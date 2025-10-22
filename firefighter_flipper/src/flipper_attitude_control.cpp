#include <rclcpp/rclcpp.hpp>
#include <postech_flipper_msgs/msg/flipper_velocity_command.hpp>
#include <postech_flipper_msgs/msg/flipper_angle_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <chrono>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "firefighter_flipper/FirefighterControl.hpp"

class FlipperAttitudeControl : public rclcpp::Node
{
public:
    FlipperAttitudeControl() : Node("flipper_attitude_control")
    {
        // 퍼블리셔: rho_dot을 FlipperVelocityCommand 형식으로 발행 (순서: FL, FR, RL, RR)
        flipper_pub_ = this->create_publisher<postech_flipper_msgs::msg::FlipperVelocityCommand>(
            "/postech/flipper_velocity_command", 10);

        // Odometry 구독자
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/postech/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){
                const auto &q = msg->pose.pose.orientation;
                qx_ = q.x; qy_ = q.y; qz_ = q.z; qw_ = q.w;
                odom_received_ = true;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "✅ Odom received");
            }
        );

        // (조인트 이름 파라미터/JointState 경로 제거: flipper_positions를 직접 사용)

        // flipper_angle_state 구독자 (degree 단위로 받아서 radian으로 변환)
        flipper_angle_sub_ = this->create_subscription<postech_flipper_msgs::msg::FlipperAngleState>(
            "/kiro/flipper_angle_state", 10,
            [this](const postech_flipper_msgs::msg::FlipperAngleState::SharedPtr msg){
                // degree를 radian으로 변환
                last_fl_ = msg->fl * M_PI / 180.0;
                last_fr_ = msg->fr * M_PI / 180.0;
                last_rl_ = msg->rl * M_PI / 180.0;
                last_rr_ = msg->rr * M_PI / 180.0;
                has_flipper_pos_ = true;
                flipper_received_ = true;

                // flipper_angle_state로부터 rho 반영 (radian)
                Eigen::Vector4d rho;
                rho << last_fl_, last_fr_, last_rl_, last_rr_;
                control_->setRho(rho);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "✅ flipper_angle_state received (degree)");
            }
        );

        // 제어 초기화
        params_ = firefighter_flipper::FirefighterParams();
        control_ = std::make_unique<firefighter_flipper::FirefighterControl>(params_);
        control_->setRho(Eigen::Vector4d::Zero());
        rho_dot_.setZero();

        // 게인/제약 파라미터 (MATLAB 스크립트와 동일 의미) - 자세제어 강화
        Kp_omega_      = this->declare_parameter<double>("Kp_omega", 3.0);          // 자세오차→각속도 (1.0→3.0)
        tau_rho_       = this->declare_parameter<double>("tau_rho", 1.0);           // ρ 수렴 시간상수 [s] (2.0→1.0)
        rho_dot_max_   = this->declare_parameter<double>("rho_dot_max_deg", 30.0);  // [deg/s] (20.0→30.0)
        w_task_        = this->declare_parameter<double>("w_task", 2.0);            // 작업 가중치 (1.0→2.0)
        w_rho_         = this->declare_parameter<double>("w_rho", 5e-4);            // ρ=0 유도 가중치(작게) (1e-3→5e-4)
        w_damp_base_   = this->declare_parameter<double>("w_damp_base", 5e-5);      // 기본 댐핑 스케일 (1e-4→5e-5)

        // 타이머 (10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FlipperAttitudeControl::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "🔥 Flipper Attitude Control 노드 시작!");
        RCLCPP_INFO(this->get_logger(), "   - MATLAB lsqlin 유사: 가중 최소제곱 + bound 클립");
        RCLCPP_INFO(this->get_logger(), "   - 10Hz 업데이트 주기");
    }

private:
    void timer_callback()
    {
        // 토픽 수신 대기 상태 (Odom/Flipper)
        if (!odom_received_ || !flipper_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "⏳ Waiting for topics... (Odom:%s, Flipper:%s)",
                odom_received_ ? "✅" : "❌",
                flipper_received_ ? "✅" : "❌");
        }

        // 루프 상단: flipper angle 수신 여부만 1초 쓰로틀로 출력
        if (has_flipper_pos_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "flipper_angle_state: received (degree)");
        }

        // rho_dot 계산 (IMU 기반 자세 오차)
        compute_rho_dot();

        // FlipperVelocityCommand 메시지로 rho_dot 발행 (순서: FL, FR, RL, RR)
        auto flipper_msg = postech_flipper_msgs::msg::FlipperVelocityCommand();
        flipper_msg.fl = rho_dot_(0);
        flipper_msg.fr = rho_dot_(1);
        flipper_msg.rl = rho_dot_(2);
        flipper_msg.rr = rho_dot_(3);
        flipper_pub_->publish(flipper_msg);

        // 1초마다 퍼블리시 상태 로그
        static int count = 0;
        if (++count >= 10) {
            RCLCPP_INFO(this->get_logger(), "Publishing flipper command (rho_dot)");
            count = 0;
        }
    }

    void compute_rho_dot()
    {
        const double dt = 0.1; // 10Hz (타이머 주기와 일치)

        // 현재 total 회전 (IMU)
        Eigen::Matrix3d R_tot = quaternion_to_rotation_matrix(qx_, qy_, qz_, qw_);

        // 목표 R_d = I (수평). 오차 회전(바디 기준) → body 각속도 지령
        Eigen::Matrix3d R_e = Eigen::Matrix3d::Identity() * R_tot.transpose();
        Eigen::Vector3d omega_e = firefighter_flipper::so3ToVec(firefighter_flipper::MatrixLog3(R_e));
        Eigen::Vector2d omega_xy = Kp_omega_ * omega_e.head<2>(); // [wx, wy]

        // Task Jacobian = J * diag(cos(rho))
        Eigen::Matrix<double,2,4> J = control_->jacobian();
        Eigen::Vector4d rho = control_->getRho();
        Eigen::Vector4d c = rho.array().cos().matrix();
        Eigen::Matrix4d S = c.asDiagonal();
        Eigen::Matrix<double,2,4> TaskJac = J * S;

        // === MATLAB의 lsqlin 구성과 동등한 가중 최소제곱 쌓기 ===
        // A = [ sqrt(w_task)*TaskJac ;
        //       sqrt(w_rho )*I4     ;
        //       sqrt(w_damp)*I4     ];
        // b = [ sqrt(w_task)*omega_xy       ;
        //       -sqrt(w_rho)*(rho/tau_rho)  ;
        //       0                           ];
        // 여기서 w_damp는 조건수 기반으로 가변화
        double cond = condition_number_2x4(TaskJac);
        double w_damp = w_damp_base_ * std::max(1.0, cond / 1e4);

        Eigen::Matrix<double,10,4> A; // (2+4+4) x 4
        Eigen::Matrix<double,10,1> b; // 10 x 1

        A.setZero(); b.setZero();

        // 블록1: 작업항
        A.block<2,4>(0,0) = std::sqrt(w_task_) * TaskJac;
        b.block<2,1>(0,0) = std::sqrt(w_task_) * omega_xy;

        // 블록2: rho를 0으로 완만히 유도 (rho_dot ≈ -rho/τ)
        A.block<4,4>(2,0) = std::sqrt(w_rho_) * Eigen::Matrix4d::Identity();
        b.block<4,1>(2,0) = -std::sqrt(w_rho_) * (rho / tau_rho_);

        // 블록3: 댐핑(정규화)
        A.block<4,4>(6,0) = std::sqrt(w_damp) * Eigen::Matrix4d::Identity();
        b.block<4,1>(6,0) = Eigen::Vector4d::Zero();

        // 정규방정식 풀이 (A^T A x = A^T b)
        Eigen::Matrix4d H = A.transpose() * A;
        Eigen::Vector4d g = A.transpose() * b;
        Eigen::Vector4d rho_dot = H.ldlt().solve(g);

        // === 속도 제약 (각도 한계 기반 + 속도 상한) ===
        // 각도한계: [lo, hi] = [P.RHO_LO, 0]
        const double lo = params_.RHO_LO; // 보통 -pi/2
        const double hi = 0.0;
        // 속도 상한: rho_dot_max_ [deg/s] → [rad/s]
        const double rho_dot_max = rho_dot_max_ * M_PI / 180.0;

        // 각도 한계를 속도 한계로 변환
        Eigen::Vector4d lb_angle = (Eigen::Vector4d::Constant(lo) - rho) / dt;
        Eigen::Vector4d ub_angle = (Eigen::Vector4d::Constant(hi) - rho) / dt;

        // 속도 상한/하한
        Eigen::Vector4d lb_rate = Eigen::Vector4d::Constant(-rho_dot_max);
        Eigen::Vector4d ub_rate = Eigen::Vector4d::Constant(+rho_dot_max);

        // 최종 bound = 교집합
        Eigen::Vector4d lb = lb_angle.cwiseMax(lb_rate);
        Eigen::Vector4d ub = ub_angle.cwiseMin(ub_rate);

        // 클리핑
        for (int i = 0; i < 4; ++i) {
            if (rho_dot(i) < lb(i)) rho_dot(i) = lb(i);
            if (rho_dot(i) > ub(i)) rho_dot(i) = ub(i);
        }

        rho_dot_ = rho_dot; // 최종 속도 명령 [rad/s]
        // (참고) 실제 rho 적분/클립은 JointState 콜백을 통해 외부에서 관리되거나, 모터/드라이버가 수행
    }

    // 2x4 행렬의 간단한 조건수(σ_max / σ_min) 계산
    static double condition_number_2x4(const Eigen::Matrix<double,2,4>& A)
    {
        // Only singular values are needed; don't request U/V on fixed-size matrices
        Eigen::JacobiSVD<Eigen::Matrix<double,2,4>> svd(A);
        const auto sv = svd.singularValues();
        double smax = (sv.size() > 0) ? sv(0) : 0.0;
        double smin = (sv.size() > 1) ? sv(1) : 0.0;
        if (smin < 1e-12) return 1e12;
        return smax / smin;
    }

    static Eigen::Matrix3d quaternion_to_rotation_matrix(double x, double y, double z, double w)
    {
        double n = std::sqrt(x*x + y*y + z*z + w*w);
        if (n < 1e-12) {
            return Eigen::Matrix3d::Identity();
        }
        x/=n; y/=n; z/=n; w/=n;
        Eigen::Matrix3d R;
        R << 1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y),
             2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x),
             2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y);
        return R;
    }

    // 멤버 변수
    rclcpp::Publisher<postech_flipper_msgs::msg::FlipperVelocityCommand>::SharedPtr flipper_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<postech_flipper_msgs::msg::FlipperAngleState>::SharedPtr flipper_angle_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    firefighter_flipper::FirefighterParams params_;
    std::unique_ptr<firefighter_flipper::FirefighterControl> control_;
    Eigen::Vector4d rho_dot_;
    double qx_ = 0.0, qy_ = 0.0, qz_ = 0.0, qw_ = 1.0;

    // flipper_positions 최신값 저장
    bool has_flipper_pos_ = false;
    double last_fl_ = 0.0;
    double last_fr_ = 0.0;
    double last_rl_ = 0.0;
    double last_rr_ = 0.0;

    // 제어/튜닝 파라미터
    double Kp_omega_;
    double tau_rho_;
    double rho_dot_max_;
    double w_task_;
    double w_rho_;
    double w_damp_base_;

    // 상태 플래그
    bool odom_received_ = false;
    bool flipper_received_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlipperAttitudeControl>());
    rclcpp::shutdown();
    return 0;
}


