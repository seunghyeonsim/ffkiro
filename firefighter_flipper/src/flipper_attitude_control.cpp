#include <rclcpp/rclcpp.hpp>
#include <firefighter_interfaces/msg/flipper_command.hpp>
#include <firefighter_interfaces/msg/flipper_positions.hpp>
#include <nav_msgs/msg/odometry.hpp>            // ✅ Odometry
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
        // 퍼블리셔: rho_dot을 FlipperCommand 형식으로 발행 (순서: FL, FR, RL, RR)
        flipper_pub_ = this->create_publisher<firefighter_interfaces::msg::FlipperCommand>(
            "firefighter/flipper_command", 10);

        // ✅ Odometry 구독자 (/odom 기본 가정)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){
                const auto &q = msg->pose.pose.orientation;
                qx_ = q.x;
                qy_ = q.y;
                qz_ = q.z;
                qw_ = q.w;
                odom_received_ = true;
                RCLCPP_INFO(this->get_logger(),
                            "✅ Odom received (qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f)",
                            qx_, qy_, qz_, qw_);
            }
        );

        // flipper_positions 구독자 (이제 여기서 rho를 직접 갱신)
        flipper_pos_sub_ = this->create_subscription<firefighter_interfaces::msg::FlipperPositions>(
            "flipper_positions", 10,
            [this](const firefighter_interfaces::msg::FlipperPositions::SharedPtr msg){
                last_fl_ = msg->fl;
                last_fr_ = msg->fr;
                last_rl_ = msg->rl;
                last_rr_ = msg->rr;
                has_flipper_pos_ = true;
                flipper_received_ = true;

                // rho를 flipper_positions로부터 직접 반영
                Eigen::Vector4d rho;
                rho << last_fl_, last_fr_, last_rl_, last_rr_;
                control_->setRho(rho);

                RCLCPP_INFO(this->get_logger(),
                    "✅ flipper_positions received (FL=%.4f, FR=%.4f, RL=%.4f, RR=%.4f)",
                    last_fl_, last_fr_, last_rl_, last_rr_);
            }
        );

        // 제어 초기화
        params_ = firefighter_flipper::FirefighterParams();
        control_ = std::make_unique<firefighter_flipper::FirefighterControl>(params_);
        control_->setRho(Eigen::Vector4d::Zero());
        rho_dot_.setZero();

        // 게인/제약 파라미터
        Kp_omega_      = this->declare_parameter<double>("Kp_omega", 1.0);          // 자세오차→각속도
        tau_rho_       = this->declare_parameter<double>("tau_rho", 2.0);           // ρ 수렴 시간상수 [s]
        rho_dot_max_   = this->declare_parameter<double>("rho_dot_max_deg", 20.0);  // [deg/s]
        w_task_        = this->declare_parameter<double>("w_task", 1.0);            // 작업 가중치
        w_rho_         = this->declare_parameter<double>("w_rho", 1e-3);            // ρ=0 유도 가중치(작게)
        w_damp_base_   = this->declare_parameter<double>("w_damp_base", 1e-4);      // 기본 댐핑 스케일

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
        // 토픽 수신 대기 상태 출력 (Odom, Flipper만 확인)
        if (!odom_received_ || !flipper_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "⏳ Waiting for topics... (Odom:%s, Flipper:%s)",
                odom_received_ ? "✅" : "❌",
                flipper_received_ ? "✅" : "❌");
        }

        // 최신 flipper position을 1초 쓰로틀로 표시
        if (has_flipper_pos_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "flipper_positions: FL=%.4f, FR=%.4f, RL=%.4f, RR=%.4f",
                last_fl_, last_fr_, last_rl_, last_rr_);
        }

        // rho_dot 계산 (Odometry의 orientation 사용)
        compute_rho_dot();

        // FlipperCommand 메시지로 rho_dot 발행
        auto flipper_msg = firefighter_interfaces::msg::FlipperCommand();
        flipper_msg.fl_velocity = rho_dot_(0);
        flipper_msg.fr_velocity = rho_dot_(1);
        flipper_msg.rl_velocity = rho_dot_(2);
        flipper_msg.rr_velocity = rho_dot_(3);
        flipper_pub_->publish(flipper_msg);

        // 1초마다 rho_dot 로그
        static int count = 0;
        if (++count >= 10) {
            RCLCPP_INFO(this->get_logger(),
                "rho_dot [rad/s]: FL=%.3f, FR=%.3f, RL=%.3f, RR=%.3f",
                rho_dot_(0), rho_dot_(1), rho_dot_(2), rho_dot_(3));
            count = 0;
        }
    }

    void compute_rho_dot()
    {
        const double dt = 0.1; // 10Hz

        // 현재 total 회전 (Odometry quaternion)
        Eigen::Matrix3d R_tot = quaternion_to_rotation_matrix(qx_, qy_, qz_, qw_);

        // 목표 R_d = I, 오차 회전 → body 각속도 지령
        Eigen::Matrix3d R_e = Eigen::Matrix3d::Identity() * R_tot.transpose();
        Eigen::Vector3d omega_e = firefighter_flipper::so3ToVec(firefighter_flipper::MatrixLog3(R_e));
        Eigen::Vector2d omega_xy = Kp_omega_ * omega_e.head<2>(); // [wx, wy]

        // Task Jacobian = J * diag(cos(rho))
        Eigen::Matrix<double,2,4> J = control_->jacobian();
        Eigen::Vector4d rho = control_->getRho();
        Eigen::Vector4d c = rho.array().cos().matrix();
        Eigen::Matrix4d S = c.asDiagonal();
        Eigen::Matrix<double,2,4> TaskJac = J * S;

        // 가중 최소제곱 구성
        double cond = condition_number_2x4(TaskJac);
        double w_damp = w_damp_base_ * std::max(1.0, cond / 1e4);

        Eigen::Matrix<double,10,4> A; // (2+4+4) x 4
        Eigen::Matrix<double,10,1> b; // 10 x 1
        A.setZero(); b.setZero();

        A.block<2,4>(0,0) = std::sqrt(w_task_) * TaskJac;
        b.block<2,1>(0,0) = std::sqrt(w_task_) * omega_xy;

        A.block<4,4>(2,0) = std::sqrt(w_rho_) * Eigen::Matrix4d::Identity();
        b.block<4,1>(2,0) = -std::sqrt(w_rho_) * (rho / tau_rho_);

        A.block<4,4>(6,0) = std::sqrt(w_damp) * Eigen::Matrix4d::Identity();
        b.block<4,1>(6,0) = Eigen::Vector4d::Zero();

        Eigen::Matrix4d H = A.transpose() * A;
        Eigen::Vector4d g = A.transpose() * b;
        Eigen::Vector4d rho_dot = H.ldlt().solve(g);

        // 속도 제약 (각도 한계 기반 + 속도 상한)
        const double lo = params_.RHO_LO; // 보통 -pi/2
        const double hi = 0.0;
        const double rho_dot_max = rho_dot_max_ * M_PI / 180.0;

        Eigen::Vector4d lb_angle = (Eigen::Vector4d::Constant(lo) - rho) / dt;
        Eigen::Vector4d ub_angle = (Eigen::Vector4d::Constant(hi) - rho) / dt;

        Eigen::Vector4d lb_rate = Eigen::Vector4d::Constant(-rho_dot_max);
        Eigen::Vector4d ub_rate = Eigen::Vector4d::Constant(+rho_dot_max);

        Eigen::Vector4d lb = lb_angle.cwiseMax(lb_rate);
        Eigen::Vector4d ub = ub_angle.cwiseMin(ub_rate);

        for (int i = 0; i < 4; ++i) {
            if (rho_dot(i) < lb(i)) rho_dot(i) = lb(i);
            if (rho_dot(i) > ub(i)) rho_dot(i) = ub(i);
        }

        rho_dot_ = rho_dot; // 최종 속도 명령 [rad/s]
    }

    // 2x4 행렬의 간단한 조건수
    static double condition_number_2x4(const Eigen::Matrix<double,2,4>& A)
    {
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
    rclcpp::Publisher<firefighter_interfaces::msg::FlipperCommand>::SharedPtr flipper_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;     // ✅ 변경: Odometry
    rclcpp::Subscription<firefighter_interfaces::msg::FlipperPositions>::SharedPtr flipper_pos_sub_;
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
    bool odom_received_ = false;         // ✅ 변경: IMU → Odom
    bool flipper_received_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlipperAttitudeControl>());
    rclcpp::shutdown();
    return 0;
}
