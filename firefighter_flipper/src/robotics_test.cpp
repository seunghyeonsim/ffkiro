#include <rclcpp/rclcpp.hpp>
#include "firefighter_flipper/Robotics.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "firefighter_flipper/FirefighterControl.hpp"
#include <iostream>

class RoboticsTest : public rclcpp::Node
{
public:
    RoboticsTest() : Node("robotics_test")
    {
        RCLCPP_INFO(this->get_logger(), "🤖 Robotics 함수 테스트 시작!");
        
        // IMU 서브스크라이버 생성
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&RoboticsTest::imu_callback, this, std::placeholders::_1));
        
        // 테스트 실행
        test_functions();

        // Control 테스트: rho_dot 계산 및 R_b 적분
        firefighter_flipper::FirefighterParams params;
        control_ = std::make_unique<firefighter_flipper::FirefighterControl>(params);
        control_->setRho(Eigen::Vector4d::Zero());
        rho_dot_.setZero();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
            const double dt = 0.1; // 10 Hz

            // 1) 현재 바디 회전 (IMU 기반, universe 기준)
            Eigen::Matrix3d R_tot = quaternion_to_rotation_matrix(qx_, qy_, qz_, qw_);

            // 2) 오차 회전 (body 기준) → 원하는 각속도
            Eigen::Matrix3d R_d = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d R_e = R_d * R_tot.transpose();
            Eigen::Matrix3d logRe = firefighter_flipper::MatrixLog3(R_e);
            Eigen::Vector3d omega_e = firefighter_flipper::so3ToVec(logRe);
            Eigen::Vector2d omega_xy = omega_e.head<2>(); // Kp=1

            // 3) 플리퍼 최적 각속도 (정규화된 최소제곱)
            Eigen::Matrix<double,2,4> J = control_->jacobian();
            Eigen::Vector4d rho = control_->getRho();
            Eigen::Vector4d c = rho.array().cos().matrix();
            Eigen::Matrix4d Scaling = c.asDiagonal();
            Eigen::Matrix<double,2,4> A = J * Scaling;
            Eigen::Vector2d b = omega_xy;

            // 조건수 추정 및 정규화 가중치
            Eigen::JacobiSVD<Eigen::Matrix<double,2,4>> svd(A); // singular values only
            double smax = 0.0, smin = 1e12;
            for (int i = 0; i < svd.singularValues().size(); ++i) {
                smax = std::max(svd.singularValues()(i), smax);
                smin = std::min(svd.singularValues()(i), smin);
            }
            double cond = (smin < 1e-12) ? 1e12 : (smax / smin);
            double alpha = 1e-4 * std::max(1.0, cond / 1e4);

            // Tikhonov regularized least squares
            Eigen::Matrix4d H = A.transpose() * A + alpha * Eigen::Matrix4d::Identity();
            Eigen::Vector4d g = A.transpose() * b;
            Eigen::Vector4d rho_dot = H.ldlt().solve(g);

            // 속도 제약 (각도 한계 기반): [-pi, 0]
            const double lo = -M_PI;
            const double hi = 0.0;
            Eigen::Vector4d lb = (Eigen::Vector4d::Constant(lo) - rho) / dt;
            Eigen::Vector4d ub = (Eigen::Vector4d::Constant(hi) - rho) / dt;
            for (int i = 0; i < 4; ++i) rho_dot(i) = std::min(std::max(rho_dot(i), lb(i)), ub(i));

            // 상태 업데이트 및 로그
            rho_dot_ = rho_dot;
            Eigen::Vector4d rho_next = rho + dt * rho_dot_;
            control_->setRho(rho_next);
            control_->integrateRb(rho_dot_, dt);

            static int tick = 0;
            if (++tick % 10 == 0) { // 1초마다 출력
                RCLCPP_INFO(this->get_logger(),
                    "rho_dot = [%.3f %.3f %.3f %.3f] | cond=%.1f alpha=%.2e",
                    rho_dot_(0), rho_dot_(1), rho_dot_(2), rho_dot_(3), cond, alpha);
            }
        });
        
        RCLCPP_INFO(this->get_logger(), "📡 /imu 토픽 구독 시작...");
    }

private:
    void test_functions()
    {
        // NearZero 테스트
        RCLCPP_INFO(this->get_logger(), "=== NearZero 테스트 ===");
        RCLCPP_INFO(this->get_logger(), "NearZero(1e-7): %s", 
            firefighter_flipper::NearZero(1e-7) ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "NearZero(1e-4): %s", 
            firefighter_flipper::NearZero(1e-4) ? "true" : "false");
        
        // VecToso3 테스트
        RCLCPP_INFO(this->get_logger(), "=== VecToso3 테스트 ===");
        Eigen::Vector3d omg(1, 2, 3);
        Eigen::Matrix3d so3mat = firefighter_flipper::VecToso3(omg);
        RCLCPP_INFO(this->get_logger(), "입력 벡터: [%.1f, %.1f, %.1f]", omg(0), omg(1), omg(2));
        RCLCPP_INFO(this->get_logger(), "출력 행렬:\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f",
            so3mat(0,0), so3mat(0,1), so3mat(0,2),
            so3mat(1,0), so3mat(1,1), so3mat(1,2),
            so3mat(2,0), so3mat(2,1), so3mat(2,2));
        
        // so3ToVec 테스트
        RCLCPP_INFO(this->get_logger(), "=== so3ToVec 테스트 ===");
        Eigen::Vector3d recovered_omg = firefighter_flipper::so3ToVec(so3mat);
        RCLCPP_INFO(this->get_logger(), "복원된 벡터: [%.1f, %.1f, %.1f]", 
            recovered_omg(0), recovered_omg(1), recovered_omg(2));
        
        // MatrixLog3 테스트
        RCLCPP_INFO(this->get_logger(), "=== MatrixLog3 테스트 ===");
        // 90도 Z축 회전 행렬
        Eigen::Matrix3d R;
        R << 0, -1, 0,
             1,  0, 0,
             0,  0, 1;
        
        Eigen::Matrix3d log_R = firefighter_flipper::MatrixLog3(R);
        RCLCPP_INFO(this->get_logger(), "회전 행렬:\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f",
            R(0,0), R(0,1), R(0,2),
            R(1,0), R(1,1), R(1,2),
            R(2,0), R(2,1), R(2,2));
        
        RCLCPP_INFO(this->get_logger(), "로그 행렬:\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f",
            log_R(0,0), log_R(0,1), log_R(0,2),
            log_R(1,0), log_R(1,1), log_R(1,2),
            log_R(2,0), log_R(2,1), log_R(2,2));
        
        RCLCPP_INFO(this->get_logger(), "✅ 모든 테스트 완료!");
    }
    
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 최신 쿼터니언 저장
        qx_ = msg->orientation.x;
        qy_ = msg->orientation.y;
        qz_ = msg->orientation.z;
        qw_ = msg->orientation.w;

        // 쿼터니언을 회전 행렬로 변환
        Eigen::Matrix3d rotation_matrix = quaternion_to_rotation_matrix(
            qx_, qy_, qz_, qw_);
        
        // 회전 행렬 출력 (1초마다)
        static int count = 0;
        if (++count >= 10) {  // 10Hz로 출력
            RCLCPP_INFO(this->get_logger(), "🔄 IMU 회전 행렬:");
            RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", 
                rotation_matrix(0,0), rotation_matrix(0,1), rotation_matrix(0,2));
            RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", 
                rotation_matrix(1,0), rotation_matrix(1,1), rotation_matrix(1,2));
            RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", 
                rotation_matrix(2,0), rotation_matrix(2,1), rotation_matrix(2,2));
            
            // MatrixLog3 함수로 로그 행렬 계산
            Eigen::Matrix3d log_matrix = firefighter_flipper::MatrixLog3(rotation_matrix);
            RCLCPP_INFO(this->get_logger(), "📊 로그 행렬:");
            RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", 
                log_matrix(0,0), log_matrix(0,1), log_matrix(0,2));
            RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", 
                log_matrix(1,0), log_matrix(1,1), log_matrix(1,2));
            RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", 
                log_matrix(2,0), log_matrix(2,1), log_matrix(2,2));
            
            count = 0;
        }
    }
    
    Eigen::Matrix3d quaternion_to_rotation_matrix(double x, double y, double z, double w)
    {
        // 쿼터니언 정규화
        double norm = std::sqrt(x*x + y*y + z*z + w*w);
        x /= norm; y /= norm; z /= norm; w /= norm;
        
        // 회전 행렬 계산
        Eigen::Matrix3d R;
        R << 1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y),
             2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x),
             2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y);
        
        return R;
    }
    
    // 멤버 변수
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::unique_ptr<firefighter_flipper::FirefighterControl> control_;
    Eigen::Vector4d rho_dot_;
    rclcpp::TimerBase::SharedPtr timer_;
    // 최신 IMU 쿼터니언 (초기값: 항등 회전)
    double qx_ = 0.0;
    double qy_ = 0.0;
    double qz_ = 0.0;
    double qw_ = 1.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboticsTest>());
    rclcpp::shutdown();
    return 0;
}
