#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <firefighter_interfaces/msg/flipper_positions.hpp>
#include <unordered_map>
#include <string>

class FlipperPositionsNode : public rclcpp::Node
{
public:
    FlipperPositionsNode()
    : rclcpp::Node("flipper_positions_node")
    {
        // Declare parameters for joint names with sensible defaults
        joint_fl_name_ = this->declare_parameter<std::string>("joint_fl", "Flipper_FL");
        joint_fr_name_ = this->declare_parameter<std::string>("joint_fr", "Flipper_FR");
        joint_rl_name_ = this->declare_parameter<std::string>("joint_rl", "Flipper_RL");
        joint_rr_name_ = this->declare_parameter<std::string>("joint_rr", "Flipper_RR");

        publisher_ = this->create_publisher<firefighter_interfaces::msg::FlipperPositions>(
            "flipper_positions", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&FlipperPositionsNode::on_joint_state, this, std::placeholders::_1));
    }

private:
    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Build name->position map for quick lookup
        std::unordered_map<std::string, double> position_by_name;
        position_by_name.reserve(msg->name.size());
        const size_t count = std::min(msg->name.size(), msg->position.size());
        for (size_t i = 0; i < count; ++i) {
            position_by_name[msg->name[i]] = msg->position[i];
        }

        firefighter_interfaces::msg::FlipperPositions out;
        out.fl = getPosition(position_by_name, joint_fl_name_);
        out.fr = getPosition(position_by_name, joint_fr_name_);
        out.rl = getPosition(position_by_name, joint_rl_name_);
        out.rr = getPosition(position_by_name, joint_rr_name_);

        publisher_->publish(out);
    }

    static double getPosition(const std::unordered_map<std::string, double>& map,
                              const std::string& key)
    {
        auto it = map.find(key);
        if (it != map.end()) return it->second;
        return 0.0;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<firefighter_interfaces::msg::FlipperPositions>::SharedPtr publisher_;

    std::string joint_fl_name_;
    std::string joint_fr_name_;
    std::string joint_rl_name_;
    std::string joint_rr_name_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlipperPositionsNode>());
    rclcpp::shutdown();
    return 0;
}


