#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // For Twist messages

class SimAriaNodeSimplified : public rclcpp::Node {
public:
    SimAriaNodeSimplified()
        : Node("sim_aria_node_simplified")
    {
        // Create a subscriber for the 'cmd_vel_team10' topic from your control_node
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_team10", 10,
            std::bind(&SimAriaNodeSimplified::cmd_vel_callback, this, std::placeholders::_1));

        // Create a publisher for the 'cmd_vel' topic that Gazebo expects
        // This is the topic Gazebo uses to control the robot's wheels/joints
        gazebo_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "SimAriaNodeSimplified ready. Passing cmd_vel_team10 to cmd_vel.");
    }

private:
    // Callback function for incoming Twist messages
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Directly publish the received Twist message to Gazebo's cmd_vel topic
        gazebo_cmd_vel_pub_->publish(*msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gazebo_cmd_vel_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimAriaNodeSimplified>());
    rclcpp::shutdown();
    return 0;
}