#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <Aria/Aria.h>

class AriaNode : public rclcpp::Node {
public:
    AriaNode(int &argc, char **argv);
    ~AriaNode();

private:
    ArRobot robot_;

    double current_forward_speed_ = 0.0;
    double current_rotation_speed_ = 0.0;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr heartbeat_sub_;

    rclcpp::Time last_heartbeat_time_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void heartbeatCallback(const std_msgs::msg::Int32::SharedPtr msg);
};
