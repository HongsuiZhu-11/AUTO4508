#include "ariaNode.hpp"

AriaNode::AriaNode(int &argc, char **argv)
    : Node("ariaNode"), robot_() {
    Aria::init();
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobotConnector connector(&parser, &robot_);
    if (!connector.connectRobot()) {
        RCLCPP_FATAL(get_logger(), "Failed to connect to Aria robot");
        rclcpp::shutdown();
        return;
    }
    robot_.runAsync(true);
    robot_.lock();
    robot_.enableMotors();
    robot_.unlock();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_team10", 10,
        std::bind(&AriaNode::cmdVelCallback, this, std::placeholders::_1)
    );

    heartbeat_sub_ = create_subscription<std_msgs::msg::Int32>(
        "heartbeat_team10", 10,
        std::bind(&AriaNode::heartbeatCallback, this, std::placeholders::_1)
    );

    last_heartbeat_time_ = now();
}

AriaNode::~AriaNode() {
    robot_.disableMotors();
    robot_.stopRunning();
    robot_.waitForRunExit();
    Aria::exit(0);
}

void AriaNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if ((now() - last_heartbeat_time_).seconds() > 1.0) {
        robot_.lock();
        robot_.setVel(0);
        robot_.setRotVel(0);
        robot_.unlock();
        return;
    }

    robot_.lock();
    robot_.setVel(msg->linear.x);
    robot_.setRotVel(msg->angular.z);
    robot_.unlock();
}

void AriaNode::heartbeatCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    last_heartbeat_time_ = now();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AriaNode>(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
