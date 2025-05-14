#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "aria_pkg/srv/drive_distance.hpp"
#include "aria_pkg/srv/turn_angle.hpp"
#include <Aria/Aria.h>
#include <cmath>

class AriaNode : public rclcpp::Node {
    public:
	AriaNode(int &argc, char **argv)
		: Node("ariaNode")
		, robot_()
	{
		Aria::init();
		ArArgumentParser parser(&argc, argv);
		parser.loadDefaultArguments();
		ArRobotConnector connector(&parser, &robot_);
		if (!connector.connectRobot()) {
			RCLCPP_FATAL(get_logger(),
				     "Failed to connect to Aria robot");
			rclcpp::shutdown();
			return;
		}
		robot_.runAsync(true);
		robot_.lock();
		robot_.enableMotors();
		robot_.unlock();

		cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
			"cmd_vel_team10", 10,
			std::bind(&AriaNode::cmdVelCallback, this,
				  std::placeholders::_1));

		heartbeat_sub_ = create_subscription<std_msgs::msg::Int32>(
			"heartbeat_team10", 10,
			std::bind(&AriaNode::heartbeatCallback, this,
				  std::placeholders::_1));

		last_heartbeat_time_ = now();

		drive_service_ = create_service<aria_pkg::srv::DriveDistance>(
			"drive_distance",
			std::bind(&AriaNode::driveDistanceCallback, this,
				  std::placeholders::_1,
				  std::placeholders::_2));

		turn_service_ = create_service<aria_pkg::srv::TurnAngle>(
			"turn_angle", std::bind(&AriaNode::turnAngleCallback,
						this, std::placeholders::_1,
						std::placeholders::_2));

		x = 0.0;
		y = 0.0;
		rot = 0.0;
		RCLCPP_INFO(get_logger(), "Aria Node started.");
	}

	~AriaNode()
	{
		robot_.disableMotors();
		robot_.stopRunning();
		robot_.waitForRunExit();
		Aria::exit(0);
	}

    private:
	void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
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

	void heartbeatCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		last_heartbeat_time_ = now();
	}

	void driveDistanceCallback(
		const std::shared_ptr<aria_pkg::srv::DriveDistance::Request>
			request,
		std::shared_ptr<aria_pkg::srv::DriveDistance::Response> response)
	{
		double distance_mm = request->distance;
		RCLCPP_INFO(get_logger(), "Driving distance: %.2f mm",
			    distance_mm);
		robot_.lock();
		robot_.move(distance_mm);
		robot_.unlock();
		response->success = true;
		response->message = "Drive command sent.";
	}

	void turnAngleCallback(
		const std::shared_ptr<aria_pkg::srv::TurnAngle::Request> request,
		std::shared_ptr<aria_pkg::srv::TurnAngle::Response> response)
	{
		double angle_degrees = request->angle;
		RCLCPP_INFO(get_logger(), "Turning angle: %.2f degrees",
			    angle_degrees);
		robot_.lock();
		robot_.setDeltaHeading(angle_degrees);
		robot_.unlock();
		response->success = true;
		response->message = "Turn command sent.";
	}

	float x, y, rot;
	double current_forward_speed_ = 0.0;
	double current_rotation_speed_ = 0.0;

	ArRobot robot_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr heartbeat_sub_;
	rclcpp::Service<aria_pkg::srv::DriveDistance>::SharedPtr drive_service_;
	rclcpp::Service<aria_pkg::srv::TurnAngle>::SharedPtr turn_service_;

	rclcpp::Time last_heartbeat_time_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<AriaNode>(argc, argv);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}