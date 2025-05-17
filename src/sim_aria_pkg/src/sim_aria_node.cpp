#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>

using namespace std::chrono_literals;

class SimController : public rclcpp::Node {
    public:
	SimController()
		: Node("sim_controller")
	{
		cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
			"cmd_vel_team10", 10);

		distance_sub_ = create_subscription<std_msgs::msg::Float32>(
			"drive_distance", 10,
			[this](const std_msgs::msg::Float32::SharedPtr msg) {
				driveDistance(msg->data);
			});

		angle_sub_ = create_subscription<std_msgs::msg::Float32>(
			"turn_angle", 10,
			[this](const std_msgs::msg::Float32::SharedPtr msg) {
				turnAngle(msg->data);
			});

		// Create a timer for continuous command publishing
		timer_ = create_wall_timer(100ms, [this]() {
			if (active_duration_ > 0s) {
				cmd_vel_pub_->publish(active_command_);
				active_duration_ -= 100ms;
			} else {
				stopRobot();
			}
		});
	}

    private:
	void driveDistance(float distance_m)
	{
		RCLCPP_INFO(get_logger(), "Driving: %.2f meters", distance_m);

		// Convert distance to time (0.2 m/s base speed)
		const float speed = 0.2;
		active_duration_ =
			std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::duration<float>(
					std::abs(distance_m) / speed));

		active_command_.linear.x = (distance_m > 0) ? speed : -speed;
		active_command_.angular.z = 0.0;
	}

	void turnAngle(float angle_rad)
	{
		RCLCPP_INFO(get_logger(), "Turning: %.2f radians", angle_rad);

		// Convert angle to time (0.5 rad/s base speed)
		const float angular_speed = 0.5;
		active_duration_ =
			std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::duration<float>(
					std::abs(angle_rad) / angular_speed));

		active_command_.linear.x = 0.0;
		active_command_.angular.z = (angle_rad > 0) ? angular_speed :
							      -angular_speed;
	}

	void stopRobot()
	{
		active_command_.linear.x = 0.0;
		active_command_.angular.z = 0.0;
		cmd_vel_pub_->publish(active_command_);
		active_duration_ = 0s;
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
	rclcpp::TimerBase::SharedPtr timer_;

	geometry_msgs::msg::Twist active_command_;
	std::chrono::milliseconds active_duration_{ 0 };
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SimController>());
	rclcpp::shutdown();
	return 0;
}