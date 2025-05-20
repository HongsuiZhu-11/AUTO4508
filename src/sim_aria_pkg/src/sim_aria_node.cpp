#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

class SimControllerSimplified : public rclcpp::Node {
    public:
	SimControllerSimplified()
		: Node("sim_controller_simplified")
	{
		cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
			"cmd_vel_team10", 10);

		// Configure speeds and time factor
		linear_speed_ = 0.5; // m/s
		angular_speed_ = 1.0; // rad/s
		time_factor_ = 1.2;

		// Subscriptions
		distance_sub_ = create_subscription<std_msgs::msg::Float32>(
			"drive_distance", 10,
			[this](const std_msgs::msg::Float32::SharedPtr msg) {
				std::lock_guard<std::mutex> lock(
					command_mutex_);
				if (!current_drive_active_) {
					float distance = msg->data;
					drive_duration_ = std::chrono::duration_cast<
						std::chrono::milliseconds>(
						std::chrono::duration<float>(
							std::abs(distance) /
							linear_speed_ *
							time_factor_));
					drive_start_ = this->now();
					current_drive_active_ = true;
					current_command_.linear.x =
						(distance > 0) ? linear_speed_ :
								 -linear_speed_;
				}
			});

		angle_sub_ = create_subscription<std_msgs::msg::Float32>(
			"turn_angle", 10,
			[this](const std_msgs::msg::Float32::SharedPtr msg) {
				std::lock_guard<std::mutex> lock(
					command_mutex_);
				if (!current_turn_active_) {
					float angle = msg->data;
					turn_duration_ = std::chrono::duration_cast<
						std::chrono::milliseconds>(
						std::chrono::duration<float>(
							std::abs(angle) /
							angular_speed_ *
							time_factor_));
					turn_start_ = this->now();
					current_turn_active_ = true;
					current_command_.angular.z =
						(angle > 0) ? angular_speed_ :
							      -angular_speed_;
				}
			});

		// Main command timer
		timer_ = create_wall_timer(50ms, [this]() {
			std::lock_guard<std::mutex> lock(command_mutex_);
			auto now = this->now();

			// Check drive timeout
			if (current_drive_active_ &&
			    (now - drive_start_) > drive_duration_) {
				current_command_.linear.x = 0.0;
				current_drive_active_ = false;
			}

			// Check turn timeout
			if (current_turn_active_ &&
			    (now - turn_start_) > turn_duration_) {
				current_command_.angular.z = 0.0;
				current_turn_active_ = false;
			}

			cmd_vel_pub_->publish(current_command_);
		});
	}

    private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
	rclcpp::TimerBase::SharedPtr timer_;

	geometry_msgs::msg::Twist current_command_;
	std::mutex command_mutex_;

	// Drive parameters
	float linear_speed_;
	float angular_speed_;
	float time_factor_;
	bool current_drive_active_ = false;
	bool current_turn_active_ = false;
	rclcpp::Time drive_start_;
	rclcpp::Time turn_start_;
	std::chrono::milliseconds drive_duration_{ 0 };
	std::chrono::milliseconds turn_duration_{ 0 };
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SimControllerSimplified>());
	rclcpp::shutdown();
	return 0;
}