#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class SimControllerSimplified : public rclcpp::Node {
public:
    SimControllerSimplified()
    : Node("sim_controller_simplified")
    {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel_team10", 10);

        distance_sub_ = create_subscription<std_msgs::msg::Float32>(
            "drive_distance", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                driveDistanceCallback(msg->data);
            });

        angle_sub_ = create_subscription<std_msgs::msg::Float32>(
            "turn_angle", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                turnAngleCallback(msg->data);
            });

        timer_ = create_wall_timer(100ms, [this]() {
            timerCallback();
        });

        linear_speed_ = 1.0;   // m/s
        angular_speed_ = 2.0;  // rad/s
        time_factor_ = 1.15;
        driving_ = false;
        turning_ = false;
    }

private:
    void driveDistanceCallback(float distance_m)
    {
        RCLCPP_INFO(get_logger(), "Driving: %.2f meters", distance_m);
        if (!driving_ && !turning_) {
            driving_ = true;
            start_time_ = now();
            active_duration_ =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<float>(
                        std::abs(distance_m) / linear_speed_ * time_factor_));
            active_command_.linear.x = (distance_m > 0) ? linear_speed_ : -linear_speed_;
            active_command_.angular.z = 0.0;
        }
    }

    void turnAngleCallback(float angle_rad)
    {
        RCLCPP_INFO(get_logger(), "Turning: %.2f radians", angle_rad);
        if (!driving_ && !turning_) {
            turning_ = true;
            start_time_ = now();
            active_duration_ =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<float>(
                        std::abs(angle_rad) / angular_speed_ * time_factor_));
            active_command_.linear.x = 0.0;
            active_command_.angular.z = (angle_rad > 0) ? angular_speed_ : -angular_speed_;
            RCLCPP_INFO(get_logger(), "Setting angular velocity: %.2f rad/s for %.2ld ms", active_command_.angular.z, active_duration_.count());
        }
    }

    void timerCallback()
    {
        if (driving_ || turning_) {
            auto elapsed_time = now() - start_time_;
            if (elapsed_time < active_duration_) {
                cmd_vel_pub_->publish(active_command_);
            } else {
                stopRobot();
                driving_ = false;
                turning_ = false;
            }
        } else {
            // Optionally publish zero velocity when not actively driving/turning
            // geometry_msgs::msg::Twist stop_command;
            // cmd_vel_pub_->publish(stop_command);
        }
    }

    void stopRobot()
    {
        geometry_msgs::msg::Twist stop_command;
        cmd_vel_pub_->publish(stop_command);
        active_command_.linear.x = 0.0;
        active_command_.angular.z = 0.0;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist active_command_;
    rclcpp::Time start_time_;
    std::chrono::milliseconds active_duration_{ 0 };
    float linear_speed_;
    float angular_speed_;
    float time_factor_;
    bool driving_;
    bool turning_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimControllerSimplified>());
    rclcpp::shutdown();
    return 0;
}