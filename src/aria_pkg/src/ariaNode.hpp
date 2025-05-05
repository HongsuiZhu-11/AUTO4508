#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <Aria/Aria.h>
#include <vector>
#include <cmath>

typedef struct {
	double x, y, th, vel, rot_vel, battery;
} GPS_DATA;

class AriaNode : public rclcpp::Node {
    public:
	AriaNode(int &argc, char **argv);
	~AriaNode();

	void run();

    private:
	double turn_kp_ = 1.0; // [deg error] → [deg/s rotation vel]
	double max_turn_vel_ = 60; // cap on rotation speed (deg/s)

	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

	bool enabled_ = false;
	float currentForwardSpeed = 0.0f;
	float currentRotationSpeed = 0.0f;

	// Joystick state
	rclcpp::Time last_joy_time_;
	bool auto_mode_   = false;
	bool manual_mode_ = false;
	bool deadman_trigger = false;

	// Dead-man switch: back pedals indices (example: L2 and R2 axes)
	static constexpr int PEDAL_L_AXIS = 4;
	static constexpr int PEDAL_R_AXIS = 5;
	static constexpr float PEDAL_THRESHOLD = 0.1f;

	// Mode buttons
	static constexpr int BUTTON_X = 2;  // “X” on PS-style pads
	static constexpr int BUTTON_O = 1;  // “O” (circle)

	// Manual driving axes
	static constexpr int AXIS_STEER = 0;    // left stick horizontal
	static constexpr int AXIS_FORWARD = 1;  // left stick vertical

	ArRobot robot_;
	bool stop_requested_;

	GPS_DATA data_;

	std::vector<GPS_DATA> waypoints_;

	// Helpers
	void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
	void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg);

	void updateData();
	void logData();
	void driveStraight(double vel);
	void driveTurn(double rot_vel);
	static double calDist(double x, double y, double x2, double y2);
	static double calTheta(double x, double y, double goal_x,
			       double goal_y);
	void turnToAngle(double target_heading, double tol_deg = 2.0);
};
