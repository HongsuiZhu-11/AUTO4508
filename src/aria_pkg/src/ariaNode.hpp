#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
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

	ArRobot robot_;

	GPS_DATA data_;

	std::vector<GPS_DATA> waypoints_;

	// Helpers
	void onGPS(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void updateData();
	void logData();
	void driveStraight(double vel);
	void driveTurn(double rot_vel);
	static double calDist(double x, double y, double x2, double y2);
	static double calTheta(double x, double y, double goal_x,
			       double goal_y);
	void turnToAngle(double target_heading, double tol_deg = 2.0);

	bool stop_requested_;
};
