#include "ariaNode.hpp"
#include <signal.h>

static bool g_stop = false;
void signalHandler(int)
{
	g_stop = true;
}

AriaNode::AriaNode(int &argc, char **argv)
	: Node("aria_node")
	, robot_()
	, stop_requested_(false)
{
	Aria::init();
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobotConnector connector(&parser, &robot_);
	if (!connector.connectRobot()) {
		RCLCPP_FATAL(get_logger(), "Could not connect to Aria robot");
		rclcpp::shutdown();
		return;
	}
	robot_.runAsync(true);
	robot_.lock();
	robot_.enableMotors();
	robot_.unlock();

	gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
		"/fix", 10,
		std::bind(&AriaNode::onGPS, this, std::placeholders::_1));

	waypoints_ = { { 1000, 500, 0, 200, 0, 0 },
		       { 1000, 1000, 0, 200, 0, 0 },
		       { 0, 1000, 0, 200, 0, 0 },
		       { 0, 0, 0, 200, 0, 0 } };

	signal(SIGINT, signalHandler);
}

AriaNode::~AriaNode()
{
	robot_.disableMotors();
	robot_.stopRunning();
	robot_.waitForRunExit();
	Aria::exit(0);
}

void AriaNode::onGPS(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
	RCLCPP_INFO(get_logger(), "GPS: lat=%.6f lon=%.6f alt=%.2f",
		    msg->latitude, msg->longitude, msg->altitude);
}

void AriaNode::updateData()
{
	robot_.lock();
	data_.x = robot_.getX();
	data_.y = robot_.getY();
	data_.th = robot_.getTh();
	data_.vel = robot_.getVel();
	data_.rot_vel = robot_.getRotVel();
	data_.battery = robot_.getBatteryVoltage();
	robot_.unlock();
}

void AriaNode::logData()
{
	RCLCPP_INFO(get_logger(),
		    "Pose=(%.2f,%.2f,%.2f) Vel=%.2f Rot=%.2f Batt=%.2fV",
		    data_.x, data_.y, data_.th, data_.vel, data_.rot_vel,
		    data_.battery);
}

void AriaNode::driveStraight(double vel)
{
	robot_.lock();
	robot_.setVel(vel);
	robot_.setRotVel(0);
	robot_.unlock();
	ArUtil::sleep(500);
}

void AriaNode::driveTurn(double rot_vel)
{
	robot_.lock();
	robot_.setVel(0);
	robot_.setRotVel(rot_vel);
	robot_.unlock();
	ArUtil::sleep(500);
}

double AriaNode::calDist(double x, double y, double x2, double y2)
{
	double dx = x2 - x, dy = y2 - y;
	return std::hypot(dx, dy);
}

double AriaNode::calTheta(double x, double y, double goal_x, double goal_y)
{
	double dx = goal_x - x, dy = goal_y - y;
	return std::atan2(dy, dx) * 180.0 / M_PI;
}

void AriaNode::turnToAngle(double target_heading, double tol_deg)
{
	rclcpp::Rate rate(10);

	while (rclcpp::ok()) {
		double current = robot_.getTh();

		double error = ArMath::subAngle(target_heading, current);

		if (std::fabs(error) <= tol_deg) {
			robot_.lock();
			robot_.setRotVel(0);
			robot_.unlock();
			break;
		}

		double vel = turn_kp_ * error; // deg/s
		vel = std::max(-max_turn_vel_, std::min(max_turn_vel_, vel));

		robot_.lock();
		robot_.setVel(0);
		robot_.setRotVel(vel);
		robot_.unlock();

		rate.sleep();
	}
}

void AriaNode::run()
{
	const double goal_tol = 50.0; // mm
	const double rot_tol = 5.0; // degrees

	for (auto &wp : waypoints_) {
		RCLCPP_INFO(get_logger(), "Moving to (%.2f,%.2f)", wp.x, wp.y);
		bool reached = false;
		while (!reached && !g_stop) {
			updateData();
			logData();

			double dist = calDist(data_.x, data_.y, wp.x, wp.y);
			double heading = calTheta(data_.x, data_.y, wp.x, wp.y);
			double diff = ArMath::subAngle(heading, data_.th);

			if (std::fabs(diff) > rot_tol) {
				double rv =
					wp.rot_vel * (diff > 0 ? 1.0 : -1.0);
				driveTurn(rv);
			} else if (dist > goal_tol) {
				driveStraight(wp.vel);
			} else {
				RCLCPP_INFO(get_logger(), "Reached (%.2f,%.2f)",
					    wp.x, wp.y);
				driveStraight(0);
				reached = true;
				ArUtil::sleep(2000);
			}
		}
		if (g_stop)
			break;
	}

	RCLCPP_INFO(get_logger(), "Shutting down.");
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<AriaNode>(argc, argv);
	node->run();
	rclcpp::shutdown();
	return 0;
}
