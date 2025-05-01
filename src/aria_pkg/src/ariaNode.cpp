/*
*   A basic node for ros2 that runs with ariaCoda
*   To run use 'ros2 run ariaNode ariaNode -rp /dev/ttyUSB0'
*
*   Author: Kieran Quirke-Brown
*   Date: 12/01/2024
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "Aria/Aria.h"

typedef struct {
	float x;
	float y;
	float th;
	float vel;
	float rot_vel;
	float battery;
} GPS_DATA;

float cal_dist(float x, float y, float x2, float y2)
{
	float dx = x2 - x;
	float dy = y2 - y;

	return std::sqrt(dx * dx + dy * dy);
}

float cal_theta(float x, float y, float goal_x, float goal_y)
{
	float dx = goal_x - x;
	float dy = goal_y - y;

	return atan2(dy, dx) * 180 / M_PI;
}

void drive(ArRobot *robot, float vel, float rot)
{
	robot->lock();
	robot->setVel(vel);
	robot->setRotVel(rot);
	robot->unlock();
	ArUtil::sleep(500);
}

void update_data(ArRobot *robot, GPS_DATA *data)
{
	robot->lock();
	data->x = robot->getX();
	data->y = robot->getY();
	data->th = robot->getTh();
	data->vel = robot->getVel();
	data->rot_vel = robot->getRotVel();
	data->battery = robot->getBatteryVoltage();
	robot->unlock();
}

void log_data(GPS_DATA *data)
{
	ArLog::log(
		ArLog::Normal,
		"AriaNode: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
		data->x, data->y, data->th, data->vel, data->rot_vel,
		data->battery);
}

int main(int argc, char **argv)
{
	Aria::init();
	ArRobot robot;
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();

	ArRobotConnector robot_connector(&parser, &robot);
	if (!robot_connector.connectRobot()) {
		ArLog::log(ArLog::Terse,
			   "AriaNode: could not connect to the robot.");
		if (parser.checkHelpAndWarnUnparsed()) {
			Aria::logOptions();
			Aria::exit(1);
			return 1;
		}
	}
	if (!Aria::parseArgs()) {
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}

	GPS_DATA data;
    std::vector<GPS_DATA> waypoints = {
        {1000.0, 500, 0.0, 200.0, 0.0, 0.0},    // Goal 1: (1m, 0m)
        {1000.0, 1000.0, 0.0, 200.0, 0.0, 0.0},   // Goal 2: (1m, 1m)
        {0.0, 1000.0, 0.0, 200.0, 0.0, 0.0},    // Goal 3: (0m, 1m)
        {0.0, 0.0, 0.0, 200.0, 0.0, 0.0}     // Goal 4: (0m, 0m)
    };

	ArLog::log(ArLog::Terse, "AriaNode: connected:");
	robot.runAsync(true);

	robot.lock();
	robot.enableMotors();
	robot.unlock();

	update_data(&robot, &data);
	log_data(&data);

	double goal_tolerance = 50.0; // mm
    double rotation_tolerance = 5.0; // degrees
    double approach_speed = 150.0;
    double approach_rotation_speed = 30.0;

    for (const auto &goal : waypoints) {
        ArLog::log(ArLog::Normal, "AriaNode: Moving to goal (%.2f, %.2f)", goal.x, goal.y);
        bool goal_reached = false;
        while (!goal_reached) {
            update_data(&robot, &data);
            log_data(&data);

            float distance_to_goal = cal_dist(data.x, data.y, goal.x, goal.y);
            float angle_to_goal = cal_theta(data.x, data.y, goal.x, goal.y);
            float angle_difference = ArMath::subAngle(angle_to_goal, data.th);

            if (std::abs(angle_difference) > rotation_tolerance) {
                float rotation_vel = approach_rotation_speed * (angle_difference > 0 ? 1.0 : -1.0);
                drive(&robot, 0.0, rotation_vel);
		ArLog::log(ArLog::Normal, "Rot:%.2f\n", rotation_vel);
            } else if (distance_to_goal > goal_tolerance) {
                drive(&robot, approach_speed, 0.0);
            } else {
                ArLog::log(ArLog::Normal, "AriaNode: Goal (%.2f, %.2f) reached.", goal.x, goal.y);
                drive(&robot, 0.0, 0.0);
                goal_reached = true;
                ArUtil::sleep(2000);
            }
            ArUtil::sleep(100);
	   ArLog::log(ArLog::Normal, "X:%0.2f, Y:%0.2f", data.x, data.y);
        }
    }

	ArLog::log(ArLog::Normal,
		   "AriaNode: Ending robot thread...");
	robot.stopRunning();

	robot.waitForRunExit();

	ArLog::log(ArLog::Normal, "AriaNode: Exiting.");
	Aria::exit(0);
	return 0;
}
