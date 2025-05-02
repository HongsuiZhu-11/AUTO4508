#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <chrono>

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Accessing LaserScan fields
    float angle_min = scan->angle_min;
    float angle_max = scan->angle_max;
    float range_min = scan->range_min;
    float range_max = scan->range_max;

    int forward_index = scan->ranges.size() / 2;


    // Accessing range data
    std::vector<float> ranges = scan->ranges;
    std::vector<float> intensities = scan->intensities;

    ROS_INFO("Angle Min: %f, Angle Max: %f", angle_min, angle_max);
    ROS_INFO("Range Min: %f, Range Max: %f", range_min, range_max);
    ROS_INFO("First range value: %f", ranges[0]);
}


void shutdownHandler(int sig) {
    ROS_WARN("Shutdown signal received. Cleaning up...");
    ros::shutdown();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_subscriber");
    signal(SIGINT, shutdownHandler);  // Capture Ctrl+C
    ros::NodeHandle LIDAR_nh;
    
    ros::Subscriber scan_sub = LIDAR_nh.subscribe("/scan", 10, scanCallback);
    while(ros::ok()){
      ros::spinOnce();
      
    }
    

    return 0;
}