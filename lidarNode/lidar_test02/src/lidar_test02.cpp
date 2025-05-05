/**
* \file
* \brief This project contains a tiny ROS-2 example. It shows how to use sick_scan_xd messages
* and services in a ROS-2 application.
*
* Copyright (C) 2022 Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2022, SICK AG, Waldkirch
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Osnabrueck University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*     * Neither the name of SICK AG nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*
*  Copyright 2022 SICK AG
*  Copyright 2022 Ing.-Buero Dr. Michael Lehning
*
*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>
#include <thread>

#include <sick_scan_xd/msg/radar_scan.hpp>           // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/msg/encoder.hpp>              // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/msg/li_doutputstate_msg.hpp>  // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/msg/lf_erec_msg.hpp>          // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/srv/cola_msg_srv.hpp>         // generated in sick_scan_xd by rosidl-generator

#define RCLCPP_LOGGER         rclcpp::get_logger("lidar_test02")
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(RCLCPP_LOGGER,__VA_ARGS__)

/*
* @brief class SickScanMessageReceiver subscribes to pointcloud, lidoutputstate and lferec messages. Tiny example how to subscribe to sick_scan messages.
*/
class SickScanMessageReceiver {
    public:
        /** Constructor */
        SickScanMessageReceiver(rclcpp::Node::SharedPtr node, const std::string& scan_topic = "scan")
        {
            if (node)
            {
                m_laser_scan_subscriber = node->create_subscription<sensor_msgs::msg::LaserScan>(
                    scan_topic, 10, std::bind(&SickScanMessageReceiver::messageCbLaserScanROS2, this, std::placeholders::_1));
            }
        }
    
        /** Getter method to retrieve the latest Laser Scan message */
        std::shared_ptr<sensor_msgs::msg::LaserScan> getLatestLaserScan() const {
            return latest_msg;
        }
    
    protected:
        /** ROS2 callback for receiving Laser Scan messages */
        void messageCbLaserScanROS2(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
        {
            latest_msg = msg;  // Store the latest received message
            ROS_INFO_STREAM("sick_scan_ros2_example: pointcloud received, angle_min: " << msg->angle_min << ", angle_max: " << msg->angle_max);
        }
    
        /** Private member to store the latest received message */
        std::shared_ptr<sensor_msgs::msg::LaserScan> latest_msg;
    
        /** Subscriber for pointcloud messages */
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_scan_subscriber;
    };
    

// /** Sends a sick_scan ColaMsg service request */
// bool sendSickScanServiceRequest(rclcpp::Node::SharedPtr node, rclcpp::Client<sick_scan_xd::srv::ColaMsgSrv>::SharedPtr sick_scan_srv_colamsg_client, const std::string& cola_msg)
// {
//     std::shared_ptr<sick_scan_xd::srv::ColaMsgSrv::Request> sick_scan_srv_request = std::make_shared<sick_scan_xd::srv::ColaMsgSrv::Request>();
//     sick_scan_srv_request->request = cola_msg;
//     ROS_INFO_STREAM("sick_scan_ros2_example: sick_scan service request: \"" << sick_scan_srv_request->request << "\"");
//     std::shared_future<std::shared_ptr<sick_scan_xd::srv::ColaMsgSrv::Response>> sick_scan_srv_result = sick_scan_srv_colamsg_client->async_send_request(sick_scan_srv_request);
//     if (rclcpp::spin_until_future_complete(node, sick_scan_srv_result) == rclcpp::FutureReturnCode::SUCCESS)
//     {
//         ROS_INFO_STREAM("sick_scan_ros2_example: sick_scan service response: \"" << sick_scan_srv_result.get()->response << "\"");
//         return true;
//     }
//     else
//     {
//         ROS_ERROR_STREAM("## ERROR sick_scan_ros2_example: sick_scan service request failed");
//         return false;
//     }
// }

// /*
// * @brief Tiny ROS-2 example to show how to use sick_scan_xd messages and services in a ROS-2 application. Example for TiM7xx topics.
// */
// int getLIDAR(rclcpp::Node::SharedPtr node)
// {
//     SickScanMessageReceiver sickscan_message_receiver(node, "/scan");

//     // Create a sick_scan service client
//     rclcpp::Client<sick_scan_xd::srv::ColaMsgSrv>::SharedPtr sick_scan_srv_colamsg_client = node->create_client<sick_scan_xd::srv::ColaMsgSrv>("ColaMsg");
//     while (rclcpp::ok() && !sick_scan_srv_colamsg_client->wait_for_service(std::chrono::seconds(1)))
//     {
//        ROS_INFO_STREAM("sick_scan_ros2_example: Waiting for sick_scan service...");
//     }
//     if (rclcpp::ok())
//     {
//         if (sick_scan_srv_colamsg_client->wait_for_service(std::chrono::milliseconds(1)))
//             ROS_INFO_STREAM("sick_scan_ros2_example: sick_scan service available");
//         else
//             ROS_ERROR_STREAM("## ERROR sick_scan_ros2_example: sick_scan service not available");
//     }

//     // Run event loop
//     // rclcpp::spin(node);
//     rclcpp::Time sick_scan_srv_request_timestamp = rclcpp::Clock().now();
//     while (rclcpp::ok())
//     {
//         rclcpp::spin_some(node);
//         rclcpp::sleep_for(std::chrono::milliseconds(100));
//         if (rclcpp::Clock().now() > sick_scan_srv_request_timestamp + std::chrono::seconds(1))
//         {
//             // Send a sick_scan service request example each second
//             sendSickScanServiceRequest(node, sick_scan_srv_colamsg_client, "sRN SCdevicestate");
//             sick_scan_srv_request_timestamp = rclcpp::Clock().now();
            
//         }
//     }

//     // Cleanup and exit
//     ROS_INFO_STREAM("sick_scan_ros2_example finished");
//     return 0;
// }

// void recieveMessage(rclcpp::Node::SharedPtr node){
//     SickScanMessageReceiver receiver(node, "/scan");

//     // Later in your event loop or another function:
//     auto laser_scan_msg = receiver.getLatestLaserScan();
//     while(rclcpp::ok() && !laser_scan_msg){
//         rclcpp::spin_some(node);
//         rclcpp::sleep_for(std::chrono::milliseconds(10));
//         laser_scan_msg = receiver.getLatestLaserScan();
//         if (laser_scan_msg) {
//             break;
//         }
//     }
//     if (laser_scan_msg) {
//         ROS_INFO_STREAM("Retrieved Laser Scan: angle_min: " << laser_scan_msg->angle_min << ", angle_max: " << laser_scan_msg->angle_max);
//     } 
//     // return laser_scan_msg;
// }

class LIDARrunner
{
    public:
        LIDARrunner()
        : node(rclcpp::Node::make_shared("lidar", "")),
        receiver(node, "/scan") // Correct member initialization
        {
            ROS_INFO_STREAM("sick_scan_ros2_example initialized");
        }


        sensor_msgs::msg::LaserScan recieveMessage()
        {
            auto laser_scan_msg = receiver.getLatestLaserScan();
            while(rclcpp::ok() && !laser_scan_msg){
                rclcpp::spin_some(node);
                
                laser_scan_msg = receiver.getLatestLaserScan();
                if (laser_scan_msg) {
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
            if (laser_scan_msg) {
                ROS_INFO_STREAM("Retrieved Laser Scan: angle_min: " << laser_scan_msg->angle_min << ", angle_max: " << laser_scan_msg->angle_max);
            } 
            return *laser_scan_msg;
        }

        float getRangeAtDegrees(sensor_msgs::msg::LaserScan& scan, float angle_degrees)
        {
            // Convert degrees to radians
            if (angle_degrees > 180.0) {
                angle_degrees -= 360.0; // Normalize to -180 to 180
            }
            float angle_radians = angle_degrees * M_PI / 180.0;
            // Calculate the index of the range value
            int center_index = static_cast<int>((0 - scan.angle_min) / scan.angle_increment); // Index of forward
            int index = center_index + static_cast<int>((angle_radians) / scan.angle_increment);

            if (index >= 0 && index < scan.ranges.size()) {
                return scan.ranges[index];
            } else {
                return -1.0; // Invalid angle
            }
        }

        float getMinRange(sensor_msgs::msg::LaserScan& scan)
        {
            float min_range = std::numeric_limits<float>::max();
            for (const auto& range : scan.ranges) {
                if (range < min_range && range > 0.1) { // Ignore too low values
                    min_range = range;
                }
            }
            return min_range;
        }

    private:
        rclcpp::Node::SharedPtr node;
        SickScanMessageReceiver receiver;
        std::shared_ptr<sensor_msgs::msg::LaserScan> laser_scan_msg;
};


void printLaserScanSummary(const sensor_msgs::msg::LaserScan& scan) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "=== LaserScan Summary ===");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Frame ID: " << scan.header.frame_id);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Timestamp: " << scan.header.stamp.sec << "." << scan.header.stamp.nanosec << " seconds");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Angle Min: " << scan.angle_min << " rad");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Angle Max: " << scan.angle_max << " rad");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Angle Increment: " << scan.angle_increment << " rad");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Scan Time: " << scan.scan_time << " seconds");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Range Min: " << scan.range_min << " meters");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Range Max: " << scan.range_max << " meters");

    // Display first few range values
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "First 5 ranges: ");
    for (size_t i = 0; i < std::min(scan.ranges.size(), size_t(5)); ++i) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), scan.ranges[i] << " meters");
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "=========================");
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::NodeOptions node_options;
    // node_options.allow_undeclared_parameters(true);
    // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("", "", node_options);
    
    // ROS_INFO_STREAM("sick_scan_ros2_example initialized");
    
    // recieveMessage(node);

    LIDARrunner lidar_runner;
    sensor_msgs::msg::LaserScan scan = lidar_runner.recieveMessage();
    printLaserScanSummary(scan);
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Range at 0 (forward): " << lidar_runner.getRangeAtDegrees(scan, 0)<< " meters");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Range at 90 (left): " << lidar_runner.getRangeAtDegrees(scan, 90)<< " meters");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Range at 180 (back): " << lidar_runner.getRangeAtDegrees(scan, 180)<< " meters");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Range at 270 (right): " << lidar_runner.getRangeAtDegrees(scan, 270)<< " meters");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidar_test"), "Minimum Range: " << lidar_runner.getMinRange(scan)<< " meters");
    return 0;
}