#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>
#include <thread>
#include <chrono>

#include <sick_scan_xd/msg/radar_scan.hpp>           // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/msg/encoder.hpp>              // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/msg/li_doutputstate_msg.hpp>  // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/msg/lf_erec_msg.hpp>          // generated in sick_scan_xd by msg-generator
#include <sick_scan_xd/srv/cola_msg_srv.hpp>         // generated in sick_scan_xd by rosidl-generator

#include <tuple>

#define RCLCPP_LOGGER         rclcpp::get_logger("lidar_test02")
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(RCLCPP_LOGGER,__VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(RCLCPP_LOGGER,__VA_ARGS__)



class SickScanNode: public rclcpp::Node {
    public:
        SickScanNode(): Node("SickScanNode") {
            printf("Create SickScanNode\n");
            m_laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&SickScanNode::messageCbLaserScanROS2, this, std::placeholders::_1));

            obstacle_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("obstacle_team10", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), // Timer period (100 milliseconds)
                std::bind(&SickScanNode::lidar_timer_callback, this));
        }


    protected:
        /** ROS2 callback for receiving Laser Scan messages */
        void messageCbLaserScanROS2(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
        {
            latest_msg = msg;  // Store the latest received message
            last_message_time_ = this->get_clock()->now();
            //ROS_INFO_STREAM("sick_scan_ros2_example: pointcloud received, angle_min: " << msg->angle_min << ", angle_max: " << msg->angle_max);

        }

        void lidar_timer_callback()
        {
            if (!latest_msg) {
                return;
            }
            rclcpp::Time now = this->get_clock()->now();
            if (now - last_message_time_ > rclcpp::Duration::from_seconds(1.0)) {
                return;
            }

            float dist[61];
            float angle = -30.0f;
            for (size_t i = 0; i < std::min((size_t)61, latest_msg->ranges.size()); ++i) {
                angle = angle + i;
                dist[i] = this->getRangeAtDegrees(*latest_msg, angle);
            }

            //Calc nearest distance
            auto minRangeTuple = this->getMinRange(dist, 61);
            float minRange = std::get<0>(minRangeTuple);
            float minAngle = std::get<1>(minRangeTuple);
            float isObtacle = 0.0f;
            if (minRange < 0.7){
                isObtacle = 1.0f;
            }
            std_msgs::msg::Float32MultiArray msg;
            msg.data.resize(3);
            msg.data[0] = isObtacle;
            msg.data[1] = minRange;
            msg.data[2] = minAngle;

            obstacle_publisher->publish(msg);
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

        std::tuple<float, float> getMinRange(float* angle_list, int len)
        {

            float min_range = std::numeric_limits<float>::max();
            int min_index = 0;
            for (int i = 0; i < len; i++) {
                float range = angle_list[i];
                if (range < min_range && range > 0.1) { // Ignore too low values
                    min_range = range;
                    min_index = i;
                }
            }
            float angle = -30.0f +  min_index;
            return {min_range, angle};
        }


        /** Private member to store the latest received message */
        std::shared_ptr<sensor_msgs::msg::LaserScan> latest_msg;
    
        /** Subscriber for pointcloud messages */
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_scan_subscriber;
        /** Publisher for obstacle detection */
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_publisher;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time last_message_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SickScanNode>());
  rclcpp::shutdown();
  return 0;
}
