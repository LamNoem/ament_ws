#include <chrono>
#include <memory>

#include <cmath>

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "rotations.h"

using namespace Eigen;
using namespace std;


#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "estimation_node.hpp"

EstimationNode::EstimationNode() : Node("estimation"), estimation_(robot::EstimationCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&EstimationNode::publishMessage, this));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  
//   lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//       "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&EstimationNode::laserCallback, this, std::placeholders::_1));


// Create a subscriber to the IMU topic
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",  // Change this to the name of your IMU topic
         10,  // Queue size
         std::bind(&IEstimationNode::imuCallback, this, std::placeholders::_1)
    );



//   // Initialize costmap parameters
//   resolution_ = 0.1; // 0.1 meters per cell
//   width_ = 100;      // 10 meters
//   height_ = 100;     // 10 meters
//   origin_x_ = 0; 
//   origin_y_ = 0; 

//   initializeCostmap();
}


// Callback function to handle incoming IMU messages
void EstimationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Extract and log orientation data
    RCLCPP_INFO(this->get_logger(), "Orientation -> x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    // Extract and log angular velocity data
    RCLCPP_INFO(this->get_logger(), "Angular Velocity -> x: %.2f, y: %.2f, z: %.2f",
                msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // Extract and log linear acceleration data
    RCLCPP_INFO(this->get_logger(), "Linear Acceleration -> x: %.2f, y: %.2f, z: %.2f",
                msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}



void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range > scan->range_min && range < scan->range_max) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  // Log the angle and range data
    RCLCPP_INFO(this->get_logger(), "LaserScan Info:");
    RCLCPP_INFO(this->get_logger(), "  Angle Min: %.2f rad, Angle Max: %.2f rad", msg->angle_min, msg->angle_max);
    RCLCPP_INFO(this->get_logger(), "  Angle Increment: %.2f rad", msg->angle_increment);
    RCLCPP_INFO(this->get_logger(), "  Time Increment: %.6f sec", msg->time_increment);
    RCLCPP_INFO(this->get_logger(), "  Scan Time: %.6f sec", msg->scan_time);
    RCLCPP_INFO(this->get_logger(), "  Range Min: %.2f m, Range Max: %.2f m", msg->range_min, msg->range_max);

    // Process range data (e.g., find the minimum range)
    float min_range = std::numeric_limits<float>::infinity();
    float max_range = -std::numeric_limits<float>::infinity();
    for (const auto &range : msg->ranges) {
        if (range < min_range) min_range = range;
        if (range > max_range) max_range = range;
    }

    RCLCPP_INFO(this->get_logger(), "  Min Range: %.2f m, Max Range: %.2f m", min_range, max_range);

}




// Define the timer to publish a message every 500ms
void EstimationNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimationNode>());
  rclcpp::shutdown();
  return 0;
}