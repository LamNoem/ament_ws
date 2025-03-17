#ifndef ESTIMATION_NODE_HPP_
#define ESTIMATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/imu.hpp" //added imu header


#include "estimation_core.hpp"

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>


#include <map> //i think u use map 

class EstimationNode : public rclcpp::Node {
  public:
    // Constants
    Matrix3d C_li;
    Vector3d t_i_li;
    double var_imu_f, var_imu_w, var_gnss, var_lidar;
    Vector3d g;
    MatrixXd l_jac, h_jac;

    // State Variables
    map<string, MatrixXd> data;
    MatrixXd p_est, v_est, q_est, imu_w, imu_f;
    vector<MatrixXd> p_cov;
    vector<double> gnss_t, lidar_t;

    
    EstimationNode();

    void MotionModel();

    // Timer callback to publish a test message
    void publishMessage();

    // Callback function to handle LaserScan messages
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

   
    tuple<Vector3d, Vector3d, Quaternion, MatrixXd> measurement_update(sensor_var, p_cov_check, y_k, p_check, v_check, q_check);


    // Callback function to handle imu messages
    void EstimationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);


  private:
    rclcpp::Time last_time_{0};


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimation_pub_;

    // Core costmap object
    robot::EstimationCore estimation_;
    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    // Subscriber to the IMU topic
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    // Timer for periodic test message publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // // Costmap parameters
    // double resolution_; // Grid resolution (meters per cell)
    // int width_;         // Number of cells in x-direction
    // int height_;        // Number of cells in y-direction
    // double origin_x_;   // Origin of the costmap in x (meters)
    // double origin_y_;   // Origin of the costmap in y (meters)

    // Costmap data
    std::vector<std::vector<int>> costmap_grid; // 2D array for storing grid costs
    //was costmap_ before
};

#endif 