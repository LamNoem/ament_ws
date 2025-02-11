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

  // Initialize constants
        C_li << 0.99376, -0.09722, 0.05466,
                0.09971, 0.99401, -0.04475,
               -0.04998, 0.04992, 0.9975;

        t_i_li = Vector3d(0.5, 0.1, 0.5);
        var_imu_f = 0.10;
        var_imu_w = 0.10;
        var_gnss = 0.10;
        var_lidar = 2.00;
        g = Vector3d(0, 0, -9.81);

        l_jac = MatrixXd::Zero(9, 6);
        l_jac.block<6, 6>(3, 0) = MatrixXd::Identity(6, 6);

        h_jac = MatrixXd::Zero(3, 9);
        h_jac.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);

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



void EstimationNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
 
  // from costmap
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range > scan->range_min && range < scan->range_max) {
      int x_grid, y_grid;
      
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

// Noemie: must be changed to handle actual data structure from inputs
tuple<Vector3d, Vector3d, Quaternion, MatrixXd> EstimationNode::measurementUpdate(
        double sensor_var, const MatrixXd &p_cov_check, const Vector3d &y_k,
        const Vector3d &p_check, const Vector3d &v_check, const Quaternion &q_check) {

        Matrix3d r_cov = Matrix3d::Identity() * sensor_var;
        MatrixXd k_gain = p_cov_check * h_jac.transpose() *
                          (h_jac * p_cov_check * h_jac.transpose() + r_cov).inverse();

        VectorXd error_state = k_gain * (y_k - p_check);

        Vector3d p_hat = p_check + error_state.head(3);
        Vector3d v_hat = v_check + error_state.segment(3, 3);
        Quaternion q_hat = Quaternion(error_state.tail(3)).quat_mult_left(q_check);

        MatrixXd p_cov_hat = (MatrixXd::Identity(9, 9) - k_gain * h_jac) * p_cov_check;
        return make_tuple(p_hat, v_hat, q_hat, p_cov_hat);
}
// Noemie: must be changed to handle actual data structure from inputs
void EstimationNode::mainLoop() {
        auto gt = data["gt"];
        auto imu_f = data["imu_f"];
        auto imu_w = data["imu_w"];
        auto gnss = data["gnss"];
        auto lidar = data["lidar"];

        // Set initial values
        p_est.row(0) = gt.row(0).head(3);
        v_est.row(0) = gt.row(0).segment(3, 3);
        q_est.row(0) = Quaternion(gt.row(0).segment(6, 3)).to_numpy();
        p_cov[0] = MatrixXd::Zero(9, 9);

        for (int k = 1; k < imu_f.rows(); ++k) {
            double delta_t = imu_f(k, 0) - imu_f(k - 1, 0);

            Quaternion q_prev(q_est.row(k - 1));
            Quaternion q_curr(imu_w.row(k - 1) * delta_t);
            Matrix3d c_ns = q_prev.to_mat();
            Vector3d f_ns = (c_ns * imu_f.row(k - 1).transpose()) + g;

            Vector3d p_check = p_est.row(k - 1).transpose() + delta_t * v_est.row(k - 1).transpose() +
                               0.5 * delta_t * delta_t * f_ns;
            Vector3d v_check = v_est.row(k - 1).transpose() + delta_t * f_ns;
            Quaternion q_check = q_prev.quat_mult_left(q_curr);

            // Linearize motion model and compute Jacobians
            MatrixXd f_jac = MatrixXd::Identity(9, 9);
            f_jac.block<3, 3>(0, 3) = MatrixXd::Identity(3, 3) * delta_t;
            f_jac.block<3, 3>(3, 6) = -skew_symmetric(c_ns * imu_f.row(k - 1).transpose()) * delta_t;

            MatrixXd q_cov = MatrixXd::Zero(6, 6);
            q_cov.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3) * delta_t * delta_t * var_imu_f;
            q_cov.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3) * delta_t * delta_t * var_imu_w;

            MatrixXd p_cov_check = f_jac * p_cov[k - 1] * f_jac.transpose() + l_jac * q_cov * l_jac.transpose();

            // GNSS and LIDAR updates
            if (find(gnss_t.begin(), gnss_t.end(), imu_f(k, 0)) != gnss_t.end()) {
                int gnss_i = distance(gnss_t.begin(), find(gnss_t.begin(), gnss_t.end(), imu_f(k, 0)));
                tie(p_check, v_check, q_check, p_cov_check) =
                    measurementUpdate(var_gnss, p_cov_check, gnss.row(gnss_i).transpose(), p_check, v_check, q_check);
            }

            if (find(lidar_t.begin(), lidar_t.end(), imu_f(k, 0)) != lidar_t.end()) {
                int lidar_i = distance(lidar_t.begin(), find(lidar_t.begin(), lidar_t.end(), imu_f(k, 0)));
                tie(p_check, v_check, q_check, p_cov_check) =
                    measurementUpdate(var_lidar, p_cov_check, lidar.row(lidar_i).transpose(), p_check, v_check, q_check);
            }

            // Update states
            p_est.row(k) = p_check.transpose();
            v_est.row(k) = v_check.transpose();
            q_est.row(k) = q_check.to_numpy();
            p_cov[k] = p_cov_check;
        }
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