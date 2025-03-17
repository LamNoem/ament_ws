#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

// Quaternion class and other helper functions (angle_normalize, skew_symmetric, etc.)
// would need to be defined here as they are used in the script.

// Load data from file (mock implementation)
void load_data(const string &filename, map<string, MatrixXd> &data) {
    // Implement loading logic (mock implementation, assuming pre-parsed data for simplicity).
    // For actual implementation, use an appropriate library to parse pickle files.
    // Example: Use Boost or pybind11 to handle Python's pickle format.
}

int main() {
    // 1. Data
    map<string, MatrixXd> data;
    load_data("data/pt1_data.pkl", data);

    auto gt = data["gt"];
    auto imu_f = data["imu_f"];
    auto imu_w = data["imu_w"];
    auto gnss = data["gnss"];
    auto lidar = data["lidar"];

    // Correct calibration rotation matrix, corresponding to Euler RPY angles (0.05, 0.05, 0.1).
    Matrix3d C_li;
    C_li <<  0.99376, -0.09722,  0.05466,
             0.09971,  0.99401, -0.04475,
            -0.04998,  0.04992,  0.9975;

    Vector3d t_i_li(0.5, 0.1, 0.5);

    // Transform from the LIDAR frame to the vehicle (IMU) frame.
    MatrixXd lidar_data = lidar; // Assuming lidar is a MatrixXd for simplicity.
    lidar_data = (C_li * lidar_data.transpose()).transpose();
    lidar_data.rowwise() += t_i_li.transpose();

    // 2. Constants
    double var_imu_f = 0.10;
    double var_imu_w = 0.10;
    double var_gnss  = 0.10;
    double var_lidar = 2.00;

    Vector3d g(0, 0, -9.81);  // gravity
    MatrixXd l_jac = MatrixXd::Zero(9, 6);
    l_jac.block<6, 6>(3, 0) = MatrixXd::Identity(6, 6);

    MatrixXd h_jac = MatrixXd::Zero(3, 9);
    h_jac.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);

    // 3. Initial Values
    int data_size = imu_f.rows(); // Assuming imu_f is a MatrixXd
    MatrixXd p_est = MatrixXd::Zero(data_size, 3);
    MatrixXd v_est = MatrixXd::Zero(data_size, 3);
    MatrixXd q_est = MatrixXd::Zero(data_size, 4);
    vector<MatrixXd> p_cov(data_size, MatrixXd::Zero(9, 9));

    // Set initial values
    p_est.row(0) = gt.row(0).head(3); //these r arbitrary
    v_est.row(0) = gt.row(0).segment(3, 3); //these r arbitrary
    q_est.row(0) = Quaternion(gt.row(0).segment(6, 3)).to_numpy(); // these r arbitrary
    p_cov[0] = MatrixXd::Zero(9, 9);

    vector<double> gnss_t(gnss.rows());
    vector<double> lidar_t(lidar.rows());

    for (int i = 0; i < gnss.rows(); ++i) gnss_t[i] = gnss(i, 0);
    for (int i = 0; i < lidar.rows(); ++i) lidar_t[i] = lidar(i, 0);

    // 4. Measurement Update Function
    auto measurement_update = [&](double sensor_var, const MatrixXd &p_cov_check, const Vector3d &y_k,
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
    };

    // 5. Main Filter Loop
    for (int k = 1; k < data_size; ++k) {
        double delta_t = imu_f(k, 0) - imu_f(k - 1, 0);

        Quaternion q_prev(q_est.row(k - 1));
        Quaternion q_curr(imu_w.row(k - 1) * delta_t);
        Matrix3d c_ns = q_prev.to_mat();
        Vector3d f_ns = (c_ns * imu_f.row(k - 1).transpose()) + g;

        Vector3d p_check = p_est.row(k - 1).transpose() + delta_t * v_est.row(k - 1).transpose() +
                           0.5 * delta_t * delta_t * f_ns;
        Vector3d v_check = v_est.row(k - 1).transpose() + delta_t * f_ns;
        Quaternion q_check = q_prev.quat_mult_left(q_curr);

        // Linearize the motion model and compute Jacobians
        MatrixXd f_jac = MatrixXd::Identity(9, 9);
        f_jac.block<3, 3>(0, 3) = MatrixXd::Identity(3, 3) * delta_t;
        f_jac.block<3, 3>(3, 6) = -skew_symmetric(c_ns * imu_f.row(k - 1).transpose()) * delta_t;

        MatrixXd q_cov = MatrixXd::Zero(6, 6);
        q_cov.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3) * delta_t * delta_t * var_imu_f;
        q_cov.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3) * delta_t * delta_t * var_imu_w;

        MatrixXd p_cov_check = f_jac * p_cov[k - 1] * f_jac.transpose() + l_jac * q_cov * l_jac.transpose();

        // GNSS and LIDAR measurement updates
        if (find(gnss_t.begin(), gnss_t.end(), imu_f(k, 0)) != gnss_t.end()) {
            int gnss_i = distance(gnss_t.begin(), find(gnss_t.begin(), gnss_t.end(), imu_f(k, 0)));
            tie(p_check, v_check, q_check, p_cov_check) = 
                measurement_update(var_gnss, p_cov_check, gnss.row(gnss_i).transpose(),
                                   p_check, v_check, q_check);
        }

        if (find(lidar_t.begin(), lidar_t.end(), imu_f(k, 0)) != lidar_t.end()) {
            int lidar_i = distance(lidar_t.begin(), find(lidar_t.begin(), lidar_t.end(), imu_f(k, 0)));
            tie(p_check, v_check, q_check, p_cov_check) = 
                measurement_update(var_lidar, p_cov_check, lidar.row(lidar_i).transpose(),
                                   p_check, v_check, q_check);
        }

        // Update states (save)
        p_est.row(k) = p_check.transpose();
        v_est.row(k) = v_check.transpose();
        q_est.row(k) = q_check.to_numpy();
        p_cov[k] = p_cov_check;
    }

    return 0;
}

