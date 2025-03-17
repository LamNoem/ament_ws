#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class KalmanFilter {
public:
    // Constants
    Matrix3d C_li;
    Vector3d t_i_li;
    double var_imu_f, var_imu_w, var_gnss, var_lidar;
    Vector3d g;
    MatrixXd l_jac, h_jac;

    // State Variables
    map<string, MatrixXd> data;
    MatrixXd p_est, v_est, q_est;
    vector<MatrixXd> p_cov;
    vector<double> gnss_t, lidar_t;

    KalmanFilter() {
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
    }
    //Noemie: change to data from ros2 subscription 
    void loadData(const string &filename) {
        // Mock data loading; replace with actual file handling logic.
        load_data(filename, data);
        auto imu_f = data["imu_f"];
        auto gnss = data["gnss"];
        auto lidar = data["lidar"];

        int data_size = imu_f.rows();
        p_est = MatrixXd::Zero(data_size, 3);
        v_est = MatrixXd::Zero(data_size, 3);
        q_est = MatrixXd::Zero(data_size, 4);
        p_cov = vector<MatrixXd>(data_size, MatrixXd::Zero(9, 9));

        for (int i = 0; i < gnss.rows(); ++i) gnss_t.push_back(gnss(i, 0));
        for (int i = 0; i < lidar.rows(); ++i) lidar_t.push_back(lidar(i, 0));
    }

    tuple<Vector3d, Vector3d, Quaternion, MatrixXd> measurementUpdate(
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
    //Noemie: i got to figure out how to implement main loop and the actual data structures
    void mainLoop() {
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
};

int main() {
    KalmanFilter kf;
    kf.loadData("data/pt1_data.pkl");
    kf.mainLoop();
    return 0;
}