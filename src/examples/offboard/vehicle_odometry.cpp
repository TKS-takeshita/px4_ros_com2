#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <sensor_msgs/msg/imu.hpp>
#include "px4_ros_com/frame_transforms.h"
#include <px4_msgs/msg/timesync_status.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <functional>
#include <cmath>
#include <array>
#include <algorithm>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem> 

using namespace px4_ros_com::frame_transforms;
using RowMat3 = Eigen::Matrix<double,3,3,Eigen::RowMajor>;

class FastLioToPX4 : public rclcpp::Node {
public:
    FastLioToPX4()
        : Node("fastlio_to_px4")
    {
        yaw_imu = 0.0;
        yaw_slam = 0.0;
        last_us = this->get_clock()->now().nanoseconds() / 1000;
        slam_last_ns = last_us * 1000;
        pre_posi = Eigen::Vector3d::Zero();
        pre_vel =  geometry_msgs::msg::Twist{};
        cov_pos_px4 = {1e-3,0,0, 0,1e-3,0, 0,0,1e-3};
        cov_rot_px4 = {1e-3,0,0, 0,1e-3,0, 0,0,1e-3};
        FLU2FRD_world = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

        const std::string dir = "/home/ros2/ws_sensor_combined/src/px4_ros_com/csv/";
        std::filesystem::create_directories(dir);

        const auto now_sys   = std::chrono::system_clock::now();
        const std::time_t tt = std::chrono::system_clock::to_time_t(now_sys);
        std::tm tm{};
        localtime_r(&tt, &tm);

        const std::string px4_fname  = dir + "px4_odom_" + ".csv";
        const std::string slam_fname = dir + "slam_odom_" + ".csv";

        csv_.open(px4_fname,  std::ios::out);
        csv_slam_.open(slam_fname, std::ios::out);

        if (!csv_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", px4_fname.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Logging px4_odom to: %s", px4_fname.c_str());
            csv_ << "timestamp_us,px,py,pz,qw,qx,qy,qz,yaw_imu,yaw_slam\n";
            csv_.flush();
        }

        if (!csv_slam_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV_SLAM file: %s", slam_fname.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Logging slam_odom to: %s", slam_fname.c_str());
            csv_slam_ << "timestamp_us,px,py,pz,qw,qx,qy,qz\n";
            csv_slam_.flush();
        }

        // FAST-LIO の Odometry を購読
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odom_callback(msg);
            });

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg){
                this->imu_callback(msg);
            }
        );

        // PX4 の `vehicle_odometry` に送信
        vehicle_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);

        time_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
            "/fmu/out/timesync_status",
            rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::TimesyncStatus::SharedPtr s) {
                if (!have_offset_.load(std::memory_order_relaxed)) {
                    px4_ros_offset_us_ = s->estimated_offset;
                    have_offset_.store(true, std::memory_order_relaxed);
                }
            }
        );
    }

private:
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr time_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_pub_;
    int64_t px4_ros_offset_us_{0};
    Eigen::Quaterniond prev_q;
    bool received_once = false;
    Eigen::Vector3d prev_pos;

    rclcpp::TimerBase::SharedPtr timer_;

    Eigen::Vector3d pre_posi;
    geometry_msgs::msg::Twist pre_vel{};
    Eigen::Vector3d pre_position_slam = Eigen::Vector3d::Zero();
    Eigen::Vector3d position_FRD = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_FRD = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond orientation_slam_FRD = Eigen::Quaterniond::Identity();

    Eigen::Quaterniond delta = Eigen::Quaterniond::Identity();//SLAM姿勢yaw、Madgwick filter 姿勢yawの誤差
    bool init_done = false;
    Eigen::Quaterniond q_init_inv;
    uint64_t slam_last_ns;
    uint64_t last_us{0};
    uint64_t odom_last_us{0};
    std::atomic<bool> have_offset_{false};
    bool update_slam{false};
    double yaw_imu;
    double yaw_slam;
    Covariance3d cov_pos_px4;
    Covariance3d cov_rot_px4;
    std::mutex vel_mutex;
    std::ofstream csv_;
    std::ofstream csv_slam_;
    Eigen::Matrix3d FLU2FRD_world;
    Eigen::Quaterniond pre_imu_pose = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond pre_slam_pose = Eigen::Quaterniond::Identity();

    const Eigen::Quaterniond q_FLU2FRD_world{
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
    };

    static std::tuple<double, double, double> quat_to_euler(const Eigen::Quaterniond& q){
        Eigen::Vector3d ypr = q.toRotationMatrix().eulerAngles(2, 1, 0);
        double yaw   = ypr[0];  // Z
        double pitch = ypr[1];  // Y
        double roll  = ypr[2];  // X

        return {roll, pitch, yaw};
    }

    static inline Eigen::Vector3d enu_to_frd_vec(const Eigen::Vector3d& v_enu){
        return { v_enu.x(), -v_enu.y(), -v_enu.z() };
    }
    static inline double wrapPi(double a) {
        return std::atan2(std::sin(a), std::cos(a));
    }

    inline std::array<double,9> rotate_cov_flu_to_frd(const std::array<double,9>& cov_flu,
                                                      const Eigen::Quaterniond& rotate_quat)
    {
        const Eigen::Matrix3d R = rotate_quat.toRotationMatrix();
        const Eigen::Map<const RowMat3> C_flu(cov_flu.data());
        const Eigen::Matrix3d C_frd = R * C_flu * R.transpose();

        std::array<double,9> out{};
        Eigen::Map<RowMat3>(out.data()) = C_frd;
        return out;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr &msg){
        if (!have_offset_.load(std::memory_order_relaxed)) {
            return;
        }

        const uint64_t now_us = this->get_clock()->now().nanoseconds() / 1000;
        double dt = static_cast<double>(now_us - last_us) / 1e6;//seconds
        last_us = now_us;

        Eigen::Vector3d curr_posi;

        Eigen::Quaterniond curr_pose(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z
        );
        if(curr_pose.dot(pre_imu_pose) < 0.0){
            curr_pose.coeffs() *= -1.0;
        }
        pre_imu_pose = curr_pose;
        if(!init_done){
            q_init_inv = curr_pose.conjugate();
            init_done = true;
        }
        curr_pose = (q_init_inv * curr_pose).normalized();//初期姿勢を基準にする FLU
        Eigen::Vector3d a_b(msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z);
        Eigen::Vector3d a_w_flu = curr_pose * a_b;
        a_w_flu.z() -= 9.80665;//重力補償
        auto a_w_frd = enu_to_frd_vec(a_w_flu);

        geometry_msgs::msg::Twist curr_vel = pre_vel;//FRD
        curr_vel.linear.x += a_w_frd.x() * dt;
        curr_vel.linear.y += a_w_frd.y() * dt;
        curr_vel.linear.z += a_w_frd.z() * dt;
        Eigen::Vector3d v_w(curr_vel.linear.x, curr_vel.linear.y, curr_vel.linear.z);
        curr_posi = pre_posi + v_w * dt;//FRD
        orientation_FRD = (q_FLU2FRD_world * curr_pose * q_FLU2FRD_world.conjugate()).normalized();
        position_FRD = curr_posi;
        vel_mutex.lock();
        pre_vel.linear.x = v_w.x();
        pre_vel.linear.y = v_w.y();
        pre_vel.linear.z = v_w.z();
        vel_mutex.unlock();
        pre_posi = position_FRD;
        
        px4_msgs::msg::VehicleOdometry px4_odom;
        const uint64_t px4_time_us = now_us + px4_ros_offset_us_;
        px4_odom.timestamp = px4_time_us;
        px4_odom.timestamp_sample = px4_time_us;

        px4_odom.pose_frame = px4_odom.POSE_FRAME_FRD;
        px4_odom.velocity_frame = px4_odom.VELOCITY_FRAME_FRD;

        px4_odom.position[0] = position_FRD.x();
        px4_odom.position[1] = position_FRD.y();
        px4_odom.position[2] = position_FRD.z();
        px4_odom.q[0] = orientation_FRD.w();
        px4_odom.q[1] = orientation_FRD.x();
        px4_odom.q[2] = orientation_FRD.y();
        px4_odom.q[3] = orientation_FRD.z();

        px4_odom.velocity[0] = v_w.x();
        px4_odom.velocity[1] = v_w.y();
        px4_odom.velocity[2] = v_w.z();
        Eigen::Vector3d w_b_flu(msg->angular_velocity.x,
                                      msg->angular_velocity.y,
                                      msg->angular_velocity.z);
        Eigen::Vector3d avel_w_flu = curr_pose * w_b_flu;
        Eigen::Vector3d avel_w_frd = enu_to_frd_vec(avel_w_flu);
        px4_odom.angular_velocity[0] = avel_w_frd.x();
        px4_odom.angular_velocity[1] = avel_w_frd.y();
        px4_odom.angular_velocity[2] = avel_w_frd.z();

        const auto& acc_cov_flu = msg->linear_acceleration_covariance;
        const auto acc_cov_frd = rotate_cov_flu_to_frd(acc_cov_flu, q_FLU2FRD_world);

        Eigen::Matrix3d D = Eigen::Map<const RowMat3>(cov_pos_px4.data());
        D = 0.5*(D + D.transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_pos(D);
        Eigen::Vector3d c = es_pos.eigenvalues().cwiseMax(1e-12);
        Eigen::Matrix3d M = es_pos.eigenvectors();
        D = M * c.asDiagonal() * M.transpose();  

        Eigen::Matrix3d C = Eigen::Map<const RowMat3>(cov_rot_px4.data());
        C = 0.5*(C + C.transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_rot(C);
        Eigen::Vector3d d = es_rot.eigenvalues().cwiseMax(1e-12);
        Eigen::Matrix3d V = es_rot.eigenvectors();
        C = V * d.asDiagonal() * V.transpose();  

        px4_odom.position_variance[0] = std::max(D(0,0), 1e-12);   // x軸の分散
        px4_odom.position_variance[1] = std::max(D(1,1), 1e-12);   // y軸の分散
        px4_odom.position_variance[2] = std::max(D(2,2), 1e-12);   // z軸の分散

        px4_odom.orientation_variance[0] = std::max(C(0,0), 1e-12);   // x軸の回転分散
        px4_odom.orientation_variance[1] = std::max(C(1,1), 1e-12);  // y軸の回転分散
        px4_odom.orientation_variance[2] = std::max(C(2,2), 1e-12);   // z軸の回転分散

        const double dt2 = dt * dt;
        px4_odom.velocity_variance[0] = std::max(1e-6, acc_cov_frd[0] * dt2);
        px4_odom.velocity_variance[1] = std::max(1e-6, acc_cov_frd[4] * dt2);
        px4_odom.velocity_variance[2] = std::max(1e-6, acc_cov_frd[8] * dt2);

        px4_odom.quality = 100;
        vehicle_odom_pub_->publish(px4_odom);

        csv_ << px4_odom.timestamp << ","
             << px4_odom.position[0] << ","
             << px4_odom.position[1] << ","
             << px4_odom.position[2] << ","
             << px4_odom.q[0] << "," << px4_odom.q[1] << "," << px4_odom.q[2] << "," << px4_odom.q[3] << ","
             << yaw_imu << "," << yaw_slam
             << "\n";
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg)
    {
        if (!have_offset_.load(std::memory_order_relaxed)) {
            return;
        }
        const uint64_t slam_now_ns = this->get_clock()->now().nanoseconds();
        double dt = static_cast<double>(slam_now_ns - slam_last_ns) / 1e9;//seconds
        update_slam = true;
        Eigen::Vector3d position_slam(msg->pose.pose.position.x,
                                     msg->pose.pose.position.y,
                                     msg->pose.pose.position.z);
        pre_posi = (FLU2FRD_world * position_slam);
        Eigen::Quaterniond orientation_slam(msg->pose.pose.orientation.w,
                                 msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z);
        if(orientation_slam.dot(pre_slam_pose) < 0.0){
            orientation_slam.coeffs() *= -1.0;
        }
        pre_slam_pose = orientation_slam;
        orientation_slam_FRD = (q_FLU2FRD_world * orientation_slam * q_FLU2FRD_world.conjugate()).normalized();

        Covariance3d cov_pos_ros ={
            msg->pose.covariance[0], msg->pose.covariance[1], msg->pose.covariance[2],
            msg->pose.covariance[6], msg->pose.covariance[7], msg->pose.covariance[8],   
            msg->pose.covariance[12], msg->pose.covariance[13], msg->pose.covariance[14] 
        };

        Covariance3d cov_rot_ros = {
            msg->pose.covariance[21], msg->pose.covariance[22], msg->pose.covariance[23],
            msg->pose.covariance[27], msg->pose.covariance[28], msg->pose.covariance[29],
            msg->pose.covariance[33], msg->pose.covariance[34], msg->pose.covariance[35]
        };
        // vel_mutex.lock();
        // auto vel = (pre_posi - pre_position_slam) / dt;
        // pre_vel.linear.x = vel.x();
        // pre_vel.linear.y = vel.y();
        // pre_vel.linear.z = vel.z();
        // vel_mutex.unlock();
        cov_pos_px4 = px4_ros_com::frame_transforms::transform_static_frame(cov_pos_ros, StaticTF::ENU_TO_NED);
        cov_rot_px4 = px4_ros_com::frame_transforms::transform_frame(cov_rot_ros, orientation_FRD);
        pre_position_slam = pre_posi;
        slam_last_ns = slam_now_ns;

        const uint64_t slam_time_us = slam_now_ns / 1000 + px4_ros_offset_us_;
        csv_slam_ << slam_time_us << ","
             << position_slam.x() << ","
             << position_slam.y() << ","
             << position_slam.z() << ","
             << msg->pose.pose.orientation.w << "," << msg->pose.pose.orientation.x << "," << msg->pose.pose.orientation.y << "," << msg->pose.pose.orientation.z
             << "\n";
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastLioToPX4>());
    rclcpp::shutdown();
    return 0;
}