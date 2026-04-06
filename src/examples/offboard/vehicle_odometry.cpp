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
        last_us = this->get_clock()->now().nanoseconds() / 1000;
        pre_position = Eigen::Vector3d::Zero();
        pre_vel =  geometry_msgs::msg::Twist{};
        cov_pos_px4 = {1e-3,0,0, 0,1e-3,0, 0,0,1e-3};
        cov_rot_px4 = {1e-3,0,0, 0,1e-3,0, 0,0,1e-3};
        FLU2FRD_world = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

        const auto now_sys   = std::chrono::system_clock::now();
        const std::time_t tt = std::chrono::system_clock::to_time_t(now_sys);
        std::tm tm{};
        localtime_r(&tt, &tm);
        std::ostringstream oss;
        oss << "/home/ros2/ws_sensor_combined/src/px4_ros_com/csv"
            << std::put_time(&tm, "/pose_log_%Y%m%d_%H%M%S.csv");

        pose_log.open(oss.str(), std::ios::out);
        if(pose_log.is_open()){
            pose_log << "time_us,"
                     << "slam_roll, slam_pitch, slam_yaw,"
                     << "imu_roll, imu_pitch, imu_yaw,"
                     << "yaw_diff,"
                     << "slam_qw, slam_qx, slam_qy, slam_qz,"
                     << "imu_qw, imu_qx, imu_qy, imu_qz,"
                     << "\n";
        };

        // FAST-LIO の Odometry を購読
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odom_callback(msg);
            });
        // madgwick filter の IMU を購読
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
    bool received_once = false;

    rclcpp::TimerBase::SharedPtr timer_;

    Eigen::Vector3d pre_position;
    geometry_msgs::msg::Twist pre_vel{};
    Eigen::Vector3d pre_position_slam = Eigen::Vector3d::Zero();
    Eigen::Vector3d curr_position_FRD = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_FRD = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond orientation_slam_FRD = Eigen::Quaterniond::Identity();

    Eigen::Quaterniond delta = Eigen::Quaterniond::Identity();//SLAM姿勢yaw、Madgwick filter 姿勢yawの誤差
    bool init_done = false;
    Eigen::Quaterniond q_init_inv;
    uint64_t last_us{0};
    std::atomic<bool> have_offset_{false};
    Covariance3d cov_pos_px4;
    Covariance3d cov_rot_px4;
    Eigen::Matrix3d FLU2FRD_world = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Quaterniond pre_slam_pose = Eigen::Quaterniond::Identity();

    std::ofstream pose_log;
    std::mutex log_mutex;

    Eigen::Quaterniond imu_pose_flu = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond slam_pose_flu = Eigen::Quaterniond::Identity();
    bool imu_pose_ready = false;
    bool slam_pose_ready = false;

    void log_pose_compare(uint64_t time_us)
    {
        if (!pose_log.is_open() || !imu_pose_ready || !slam_pose_ready) {
            return;
        }

        auto [slam_r, slam_p, slam_y] = quat_to_euler(orientation_slam_FRD);
        auto [imu_r,  imu_p,  imu_y ] = quat_to_euler(orientation_FRD);

        double yaw_diff = wrapPi(slam_y - imu_y);

        std::lock_guard<std::mutex> lock(log_mutex);
        pose_log << time_us << ","
                  << slam_r << "," << slam_p << "," << slam_y << ","
                  << imu_r  << "," << imu_p  << "," << imu_y  << ","
                  << yaw_diff << ","
                  << orientation_slam_FRD.w() << "," << orientation_slam_FRD.x() << ","
                  << orientation_slam_FRD.y() << "," << orientation_slam_FRD.z() << ","
                  << orientation_FRD.w()  << "," << orientation_FRD.x()  << ","
                  << orientation_FRD.y()  << "," << orientation_FRD.z()
                  << "\n";
    };

    // FAST-LIOのFLU座標系をPX4のFRD座標系に変換
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

    static inline Eigen::Vector3d flu_to_frd_vec(const Eigen::Vector3d& v_enu){
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

        // 現在姿勢(madgwick filter の IMU から)
        Eigen::Quaterniond curr_pose(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z
        );
        if(!init_done){
            q_init_inv = curr_pose.conjugate();
            init_done = true;
        }
        curr_pose = (q_init_inv * curr_pose).normalized();//初期姿勢を基準にする FLU
        imu_pose_ready = true;
        // 加速度の処理
        Eigen::Vector3d a_b(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
        Eigen::Vector3d a_w_flu = curr_pose * a_b; //世界座標系(FAST-LIO world, FLU系)での加速度
        a_w_flu.z() -= 9.80665;//重力補償
        auto a_w_frd = flu_to_frd_vec(a_w_flu);//FRD系での加速度

        // 姿勢の処理
        orientation_FRD = (q_FLU2FRD_world * curr_pose * q_FLU2FRD_world.conjugate()).normalized();

        // 速度の処理
        Eigen::Vector3d curr_vel(pre_vel.linear.x, pre_vel.linear.y, pre_vel.linear.z);
        curr_vel += a_w_frd * dt;
        pre_vel.linear.x = curr_vel.x();
        pre_vel.linear.y = curr_vel.y();
        pre_vel.linear.z = curr_vel.z();

        // 位置の処理
        curr_position_FRD = pre_position + curr_vel * dt;//FRD
        pre_position = curr_position_FRD;
        
        px4_msgs::msg::VehicleOdometry px4_odom;
        const uint64_t px4_time_us = now_us + px4_ros_offset_us_;
        px4_odom.timestamp = px4_time_us;
        px4_odom.timestamp_sample = px4_time_us;

        px4_odom.pose_frame = px4_odom.POSE_FRAME_FRD;
        px4_odom.velocity_frame = px4_odom.VELOCITY_FRAME_FRD;

        px4_odom.position[0] = curr_position_FRD.x();
        px4_odom.position[1] = curr_position_FRD.y();
        px4_odom.position[2] = curr_position_FRD.z();

        px4_odom.q[0] = orientation_FRD.w();
        px4_odom.q[1] = orientation_FRD.x();
        px4_odom.q[2] = orientation_FRD.y();
        px4_odom.q[3] = orientation_FRD.z();

        px4_odom.velocity[0] = curr_vel.x();
        px4_odom.velocity[1] = curr_vel.y();
        px4_odom.velocity[2] = curr_vel.z();

        Eigen::Vector3d angular_vel_flu(msg->angular_velocity.x,
                                      msg->angular_velocity.y,
                                      msg->angular_velocity.z);
        Eigen::Vector3d angular_vel_frd = flu_to_frd_vec(angular_vel_flu);
        px4_odom.angular_velocity[0] = angular_vel_frd.x();
        px4_odom.angular_velocity[1] = angular_vel_frd.y();
        px4_odom.angular_velocity[2] = angular_vel_frd.z();

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
        log_pose_compare(px4_time_us);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg)
    {
        if (!have_offset_.load(std::memory_order_relaxed)) {
            return;
        }
        Eigen::Vector3d position_slam(msg->pose.pose.position.x,
                                     msg->pose.pose.position.y,
                                     msg->pose.pose.position.z);
        
        Eigen::Quaterniond orientation_slam(msg->pose.pose.orientation.w,
                                 msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z);
        if(orientation_slam.dot(pre_slam_pose) < 0.0){
            orientation_slam.coeffs() *= -1.0;
        }
        slam_pose_ready = true;
        // センサ原点から見た重心位置 [FLU]
        Eigen::Vector3d r_cg_from_sensor_flu(0.0, 0.0, -0.01);

        // 世界座標系(FAST-LIO world, FLU系)での重心位置
        Eigen::Vector3d position_cg_world = position_slam + orientation_slam.toRotationMatrix() * r_cg_from_sensor_flu;
        pre_position = FLU2FRD_world * position_cg_world;

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
        
        cov_pos_px4 = px4_ros_com::frame_transforms::transform_static_frame(cov_pos_ros, StaticTF::ENU_TO_NED);
        cov_rot_px4 = px4_ros_com::frame_transforms::transform_frame(cov_rot_ros, orientation_FRD);

    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastLioToPX4>());
    rclcpp::shutdown();
    return 0;
}