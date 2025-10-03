#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <time.h>
#include <sys/time.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <functional>
#include <cmath>
#include <array>
#include <chrono>
#include <iostream>
#include <functional>
#include <algorithm>
#include <stdint.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include "px4_ros_com/frame_transforms.h"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>

using namespace px4_ros_com::frame_transforms;
using RowMat3 = Eigen::Matrix<double,3,3,Eigen::RowMajor>;
using namespace std::chrono_literals;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::ActuatorMotors;

class OffboardPIDControl : public rclcpp::Node
{
public:
    OffboardPIDControl();

private:
    void get_param();
    void arm();
    void disarm();
    void controlLoop();
    void publish_offboard_actuator_control_mode();
    void publish_offboard_position_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_motor_setpoint();
    void publish_actuator_setpoint();
    void key_input();
    double throttle_thrust(double thr);
    double wrapPi(double a);
    void VehicleCallback(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg);
    void quat_to_rpy(double &roll, double &pitch, double &yaw, const geometry_msgs::msg::Quaternion &q_msg);

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr motor_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub;

    Eigen::Vector3d pre_position_ = {0.0, 0.0, 0.0};//前回位置
    Eigen::Vector3d prev_velocity_ = Eigen::Vector3d::Zero();//前回速度
    Eigen::Vector3d prev_angular_velocity_ = Eigen::Vector3d::Zero();//前回角速度

    Eigen::Vector3d current_position_ = {0.0, 0.0, 0.0};//現在位置
    Eigen::Quaterniond current_orientation_;
    Eigen::Quaterniond pre_slam_pose = Eigen::Quaterniond::Identity();
    Eigen::Vector3d current_velocity_;//現在速度
    Eigen::Vector3d current_angular_velocity_;//現在角速度
    Eigen::Matrix3d current_rot_matrix_; //現在姿勢(回転行列)

    Eigen::Vector3d target_position_;//目標位置
    Eigen::Vector3d target_velocity_;//目標速度
    Eigen::Vector3d target_angular_velocity_;//目標角速度
    Eigen::Quaterniond target_orientation_;//目標姿勢(クォータニオン)

    Eigen::Vector3d target_acceleration_;//目標加速度
    Eigen::Matrix3d FLU2FRD_world;
    Eigen::Quaterniond q_init;//初期姿勢
    bool init_done = false;
    uint64_t slam_last_ns;
    uint64_t last_us{0};
    uint64_t odom_last_us{0};
    std::atomic<bool> have_offset_{false};
    Covariance3d cov_pos_px4;
    Covariance3d cov_rot_px4;
    std::ofstream csv_;
    const Eigen::Quaterniond q_FLU2FRD_world{
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
    };

    int offboard_setpoint_counter_ = 0;

    static constexpr double MAX_THRUST = 53.6; //最大スロットル値
    static constexpr double V_MAX = 1.0;
    static constexpr double d_slow = 1.0; //減速距離
    static constexpr double gravity_acceleration = 9.81; //重力加速度

    double VK_p = 1.0; //位置制御の比例ゲイン
    double AK_p = 1.0; //速度制御の比例ゲイン

    double I_max; //積分項の最大値
    double Kp_r;
    double Kp_p;
    double Kp_y;
    double Kd_r;
    double Kd_p;
    double Kd_y;
    double Ki_r;
    double Ki_p;
    double Ki_y;
    bool init_pix_att = false;
    
    double mass; //機体質量[kg]

    double target_yaw;
    double prev_roll=0.0;
    double prev_pitch=0.0;
    double prev_yaw=0.0;
    Eigen::Vector3d target_I = Eigen::Vector3d::Zero();
    const double THROTTLE_OFF = 0.0;
    const double THROTTLE_MEDIUM = 0.60;
    const double THROTTLE_FULL = 0.99;

    double HOOVER = 0.73;
    int rate = 5; //制御周期　5ms = 200Hz

    double TARGET_X = 0.0;
    double TARGET_Y = 0.0;
    double TARGET_Z = -1.0;
    static const int Motor_FR = 0;
    static const int Motor_BR = 1;
    static const int Motor_BL = 2;
    static const int Motor_FL = 3;
    double new_speeds[4] = {0.0, 0.0, 0.0, 0.0};//各モータへの出力
    bool run = true;//実行フラグ
    bool start_motor = false; //モータ始動フラグ
    std::thread key_thread_;//キーボード入力用スレッド
    bool start_actuator = false; //アクチュエータ始動フラグ
};