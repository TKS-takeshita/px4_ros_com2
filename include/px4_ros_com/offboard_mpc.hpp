#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_rate_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include "MCMPC.cuh"
#include "plant.cuh"
#include "common.cuh"

using namespace px4_ros_com::frame_transforms;
using px4_msgs::msg::VehicleCommand;

class OffboardMPC : public rclcpp::Node
{
public:
    OffboardMPC();

private:
    void arm();
    void disarm();
    void control_loop();
    void publish_offboard_avel(Eigen::Vector3d control_input);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    // void VehicleCallback(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg);
    void FastLioCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void IMUCallback(sensor_msgs::msg::Imu::SharedPtr msg);
    void publish_offboard_control_mode();

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode;
    rclcpp::Publisher<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_vel_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timer;

    Eigen::Vector3d current_position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d current_velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond current_orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d angular_vel{Eigen::Vector3d::Zero()};

    bool odom_received = false;
    int offboard_counter = 0;

    const float DT = 0.005f; // 200Hz

    float PLANT_STT_INIT[DIMENTION_OF_STATE];
    float PLANT_PARAM[DIMENTION_OF_PARAM];
    float PLANT_INPUT_INIT[DIMENTION_OF_INPUT];
    float PLANT_OUTPUT_INIT[DIMENTION_OF_OUTPUT];
    
    float INPUT_SDEV[DIMENTION_OF_INPUT];
    float INPUT_MIN[DIMENTION_OF_INPUT];
    float INPUT_MAX[DIMENTION_OF_INPUT];
    float STATE_MIN[DIMENTION_OF_STATE];
    float STATE_MAX[DIMENTION_OF_STATE];
    float MCMPC_COST_R[DIMENTION_OF_INPUT];
    float MCMPC_COST_Q[DIMENTION_OF_STATE];
    float MCMPC_COST_QF[DIMENTION_OF_STATE];
    float MCMPC_COST_BARRIER;
    float MCMPC_LAMBDA;
    int MCMPC_ITERATION;

    enum solver MCMPC_SOLVER;
    float MCMPC_STATE_REF[DIMENTION_OF_STATE];

    float current_state[DIMENTION_OF_STATE];
    float current_ref[DIMENTION_OF_STATE];

    MCMPC mcmpc;
};