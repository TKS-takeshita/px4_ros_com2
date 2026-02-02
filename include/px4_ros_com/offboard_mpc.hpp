#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <Eigen/Dense>
#include "MCMPC.cuh"

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
    void VehicleCallback(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg);
    void publish_offboard_control_mode();

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode;
    rclcpp::Publisher<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_vel_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer;

    Eigen::Vector3d current_position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d current_velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond current_orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d angular_vel{Eigen::Vector3d::Zero()};

    bool odom_received = false;
    int offboard_counter = 0;

    MCMPC mcmpc;

};