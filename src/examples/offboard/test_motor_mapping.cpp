#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include "px4_msgs/msg/actuator_motors.hpp"

using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class MotorTestNode : public rclcpp::Node {
public:
    MotorTestNode() : Node("motor_test_node") {
        timer_ = this->create_wall_timer(5ms, std::bind(&MotorTestNode::timer_callback, this));
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    rclcpp::TimerBase::SharedPtr timer_;
    int offboard_setpoint_counter_ = 0;
    void timer_callback() {
        if(offboard_setpoint_counter_ == 10){
            arm();
        }
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_MOTOR_TEST, 1, 2);
        offboard_setpoint_counter_++;
    }

    void arm(){
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command send");
    }

    void disarm(){
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command send");
    }

    void publish_vehicle_command(uint16_t command, float param1=0.0, float param2=0.0){
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = 0.8;
        msg.param4 = 10.0;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.confirmation = 0;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}