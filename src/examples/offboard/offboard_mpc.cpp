#include <px4_ros_com/offboard_mpc.hpp>

OffboardMPC::OffboardMPC()
    : Node("offboard_mpc"),
      mcmpc()
{
    offboard_control_mode = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);

    angular_vel_pub = this->create_publisher<px4_msgs::msg::VehicleAngularVelocity>(
        "/fmu/in/vehicle_angular_velocity", 10);

    vehicle_cmd_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", 10,
        std::bind(&OffboardMPC::VehicleCallback, this, std::placeholders::_1));

    publish_offboard_control_mode();
    auto timer_callback = [this]() -> void {
        
        if (offboard_setpoint_counter_ == 100) {
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            this->arm();
		}

        control_loop();
        offboard_setpoint_counter_++;
    };
    timer = this->create_wall_timer(5ms, timer_callback);//200Hz
    RCLCPP_INFO(this->get_logger(), "Offboard MPC Node has been started.");
}

void OffboardMPC::VehicleCallback(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg)
{
    //FRD frame 
    // 慣性系
    current_position.x() = msg->position[0];
    current_position.y() = msg->position[1];
    current_position.z() = msg->position[2];

    current_velocity.x() = msg->velocity[0];
    current_velocity.y() = msg->velocity[1];
    current_velocity.z() = msg->velocity[2];

    current_orientation.w() = msg->q[0];
    current_orientation.x() = msg->q[1];
    current_orientation.y() = msg->q[2];
    current_orientation.z() = msg->q[3];

    odom_received = true;
}

void OffboardMPC::control_loop()
{
    publish_offboard_control_mode();
    if (!odom_received) {
        RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
        return;
    }
    MCMPCState state;
    state.position = current_position;
    state.velocity = current_velocity;
    state.orientation = current_orientation;

    Eigen::Vector3d control_input =  mcmpc.computeAngularVelocity(state);
    publish_offboard_avel(control_input);
}

void OffboardMPC::publisho_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.direct_actuator = false; //
    msg.thrust_and_torque = true;
    msg.timestamp = steady_clock_.now().nanoseconds() / 1000;
    offboard_control_mode->publish(msg);
}

void OffboardMPC::publish_offboard_avel(Eigen::Vector3d control_input)
{
    px4_msgs::msg::VehicleAngularVelocity msg{};
    msg.timestamp = steady_clock_.now().nanoseconds() / 1000;
    msg.xyz[0] = control_input[0];
    msg.xyz[1] = control_input[1];
    msg.xyz[2] = control_input[2];
    angular_vel_pub->publish(msg);
}

void OffboardPIDControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}
void OffboardPIDControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardMPC::publish_vehicle_command(uint16_t command, float param1)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_cmd_pub_->publish(msg);
}

