#include <px4_ros_com/offboard_mpc.hpp>

OffboardMPC::OffboardMPC()
    : Node("offboard_mpc"),
      mcmpc()
{
    // Initialize MPC parameters
    INPUT_SDEV = {1.0f, 0.4f, 0.4f, 0.125f};
    INPUT_MIN = {-55, -1e3f, -1e3f, -1e3f};
    INPUT_MAX = {55f, 1e3f, 1e3f, 1e3f};
    STATE_MIN = {-1E4, -1E4, -1E4, -1E4, -1E4, -1E4, -1E4, -1E4, -1E4, -1E4, -1E4, -1E4, -1E4};
    STATE_MAX = {1E4, 1E4, 1E4, 1E4, 1E4, 1E4, 1E4, 1E4, 1E4, 1E4, 1E4, 1E4, 1E4};
    MCMPC_COST_R = {0.01, 0.001f, 0.001f, 0.01f};
    MCMPC_COST_Q = {0, 150, 150, 36, 36, 9, 3, 3, 5, 3, 3, 5};
    MCMPC_COST_QF = {0, 150, 150, 150, 36, 36, 9, 3, 3, 5, 3, 3, 5};
    MCMPC_COST_BARRIER = 1e30;
    MCMPC_LAMBDA = 5000;
    MCMPC_ITERATION = 3;
    MCMPC_SOLVER = rk4;
    MCMPC_STATE_REF = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    mcmpc = MCMPC(DT, INPUT_SDEV, INPUT_MIN, INPUT_MAX, PLANT_PARAM,
                   STATE_MIN, STATE_MAX, MCMPC_COST_R, MCMPC_COST_Q, MCMPC_COST_QF,
                   MCMPC_COST_BARRIER, MCMPC_LAMBDA, MCMPC_ITERATION, MCMPC_SOLVER, MCMPC_STATE_REF);

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
    msg.body_rate = true;
    msg.direct_actuator = false; //
    msg.thrust_and_torque = false;
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

