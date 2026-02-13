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

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10,
        std::bind(&OffboardMPC::FastLioCallback, this, std::placeholders::_1));

    pose_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, 
        std::bind(&OffboardMPC::IMUCallback, this, std::placeholders::_1);
    )

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

void OffboardMPC::FastLioCallback(nav_msgs::msg::Odometry::SharedPtr &msg){
    current_position.x() = msg->pose.pose.position.x;
    current_position.y() = msg->pose.pose.position.y;
    current_position.z() = msg->pose.pose.position.z;

    odom_received = true;
}

void OffboardMPC::CalcStateRef(){
    tf2::Quaternion q;
    float roll = 0.0;//[rad]
    float pitch = 0.0;//[rad]
    float yaw = 0.0;//[rad]
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    double w = q.w();
    double x = q.x();
    double y = q.y();
    double z = q.z();

    current_ref = [w, x, y, z, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
}

void OffboardMPC::IMUCallback(sensor_msgs::msg::Imu::SharedPtr &msg){
    current_orientation.w() = msg->orientation.w;
    current_orientation.x() = msg->orientation.x;
    current_orientation.y() = msg->orientation.y;
    current_orientation.z() = msg->orientation.z;
}

void OffboardMPC::control_loop()
{
    publish_offboard_control_mode();
    if (!odom_received) {
        RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
        return;
    }

    float control_input[DIMENTION_OF_INPUT];
    
    float current_ref[DIMENTION_OF_STATE];
    mcmpc.get_opt_in(control_input, current_state, current_ref);
    

    Eigen::Vector4d control_input =  mcmpc.computeAngularVelocity(state);
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

void OffboardMPC::publish_offboard_avel(Eigen::Vector4d control_input)
{
    px4_msgs::msg::VehicleRateSetpoint msg{};
    msg.timestamp = steady_clock_.now().nanoseconds() / 1000;
    //FLU frame から FRD frameへ変換
    msg.roll = control_input[0];
    msg.pitch = -control_input[1];
    msg.yaw = -control_input[2];

    msg.thrust_body = [0, 0, 0, control_input[3]]

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

