#include "px4_ros_com/offboard_PID.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

OffboardPIDControl::OffboardPIDControl() : rclcpp::Node("offboard_pid_control"){
    get_param();
    last_us = this->get_clock()->now().nanoseconds() / 1000;
    slam_last_ns = last_us * 1000;

    // Publisher
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    actuator_publisher_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors",10);
    motor_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint",10);
    // Subscriber
    vehicle_odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/in/vehicle_visual_odometry",                 // 読み取りは out 側
    rclcpp::SensorDataQoS(),
    std::bind(&OffboardPIDControl::VehicleCallback, this, std::placeholders::_1));

    auto timer_callback = [this]() -> void {
        if(!start_motor){
            return;
        }
        if (offboard_setpoint_counter_ == 50){
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        }

        if (offboard_setpoint_counter_ == 100) {
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            this->arm();
		}
        
        
        if(start_actuator){
            publish_offboard_actuator_control_mode();
            publish_actuator_setpoint();
        }
        else{
            publish_offboard_position_control_mode();
            publish_motor_setpoint();
        }
        // publish_actuator_setpoint();
        if (offboard_setpoint_counter_ < 101) {
            offboard_setpoint_counter_++;
        }
    };
    key_input();
    timer_ = this->create_wall_timer(5ms, timer_callback);//200Hz
    target_position_ << TARGET_X, TARGET_Y, TARGET_Z; //目標位置(SLAM座標系)
    target_yaw = 0.0; //目標姿勢(ヨー角)
}

void OffboardPIDControl::get_param(){
    VK_p = this->declare_parameter<double>("VK_p", 5.0);
    AK_p = this->declare_parameter<double>("AK_p", 5.0);
    Kp_r = this->declare_parameter<double>("Kp_r", 0.5);
    Kd_r = this->declare_parameter<double>("Kd_r", 0.1);
    Ki_r = this->declare_parameter<double>("Ki_r", 0.001);
    Kp_p = this->declare_parameter<double>("Kp_p", 0.5);
    Kd_p = this->declare_parameter<double>("Kd_p", 0.1);
    Ki_p = this->declare_parameter<double>("Ki_p", 0.001);
    Kp_y = this->declare_parameter<double>("Kp_y", 0.5);
    Kd_y = this->declare_parameter<double>("Kd_y", 0.1);
    Ki_y = this->declare_parameter<double>("Ki_y", 0.001);
    I_max = this->declare_parameter<double>("I_MAX", 0.5);
    mass = this->declare_parameter<double>("mass", 3.14);
    HOOVER = this->declare_parameter<double>("HOOVER", 0.70);
}
void OffboardPIDControl::key_input(){
   key_thread_ = std::thread([this]() {
       initscr();
       cbreak(); 
       noecho();               
       keypad(stdscr, TRUE);
       nodelay(stdscr, FALSE);
       while (rclcpp::ok()) {
           int ch = getch();
           if (ch == 's' || ch == 'S') {
               start_motor = true;
               RCLCPP_WARN(this->get_logger(), "'s' received: control loop enabled.");
           } else if (ch == 'a' || ch == 'A') {
               start_actuator = true;
               RCLCPP_WARN(this->get_logger(), "'a' received: actuator control enabled.");
           }
           else if(ch == KEY_UP){
               TARGET_X += 0.5;
               RCLCPP_WARN(this->get_logger(), "TARGET_X: %f, TARGET_Y: %f, TARGET_Z: %f", TARGET_X, TARGET_Y, TARGET_Z);
           }
           else if(ch == KEY_DOWN){
               TARGET_X -= 0.5;
               RCLCPP_WARN(this->get_logger(), "TARGET_X: %f, TARGET_Y: %f, TARGET_Z: %f", TARGET_X, TARGET_Y, TARGET_Z);
           }
           else if(ch == KEY_RIGHT){
               TARGET_Y += 0.5;
               RCLCPP_WARN(this->get_logger(), "TARGET_X: %f, TARGET_Y: %f, TARGET_Z: %f", TARGET_X, TARGET_Y, TARGET_Z);
           }
           else if(ch == KEY_LEFT){
               TARGET_Y -= 0.5;
               RCLCPP_WARN(this->get_logger(), "TARGET_X: %f, TARGET_Y: %f, TARGET_Z: %f", TARGET_X, TARGET_Y, TARGET_Z);
           }
           else if(ch == KEY_PPAGE){
               TARGET_Z += 0.5;
               RCLCPP_WARN(this->get_logger(), "TARGET_X: %f, TARGET_Y: %f, TARGET_Z: %f", TARGET_X, TARGET_Y, TARGET_Z);
           }
           else if(ch == KEY_NPAGE){
               TARGET_Z -= 0.5;
               RCLCPP_WARN(this->get_logger(), "TARGET_X: %f, TARGET_Y: %f, TARGET_Z: %f", TARGET_X, TARGET_Y, TARGET_Z);
           }
           target_position_ << TARGET_X, TARGET_Y, TARGET_Z;
       }
       endwin();
   });
}

double OffboardPIDControl::throttle_thrust(double thr){
    return (thr - 3.6535)/17.326;
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

double OffboardPIDControl::wrapPi(double a){
    a = std::fmod(a + M_PI, 2.0*M_PI);
    if (a < 0) a += 2.0*M_PI;
    return a - M_PI;
}

void OffboardPIDControl::VehicleCallback(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg){
    // std::cout << "VehicleCallback" << std::endl;
    current_position_.x() = msg->position[0];//FRD
    current_position_.y() = msg->position[1];
    current_position_.z() = msg->position[2];
    current_velocity_.x() = msg->velocity[0];//FRD
    current_velocity_.y() = msg->velocity[1];
    current_velocity_.z() = msg->velocity[2];
    current_angular_velocity_.x() = msg->angular_velocity[0];//FRD
    current_angular_velocity_.y() = msg->angular_velocity[1];
    current_angular_velocity_.z() = msg->angular_velocity[2];
    current_orientation_.w() = msg->q[0];//FRD
    current_orientation_.x() = msg->q[1];
    current_orientation_.y() = msg->q[2];
    current_orientation_.z() = msg->q[3];
    current_rot_matrix_ = current_orientation_.toRotationMatrix();

    controlLoop();
}

void OffboardPIDControl::controlLoop(){
    Eigen::Vector3d position_error = target_position_ - current_position_;//global FRD
    Eigen::Vector3d target_velocity_ = VK_p * position_error.normalized();//FRD
    Eigen::Vector3d velocity_error = target_velocity_ - current_velocity_;//gloabl FRD
    target_acceleration_ = AK_p* velocity_error;//global FRD
    prev_velocity_ = current_velocity_;
    target_acceleration_[2] -= gravity_acceleration; // 重力加速度を補償
    // RCLCPP_INFO(this->get_logger(), "target_acceleration_: [%f, %f, %f]", target_acceleration_.x(), target_acceleration_.y(), target_acceleration_.z());
    Eigen::Vector3d target_thrust = target_acceleration_ * mass; // 推力 = 加速度 * 質量
    double thrust_magnitude = target_thrust.norm();
    Eigen::Vector3d Z_b = -target_thrust.normalized();//機体のZ軸(推力方向) FRD
    Eigen::Vector3d X_c = {std::cos(target_yaw), std::sin(target_yaw), 0.0};
    Eigen::Vector3d Y_b = Z_b.cross(X_c) / ((Z_b.cross(X_c)).norm());
    Eigen::Vector3d X_b = (Y_b.cross(Z_b)).normalized();
    Eigen::Matrix3d target_R;
    target_R.col(0) = X_b;
    target_R.col(1) = Y_b;
    target_R.col(2) = Z_b;
    target_orientation_ = Eigen::Quaterniond(target_R);
    Eigen::Vector3d target_euler = target_R.eulerAngles(2, 1, 0);
    Eigen::Vector3d current_euler = current_rot_matrix_.eulerAngles(2, 1, 0);
    double target_roll = target_euler[2];
    double target_pitch = target_euler[1];
    double current_roll = current_euler[2];
    double current_pitch = current_euler[1];
    double current_yaw = current_euler[0];
    // Eigen::Quaterniond q_err = target_orientation_ * current_orientation_.conjugate();
    // Eigen::Vector3d eZYX = q_err.toRotationMatrix().eulerAngles(2, 1, 0);

    double yaw_err = wrapPi(target_yaw - current_yaw);
    double pitch_err = wrapPi(target_pitch - current_pitch);
    double roll_err  = wrapPi(target_roll  - current_roll);

    Eigen::Vector3d att_control;
    att_control[0] = Kp_r* roll_err - Kd_r*(current_roll - prev_roll)*rate/1000 + Ki_r*target_I[0];
    att_control[1] = Kp_p* pitch_err - Kd_p*(current_pitch - prev_pitch)*rate/1000 + Ki_p*target_I[1];
    att_control[2] = Kp_y* yaw_err - Kd_y*(current_yaw - prev_yaw)*rate/1000 + Ki_y*target_I[2];//

    double U1 = throttle_thrust(thrust_magnitude);
    double U2 = throttle_thrust(att_control[0]);
    double U3 = throttle_thrust(att_control[1]);
    double U4 = throttle_thrust(att_control[2]);

    target_I[0] += roll_err * (1.0/rate*1000);
    target_I[1] += pitch_err * (1.0/rate*1000);
    target_I[2] += yaw_err * (1.0/rate*1000);
    for (size_t i = 0; i < 3; i++)
    {
        if (target_I[i] > I_max) target_I[i] = I_max;
        if (target_I[i] < -I_max) target_I[i] = -I_max;
    }
    
    prev_roll = current_roll;
    prev_pitch = current_pitch;
    prev_yaw = current_yaw;

    //実機
    // new_speeds[0] = (U1-U4)/4.0 + U3/2.0;//F CW
    // new_speeds[1] = (U1-U4)/4.0 - U3/2.0;//B CW
    // new_speeds[2] = (U1+U4)/4.0 + U2/2.0;//R CCW
    // new_speeds[3] = (U1+U4)/4.0 - U2/2.0;//L CCW

    //simulator
    new_speeds[0] = (U1+U4)/4.0 + U3/2.0;//F CCW
    new_speeds[1] = (U1-U4)/4.0 - U3/2.0;//B CCW
    new_speeds[2] = (U1+U4)/4.0 + U2/2.0;//R CW
    new_speeds[3] = (U1-U4)/4.0 - U2/2.0;//L CW
    // new_speeds[0] = 0.95;//F CCW
    // new_speeds[1] = 0.95;//B CCW
    // new_speeds[2] = 0.95;//R CW
    // new_speeds[3] = 0.95;//L CW

    // RCLCPP_INFO(this->get_logger(), "new_speeds: [%f, %f, %f, %f]", new_speeds[0], new_speeds[1], new_speeds[2], new_speeds[3]);

    for (int i = 0; i < 4; ++i) {
        if (new_speeds[i] < THROTTLE_OFF) new_speeds[i] = THROTTLE_OFF;
        if (new_speeds[i] > THROTTLE_FULL) new_speeds[i] = THROTTLE_FULL;
    }
}

void OffboardPIDControl::publish_offboard_position_control_mode(){
    OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.direct_actuator = false;//
	msg.timestamp = steady_clock_.now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void OffboardPIDControl::publish_offboard_actuator_control_mode(){
    OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.direct_actuator = true;//
	msg.timestamp = steady_clock_.now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void OffboardPIDControl::publish_vehicle_command(uint16_t command, float param1, float param2){
    VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = steady_clock_.now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}
void OffboardPIDControl::publish_actuator_setpoint(){
    ActuatorMotors msg{};
    msg.timestamp = steady_clock_.now().nanoseconds() / 1000;
    msg.control[0] = new_speeds[0];
    msg.control[1] = new_speeds[1];
    msg.control[2] = new_speeds[2];
    msg.control[3] = new_speeds[3];
    actuator_publisher_->publish(msg);
}
void OffboardPIDControl::publish_motor_setpoint(){
    TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -1.0};
    msg.velocity = {NAN, NAN, NAN};
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    msg.yawspeed = NAN;
    msg.yaw = target_yaw;
    msg.timestamp = steady_clock_.now().nanoseconds() / 1000;
	motor_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardPIDControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}