#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <atomic>
#include <algorithm>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"/joy", 10,
			std::bind(&OffboardControl::joy_callback, this, std::placeholders::_1));

		auto qos = rclcpp::SensorDataQoS();

		local_pos_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"/fmu/out/vehicle_odometry", qos,
			std::bind(&OffboardControl::local_position_callback, this, std::placeholders::_1));

		offboard_setpoint_counter_ = 0;
		is_armed_ = false;
		offboard_enabled_ = false;
		emergency_stop_ = false;
		got_local_pos_ = false;

		target_x_ = 0.0f;
		target_y_ = 0.0f;
		target_z_ = 0.0f;
		target_yaw_ = 0.0f;

		prev_a_button_ = 0;
		prev_b_button_ = 0;
		prev_x_button_ = 0;

		auto timer_callback = [this]() -> void {

			// 1) arm前のoffboard移行用 予送信
			if (!offboard_enabled_ && !emergency_stop_) {
				publish_offboard_control_mode();
				publish_trajectory_setpoint();

				if (offboard_setpoint_counter_ < 20) {
					offboard_setpoint_counter_++;
				}
			}

			// 2) arm要求が来たら offboard + arm
			if (arm_request_ && !offboard_enabled_ && offboard_setpoint_counter_ >= 10) {
				publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				arm();
				offboard_enabled_ = true;
				arm_request_ = false;
			}

			// 3) 通常飛行中のみ継続送信
			if (offboard_enabled_ && is_armed_ && !emergency_stop_) {
				publish_offboard_control_mode();
				publish_trajectory_setpoint();
			}
		};

		timer_ = this->create_wall_timer(10ms, timer_callback);
	}

	void arm()
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
		is_armed_ = true;
		RCLCPP_INFO(this->get_logger(), "Arm command sent");
	}

	void disarm()
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
		is_armed_ = false;
		offboard_enabled_ = false;
		arm_request_ = false;
		RCLCPP_WARN(this->get_logger(), "Disarm command sent");
	}

	void kill()
	{
		publish_vehicle_command(
			VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
			0.0f,
			21196.0f   // ← 強制停止
		);

		is_armed_ = false;
		offboard_enabled_ = false;
		arm_request_ = false;
		emergency_stop_ = true;

		RCLCPP_ERROR(this->get_logger(), "!!! KILL ACTIVATED !!!");
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr local_pos_subscriber_;

	uint64_t offboard_setpoint_counter_;

	bool is_armed_;
	bool offboard_enabled_;
	bool emergency_stop_;
	bool got_local_pos_;
	bool arm_request_{false};

	float target_x_;
	float target_y_;
	float target_z_;
	float target_yaw_;

	float current_x_{0.0f};
	float current_y_{0.0f};
	float current_z_{0.0f};
	float current_yaw_{0.0f};

	int prev_a_button_;
	int prev_b_button_;
	int prev_x_button_;

	float wrap_pi(float yaw)
	{
		return std::atan2(std::sin(yaw), std::cos(yaw));
	}

	void land()
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
		RCLCPP_INFO(this->get_logger(), "Land command sent");
	}

	void publish_offboard_control_mode()
	{
		OffboardControlMode msg{};
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		offboard_control_mode_publisher_->publish(msg);
	}

	void publish_trajectory_setpoint()
	{
		TrajectorySetpoint msg{};
		msg.position = {target_x_, target_y_, target_z_};
		msg.yaw = target_yaw_;
		// msg.yawspeed = 0.0;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
	{
		VehicleCommand msg{};
		msg.param1 = param1;
		msg.param2 = param2;
		msg.command = command;
		msg.target_system = 1;
		msg.target_component = 1;
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		vehicle_command_publisher_->publish(msg);
	}

	void local_position_callback(const px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg )
	{
		current_x_ = msg->position[0];
		current_y_ = msg->position[1];
		current_z_ = msg->position[2];
		double q_w = msg->q[0];
		double q_x = msg->q[1];
		double q_y = msg->q[2];
		double q_z = msg->q[3];
		double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
		double cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
		current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
		got_local_pos_ = true;
		RCLCPP_INFO(this->get_logger(), "X = %0.2F, Y = %0.2F, Z = %0.2F, yaw = %0.2f", current_x_, current_y_, current_z_, current_yaw_);
	}

	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		// joy_nodeの一般的な割当例
		// A = buttons[0], B = buttons[1], X = buttons[2]
		// 実機で /joy を確認して必要なら番号変更

		int a_button 			= (msg->buttons.size() > 0) 	? msg->buttons[0] : 0;
		int b_button 			= (msg->buttons.size() > 1) 	? msg->buttons[1] : 0;
		int x_button 			= (msg->buttons.size() > 2) 	? msg->buttons[2] : 0;
		int y_button 			= (msg->buttons.size() > 3) 	? msg->buttons[3] : 0;
		int lb_button 			= (msg->buttons.size() > 4) 	? msg->buttons[4] : 0;
		int rb_button 			= (msg->buttons.size() > 5) 	? msg->buttons[5] : 0;
		int back_button 		= (msg->buttons.size() > 6)		? msg->buttons[6] : 0;
		int start_button 		= (msg->buttons.size() > 7) 	? msg->buttons[7] : 0;
		int power_button 		= (msg->buttons.size() > 8) 	? msg->buttons[8] : 0;
		int stick_left_button 	= (msg->buttons.size() > 9) 	? msg->buttons[9] : 0;
		int stick_right_button 	= (msg->buttons.size() > 10) 	? msg->buttons[10] : 0;

		// Aボタン立ち上がり arm + 浮上
		if (a_button == 1) {
			if (!is_armed_) {
				emergency_stop_ = false;
				arm_request_ = true;
				RCLCPP_INFO(this->get_logger(), "A pressed -> request ARM + OFFBOARD");
			} else {
				target_x_ = current_x_;
				target_y_ = current_y_;
				target_z_ = -1.2f;
				target_yaw_ = current_yaw_;
				
			}
		}

		// Bボタンで緊急停止（即disarm）
		if (b_button == 1) {
			emergency_stop_ = true;
			disarm();
			RCLCPP_WARN(this->get_logger(), "B pressed -> EMERGENCY STOP / DISARM");
		}

		// Xボタンで現在位置でホバリング
		if (x_button == 1) {
			if (got_local_pos_) {
				target_x_ = current_x_;
				target_y_ = current_y_;
				target_z_ = current_z_;
				target_yaw_ = current_yaw_;
				RCLCPP_INFO(this->get_logger(), "X pressed -> hold current position");
			}
		}

		// Yボタンで離陸
		if(y_button == 1){
			if(got_local_pos_){
				target_x_ = current_x_;
				target_y_ = current_y_;
				target_z_ = 0.0;
				target_yaw_ = current_yaw_;				
			}
		}

		//LBボタンで前方に移動
		if(lb_button == 1){
			target_x_ = current_x_ + 1.5f; // 前方に1m移動
			// target_y_ = current_y_;
			// target_z_ = -1.2f;
			// target_yaw_ = current_yaw_;
			RCLCPP_INFO(this->get_logger(), "LB pressed -> move forward");
		}

		//RBボタンで後方に移動
		if(rb_button == 1){
			target_x_ = current_x_ - 1.5f; // 後方に1m移動
			// target_y_ = current_y_;
			// target_z_ = -1.2f;
			// target_yaw_ = current_yaw_;
			RCLCPP_INFO(this->get_logger(), "RB pressed -> move backward");
		}

		// Startボタンで左に移動
		if(start_button == 1){
			// target_x_ = current_x_;
			target_y_ = current_y_ - 1.5f; // 左に1m移動
			// target_z_ = -1.2f;
			// target_yaw_ = current_yaw_;
			RCLCPP_INFO(this->get_logger(), "Start pressed -> move left");
		}

		// Backボタンで右に移動
		if(back_button == 1){
			// target_x_ = current_x_;
			target_y_ = current_y_ + 1.5f; // 右に
			// target_z_ = -1.2f;
			// target_yaw_ = current_yaw_;
			RCLCPP_INFO(this->get_logger(), "Back pressed -> move right");
		}

		if (power_button == 1) {
			kill();
		}

		// stick_left_buttonで時計回りにyawを変更
		if(stick_left_button == 1){
			// target_x_ = current_x_;
			// target_y_ = current_y_;
			// target_z_ = -1.2f;
			target_yaw_ = wrap_pi(current_yaw_ + M_PI / 4.0f); // 時計回りに45度回転
			RCLCPP_INFO(this->get_logger(), "Left stick button pressed -> rotate CW");
		}

		// stick_right_buttonで反時計回りにyawを変更
		if(stick_right_button == 1){
			// target_x_ = current_x_;
			// target_y_ = current_y_;
			// target_z_ = -1.2f;
			target_yaw_ = wrap_pi(current_yaw_ - M_PI / 4.0f); // 反時計回りに45度回転
			RCLCPP_INFO(this->get_logger(), "Right stick button pressed -> rotate CCW");
		}

	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}
