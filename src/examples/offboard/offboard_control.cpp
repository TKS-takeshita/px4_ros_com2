#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/takeoff_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/rate_ctrl_status.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <stdint.h>

#include <array>
#include <chrono>
#include <iostream>
#include <atomic>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#ifndef OFFBOARD_CONTROL_LOG_PATH
#define OFFBOARD_CONTROL_LOG_PATH "offboard_control_log.csv"
#endif

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

		local_position_setpoint_subscriber_ = this->create_subscription<VehicleLocalPositionSetpoint>(
			"/fmu/out/vehicle_local_position_setpoint", qos,
			std::bind(&OffboardControl::local_position_setpoint_callback, this, std::placeholders::_1));

		attitude_setpoint_subscriber_ = this->create_subscription<VehicleAttitudeSetpoint>(
			"/fmu/out/vehicle_attitude_setpoint_v1", qos,
			std::bind(&OffboardControl::attitude_setpoint_callback, this, std::placeholders::_1));
		// attitude_setpoint_subscriber_ = this->create_subscription<VehicleAttitudeSetpoint>(
		// 	"/fmu/out/vehicle_attitude_setpoint", qos,
		// 	std::bind(&OffboardControl::attitude_setpoint_callback, this, std::placeholders::_1));

		rates_setpoint_subscriber_ = this->create_subscription<VehicleRatesSetpoint>(
			"/fmu/out/vehicle_rates_setpoint", qos,
			std::bind(&OffboardControl::rates_setpoint_callback, this, std::placeholders::_1));

		rate_ctrl_status_subscriber_ = this->create_subscription<RateCtrlStatus>(
			"/fmu/out/rate_ctrl_status", qos,
			std::bind(&OffboardControl::rate_ctrl_status_callback, this, std::placeholders::_1));

		thrust_setpoint_subscriber_ = this->create_subscription<VehicleThrustSetpoint>(
			"/fmu/out/vehicle_thrust_setpoint", qos,
			std::bind(&OffboardControl::thrust_setpoint_callback, this, std::placeholders::_1));

		torque_setpoint_subscriber_ = this->create_subscription<VehicleTorqueSetpoint>(
			"/fmu/out/vehicle_torque_setpoint", qos,
			std::bind(&OffboardControl::torque_setpoint_callback, this, std::placeholders::_1));

		actuator_motors_subscriber_ = this->create_subscription<ActuatorMotors>(
			"/fmu/out/actuator_motors", qos,
			std::bind(&OffboardControl::actuator_motors_callback, this, std::placeholders::_1));

		local_pos_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
			"/fmu/out/vehicle_local_position", qos,
			std::bind(&OffboardControl::local_position_callback, this, std::placeholders::_1));

		attitude_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
			"/fmu/out/vehicle_attitude", qos,
			std::bind(&OffboardControl::vehicle_attitude_callback, this, std::placeholders::_1));

		angular_velocity_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
			"/fmu/out/vehicle_angular_velocity", qos,
			std::bind(&OffboardControl::angular_velocity_callback, this, std::placeholders::_1));

		livox_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
			"/livox/imu", qos,
			std::bind(&OffboardControl::livox_imu_callback, this, std::placeholders::_1));

		hover_thrust_estimate_subscriber_ = this->create_subscription<px4_msgs::msg::HoverThrustEstimate>(
			"/fmu/out/hover_thrust_estimate", qos,
			std::bind(&OffboardControl::hover_thrust_estimate_callback, this, std::placeholders::_1));

		takeoff_status_subscriber_ = this->create_subscription<px4_msgs::msg::TakeoffStatus>(
			"/fmu/out/takeoff_status", qos,
			std::bind(&OffboardControl::takeoff_status_callback, this, std::placeholders::_1));

		vehicle_land_detected_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
			"/fmu/out/vehicle_land_detected", qos,
			std::bind(&OffboardControl::vehicle_land_detected_callback, this, std::placeholders::_1));

		open_log_file();
		declare_trajectory_parameters();

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
		prev_y_button_ = 0;

		phase_start_time_ = this->get_clock()->now();
		control_start_time_ = phase_start_time_;

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
				update_auto_trajectory();
				publish_offboard_control_mode();
				publish_trajectory_setpoint();
			}
		};

		timer_ = this->create_wall_timer(20ms, timer_callback);
	}

	~OffboardControl()
	{
		if (log_file_.is_open()) {
			log_file_.close();
		}
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
	rclcpp::Subscription<VehicleLocalPositionSetpoint>::SharedPtr local_position_setpoint_subscriber_;
	rclcpp::Subscription<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_subscriber_;
	rclcpp::Subscription<VehicleRatesSetpoint>::SharedPtr rates_setpoint_subscriber_;
	rclcpp::Subscription<RateCtrlStatus>::SharedPtr rate_ctrl_status_subscriber_;
	rclcpp::Subscription<VehicleThrustSetpoint>::SharedPtr thrust_setpoint_subscriber_;
	rclcpp::Subscription<VehicleTorqueSetpoint>::SharedPtr torque_setpoint_subscriber_;
	rclcpp::Subscription<ActuatorMotors>::SharedPtr actuator_motors_subscriber_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_subscriber_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_subscriber_;
	rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr angular_velocity_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr livox_imu_subscriber_;
	rclcpp::Subscription<HoverThrustEstimate>::SharedPtr hover_thrust_estimate_subscriber_;
	rclcpp::Subscription<TakeoffStatus>::SharedPtr takeoff_status_subscriber_;
	rclcpp::Subscription<VehicleLandDetected>::SharedPtr vehicle_land_detected_subscriber_;

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
	float current_vx_{0.0f};
	float current_vy_{0.0f};
	float current_vz_{0.0f};
	float current_z_deriv_{0.0f};
	float current_roll_{0.0f};
	float current_pitch_{0.0f};
	float current_yaw_{0.0f};
	float current_wx_{0.0f};
	float current_wy_{0.0f};
	float current_wz_{0.0f};
	uint64_t current_local_position_timestamp_{0};
	uint64_t current_local_position_timestamp_sample_{0};
	uint64_t current_attitude_timestamp_{0};
	uint64_t current_attitude_timestamp_sample_{0};
	uint64_t current_angular_velocity_timestamp_{0};
	uint64_t current_angular_velocity_timestamp_sample_{0};
	bool got_attitude_{false};
	bool got_angular_velocity_{false};

	float setpoint_vx_{0.0f};
	float setpoint_vy_{0.0f};
	float setpoint_vz_{0.0f};
	float setpoint_yawspeed_{0.0f};

	std::mutex sp_mutex_;
	VehicleLocalPositionSetpoint latest_traj_sp_{};
	VehicleAttitudeSetpoint latest_att_sp_{};
	VehicleRatesSetpoint latest_rate_sp_{};
	RateCtrlStatus latest_rate_ctrl_status_{};
	VehicleThrustSetpoint latest_thrust_sp_{};
	VehicleTorqueSetpoint latest_torque_sp_{};
	ActuatorMotors latest_actuator_motors_{};
	VehicleAngularVelocity latest_angular_velocity_{};
	sensor_msgs::msg::Imu latest_livox_imu_{};
	HoverThrustEstimate latest_hover_thrust_estimate_{};
	TakeoffStatus latest_takeoff_status_{};
	VehicleLandDetected latest_vehicle_land_detected_{};
	bool has_traj_sp_{false};
	bool has_att_sp_{false};
	bool has_rate_sp_{false};
	bool has_rate_ctrl_status_{false};
	bool has_thrust_sp_{false};
	bool has_torque_sp_{false};
	bool has_actuator_motors_{false};
	bool has_angular_velocity_{false};
	bool has_livox_imu_{false};
	bool has_hover_thrust_estimate_{false};
	bool has_takeoff_status_{false};
	bool has_vehicle_land_detected_{false};

	std::mutex log_event_mutex_;
	std::string pending_log_event_;

	enum class TrajectoryPhase {
		Idle,
		Waypoint,
		SinX,
		SinY,
		SinZ,
		StepHold,
		StepApplied,
		ConfiguredInput,
		Done
	};

	enum class AxisMotionMode {
		Hold,
		Step,
		Sin
	};

	struct Waypoint {
		float x;
		float y;
		float z;
		float yaw;
	};

	struct AxisMotionConfig {
		AxisMotionMode x{AxisMotionMode::Sin};
		AxisMotionMode y{AxisMotionMode::Hold};
		AxisMotionMode z{AxisMotionMode::Hold};
	};

	TrajectoryPhase trajectory_phase_{TrajectoryPhase::Idle};
	AxisMotionConfig axis_motion_config_{};
	std::vector<Waypoint> waypoints_;
	std::size_t waypoint_index_{0};
	rclcpp::Time phase_start_time_;
	rclcpp::Time control_start_time_;

	const float fixed_altitude_{-1.0f};
	const float waypoint_acceptance_radius_{0.15f};
	const double waypoint_min_hold_sec_{1.0};
	const double sin_period_sec_{5.0};
	const double sin_phase_duration_sec_{50.0};
	const float x_sin_amplitude_{1.0f};
	const float sin_xy_amplitude_{2.0f};
	const float sin_z_amplitude_{0.5f};
	const double step_hold_duration_sec_{2.0};
	const float pi_{3.14159265359f};
	bool x_sine_only_{false};

	std::ofstream log_file_;
	std::string log_path_{OFFBOARD_CONTROL_LOG_PATH};

	int prev_a_button_;
	int prev_b_button_;
	int prev_x_button_;
	int prev_y_button_;

	float distance_to_target(const Waypoint &wp) const
	{
		const float dx = current_x_ - wp.x;
		const float dy = current_y_ - wp.y;
		const float dz = current_z_ - wp.z;
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	}

	double phase_elapsed_sec()
	{
		return (this->get_clock()->now() - phase_start_time_).seconds();
	}

	double control_elapsed_sec()
	{
		return (this->get_clock()->now() - control_start_time_).seconds();
	}

	void declare_trajectory_parameters()
	{
		this->declare_parameter<std::string>("trajectory.x_mode", "sin");
		this->declare_parameter<std::string>("trajectory.y_mode", "hold");
		this->declare_parameter<std::string>("trajectory.z_mode", "hold");
	}

	AxisMotionMode parse_axis_motion_mode(const std::string &mode, const char *axis) const
	{
		std::string normalized = mode;
		std::transform(normalized.begin(), normalized.end(), normalized.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });

		if (normalized == "sin" || normalized == "sine") {
			return AxisMotionMode::Sin;
		}
		if (normalized == "step") {
			return AxisMotionMode::Step;
		}
		if (normalized == "hold" || normalized == "none" || normalized == "off") {
			return AxisMotionMode::Hold;
		}

		RCLCPP_WARN(
			this->get_logger(),
			"Unknown trajectory.%s_mode '%s'; using hold",
			axis,
			mode.c_str());
		return AxisMotionMode::Hold;
	}

	AxisMotionConfig read_axis_motion_config()
	{
		return {
			parse_axis_motion_mode(this->get_parameter("trajectory.x_mode").as_string(), "x"),
			parse_axis_motion_mode(this->get_parameter("trajectory.y_mode").as_string(), "y"),
			parse_axis_motion_mode(this->get_parameter("trajectory.z_mode").as_string(), "z")
		};
	}

	const char *axis_motion_mode_name(AxisMotionMode mode) const
	{
		switch (mode) {
		case AxisMotionMode::Hold:
			return "hold";
		case AxisMotionMode::Step:
			return "step";
		case AxisMotionMode::Sin:
			return "sin";
		}
		return "unknown";
	}

	void open_log_file()
	{
		log_file_.open(log_path_, std::ios::out | std::ios::trunc);
		if (!log_file_.is_open()) {
			RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_path_.c_str());
			return;
		}

		log_file_
			<< "control_time_s,"
			<< "local_pos_timestamp,local_pos_timestamp_sample,"
			<< "pos_x,pos_y,pos_z,"
			<< "vel_x,vel_y,vel_z,"
			<< "local_pos_z_deriv,"
			<< "attitude_timestamp,attitude_timestamp_sample,"
			<< "roll,pitch,yaw,"
			<< "angular_velocity_timestamp,angular_velocity_timestamp_sample,"
			<< "angular_vel_x,angular_vel_y,angular_vel_z,"
			<< "local_sp_timestamp,"
			<< "local_sp_x,local_sp_y,local_sp_z,local_sp_yaw,"
			<< "local_sp_vx,local_sp_vy,local_sp_vz,"
			<< "local_sp_ax,local_sp_ay,local_sp_az,local_sp_yawspeed,"
			<< "att_sp_timestamp,"
			<< "att_sp_qw,att_sp_qx,att_sp_qy,att_sp_qz,"
			<< "rate_sp_timestamp,"
			<< "rate_sp_roll,rate_sp_pitch,rate_sp_yaw,"
			<< "rate_sp_thrust_x,rate_sp_thrust_y,rate_sp_thrust_z,"
			<< "target_x,target_y,target_z,target_yaw,"
			<< "phase,event,"
			<< "hover_thrust_timestamp,"
			<< "hover_thrust,hover_thrust_valid,"
			<< "takeoff_status_timestamp,"
			<< "takeoff_state,takeoff_tilt_limit,"
			<< "land_detected_timestamp,"
			<< "landed,ground_contact,maybe_landed,"
			<< "thrust_sp_timestamp,"
			<< "thrust_sp_x,thrust_sp_y,thrust_sp_z,"
			<< "torque_sp_timestamp,"
			<< "torque_sp_x,torque_sp_y,torque_sp_z";
		log_file_
			<< ",actuator_timestamp"
			<< ",actuator_timestamp_sample";
		for (int i = 0; i < ActuatorMotors::NUM_CONTROLS; ++i) {
			log_file_ << ",actuator_motor_" << i;
		}
		log_file_
			<< ",rate_ctrl_status_timestamp"
			<< ",rollspeed_integ"
			<< ",pitchspeed_integ"
			<< ",yawspeed_integ"
			<< ",livox_imu_time_s"
			<< ",livox_imu_stamp_sec"
			<< ",livox_imu_stamp_nanosec"
			<< ",livox_imu_orientation_x"
			<< ",livox_imu_orientation_y"
			<< ",livox_imu_orientation_z"
			<< ",livox_imu_orientation_w"
			<< ",livox_imu_angular_velocity_x"
			<< ",livox_imu_angular_velocity_y"
			<< ",livox_imu_angular_velocity_z"
			<< ",livox_imu_linear_acceleration_x"
			<< ",livox_imu_linear_acceleration_y"
			<< ",livox_imu_linear_acceleration_z\n";
		log_file_.flush();
		RCLCPP_INFO(this->get_logger(), "Logging to: %s", log_path_.c_str());
	}

	void mark_log_event(const std::string &event)
	{
		std::lock_guard<std::mutex> lock(log_event_mutex_);
		if (pending_log_event_.empty()) {
			pending_log_event_ = event;
		} else {
			pending_log_event_ += "|";
			pending_log_event_ += event;
		}
	}

	std::string consume_log_event()
	{
		std::lock_guard<std::mutex> lock(log_event_mutex_);
		std::string event = pending_log_event_;
		pending_log_event_.clear();
		return event;
	}

	void set_takeoff_target()
	{
		stop_auto_trajectory();
		control_start_time_ = this->get_clock()->now();
		if (got_local_pos_) {
			target_x_ = current_x_;
			target_y_ = current_y_;
			target_yaw_ = current_yaw_;
		} else {
			target_x_ = 0.0f;
			target_y_ = 0.0f;
			target_yaw_ = 0.0f;
		}
		target_z_ = fixed_altitude_;
	}

	void start_square_trajectory()
	{
		const float square_yaw = got_local_pos_ ? current_yaw_ : target_yaw_;

		x_sine_only_ = false;
		waypoints_.clear();
		waypoints_.push_back({0.0f, 0.0f, fixed_altitude_, square_yaw});
		append_square_laps(1.0f, 2, square_yaw);
		append_square_laps(2.0f, 2, square_yaw);
		waypoints_.push_back({0.0f, 0.0f, fixed_altitude_, square_yaw});

		waypoint_index_ = 0;
		trajectory_phase_ = TrajectoryPhase::Waypoint;
		phase_start_time_ = this->get_clock()->now();
		control_start_time_ = phase_start_time_;
		set_target_from_waypoint(waypoints_[waypoint_index_]);
		mark_log_event("square_start");
		RCLCPP_INFO(this->get_logger(), "Square trajectory started");
	}

	void start_x_sine_input()
	{
		start_axis_input(AxisMotionMode::Sin, AxisMotionMode::Hold, AxisMotionMode::Hold);
	}

	void start_step_input()
	{
		start_axis_input(AxisMotionMode::Step, AxisMotionMode::Hold, AxisMotionMode::Hold);
	}

	void start_configured_axis_input()
	{
		const AxisMotionConfig config = read_axis_motion_config();
		start_axis_input(config.x, config.y, config.z);
	}

	void start_axis_input(AxisMotionMode x_mode, AxisMotionMode y_mode, AxisMotionMode z_mode)
	{
		axis_motion_config_ = {x_mode, y_mode, z_mode};
		trajectory_phase_ = TrajectoryPhase::ConfiguredInput;
		x_sine_only_ = false;
		phase_start_time_ = this->get_clock()->now();
		control_start_time_ = phase_start_time_;
		target_x_ = 0.0f;
		target_y_ = 0.0f;
		target_z_ = fixed_altitude_;
		target_yaw_ = got_local_pos_ ? current_yaw_ : target_yaw_;
		setpoint_vx_ = 0.0f;
		setpoint_vy_ = 0.0f;
		setpoint_vz_ = 0.0f;
		setpoint_yawspeed_ = 0.0f;
		mark_log_event("axis_input_start");
		RCLCPP_INFO(
			this->get_logger(),
			"Axis input started: x=%s, y=%s, z=%s",
			axis_motion_mode_name(axis_motion_config_.x),
			axis_motion_mode_name(axis_motion_config_.y),
			axis_motion_mode_name(axis_motion_config_.z));
	}

	void append_square_laps(float half_extent, int laps, float yaw)
	{
		for (int lap = 0; lap < laps; ++lap) {
			waypoints_.push_back({half_extent, 0.0f, fixed_altitude_, yaw});
			waypoints_.push_back({half_extent, half_extent, fixed_altitude_, yaw});
			waypoints_.push_back({-half_extent, half_extent, fixed_altitude_, yaw});
			waypoints_.push_back({-half_extent, -half_extent, fixed_altitude_, yaw});
			waypoints_.push_back({half_extent, -half_extent, fixed_altitude_, yaw});
			waypoints_.push_back({half_extent, 0.0f, fixed_altitude_, yaw});
		}
	}

	void set_target_from_waypoint(const Waypoint &wp)
	{
		target_x_ = wp.x;
		target_y_ = wp.y;
		target_z_ = wp.z;
		target_yaw_ = wp.yaw;
		setpoint_vx_ = 0.0f;
		setpoint_vy_ = 0.0f;
		setpoint_vz_ = 0.0f;
		setpoint_yawspeed_ = 0.0f;
	}

	void enter_phase(TrajectoryPhase phase)
	{
		trajectory_phase_ = phase;
		phase_start_time_ = this->get_clock()->now();
		setpoint_vx_ = 0.0f;
		setpoint_vy_ = 0.0f;
		setpoint_vz_ = 0.0f;
		setpoint_yawspeed_ = 0.0f;
	}

	void stop_auto_trajectory()
	{
		trajectory_phase_ = TrajectoryPhase::Idle;
		x_sine_only_ = false;
		setpoint_vx_ = 0.0f;
		setpoint_vy_ = 0.0f;
		setpoint_vz_ = 0.0f;
		setpoint_yawspeed_ = 0.0f;
	}

	void update_auto_trajectory()
	{
		if (trajectory_phase_ == TrajectoryPhase::Idle || trajectory_phase_ == TrajectoryPhase::Done) {
			return;
		}

		if (trajectory_phase_ == TrajectoryPhase::Waypoint) {
			if (waypoint_index_ < waypoints_.size()) {
				set_target_from_waypoint(waypoints_[waypoint_index_]);
				if (got_local_pos_ &&
					distance_to_target(waypoints_[waypoint_index_]) < waypoint_acceptance_radius_ &&
					phase_elapsed_sec() > waypoint_min_hold_sec_) {
					waypoint_index_++;
					phase_start_time_ = this->get_clock()->now();
					if (waypoint_index_ >= waypoints_.size()) {
						enter_phase(TrajectoryPhase::SinX);
						RCLCPP_INFO(this->get_logger(), "Square trajectory completed -> SinX");
					}
				}
			}
			return;
		}

		if (trajectory_phase_ == TrajectoryPhase::StepHold) {
			target_x_ = 0.0f;
			target_y_ = 0.0f;
			target_z_ = fixed_altitude_;
			if (phase_elapsed_sec() >= step_hold_duration_sec_) {
				enter_phase(TrajectoryPhase::StepApplied);
				target_x_ = 1.0f;
				RCLCPP_INFO(this->get_logger(), "Step applied: target changed to (1, 0, -1)");
			}
			return;
		}

		if (trajectory_phase_ == TrajectoryPhase::StepApplied) {
			target_x_ = 1.0f;
			target_y_ = 0.0f;
			target_z_ = fixed_altitude_;
			return;
		}

		if (trajectory_phase_ == TrajectoryPhase::ConfiguredInput) {
			update_configured_axis_input();
			return;
		}

		const double t = phase_elapsed_sec();
		const double omega = 2.0 * pi_ / sin_period_sec_;
		const float s = std::sin(omega * t);
		const float c = std::cos(omega * t);
		const float hold_yaw = target_yaw_;

		target_x_ = 0.0f;
		target_y_ = 0.0f;
		target_z_ = fixed_altitude_;
		target_yaw_ = hold_yaw;
		setpoint_vx_ = 0.0f;
		setpoint_vy_ = 0.0f;
		setpoint_vz_ = 0.0f;
		setpoint_yawspeed_ = 0.0f;

		if (trajectory_phase_ == TrajectoryPhase::SinX) {
			const float amplitude = x_sine_only_ ? x_sin_amplitude_ : sin_xy_amplitude_;
			target_x_ = amplitude * s;
			setpoint_vx_ = 0.0f;
			if (t >= sin_phase_duration_sec_) {
				if (x_sine_only_) {
					enter_phase(TrajectoryPhase::Done);
					x_sine_only_ = false;
					target_x_ = 0.0f;
					target_y_ = 0.0f;
					target_z_ = fixed_altitude_;
					target_yaw_ = hold_yaw;
					mark_log_event("x_sine_done");
					RCLCPP_INFO(this->get_logger(), "X sine input completed");
				} else {
					enter_phase(TrajectoryPhase::SinY);
					RCLCPP_INFO(this->get_logger(), "SinX completed -> SinY");
				}
			}
		} else if (trajectory_phase_ == TrajectoryPhase::SinY) {
			target_y_ = sin_xy_amplitude_ * s;
			setpoint_vy_ = sin_xy_amplitude_ * omega * c;
			if (t >= sin_phase_duration_sec_) {
				enter_phase(TrajectoryPhase::SinZ);
				RCLCPP_INFO(this->get_logger(), "SinY completed -> SinZ");
			}
		} else if (trajectory_phase_ == TrajectoryPhase::SinZ) {
			target_z_ = fixed_altitude_ + sin_z_amplitude_ * s;
			setpoint_vz_ = sin_z_amplitude_ * omega * c;
			if (t >= sin_phase_duration_sec_) {
				enter_phase(TrajectoryPhase::Done);
				target_x_ = 0.0f;
				target_y_ = 0.0f;
				target_z_ = fixed_altitude_;
				target_yaw_ = hold_yaw;
				RCLCPP_INFO(this->get_logger(), "XYZ sine trajectory completed");
			}
		}
	}

	void update_configured_axis_input()
	{
		const double t = phase_elapsed_sec();
		const double omega = 2.0 * pi_ / sin_period_sec_;
		const float s = std::sin(omega * t);
		const float c = std::cos(omega * t);
		const bool step_applied = t >= step_hold_duration_sec_;

		target_x_ = axis_target(axis_motion_config_.x, 0.0f, x_sin_amplitude_, 1.0f, s, step_applied);
		target_y_ = axis_target(axis_motion_config_.y, 0.0f, sin_xy_amplitude_, 1.0f, s, step_applied);
		target_z_ = axis_target(axis_motion_config_.z, fixed_altitude_, sin_z_amplitude_, -0.5f, s, step_applied);
		setpoint_vx_ = axis_velocity(axis_motion_config_.x, x_sin_amplitude_, omega, c);
		setpoint_vy_ = axis_velocity(axis_motion_config_.y, sin_xy_amplitude_, omega, c);
		setpoint_vz_ = axis_velocity(axis_motion_config_.z, sin_z_amplitude_, omega, c);
		setpoint_yawspeed_ = 0.0f;

		if (t >= sin_phase_duration_sec_) {
			enter_phase(TrajectoryPhase::Done);
			target_x_ = 0.0f;
			target_y_ = 0.0f;
			target_z_ = fixed_altitude_;
			setpoint_vx_ = 0.0f;
			setpoint_vy_ = 0.0f;
			setpoint_vz_ = 0.0f;
			mark_log_event("axis_input_done");
			RCLCPP_INFO(this->get_logger(), "Axis input completed");
		}
	}

	float axis_target(
		AxisMotionMode mode,
		float center,
		float sin_amplitude,
		float step_offset,
		float sin_value,
		bool step_applied) const
	{
		if (mode == AxisMotionMode::Sin) {
			return center + sin_amplitude * sin_value;
		}
		if (mode == AxisMotionMode::Step) {
			return center + (step_applied ? step_offset : 0.0f);
		}
		return center;
	}

	float axis_velocity(AxisMotionMode mode, float amplitude, double omega, float cos_value) const
	{
		if (mode == AxisMotionMode::Sin) {
			return amplitude * omega * cos_value;
		}
		return 0.0f;
	}

	const char *phase_name() const
	{
		switch (trajectory_phase_) {
		case TrajectoryPhase::Idle:
			return "Idle";
		case TrajectoryPhase::Waypoint:
			return "Waypoint";
		case TrajectoryPhase::SinX:
			return "SinX";
		case TrajectoryPhase::SinY:
			return "SinY";
		case TrajectoryPhase::SinZ:
			return "SinZ";
		case TrajectoryPhase::StepHold:
			return "StepHold";
		case TrajectoryPhase::StepApplied:
			return "StepApplied";
		case TrajectoryPhase::ConfiguredInput:
			return "ConfiguredInput";
		case TrajectoryPhase::Done:
			return "Done";
		}
		return "Unknown";
	}

	void log_control_state()
	{
		if (!log_file_.is_open()) {
			return;
		}

		std::lock_guard<std::mutex> lock(sp_mutex_);

		log_file_
			<< control_elapsed_sec() << ","
			<< current_local_position_timestamp_ << "," << current_local_position_timestamp_sample_ << ","
			<< current_x_ << "," << current_y_ << "," << current_z_ << ","
			<< current_vx_ << "," << current_vy_ << "," << current_vz_ << ","
			<< current_z_deriv_;

		if (got_attitude_) {
			log_file_ << "," << current_attitude_timestamp_
				<< "," << current_attitude_timestamp_sample_
				<< "," << current_roll_
				<< "," << current_pitch_
				<< "," << current_yaw_;
		} else {
			log_file_ << ",nan,nan,nan,nan,nan";
		}

		if (has_angular_velocity_) {
			log_file_ << "," << latest_angular_velocity_.timestamp
				<< "," << latest_angular_velocity_.timestamp_sample
				<< "," << latest_angular_velocity_.xyz[0]
				<< "," << latest_angular_velocity_.xyz[1]
				<< "," << latest_angular_velocity_.xyz[2];
		} else {
			log_file_ << ",nan,nan,nan,nan,nan";
		}

		if (has_traj_sp_) {
			log_file_ << "," << latest_traj_sp_.timestamp
				<< "," << latest_traj_sp_.x
				<< "," << latest_traj_sp_.y
				<< "," << latest_traj_sp_.z
				<< "," << latest_traj_sp_.yaw
				<< "," << latest_traj_sp_.vx
				<< "," << latest_traj_sp_.vy
				<< "," << latest_traj_sp_.vz
				<< "," << latest_traj_sp_.acceleration[0]
				<< "," << latest_traj_sp_.acceleration[1]
				<< "," << latest_traj_sp_.acceleration[2]
				<< "," << latest_traj_sp_.yawspeed;
		} else {
			log_file_ << ",nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan";
		}

		if (has_att_sp_) {
			log_file_ << "," << latest_att_sp_.timestamp
				<< "," << latest_att_sp_.q_d[0]
				<< "," << latest_att_sp_.q_d[1]
				<< "," << latest_att_sp_.q_d[2]
				<< "," << latest_att_sp_.q_d[3];
		} else {
			log_file_ << ",nan,nan,nan,nan,nan";
		}

		if (has_rate_sp_) {
			log_file_ << "," << latest_rate_sp_.timestamp
				<< "," << latest_rate_sp_.roll
				<< "," << latest_rate_sp_.pitch
				<< "," << latest_rate_sp_.yaw
				<< "," << latest_rate_sp_.thrust_body[0]
				<< "," << latest_rate_sp_.thrust_body[1]
				<< "," << latest_rate_sp_.thrust_body[2];
		} else {
			log_file_ << ",nan,nan,nan,nan,nan,nan,nan";
		}

		const std::string event = consume_log_event();
		log_file_ << ","
			<< target_x_ << "," << target_y_ << "," << target_z_ << "," << target_yaw_ << ","
			<< phase_name() << "," << event;

		if (has_hover_thrust_estimate_) {
			log_file_ << "," << latest_hover_thrust_estimate_.timestamp
				<< "," << latest_hover_thrust_estimate_.hover_thrust
				<< "," << static_cast<int>(latest_hover_thrust_estimate_.valid);
		} else {
			log_file_ << ",nan,nan,nan";
		}

		if (has_takeoff_status_) {
			log_file_ << "," << latest_takeoff_status_.timestamp
				<< "," << static_cast<int>(latest_takeoff_status_.takeoff_state)
				<< "," << latest_takeoff_status_.tilt_limit;
		} else {
			log_file_ << ",nan,nan,nan";
		}

		if (has_vehicle_land_detected_) {
			log_file_ << "," << latest_vehicle_land_detected_.timestamp
				<< "," << static_cast<int>(latest_vehicle_land_detected_.landed)
				<< "," << static_cast<int>(latest_vehicle_land_detected_.ground_contact)
				<< "," << static_cast<int>(latest_vehicle_land_detected_.maybe_landed);
		} else {
			log_file_ << ",nan,nan,nan,nan";
		}

		if (has_thrust_sp_) {
			log_file_ << "," << latest_thrust_sp_.timestamp
				<< "," << latest_thrust_sp_.xyz[0]
				<< "," << latest_thrust_sp_.xyz[1]
				<< "," << latest_thrust_sp_.xyz[2];
		} else {
			log_file_ << ",nan,nan,nan,nan";
		}

		if (has_torque_sp_) {
			log_file_ << "," << latest_torque_sp_.timestamp
				<< "," << latest_torque_sp_.xyz[0]
				<< "," << latest_torque_sp_.xyz[1]
				<< "," << latest_torque_sp_.xyz[2];
		} else {
			log_file_ << ",nan,nan,nan,nan";
		}

		if (has_actuator_motors_) {
			log_file_ << "," << latest_actuator_motors_.timestamp
				<< "," << latest_actuator_motors_.timestamp_sample;
		} else {
			log_file_ << ",nan,nan";
		}

		if (has_actuator_motors_) {
			for (int i = 0; i < ActuatorMotors::NUM_CONTROLS; ++i) {
				log_file_ << "," << latest_actuator_motors_.control[i];
			}
		} else {
			for (int i = 0; i < ActuatorMotors::NUM_CONTROLS; ++i) {
				log_file_ << ",nan";
			}
		}

		if (has_rate_ctrl_status_) {
			log_file_ << "," << latest_rate_ctrl_status_.timestamp
				<< "," << latest_rate_ctrl_status_.rollspeed_integ
				<< "," << latest_rate_ctrl_status_.pitchspeed_integ
				<< "," << latest_rate_ctrl_status_.yawspeed_integ;
		} else {
			log_file_ << ",nan,nan,nan,nan";
		}

		if (has_livox_imu_) {
			const double livox_imu_time_s =
				static_cast<double>(latest_livox_imu_.header.stamp.sec) +
				static_cast<double>(latest_livox_imu_.header.stamp.nanosec) * 1.0e-9;
			log_file_ << "," << livox_imu_time_s
				<< "," << latest_livox_imu_.header.stamp.sec
				<< "," << latest_livox_imu_.header.stamp.nanosec
				<< "," << latest_livox_imu_.orientation.x
				<< "," << latest_livox_imu_.orientation.y
				<< "," << latest_livox_imu_.orientation.z
				<< "," << latest_livox_imu_.orientation.w
				<< "," << latest_livox_imu_.angular_velocity.x
				<< "," << latest_livox_imu_.angular_velocity.y
				<< "," << latest_livox_imu_.angular_velocity.z
				<< "," << latest_livox_imu_.linear_acceleration.x
				<< "," << latest_livox_imu_.linear_acceleration.y
				<< "," << latest_livox_imu_.linear_acceleration.z;
		} else {
			log_file_ << ",nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan";
		}

		log_file_ << "\n";
		log_file_.flush();
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
		msg.velocity = {NAN, NAN, NAN};
		msg.yaw = target_yaw_;
		msg.yawspeed = NAN;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}

	void local_position_setpoint_callback(const VehicleLocalPositionSetpoint::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_traj_sp_ = *msg;
		has_traj_sp_ = true;
	}

	void attitude_setpoint_callback(const VehicleAttitudeSetpoint::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_att_sp_ = *msg;
		has_att_sp_ = true;
	}

	void rates_setpoint_callback(const VehicleRatesSetpoint::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_rate_sp_ = *msg;
		has_rate_sp_ = true;
	}

	void rate_ctrl_status_callback(const RateCtrlStatus::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_rate_ctrl_status_ = *msg;
		has_rate_ctrl_status_ = true;
	}

	void thrust_setpoint_callback(const VehicleThrustSetpoint::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_thrust_sp_ = *msg;
		has_thrust_sp_ = true;
	}

	void torque_setpoint_callback(const VehicleTorqueSetpoint::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_torque_sp_ = *msg;
		has_torque_sp_ = true;
	}

	void actuator_motors_callback(const ActuatorMotors::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_actuator_motors_ = *msg;
		has_actuator_motors_ = true;
	}

	void hover_thrust_estimate_callback(const HoverThrustEstimate::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_hover_thrust_estimate_ = *msg;
		has_hover_thrust_estimate_ = true;
	}

	void takeoff_status_callback(const TakeoffStatus::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_takeoff_status_ = *msg;
		has_takeoff_status_ = true;
	}

	void vehicle_land_detected_callback(const VehicleLandDetected::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_vehicle_land_detected_ = *msg;
		has_vehicle_land_detected_ = true;
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

	void update_attitude_from_quaternion(const std::array<float, 4> &q)
	{
		const double q_w = q[0];
		const double q_x = q[1];
		const double q_y = q[2];
		const double q_z = q[3];
		double sinr_cosp = 2.0 * (q_w * q_x + q_y * q_z);
		double cosr_cosp = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);
		current_roll_ = std::atan2(sinr_cosp, cosr_cosp);
		double sinp = 2.0 * (q_w * q_y - q_z * q_x);
		if (std::abs(sinp) >= 1.0) {
			current_pitch_ = std::copysign(pi_ / 2.0f, sinp);
		} else {
			current_pitch_ = std::asin(sinp);
		}
		double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
		double cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
		current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
	}

	void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg)
	{
		{
			std::lock_guard<std::mutex> lock(sp_mutex_);
			current_local_position_timestamp_ = msg->timestamp;
			current_local_position_timestamp_sample_ = msg->timestamp_sample;
			current_x_ = msg->x;
			current_y_ = msg->y;
			current_z_ = msg->z;
			current_vx_ = msg->vx;
			current_vy_ = msg->vy;
			current_vz_ = msg->vz;
			current_z_deriv_ = msg->z_deriv;
			got_local_pos_ = true;
		}

		if (offboard_enabled_ && is_armed_ && !emergency_stop_) {
			log_control_state();
		}
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
			"X = %0.2F, Y = %0.2F, Z = %0.2F, yaw = %0.2f",
			current_x_, current_y_, current_z_, current_yaw_);
	}

	void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		current_attitude_timestamp_ = msg->timestamp;
		current_attitude_timestamp_sample_ = msg->timestamp_sample;
		update_attitude_from_quaternion(msg->q);
		got_attitude_ = true;
	}

	void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_angular_velocity_ = *msg;
		current_angular_velocity_timestamp_ = msg->timestamp;
		current_angular_velocity_timestamp_sample_ = msg->timestamp_sample;
		current_wx_ = msg->xyz[0];
		current_wy_ = msg->xyz[1];
		current_wz_ = msg->xyz[2];
		has_angular_velocity_ = true;
		got_angular_velocity_ = true;
	}

	void livox_imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(sp_mutex_);
		latest_livox_imu_ = *msg;
		has_livox_imu_ = true;
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

		const bool a_pressed = (a_button == 1 && prev_a_button_ == 0);
		const bool b_pressed = (b_button == 1 && prev_b_button_ == 0);
		const bool x_pressed = (x_button == 1 && prev_x_button_ == 0);
		const bool y_pressed = (y_button == 1 && prev_y_button_ == 0);

		// Aボタン立ち上がり arm + 浮上
		if (a_pressed) {
			if (!is_armed_) {
				emergency_stop_ = false;
				set_takeoff_target();
				arm_request_ = true;
				RCLCPP_INFO(this->get_logger(), "A pressed -> request ARM + takeoff");
			} else {
				// start_square_trajectory();
				// start_step_input();
				start_configured_axis_input();
			}
		}

		// Bボタンで緊急停止（即disarm）
		if (b_pressed) {
			emergency_stop_ = true;
			disarm();
			RCLCPP_WARN(this->get_logger(), "B pressed -> EMERGENCY STOP / DISARM");
		}

		// Xボタンで現在位置でホバリング
		if (x_pressed) {
			if (got_local_pos_) {
				stop_auto_trajectory();
				target_x_ = current_x_;
				target_y_ = current_y_;
				target_z_ = current_z_;
				target_yaw_ = current_yaw_;
				RCLCPP_INFO(this->get_logger(), "X pressed -> hold current position");
			}
		}

		// Yボタンで着陸
		if(y_pressed){
			if(got_local_pos_){
				stop_auto_trajectory();
				target_x_ = current_x_;
				target_y_ = current_y_;
				target_z_ = 0.0;
				target_yaw_ = current_yaw_;				
				mark_log_event("landing_start");
				RCLCPP_INFO(this->get_logger(), "Y pressed -> landing target");
			}
		}

		//LBボタンで前方に移動
		if(lb_button == 1){
			stop_auto_trajectory();
			target_x_ = current_x_ + 3.0f; // 前方に3m移動
			// target_y_ = current_y_;
			// target_z_ = -1.2f;
			// target_yaw_ = current_yaw_;
			RCLCPP_INFO(this->get_logger(), "LB pressed -> move forward");
		}

		//RBボタンで後方に移動
		if(rb_button == 1){
			stop_auto_trajectory();
			target_x_ = current_x_ - 3.0f; // 後方に3m移動
			// target_y_ = current_y_;
			// target_z_ = -1.2f;
			// target_yaw_ = current_yaw_;
			RCLCPP_INFO(this->get_logger(), "RB pressed -> move backward");
		}

		// Startボタンで左に移動
		if(start_button == 1){
			stop_auto_trajectory();
			// target_x_ = current_x_;
			target_y_ = current_y_ - 3.0f; // 左に3m移動
			// target_z_ = -1.2f;
			// target_yaw_ = current_yaw_;
			RCLCPP_INFO(this->get_logger(), "Start pressed -> move left");
		}

		// Backボタンで右に移動
		if(back_button == 1){
			stop_auto_trajectory();
			// target_x_ = current_x_;
			target_y_ = current_y_ + 3.0f; // 右に3m移動
			// target_z_ = -1.2f;
			// target_yaw_ = current_yaw_;
			RCLCPP_INFO(this->get_logger(), "Back pressed -> move right");
		}

		if (power_button == 1) {
			kill();
		}

		prev_a_button_ = a_button;
		prev_b_button_ = b_button;
		prev_x_button_ = x_button;
		prev_y_button_ = y_button;
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
