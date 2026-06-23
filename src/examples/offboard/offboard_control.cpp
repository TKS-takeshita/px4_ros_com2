#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/takeoff_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <stdint.h>

#include <array>
#include <chrono>
#include <iostream>
#include <atomic>
#include <algorithm>
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

		rates_setpoint_subscriber_ = this->create_subscription<VehicleRatesSetpoint>(
			"/fmu/out/vehicle_rates_setpoint", qos,
			std::bind(&OffboardControl::rates_setpoint_callback, this, std::placeholders::_1));

		thrust_setpoint_subscriber_ = this->create_subscription<VehicleThrustSetpoint>(
			"/fmu/out/vehicle_thrust_setpoint", qos,
			std::bind(&OffboardControl::thrust_setpoint_callback, this, std::placeholders::_1));

		torque_setpoint_subscriber_ = this->create_subscription<VehicleTorqueSetpoint>(
			"/fmu/out/vehicle_torque_setpoint", qos,
			std::bind(&OffboardControl::torque_setpoint_callback, this, std::placeholders::_1));

		actuator_motors_subscriber_ = this->create_subscription<ActuatorMotors>(
			"/fmu/out/actuator_motors", qos,
			std::bind(&OffboardControl::actuator_motors_callback, this, std::placeholders::_1));

		local_pos_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"/fmu/out/vehicle_odometry", qos,
			std::bind(&OffboardControl::local_position_callback, this, std::placeholders::_1));

		angular_velocity_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
			"/fmu/out/vehicle_angular_velocity", qos,
			std::bind(&OffboardControl::angular_velocity_callback, this, std::placeholders::_1));

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
	rclcpp::Subscription<VehicleThrustSetpoint>::SharedPtr thrust_setpoint_subscriber_;
	rclcpp::Subscription<VehicleTorqueSetpoint>::SharedPtr torque_setpoint_subscriber_;
	rclcpp::Subscription<ActuatorMotors>::SharedPtr actuator_motors_subscriber_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr local_pos_subscriber_;
	rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr angular_velocity_subscriber_;
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
	float current_roll_{0.0f};
	float current_pitch_{0.0f};
	float current_yaw_{0.0f};
	float current_wx_{0.0f};
	float current_wy_{0.0f};
	float current_wz_{0.0f};

	float setpoint_vx_{0.0f};
	float setpoint_vy_{0.0f};
	float setpoint_vz_{0.0f};
	float setpoint_yawspeed_{0.0f};

	std::mutex sp_mutex_;
	VehicleLocalPositionSetpoint latest_traj_sp_{};
	VehicleAttitudeSetpoint latest_att_sp_{};
	VehicleRatesSetpoint latest_rate_sp_{};
	VehicleThrustSetpoint latest_thrust_sp_{};
	VehicleTorqueSetpoint latest_torque_sp_{};
	ActuatorMotors latest_actuator_motors_{};
	HoverThrustEstimate latest_hover_thrust_estimate_{};
	TakeoffStatus latest_takeoff_status_{};
	VehicleLandDetected latest_vehicle_land_detected_{};
	bool has_traj_sp_{false};
	bool has_att_sp_{false};
	bool has_rate_sp_{false};
	bool has_thrust_sp_{false};
	bool has_torque_sp_{false};
	bool has_actuator_motors_{false};
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
		Done
	};

	struct Waypoint {
		float x;
		float y;
		float z;
		float yaw;
	};

	TrajectoryPhase trajectory_phase_{TrajectoryPhase::Idle};
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

	void open_log_file()
	{
		log_file_.open(log_path_, std::ios::out | std::ios::trunc);
		if (!log_file_.is_open()) {
			RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_path_.c_str());
			return;
		}

		log_file_
			<< "control_time_s,"
			<< "pos_x,pos_y,pos_z,"
			<< "vel_x,vel_y,vel_z,"
			<< "roll,pitch,yaw,"
			<< "angular_vel_x,angular_vel_y,angular_vel_z,"
			<< "local_sp_x,local_sp_y,local_sp_z,local_sp_yaw,"
			<< "local_sp_vx,local_sp_vy,local_sp_vz,"
			<< "local_sp_ax,local_sp_ay,local_sp_az,local_sp_yawspeed,"
			<< "att_sp_qw,att_sp_qx,att_sp_qy,att_sp_qz,"
			<< "rate_sp_roll,rate_sp_pitch,rate_sp_yaw,"
			<< "rate_sp_thrust_x,rate_sp_thrust_y,rate_sp_thrust_z,"
			<< "target_x,target_y,target_z,target_yaw,"
			<< "phase,event,"
			<< "hover_thrust,hover_thrust_valid,"
			<< "takeoff_state,takeoff_tilt_limit,"
			<< "landed,ground_contact,maybe_landed,"
			<< "thrust_sp_x,thrust_sp_y,thrust_sp_z,"
			<< "torque_sp_x,torque_sp_y,torque_sp_z";
		for (int i = 0; i < ActuatorMotors::NUM_CONTROLS; ++i) {
			log_file_ << ",actuator_motor_" << i;
		}
		log_file_ << "\n";
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
		trajectory_phase_ = TrajectoryPhase::SinX;
		x_sine_only_ = true;
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
		mark_log_event("x_sine_start");
		RCLCPP_INFO(
			this->get_logger(),
			"X sine input started: amplitude %.1f m, period %.1f seconds",
			x_sin_amplitude_,
			sin_period_sec_);
	}

	void start_step_input()
	{
		trajectory_phase_ = TrajectoryPhase::StepHold;
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
		mark_log_event("step_start");
		RCLCPP_INFO(
			this->get_logger(),
			"Step input started: hold (0, 0, -1) for %.1f seconds",
			step_hold_duration_sec_);
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
			<< current_x_ << "," << current_y_ << "," << current_z_ << ","
			<< current_vx_ << "," << current_vy_ << "," << current_vz_ << ","
			<< current_roll_ << "," << current_pitch_ << "," << current_yaw_ << ","
			<< current_wx_ << "," << current_wy_ << "," << current_wz_;

		if (has_traj_sp_) {
			log_file_ << "," << latest_traj_sp_.x
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
			log_file_ << ",nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan";
		}

		if (has_att_sp_) {
			log_file_ << "," << latest_att_sp_.q_d[0]
				<< "," << latest_att_sp_.q_d[1]
				<< "," << latest_att_sp_.q_d[2]
				<< "," << latest_att_sp_.q_d[3];
		} else {
			log_file_ << ",nan,nan,nan,nan";
		}

		if (has_rate_sp_) {
			log_file_ << "," << latest_rate_sp_.roll
				<< "," << latest_rate_sp_.pitch
				<< "," << latest_rate_sp_.yaw
				<< "," << latest_rate_sp_.thrust_body[0]
				<< "," << latest_rate_sp_.thrust_body[1]
				<< "," << latest_rate_sp_.thrust_body[2];
		} else {
			log_file_ << ",nan,nan,nan,nan,nan,nan";
		}

		const std::string event = consume_log_event();
		log_file_ << ","
			<< target_x_ << "," << target_y_ << "," << target_z_ << "," << target_yaw_ << ","
			<< phase_name() << "," << event;

		if (has_hover_thrust_estimate_) {
			log_file_ << "," << latest_hover_thrust_estimate_.hover_thrust
				<< "," << static_cast<int>(latest_hover_thrust_estimate_.valid);
		} else {
			log_file_ << ",nan,nan";
		}

		if (has_takeoff_status_) {
			log_file_ << "," << static_cast<int>(latest_takeoff_status_.takeoff_state)
				<< "," << latest_takeoff_status_.tilt_limit;
		} else {
			log_file_ << ",nan,nan";
		}

		if (has_vehicle_land_detected_) {
			log_file_ << "," << static_cast<int>(latest_vehicle_land_detected_.landed)
				<< "," << static_cast<int>(latest_vehicle_land_detected_.ground_contact)
				<< "," << static_cast<int>(latest_vehicle_land_detected_.maybe_landed);
		} else {
			log_file_ << ",nan,nan,nan";
		}

		if (has_thrust_sp_) {
			log_file_ << "," << latest_thrust_sp_.xyz[0]
				<< "," << latest_thrust_sp_.xyz[1]
				<< "," << latest_thrust_sp_.xyz[2];
		} else {
			log_file_ << ",nan,nan,nan";
		}

		if (has_torque_sp_) {
			log_file_ << "," << latest_torque_sp_.xyz[0]
				<< "," << latest_torque_sp_.xyz[1]
				<< "," << latest_torque_sp_.xyz[2];
		} else {
			log_file_ << ",nan,nan,nan";
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
		msg.velocity = {0.0f, 0.0f, 0.0f};
		msg.yaw = target_yaw_;
		msg.yawspeed = 0.0f;
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

	void local_position_callback(const px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg )
	{
		current_x_ = msg->position[0];
		current_y_ = msg->position[1];
		current_z_ = msg->position[2];
		current_vx_ = msg->velocity[0];
		current_vy_ = msg->velocity[1];
		current_vz_ = msg->velocity[2];
		current_wx_ = msg->angular_velocity[0];
		current_wy_ = msg->angular_velocity[1];
		current_wz_ = msg->angular_velocity[2];
		double q_w = msg->q[0];
		double q_x = msg->q[1];
		double q_y = msg->q[2];
		double q_z = msg->q[3];
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
		got_local_pos_ = true;

		if (offboard_enabled_ && is_armed_ && !emergency_stop_) {
			log_control_state();
		}
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
			"X = %0.2F, Y = %0.2F, Z = %0.2F, yaw = %0.2f",
			current_x_, current_y_, current_z_, current_yaw_);
	}

	void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::ConstSharedPtr msg)
	{
		current_wx_ = msg->xyz[0];
		current_wy_ = msg->xyz[1];
		current_wz_ = msg->xyz[2];
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
				start_x_sine_input();
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
