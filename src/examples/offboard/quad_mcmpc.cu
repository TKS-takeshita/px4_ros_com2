#include <iostream>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
// csv保存用
#include <fstream>
#include <filesystem>
#include <sstream>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include "rclcpp/qos.hpp"

#include "px4_ros_com/const_params.hpp"
#include "px4_ros_com/mcmpc_controller.cuh"

using namespace qc_mcmpc;

#if defined(UNPREDICTABLE_COLLISION_WITH_WALL) || defined(PREDICTABLE_COLLISION_WITH_WALL)
#include <Eigen/Dense>
#endif

enum class ControlMode {
    PX4_POSITION,
    MCMPC_ATTITUDE
};

static ControlMode control_mode = ControlMode::PX4_POSITION;

static bool offboard_enabled = false;
static bool arm_request = false;
static bool disarm_request = false;
static bool kill_request = false;

static float move_dist = 1.5f;
static float rotate_angle = M_PI/4.0f;

static uint64_t offboard_setpoint_counter = 0;

// 現在状態
static float current_x = 0.0f;
static float current_y = 0.0f;
// static float current_z = 0.0f;
static float current_yaw = 0.0f;

float target_yaw = 0.0f;

// 飛行ログ
std::ofstream log_csv;

// 目標
qc_mcmpc::target_state_t target{
    CONST_PARAM_FLOAT::INIT_TARGET_E0,
    CONST_PARAM_FLOAT::INIT_TARGET_E1,
    CONST_PARAM_FLOAT::INIT_TARGET_E2,
    CONST_PARAM_FLOAT::INIT_TARGET_E3,
    CONST_PARAM_FLOAT::INIT_TARGET_WX,
    CONST_PARAM_FLOAT::INIT_TARGET_WY,
    CONST_PARAM_FLOAT::INIT_TARGET_WZ,
    CONST_PARAM_FLOAT::INIT_TARGET_X,
    CONST_PARAM_FLOAT::INIT_TARGET_Y,
    CONST_PARAM_FLOAT::INIT_TARGET_Z,
    CONST_PARAM_FLOAT::INIT_TARGET_XP,
    CONST_PARAM_FLOAT::INIT_TARGET_YP,
    CONST_PARAM_FLOAT::INIT_TARGET_ZP
};

float wrap_pi(float yaw)
{
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    int a_button            = msg->buttons.size() > 0   ? msg->buttons[0] : 0;
    static int pre_a        = false;
    int b_button            = msg->buttons.size() > 1   ? msg->buttons[1] : 0;
    static int pre_b        = false;
    int x_button            = msg->buttons.size() > 2   ? msg->buttons[2] : 0;
    static int pre_x        = false;
    int y_button            = msg->buttons.size() > 3   ? msg->buttons[3] : 0;
    static int pre_y        = false;
    int lb_button 			= msg->buttons.size() > 4 	? msg->buttons[4] : 0;
    static int pre_lb       = false;
    int rb_button 			= msg->buttons.size() > 5	? msg->buttons[5] : 0;
    static int pre_rb       = false;
    int back_button 		= msg->buttons.size() > 6	? msg->buttons[6] : 0;
    static int pre_back     = false;
    int start_button 		= msg->buttons.size() > 7 	? msg->buttons[7] : 0;
    static int pre_start    = false;
    int power_button 		= msg->buttons.size() > 8 	? msg->buttons[8] : 0;
    static int pre_power    = false;
    int stick_left_button 	= msg->buttons.size() > 9 	? msg->buttons[9] : 0;
    static int pre_stick_left = false;
    int stick_right_button 	= msg->buttons.size() > 10 	? msg->buttons[10]: 0;
    static int pre_stick_right = false;

    if (a_button & !pre_a) {
        arm_request = true;
        control_mode = ControlMode::PX4_POSITION;
        RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "request ARM +  OFFBOARD");
        target.x = current_x;
        target.y = current_y;
        target.z = -1.2f;
        target_yaw = current_yaw;
        pre_a = true;
        RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "A -> ARM + takeoff");
    }
    else if(!a_button)
        pre_a = false;

    if (x_button & !pre_x) {
        control_mode = ControlMode::MCMPC_ATTITUDE;
        pre_x = true;
        RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "X -> MCMPC mode");
    }
    else if(!x_button)
        pre_x = false;

    if (y_button & !pre_y) {
        control_mode = ControlMode::PX4_POSITION;
        target.x = current_x;
        target.y = current_y;
        target.z = 0.0f;
        target_yaw = current_yaw;
        pre_y = true;
        RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "Y -> LAND");
    }
    else if(!y_button){
        pre_y = false;
    }

    if (b_button & !pre_b) {
        disarm_request = true;
        pre_b = true;
        RCLCPP_WARN(rclcpp::get_logger("mcmpc"), "B -> DISARM");
    }
    else if(!b_button){
        pre_b = false;
    }

    if (power_button){
        kill_request = true;
    }

    if (lb_button || rb_button || back_button || start_button || stick_left_button || stick_right_button){
        if (lb_button & !pre_lb){
            target.x += move_dist;
            pre_lb = true;
            RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "LB pressed -> move forward");
        }
        else if(!lb_button)
            pre_lb = false;
        if(rb_button & !pre_rb){
            target.x -= move_dist;
            pre_rb = true;
            RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "RB pressed -> move backward");
        }
        else if(!rb_button)
            pre_rb = false;
        if(back_button & !pre_back){
            target.y += move_dist;//move to right
            pre_back = true;
            RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "Back pressed -> move right");
        }
        else if(!back_button)
            pre_back = false;
        if(start_button & !pre_start){
            target.y -= move_dist;//move to left
            pre_start = true;
            RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "Start pressed -> move left");
        }
        else if(!start_button)
            pre_start = false;
        if(stick_left_button & !pre_stick_left){
            target_yaw = wrap_pi(current_yaw + rotate_angle); // 時計回りに回転
            float half_yaw = 0.5f * target_yaw;
            target.e0 = std::cos(half_yaw);  // w
            target.e1 = 0.0f;                // x
            target.e2 = 0.0f;                // y
            target.e3 = std::sin(half_yaw);  // z
            pre_stick_left = true;
            RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "Left stick button pressed -> rotate CW");
        }
        else if(stick_left_button)
            pre_stick_left = false;
        if(stick_right_button & !pre_stick_right){
            target_yaw = wrap_pi(current_yaw - rotate_angle); // 反時計回りに回転
            float half_yaw = 0.5f * target_yaw;
            target.e0 = std::cos(half_yaw);  // w
            target.e1 = 0.0f;                // x
            target.e2 = 0.0f;                // y
            target.e3 = std::sin(half_yaw);  // z
            pre_stick_right = true;
            RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "right stick button pressed -> rotate CCW");
        }
        else if(!stick_right_button){
            pre_stick_right = false;
        }
    }
}


void publish_vehicle_command(
    const rclcpp::Node::SharedPtr &node,
    const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr &pub,
    uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = node->get_clock()->now().nanoseconds() / 1000;
    pub->publish(msg);
}

// 実測の変わりにシミュレーションを使うための1stepシミュレーション関数
void verification_simulation_one_step(float var_and_z_i_device[], float rps_cw1, float rps_cw2, float rps_ccw1, float rps_ccw2);

namespace quad_sim_base
{
    void do_simulation(float var_array_to_integrate[]);
    void output_screen(); //画面出力用関数

    // 最新のオドメトリデータ
    static px4_msgs::msg::VehicleOdometry::SharedPtr latest_odom;
    static std::mutex odom_mutex;

    float var_array_to_integrate[_N_OF_ODES];//現在の状態
    double input1, input2, input3, input4;

    float cost = 0.0f;

    // シミュレーション全体の評価用
    float sum_cost = 0.0f; //コストの全時刻分総和(積分値の積分なので物理的意味が見出しづらく、非推奨)

    // 現在状態からMCMPCを回し、そのときの最適入力を得る
    void do_simulation(float var_array_to_integrate[]){
        float var_and_z_i_temp_float[_N_OF_ODES + 1];

        // time measurement
        struct timespec start_time;
        struct timespec finish_time;
        std::vector<long> process_time_buff;//時間測定しているが毎回消えるので後で修正

        for(int i=0; i < _N_OF_ODES; i++)
            var_and_z_i_temp_float[i] = var_array_to_integrate[i];
        var_and_z_i_temp_float[_N_OF_ODES] = 0.0f;

        // コンストラクタを実行するため、get_instance()
        qc_mcmpc::mcmpc_controller::get_instance();//mcmpc_controllerのインスタンス生成

        // time measurement
        clock_gettime(CLOCK_REALTIME, &start_time);
        cost = (mcmpc_controller::get_instance()).calc_optimal_input(var_and_z_i_temp_float, input1, input2, input3, input4);
        sum_cost += cost;

        // time measurement
        clock_gettime(CLOCK_REALTIME, &finish_time);

        int sec = (int)(finish_time.tv_sec - start_time.tv_sec);
        long ns = (long)(finish_time.tv_nsec - start_time.tv_nsec);

        process_time_buff.push_back(((long)(sec * 1e9)) + ns);
    }

    void output_screen(){
        static int cnt = 0;
        float e0 = quad_sim_base::var_array_to_integrate[0]; // scalar (w)
        float e1 = quad_sim_base::var_array_to_integrate[1]; // x
        float e2 = quad_sim_base::var_array_to_integrate[2]; // y
        float e3 = quad_sim_base::var_array_to_integrate[3]; // z
        float roll  = atan2(2.0 * (e0 * e1 + e2 * e3), 1.0 - 2.0 * (e1 * e1 + e2 * e2));
        float sin_pitch = 2.0 * (e0 * e2 - e3 * e1);
        float pitch = asin(fmaxf(-1.0, fminf(1.0, sin_pitch))); // 範囲外エラー防止
        float yaw   = atan2(2.0 * (e0 * e3 + e1 * e2), 1.0 - 2.0 * (e2 * e2 + e3 * e3));
        float roll_deg  = roll  * (180.0 / M_PI);
        float pitch_deg = pitch * (180.0 / M_PI);
        float yaw_deg   = yaw   * (180.0 / M_PI);
        // printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
        //        cnt*0.02f,
        //        quad_sim_base::var_array_to_integrate[0],
        //        quad_sim_base::var_array_to_integrate[1],
        //        quad_sim_base::var_array_to_integrate[2],
        //        quad_sim_base::var_array_to_integrate[3],
        //        quad_sim_base::var_array_to_integrate[4] * (180.0 / M_PI),
        //        quad_sim_base::var_array_to_integrate[5] * (180.0 / M_PI),
        //        quad_sim_base::var_array_to_integrate[6] * (180.0 / M_PI),
        //        quad_sim_base::var_array_to_integrate[7],
        //        quad_sim_base::var_array_to_integrate[8],
        //        quad_sim_base::var_array_to_integrate[9],
        //        quad_sim_base::var_array_to_integrate[10],
        //        quad_sim_base::var_array_to_integrate[11],
        //        quad_sim_base::var_array_to_integrate[12],
        //        roll_deg, pitch_deg, yaw_deg,
        //        quad_sim_base::input1, quad_sim_base::input1/CONST_PARAM::MAX_THRUST,
        //        quad_sim_base::input2, quad_sim_base::input3, quad_sim_base::input4
        //        );
    }
}

void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(quad_sim_base::odom_mutex);
    
    quad_sim_base::latest_odom = msg;

    current_x = msg->position[0];
    current_y = msg->position[1];
    // current_z = msg->position[2];

    double qw = msg->q[0];
    double qx = msg->q[1];
    double qy = msg->q[2];
    double qz = msg->q[3];

    double siny = 2.0 * (qw*qz + qx*qy);
    double cosy = 1.0 - 2.0 * (qy*qy + qz*qz);
    current_yaw = std::atan2(siny, cosy);
}

int main(int argc, char *argv[])
{
    // quad_sim_base::init_values();
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("quadcopter_mcmpc");
    auto qos  = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto attitude_thrust_pub = node->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", qos);
    auto offboard_ctrl_pub = node->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
    auto subscription      = node->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, odometry_callback);
    auto traj_pub = node->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    auto cmd_pub = node->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, joy_callback);

    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;

    log_csv.open("/home/ros2/ws_sensor_combined/src/px4_ros_com/csv/mcmpc_log.csv", std::ios::out);
    log_csv << "time" << ","
            << "e0"  << ","
            << "e1"  << ","
            << "e2"  << ","
            << "e3"  << ","
            << "wx"  << ","
            << "wy"  << ","
            << "wz"  << ","
            << "x"   << ","
            << "y"   << ","
            << "z"   << ","
            << "xp"  << ","
            << "yp"  << ","
            << "zp"  << ","
            << "input1" << ","
            << "input2" << ","
            << "input3" << ","
            << "input4" << "\n";


    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
        qc_mcmpc::update_target_state_device(target);

        rclcpp::spin_some(node);
        std::lock_guard<std::mutex> lock(quad_sim_base::odom_mutex);
        if (!quad_sim_base::latest_odom) {
            rate.sleep();
            continue;
        }

        // 相対座標系に変換
        auto msg = quad_sim_base::latest_odom;
        // 状態更新
        quad_sim_base::var_array_to_integrate[0]  = msg->q[0];//qw
        quad_sim_base::var_array_to_integrate[1]  = msg->q[1];//qx
        quad_sim_base::var_array_to_integrate[2]  = msg->q[2];//qy
        quad_sim_base::var_array_to_integrate[3]  = msg->q[3];//qz
        quad_sim_base::var_array_to_integrate[4]  = msg->angular_velocity[0];//wx
        quad_sim_base::var_array_to_integrate[5]  = msg->angular_velocity[1];//wy
        quad_sim_base::var_array_to_integrate[6]  = msg->angular_velocity[2];//wz
        quad_sim_base::var_array_to_integrate[7]  = msg->position[0];//x
        quad_sim_base::var_array_to_integrate[8]  = msg->position[1];//y
        quad_sim_base::var_array_to_integrate[9]  = msg->position[2];//z
        quad_sim_base::var_array_to_integrate[10] = msg->velocity[0];//vx
        quad_sim_base::var_array_to_integrate[11] = msg->velocity[1];//vy
        quad_sim_base::var_array_to_integrate[12] = msg->velocity[2];//vz

        // MPC計算
        quad_sim_base::do_simulation(quad_sim_base::var_array_to_integrate);
        
        px4_msgs::msg::OffboardControlMode ocm{};
        ocm.timestamp = node->get_clock()->now().nanoseconds() / 1000;
        ocm.position = false;
        ocm.velocity = false;
        ocm.acceleration = false;
        ocm.attitude = false;
        ocm.body_rate = false;
        if (control_mode == ControlMode::PX4_POSITION) {
            ocm.position = true;
        } else {
            ocm.attitude = true;
        }
        offboard_ctrl_pub->publish(ocm);

        if (!offboard_enabled) {
            if (offboard_setpoint_counter < 20)
                offboard_setpoint_counter++;
            if (arm_request && offboard_setpoint_counter >= 10) {
                publish_vehicle_command(node, cmd_pub, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_vehicle_command(node, cmd_pub, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                offboard_enabled = true;
                arm_request = false;
            }
        }
        else if(disarm_request){
            publish_vehicle_command(node, cmd_pub, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
            offboard_enabled = false;
            arm_request = false;
            disarm_request = false;
            RCLCPP_WARN(rclcpp::get_logger("mcmpc"), "Disarm command sent");
        }

        if(kill_request){
            publish_vehicle_command(node, cmd_pub, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f);   // ← 強制停止
            kill_request = false;
            RCLCPP_ERROR(rclcpp::get_logger("mcmpc"), "!!! KILL ACTIVATED !!!");
        }
        // ======================
        // PX4位置制御
        // ======================

        if (control_mode == ControlMode::PX4_POSITION) {
            px4_msgs::msg::TrajectorySetpoint sp{};
            sp.timestamp = node->get_clock()->now().nanoseconds() / 1000;
            sp.position = {target.x, target.y, target.z};
            sp.yaw = target_yaw;
            traj_pub->publish(sp);
        }


        // 姿勢・スラスト指令
        if (control_mode == ControlMode::MCMPC_ATTITUDE) {
            px4_msgs::msg::VehicleAttitudeSetpoint sp_att{};
            sp_att.timestamp = node->get_clock()->now().nanoseconds() / 1000;

            const float ref_th = fmaxf(0.0f, fminf((float)CONST_PARAM::MAX_THRUST, (float)quad_sim_base::input1));
            const float ref_qx = fmaxf(-1.0f, fminf(1.0f, (float)quad_sim_base::input2));
            const float ref_qy = fmaxf(-1.0f, fminf(1.0f, (float)quad_sim_base::input3));
            const float ref_qz = fmaxf(-1.0f, fminf(1.0f, (float)quad_sim_base::input4));

            sp_att.q_d[0] = sqrtf(fmaxf(0.0f, 1.0f - ref_qx*ref_qx - ref_qy*ref_qy - ref_qz*ref_qz));
            sp_att.q_d[1] = ref_qx;
            sp_att.q_d[2] = ref_qy;
            sp_att.q_d[3] = ref_qz;

            // PX4は body z 正方向が下向きなので負で送る
            sp_att.thrust_body[0] = 0.0f;
            sp_att.thrust_body[1] = 0.0f;
            sp_att.thrust_body[2] = -ref_th / (float)CONST_PARAM::MAX_THRUST;

            attitude_thrust_pub->publish(sp_att);
}
        // 標準出力
        // quad_sim_base::output_screen();

        static double log_time = 0.0;

        log_csv << log_time << ","
                << quad_sim_base::var_array_to_integrate[0]  << ","
                << quad_sim_base::var_array_to_integrate[1]  << ","
                << quad_sim_base::var_array_to_integrate[2]  << ","
                << quad_sim_base::var_array_to_integrate[3]  << ","
                << quad_sim_base::var_array_to_integrate[4]  << ","
                << quad_sim_base::var_array_to_integrate[5]  << ","
                << quad_sim_base::var_array_to_integrate[6]  << ","
                << quad_sim_base::var_array_to_integrate[7]  << ","
                << quad_sim_base::var_array_to_integrate[8]  << ","
                << quad_sim_base::var_array_to_integrate[9]  << ","
                << quad_sim_base::var_array_to_integrate[10] << ","
                << quad_sim_base::var_array_to_integrate[11] << ","
                << quad_sim_base::var_array_to_integrate[12] << ","
                << quad_sim_base::input1 << ","
                << quad_sim_base::input2 << ","
                << quad_sim_base::input3 << ","
                << quad_sim_base::input4 << "\n";

        log_time += 0.02;   // 50 Hzなら

        // 実測の代わりにシミュレーションを使うための1stepシミュレーション関数
        // verification_simulation_one_step(quad_sim_base::var_array_to_integrate,
        //         quad_sim_base::input1, quad_sim_base::input2, quad_sim_base::input3, quad_sim_base::input4);

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

// 次の時刻の状態の推定にシミュレーションを利用->橘さんに確認（実測値を使用しないのかどうか）後で修正点
void verification_simulation_one_step(float var_and_z_i_device[], float input1, float input2, float input3, float input4){
    float decoupled_rps[4];
    decoupled_rps[0] = input1;//thrust
    decoupled_rps[1] = input2;//qx
    decoupled_rps[2] = input3;//qy
    decoupled_rps[3] = input4;//qz

    float control_period_device = (float)CONST_PARAM::CONTROL_PERIOD;
    float integration_step_size_device = (float)CONST_PARAM::CONTROL_PERIOD / 2.0f;
    float mass_of_machine_device = (float)CONST_PARAM::MASS_OF_MACHINE;
    float a_of_gravity_device = (float)CONST_PARAM::A_OF_GRAVITY;
    float max_thrust_device = (float)CONST_PARAM::MAX_THRUST;

    // __constant__ メモリから状態量をコピー
    float var_and_z_i_temp[_N_OF_ODES + 1];
    for ( int i = 0; i < _N_OF_ODES + 1; i++ )
        var_and_z_i_temp[i] = var_and_z_i_device[i];
    
    float var_p_temp[_N_OF_ODES];

    // PIDカスケード用修正
    float ref_th = fmaxf(0.0f, fminf(max_thrust_device, decoupled_rps[vz]));
    float ref_qx = fmaxf(-1.0f, fminf(1.0f, decoupled_rps[vwx]));
    float ref_qy = fmaxf(-1.0f, fminf(1.0f, decoupled_rps[vwy]));
    float ref_qz = fmaxf(-1.0f, fminf(1.0f, decoupled_rps[vwz]));
    decoupled_rps[vz]  = ref_th;
    decoupled_rps[vwx] = ref_qx;
    decoupled_rps[vwy] = ref_qy;
    decoupled_rps[vwz] = ref_qz;
    
    for ( float t = 0.0f; t < control_period_device - integration_step_size_device / 2; t += integration_step_size_device ){
        // PIDカスケード用修正
        float inv_mass = 1.0f / mass_of_machine_device;
        for ( int k = 0; k < _N_OF_ODES; k++ ) var_p_temp[k] = var_and_z_i_temp[k];
        /* e0p */ var_and_z_i_temp[0]   = sqrtf(fmaxf(0.0f, 1 - ref_qx*ref_qx - ref_qy*ref_qy - ref_qz*ref_qz));
        /* e1p */ var_and_z_i_temp[1]   = ref_qx;
        /* e2p */ var_and_z_i_temp[2]   = ref_qy;
        /* e3p */ var_and_z_i_temp[3]   = ref_qz;


        // omega = 2/dt * qev 
        float sign_w = copysignf(2.0f / integration_step_size_device, var_p_temp[0]*var_and_z_i_temp[0] + var_p_temp[1]*ref_qx + var_p_temp[2]*ref_qy + var_p_temp[3]*ref_qz);
        /* wxp */ var_and_z_i_temp[4]   = sign_w *                  (-var_p_temp[1]*var_and_z_i_temp[0] + var_p_temp[0]*ref_qx + var_p_temp[3]*ref_qy - var_p_temp[2]*ref_qz);
        /* wyp */ var_and_z_i_temp[5]   = sign_w *                  (-var_p_temp[2]*var_and_z_i_temp[0] - var_p_temp[3]*ref_qx + var_p_temp[0]*ref_qy + var_p_temp[1]*ref_qz); 
        /* wzp */ var_and_z_i_temp[6]   = sign_w *                  (-var_p_temp[3]*var_and_z_i_temp[0] + var_p_temp[2]*ref_qx - var_p_temp[1]*ref_qy + var_p_temp[0]*ref_qz); 

        /* xp  */ var_and_z_i_temp[7]  +=  var_p_temp[10] * integration_step_size_device;
        /* yp  */ var_and_z_i_temp[8]  +=  var_p_temp[11] * integration_step_size_device;
        /* zp  */ var_and_z_i_temp[9]  +=  var_p_temp[12] * integration_step_size_device;

        /* xpp */ var_and_z_i_temp[10] += integration_step_size_device * (-ref_th) * inv_mass * (2.0f*var_p_temp[0]*var_p_temp[2] + 2.0f*var_p_temp[1]*var_p_temp[3]); 
        /* ypp */ var_and_z_i_temp[11] += integration_step_size_device * (-ref_th) * inv_mass * (2.0f*var_p_temp[2]*var_p_temp[3] - 2.0f*var_p_temp[0]*var_p_temp[1]);
        /* zpp */ var_and_z_i_temp[12] += integration_step_size_device *((-ref_th) * inv_mass * (2.0f*var_p_temp[0]*var_p_temp[0] + 2.0f*var_p_temp[3]*var_p_temp[3] - 1.0f) + a_of_gravity_device);

        /* z_i */ var_and_z_i_temp[_N_OF_ODES] += var_and_z_i_temp[9] * integration_step_size_device;
    }

    for ( int i = 0; i < _N_OF_ODES + 1; i++ )
        var_and_z_i_device[i] = var_and_z_i_temp[i];
}
