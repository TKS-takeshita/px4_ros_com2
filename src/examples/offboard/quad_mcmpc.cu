#include <iostream>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include "rclcpp/qos.hpp"

#include "px4_ros_com/const_params.hpp"
#include "px4_ros_com/mcmpc_controller.cuh"


#if defined(UNPREDICTABLE_COLLISION_WITH_WALL) || defined(PREDICTABLE_COLLISION_WITH_WALL)
#include <Eigen/Dense>
#endif

enum class ControlMode {
    PX4_POSITION,
    MCMPC_ATTITUDE
};

static ControlMode control_mode = ControlMode::PX4_POSITION;

static bool is_armed = false;
static bool offboard_enabled = false;
static bool arm_request = false;
static bool land_request = false;
static bool disarm_request = false;
static bool got_local_pos = false;

static uint64_t offboard_setpoint_counter = 0;

// 現在状態
static float current_x = 0.0f;
static float current_y = 0.0f;
static float current_z = 0.0f;
static float current_yaw = 0.0f;

// 目標
static float target_x = 0.0f;
static float target_y = 0.0f;
static float target_z = -1.2f;
static float target_yaw = 0.0f;

float wrap_pi(float yaw)
{
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    int a_button = msg->buttons.size() > 0 ? msg->buttons[0] : 0;
    int b_button = msg->buttons.size() > 1 ? msg->buttons[1] : 0;
    int x_button = msg->buttons.size() > 2 ? msg->buttons[2] : 0;
    int y_button = msg->buttons.size() > 3 ? msg->buttons[3] : 0;

    if (a_button) {
        arm_request = true;
        control_mode = ControlMode::PX4_POSITION;
        RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "request ARM +  OFFBOARD");
        target_x = current_x;
        target_y = current_y;
        target_z = -1.2f;
        target_yaw = current_yaw;

        RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "A -> ARM + takeoff");
    }

    if (x_button) {
        control_mode = ControlMode::MCMPC_ATTITUDE;
        RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "X -> MCMPC mode");
    }

    if (y_button) {
        control_mode = ControlMode::PX4_POSITION;
        land_request = true;
        RCLCPP_INFO(rclcpp::get_logger("mcmpc"), "Y -> LAND");
    }

    if (b_button) {
        disarm_request = true;
        RCLCPP_WARN(rclcpp::get_logger("mcmpc"), "B -> DISARM");
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

    // 相対座標系用の初期値
    static bool is_first_odometry = true;
    static double init_x = 0.0;
    static double init_y = 0.0;
    static double init_z = 0.0;

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
        cost = (qc_mcmpc::mcmpc_controller::get_instance()).calc_optimal_input(var_and_z_i_temp_float, input1, input2, input3, input4);
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
        printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
               cnt*0.02f,
               quad_sim_base::var_array_to_integrate[0],
               quad_sim_base::var_array_to_integrate[1],
               quad_sim_base::var_array_to_integrate[2],
               quad_sim_base::var_array_to_integrate[3],
               quad_sim_base::var_array_to_integrate[4] * (180.0 / M_PI),
               quad_sim_base::var_array_to_integrate[5] * (180.0 / M_PI),
               quad_sim_base::var_array_to_integrate[6] * (180.0 / M_PI),
               quad_sim_base::var_array_to_integrate[7],
               quad_sim_base::var_array_to_integrate[8],
               quad_sim_base::var_array_to_integrate[9],
               quad_sim_base::var_array_to_integrate[10],
               quad_sim_base::var_array_to_integrate[11],
               quad_sim_base::var_array_to_integrate[12],
               roll_deg, pitch_deg, yaw_deg,
               quad_sim_base::input1, quad_sim_base::input1/CONST_PARAM::MAX_THRUST,
               quad_sim_base::input2, quad_sim_base::input3, quad_sim_base::input4
               );
        cnt++;
    }
}

void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(quad_sim_base::odom_mutex);
    
    // 初回のみ初期値を保存
    if (quad_sim_base::is_first_odometry) {
        quad_sim_base::init_x = msg->position[0];
        quad_sim_base::init_y = msg->position[1];
        quad_sim_base::init_z = msg->position[2];
        quad_sim_base::is_first_odometry = false;
    }
    quad_sim_base::latest_odom = msg;

    current_x = msg->position[0];
    current_y = msg->position[1];
    current_z = msg->position[2];

    double qw = msg->q[0];
    double qx = msg->q[1];
    double qy = msg->q[2];
    double qz = msg->q[3];

    double siny = 2.0 * (qw*qz + qx*qy);
    double cosy = 1.0 - 2.0 * (qy*qy + qz*qz);
    current_yaw = std::atan2(siny, cosy);

    got_local_pos = true;
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

    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {

        rclcpp::spin_some(node);
        std::lock_guard<std::mutex> lock(quad_sim_base::odom_mutex);
        if (!quad_sim_base::latest_odom) {
            rate.sleep();
            continue;
        }
        // 相対座標系に変換
        auto msg = quad_sim_base::latest_odom;
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
                is_armed = true;
                arm_request = false;
            }
        }
        else if(disarm_request){
            publish_vehicle_command(node, cmd_pub, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
            is_armed = false;
            offboard_enabled = false;
            arm_request = false;
            disarm_request = false;
            RCLCPP_WARN(rclcpp::get_logger("mcmpc"), "Disarm command sent");
        }
        // ======================
        // PX4位置制御
        // ======================

        if (control_mode == ControlMode::PX4_POSITION) {
            px4_msgs::msg::TrajectorySetpoint sp{};
            sp.timestamp = node->get_clock()->now().nanoseconds() / 1000;
            sp.position = {target_x, target_y, target_z};
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
        quad_sim_base::output_screen();

        // 実測の代わりにシミュレーションを使うための1stepシミュレーション関数
        verification_simulation_one_step(quad_sim_base::var_array_to_integrate,
                quad_sim_base::input1, quad_sim_base::input2, quad_sim_base::input3, quad_sim_base::input4);

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
