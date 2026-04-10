#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>
#include <limits>

class FastLioToPx4 : public rclcpp::Node {
public:
    FastLioToPx4() : Node("odom_fast_lio_to_px4") {
        // FAST-LIOからのトピック
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&FastLioToPx4::callback, this, std::placeholders::_1));
        
        // PX4へのトピック
        pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto out_msg = px4_msgs::msg::VehicleOdometry();

        // 1. タイムスタンプの変換 (マイクロ秒)
        out_msg.timestamp = msg->header.stamp.sec * 1000000LL + msg->header.stamp.nanosec / 1000LL;

        // 2. 位置の座標変換 (FLU -> FRD変換)
        out_msg.position[0] = msg->pose.pose.position.x;
        out_msg.position[1] = -msg->pose.pose.position.y;
        out_msg.position[2] = -msg->pose.pose.position.z;

        // 3. 姿勢の座標変換 
        Eigen::Quaterniond q_enu(
            static_cast<double>(msg->pose.pose.orientation.w),
            static_cast<double>(msg->pose.pose.orientation.x),
            static_cast<double>(msg->pose.pose.orientation.y),
            static_cast<double>(msg->pose.pose.orientation.z)
        );
        Eigen::Matrix3d R_enu2ned;
        R_enu2ned << 0, 1, 0,
                    1, 0, 0,
                    0, 0,-1;
        // 回転変換
        Eigen::Matrix3d R_body_enu = q_enu.toRotationMatrix();
        Eigen::Matrix3d R_body_ned = R_enu2ned * R_body_enu * R_enu2ned.transpose();

        Eigen::Quaterniond q_ned(R_body_ned);

        out_msg.q[0] = q_ned.w();
        out_msg.q[1] = q_ned.x();
        out_msg.q[2] = q_ned.y();
        out_msg.q[3] = q_ned.z();

        // 4. 速度・角速度は取得できないため NaN を代入
        out_msg.velocity.fill(std::numeric_limits<float>::quiet_NaN());
        out_msg.angular_velocity.fill(std::numeric_limits<float>::quiet_NaN());

        // 5. 分散（Noise）の設定
        out_msg.position_variance = {0.01f, 0.01f, 0.01f};
        out_msg.orientation_variance = {0.01f, 0.01f, 0.01f};

        // 6. frame の設定
        out_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        out_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

        pub_->publish(out_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastLioToPx4>());
    rclcpp::shutdown();
    return 0;
}