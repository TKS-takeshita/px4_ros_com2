#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <mutex>
#include <fstream>
#include <iomanip>
#include <sstream>

class RecordOdom : public rclcpp::Node {
public:
    RecordOdom()
        : Node("record_odom")
    {
        current_position_ = Eigen::Vector3d::Zero();
        //Subscription
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                this->odom_callback(msg);
            }
        );
        const auto now_sys   = std::chrono::system_clock::now();
        const std::time_t tt = std::chrono::system_clock::to_time_t(now_sys);
        std::tm tm{};
        localtime_r(&tt, &tm);
        std::ostringstream oss;
        oss << "/home/ros2/ws_sensor_combined/src/px4_ros_com/csv/" << "pixhawk_odom" << ".csv";
        const std::string fname = oss.str();
        csv_.open(fname, std::ios::out);
        if (!csv_) {
            RCLCPP_WARN(this->get_logger(), "Failed to open CSV file: %s", fname.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Logging px4_odom to: %s", fname.c_str());
            // ヘッダ（必要に応じて列を足してOK）
            csv_ << "timestamp_us,"
                << "px,py,pz,"
                << "qw,qx,qy,qz"
                << "\n";
            csv_.flush();
        }

    }
private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    std::ofstream csv_;
    Eigen::Vector3d current_position_;
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr &msg)
    {
        const uint64_t now_us = (msg -> timestamp) / 1000;

        current_position_ << msg->position[0], msg->position[1], msg->position[2];

        csv_ << now_us << ","
             << current_position_.x() << ","
             << current_position_.y() << ","
             << current_position_.z() << ","
             << msg->q[0] << "," << msg->q[1] << "," << msg->q[2] << "," << msg->q[3]
             << "\n";
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RecordOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}