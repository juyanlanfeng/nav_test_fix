#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

class CloudTfPublisher : public rclcpp::Node {
private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // 发布tf,用于将实时点云与pcd点云对齐
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> lidar_tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    // 配置值统一由 launch 加载的 YAML 提供，安装外参的单位及来源见参数文件。
    std::string odom_topic_;
    std::string base_frame_;
    std::string lidar_frame_;
    std::vector<double> lidar_xyz_;
    std::vector<double> lidar_rpy_;

    bool need_static_tf_; // 仿真下不发布静态tf
    bool need_odom_tf_; // 仿真下不发布动态tf
public:
    CloudTfPublisher(const std::string& node_name): Node(node_name) {
        // 声明参数类型并读取 YAML 值；实车部署时在 YAML 中填写标定结果。
        odom_topic_ = this->declare_parameter<std::string>("odom_topic");
        base_frame_ = this->declare_parameter<std::string>("base_frame");
        lidar_frame_ = this->declare_parameter<std::string>("lidar_frame");
        lidar_xyz_ = this->declare_parameter<std::vector<double>>("lidar_xyz");
        lidar_rpy_ = this->declare_parameter<std::vector<double>>("lidar_rpy");
        need_static_tf_ = this->declare_parameter<bool>("need_static_tf");
        need_odom_tf_ = this->declare_parameter<bool>("need_odom_tf");

        lidar_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&CloudTfPublisher::odomCallback, this, std::placeholders::_1)
        );

        // 安装外参固定，只需启动时发布一次；/tf_static 会保留它供后来加入的节点读取。
        // 仿真器自己会发布tf,所以需要区分
        if (need_static_tf_) {
            publishLidarTf();
        }
    }

    void publishLidarTf() {
        if (base_frame_.empty() || lidar_frame_.empty() || base_frame_ == lidar_frame_) {
            throw std::invalid_argument("雷达 TF 的父子坐标系必须非空且不同!");
        }
        if (lidar_xyz_.size() != 3 || lidar_rpy_.size() != 3) {
            throw std::invalid_argument("lidar_xyz 和 lidar_rpy 必须各包含 3 个数值!");
        }

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = base_frame_;
        transform.child_frame_id = lidar_frame_;
        transform.transform.translation.x = lidar_xyz_[0];
        transform.transform.translation.y = lidar_xyz_[1];
        transform.transform.translation.z = lidar_xyz_[2];

        // 将安装角度转换为四元数；默认无旋转，即两个坐标系的轴方向相同。
        tf2::Quaternion rotation;
        rotation.setRPY(lidar_rpy_[0], lidar_rpy_[1], lidar_rpy_[2]);
        transform.transform.rotation.x = rotation.x();
        transform.transform.rotation.y = rotation.y();
        transform.transform.rotation.z = rotation.z();
        transform.transform.rotation.w = rotation.w();

        lidar_tf_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "发布静态 TF: %s -> %s，XYZ=[%.3f, %.3f, %.3f] m",
            base_frame_.c_str(), lidar_frame_.c_str(), lidar_xyz_[0], lidar_xyz_[1], lidar_xyz_[2]);
    }

    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

        if (!need_odom_tf_) {
            return;
        }

        if (msg->header.frame_id.empty() || msg->child_frame_id.empty()) {
            RCLCPP_WARN(this->get_logger(), "父子坐标系必须非空!");
            return;
        }

        const auto& position = msg->pose.pose.position;
        const auto& orientation = msg->pose.pose.orientation;
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
            !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
            !std::isfinite(orientation.w)) {
            RCLCPP_WARN(this->get_logger(), "跳过里程计,里程计存在非有限数值!");
            return;
        }

        // TF 要求单位四元数；轻微数值误差可以归一化，全零四元数没有有效朝向。
        // norm:四元数长度
        const double norm = std::hypot(
            std::hypot(orientation.x, orientation.y),
            std::hypot(orientation.z, orientation.w)
        );
        if (!std::isfinite(norm) || norm < 1e-12) {
            RCLCPP_WARN(this->get_logger(), "跳过里程计,四元数无效!");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        transform.header = msg->header;
        transform.child_frame_id = msg->child_frame_id;

        // pose.position 对应子坐标系原点在父坐标系中的位置 [m]。
        transform.transform.translation.x = position.x;
        transform.transform.translation.y = position.y;
        transform.transform.translation.z = position.z;

        // 完整保留三维朝向，包含坡道行驶时的 roll、pitch，不只提取 yaw。
        transform.transform.rotation.x = orientation.x / norm;
        transform.transform.rotation.y = orientation.y / norm;
        transform.transform.rotation.z = orientation.z / norm;
        transform.transform.rotation.w = orientation.w / norm;

        // 例如收到 odom -> base_link 的里程计，就广播同一关系的动态 TF。
        // 车体 -> 雷达外参由 publishLidarTf() 发布；map -> odom 对齐仍由定位模块提供。
        tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudTfPublisher>("cloud_tf_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
