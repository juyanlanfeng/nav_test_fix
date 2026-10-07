#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/segment_differences.h>

class ObstacleSubNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_obstacles_publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 配置值统一由 launch 加载的 YAML 提供，CPP 中只保留成员及参数类型。
    double distance_threshold_;
    std::string static_pcd_file_;
    std::string input_topic_;
    std::string output_topic_;
    std::string base_frame_;
    std::string lidar_frame_;
    std::string odom_frame_;

    // 全局静态点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud_;
    // 动态障碍物点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
    // 转换后的PCL类型点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_odom_pcl_;
    // 转换后的pointcloud2类型点云
    sensor_msgs::msg::PointCloud2 current_cloud_odom_;

    geometry_msgs::msg::TransformStamped odom_to_footprint_;
    geometry_msgs::msg::TransformStamped footprint_to_lidar_;
    geometry_msgs::msg::TransformStamped lidar_to_odom_;
    geometry_msgs::msg::TransformStamped odom_to_lidar_;

public:
    ObstacleSubNode(const std::string& node_name): Node(node_name) {
      
        // 声明参数类型并读取 YAML 值；漏配参数时直接报告错误，不使用隐藏默认值。
        distance_threshold_ = this->declare_parameter<double>("distance_threshold");
        static_pcd_file_ = this->declare_parameter<std::string>("pcd_file_path");
        input_topic_ = this->declare_parameter<std::string>("input_cloud_topic");
        output_topic_ = this->declare_parameter<std::string>("output_cloud_topic");
        base_frame_ = this->declare_parameter<std::string>("base_frame");
        lidar_frame_ = this->declare_parameter<std::string>("lidar_frame");
        odom_frame_ = this->declare_parameter<std::string>("odom_frame");

        this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        // 加载静态点云
        static_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(static_pcd_file_, *static_cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "无法读取文件 %s", static_pcd_file_.c_str());
        }

        // 当前仿真 PCD 的坐标与 odom 重合；这里只标记坐标系，不再次平移地图。
        static_cloud_->header.frame_id = odom_frame_;
        RCLCPP_INFO(this->get_logger(), "Loaded background point cloud with %zu points.", static_cast<size_t>(static_cloud_->size()));

        // 初始化动态障碍物点云
        obstacle_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        current_cloud_odom_pcl_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        // 创建订阅器和发布器
        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&ObstacleSubNode::inputCloudCallback, this, std::placeholders::_1)
        );

        dynamic_obstacles_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    }

    // 处理输入的点云
    void inputCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN(this->get_logger(), "跳过点云: header.frame_id 为空!");
            return;
        }

        try {
            // 目标系在前、源系在后；使用点云消息中的采样时间查询。
            odom_to_footprint_ = tf_buffer_->lookupTransform(
                base_frame_, odom_frame_, rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(0.05)
              );

            footprint_to_lidar_ = tf_buffer_->lookupTransform(
                lidar_frame_, base_frame_, rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(0.05)
              );
        }
        catch(const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "未查找到TF信息");
            return;
        }

        // 计算odom->lidar
        tf2::Transform T_odom_footprint;
        tf2::Transform T_footprint_lidar;

        tf2::fromMsg(odom_to_footprint_.transform, T_odom_footprint);
        tf2::fromMsg(footprint_to_lidar_.transform, T_footprint_lidar);
        tf2::Transform T_odom_lidar = (T_footprint_lidar * T_odom_footprint).inverse();

        // --- gazebo发布的tf与点云中心存在偏差,这里直接强制纠正,后面需要去除 ---
        // Gazebo 测量原点相对雷达 link，沿局部 Z 轴偏移 +0.03 m。
        tf2::Transform T_lidar_sensor;
        T_lidar_sensor.setIdentity();
        T_lidar_sensor.setOrigin(tf2::Vector3(0.0, 0.0, 0.03));
        // 将测量原点的偏移合入点云到 odom 的变换。
        T_odom_lidar = T_odom_lidar * T_lidar_sensor;
        // ----------------------------------------------------------------------

        lidar_to_odom_.header.stamp = msg->header.stamp;
        lidar_to_odom_.header.frame_id = odom_frame_;
        lidar_to_odom_.child_frame_id = lidar_frame_;
        lidar_to_odom_.transform = tf2::toMsg(T_odom_lidar);

        tf2::doTransform(*msg, current_cloud_odom_, lidar_to_odom_);

        // 将当前单帧点云转为 PCL；此时点坐标仍在消息标记的源坐标系中。
        pcl::fromROSMsg(current_cloud_odom_, *current_cloud_odom_pcl_);

        pcl::SegmentDifferences<pcl::PointXYZ> segment;
        segment.setInputCloud(current_cloud_odom_pcl_);  // 已变换到 odom 的实时扫描
        segment.setTargetCloud(static_cloud_); // 同一坐标系下的固定背景
        // 此接口接收距离阈值平方，输出扫描中与背景不匹配的点。
        segment.setDistanceThreshold(distance_threshold_ * distance_threshold_);
        segment.segment(*obstacle_cloud_);

        sensor_msgs::msg::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_cloud_, obstacle_msg);

        // ----------------------- 变回车体系 -----------------------------------
        // mesh的obstacle_layer中的max_obstacle_dist是以obstacle点云中心来计算的,所以obstacle点云必须变换回车体系
        odom_to_lidar_.header.stamp = msg->header.stamp;
        odom_to_lidar_.header.frame_id = lidar_frame_;
        odom_to_lidar_.child_frame_id = odom_frame_;
        odom_to_lidar_.transform = tf2::toMsg(T_footprint_lidar * T_odom_footprint);

        tf2::doTransform(obstacle_msg, obstacle_msg, odom_to_lidar_);
        // ----------------------------------------------------------------------

        obstacle_msg.header = msg->header; // 保留原单帧点云的采样时间
        obstacle_msg.header.frame_id = lidar_frame_;
        dynamic_obstacles_publisher_->publish(obstacle_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleSubNode>("obstacle_sub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
