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
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Geometry>

class ObstacleSubNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_obstacles_publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 配置值统一由 launch 加载的 YAML 提供，CPP 中只保留成员及参数类型。
    double distance_threshold_;
    double robot_radius_;
    double horizon_;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_pcl_;
    // 转换后的pointcloud2类型点云
    sensor_msgs::msg::PointCloud2 current_cloud_odom_;

    geometry_msgs::msg::TransformStamped odom_to_lidar_;
    geometry_msgs::msg::TransformStamped lidar_to_footprint_;

    pcl::search::Search<pcl::PointXYZ>::Ptr background_tree_;

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
        robot_radius_ = this->declare_parameter<double>("robot_radius");
        horizon_ = this->declare_parameter<double>("horizon");

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
        current_cloud_pcl_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        // 只排队最新扫描；保留采样时间，避免积压点云被下游新鲜度检查拒收。
        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&ObstacleSubNode::inputCloudCallback, this, std::placeholders::_1)
        );

        dynamic_obstacles_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)));

        // SegmentDifferences::segment() 每帧都会重建背景 KD 树。
        // 背景固定，建树一次并复用相同的差分算法，避免旧扫描积压。
        background_tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
        if (!static_cloud_->empty()) background_tree_->setInputCloud(static_cloud_);
    }

    // 处理输入的点云
    void inputCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN(this->get_logger(), "跳过点云: header.frame_id 为空!");
            return;
        }

        try {
            // 目标系在前、源系在后；使用点云消息中的采样时间查询。
            odom_to_lidar_ = tf_buffer_->lookupTransform(
                lidar_frame_, odom_frame_, rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(0.05)
              );
        }
        catch(const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "未查找到TF信息%s -> %s", odom_frame_.c_str(), lidar_frame_.c_str());
            return;
        }

        try{
            lidar_to_footprint_ = tf_buffer_->lookupTransform(
                base_frame_, lidar_frame_, rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(0.05)
              );
        }
        catch(const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "未查找到TF信息%s -> %s", lidar_frame_.c_str(), base_frame_.c_str());
            return;
        }

        // 计算odom->basefootprint
        tf2::Transform T_odom_lidar;
        tf2::Transform T_lidar_footprint;

        tf2::fromMsg(odom_to_lidar_.transform, T_odom_lidar);
        tf2::fromMsg(lidar_to_footprint_.transform, T_lidar_footprint);
        
        tf2::Transform T_odom_footprint = T_lidar_footprint * T_odom_lidar;

        // --- gazebo发布的tf与点云中心存在偏差,这里直接强制纠正,后面需要去除 ---
        // Gazebo 测量原点相对雷达 link，沿局部 Z 轴偏移 +0.03 m。
        tf2::Transform T_lidar_sensor;
        T_lidar_sensor.setIdentity();
        T_lidar_sensor.setOrigin(tf2::Vector3(0.0, 0.0, 0.03));
        // 将测量原点的偏移合入点云到 odom 的变换。
        T_odom_lidar = T_odom_lidar * T_lidar_sensor;
        // ----------------------------------------------------------------------

        pcl::fromROSMsg(*msg, *current_cloud_pcl_);

        // 点云：lidar->basefootprint
        auto lidar_to_footprint = tf2::toMsg(T_lidar_footprint);

        Eigen::Affine3f T_lidar_footprint_matrix = Eigen::Affine3f::Identity();
        T_lidar_footprint_matrix.translation() = Eigen::Vector3f(
            lidar_to_footprint.translation.x,
            lidar_to_footprint.translation.y,
            lidar_to_footprint.translation.z
        );
        T_lidar_footprint_matrix.rotate(Eigen::Quaternionf(
            lidar_to_footprint.rotation.w,
            lidar_to_footprint.rotation.x,
            lidar_to_footprint.rotation.y,
            lidar_to_footprint.rotation.z
        ));
        
        pcl::PointCloud<pcl::PointXYZ> cloud_in_footprint;
        pcl::transformPointCloud(*current_cloud_pcl_, cloud_in_footprint, T_lidar_footprint_matrix);

        // 按距离过滤点云
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
        for (const auto &point : cloud_in_footprint) {
            float distance = std::hypot(point.x, point.y);
            if (distance >= robot_radius_ && distance <= horizon_)
            {
                filtered_cloud.push_back(point);
            }
        }

        // 点云：basefootprint->odom
        auto footprint_to_odom = tf2::toMsg(T_odom_footprint.inverse());

        Eigen::Affine3f T_footprint_odom_matrix = Eigen::Affine3f::Identity();
        T_footprint_odom_matrix.translation() = Eigen::Vector3f(
            footprint_to_odom.translation.x,
            footprint_to_odom.translation.y,
            footprint_to_odom.translation.z
        );
        T_footprint_odom_matrix.rotate(Eigen::Quaternionf(
            footprint_to_odom.rotation.w,
            footprint_to_odom.rotation.x,
            footprint_to_odom.rotation.y,
            footprint_to_odom.rotation.z
        ));
        
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud_in_odom;
        pcl::transformPointCloud(filtered_cloud, filtered_cloud_in_odom, T_footprint_odom_matrix);

        // 与 SegmentDifferences 相同的最近邻差分；阈值仍为距离平方。
        if (static_cloud_->empty())
            *obstacle_cloud_ = filtered_cloud_in_odom;
        else
            pcl::getPointCloudDifference(filtered_cloud_in_odom,
                distance_threshold_ * distance_threshold_, background_tree_, *obstacle_cloud_);

        // 点云：odom->basefootprint
        // mesh的obstacle_layer中的max_obstacle_dist是以obstacle点云中心来计算的,所以obstacle点云必须变换回车体系
        auto odom_to_footprint = tf2::toMsg(T_odom_footprint);

        Eigen::Affine3f T_odom_footprint_matrix = Eigen::Affine3f::Identity();
        T_odom_footprint_matrix.translation() = Eigen::Vector3f(
            odom_to_footprint.translation.x,
            odom_to_footprint.translation.y,
            odom_to_footprint.translation.z
        );
        T_odom_footprint_matrix.rotate(Eigen::Quaternionf(
            odom_to_footprint.rotation.w,
            odom_to_footprint.rotation.x,
            odom_to_footprint.rotation.y,
            odom_to_footprint.rotation.z
        ));
        
        pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_in_footprint;
        pcl::transformPointCloud(*obstacle_cloud_, obstacle_cloud_in_footprint, T_odom_footprint_matrix);

        sensor_msgs::msg::PointCloud2 obstacle_msg;
        pcl::toROSMsg(obstacle_cloud_in_footprint, obstacle_msg);
        obstacle_msg.header = msg->header; // 保留原单帧点云的采样时间
        obstacle_msg.header.frame_id = base_frame_;
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
