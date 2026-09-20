/**
 * @file rog_map_simple_node.cpp
 * @brief 简化的ROG-Map节点，使用ProbMap核心功能
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rog_map/prob_map.h"

using namespace rog_map;

// 简单的ROGMap实现
class SimpleROGMap : public ProbMap {
public:
    // 不需要override，ProbMap没有这个虚函数
    double getSystemWalltimeNow() {
        return std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    void initialize(const Config& config) {
        cfg_ = config;
        initProbMap();
    }

    void update(const PointCloud& cloud, const Pose& pose) {
        updateProbMap(cloud, pose, pose.first);
    }
};

class ROGMapSimpleNode : public rclcpp::Node {
public:
    ROGMapSimpleNode() : Node("rog_map_simple"), has_odom_(false) {
        RCLCPP_INFO(this->get_logger(), "=== ROG-Map Simple Node Starting ===");

        // 声明参数 - 使用极小的地图尺寸以避免内存溢出
        this->declare_parameter("cloud_topic", "/cloud_registered_full");
        this->declare_parameter("odom_topic", "/aft_mapped_to_init");
        this->declare_parameter("output_grid_topic", "/rog_map/occupancy_grid");
        this->declare_parameter("map_resolution", 0.4);  // 更大的分辨率
        this->declare_parameter("map_half_size_x", 4.0);  // 更小的范围：8m x 8m
        this->declare_parameter("map_half_size_y", 4.0);
        this->declare_parameter("map_half_size_z", 0.8);  // 更低的高度
        this->declare_parameter("grid_width", 80);  // 更小的栅格：80x80
        this->declare_parameter("grid_height", 80);
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("frame_id", "map");

        // 获取参数
        std::string cloud_topic = this->get_parameter("cloud_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string output_topic = this->get_parameter("output_grid_topic").as_string();
        resolution_ = this->get_parameter("map_resolution").as_double();
        double half_x = this->get_parameter("map_half_size_x").as_double();
        double half_y = this->get_parameter("map_half_size_y").as_double();
        double half_z = this->get_parameter("map_half_size_z").as_double();
        grid_width_ = this->get_parameter("grid_width").as_int();
        grid_height_ = this->get_parameter("grid_height").as_int();
        double pub_rate = this->get_parameter("publish_rate").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();

        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  - cloud_topic: %s", cloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  - odom_topic: %s", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  - resolution: %.2f m", resolution_);
        RCLCPP_INFO(this->get_logger(), "  - grid_size: %d x %d", grid_width_, grid_height_);

        // 初始化ROG-Map
        initROGMap(resolution_, half_x, half_y, half_z);

        // 订阅
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, 10,
            std::bind(&ROGMapSimpleNode::cloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&ROGMapSimpleNode::odomCallback, this, std::placeholders::_1));

        // 发布
        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic, 10);

        // 定时器
        auto period_ms = static_cast<int>(1000.0 / pub_rate);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&ROGMapSimpleNode::publishGridMap, this));

        RCLCPP_INFO(this->get_logger(), "=== ROG-Map Simple Node Ready ===");
    }

private:
    void initROGMap(double resolution, double half_x, double half_y, double half_z) {
        rog_map_ = std::make_shared<SimpleROGMap>();

        Config cfg;
        cfg.resolution = resolution;

        // inflation_resolution必须 >= resolution
        cfg.inflation_resolution = resolution + 0.001;  // 稍大于resolution
        cfg.inflation_step = 1;  // 膨胀步数

        // ESDF分辨率必须 >= inflation_resolution（根据ESDFMap的要求）
        cfg.esdf_resolution = resolution + 0.002;  // 稍大于inflation_resolution
        cfg.esdf_en = true;

        // Config需要map_size_d（全尺寸）
        cfg.map_size_d = Vec3f(half_x * 2.0, half_y * 2.0, half_z * 2.0);

        cfg.map_sliding_en = true;
        cfg.fix_map_origin = Vec3f(0, 0, 0);

        // 设置概率参数
        cfg.p_hit = 0.85;
        cfg.p_miss = 0.4;
        cfg.p_min = 0.12;
        cfg.p_max = 0.97;
        cfg.p_occ = 0.7;

        cfg.odom_timeout = 0.5;

        // 设置虚拟地面和天花板高度
        cfg.virtual_ground_height = -0.8;
        cfg.virtual_ceil_height = 1.8;

        // 关闭不需要的功能以减少内存
        cfg.frontier_extraction_en = false;
        cfg.unk_inflation_en = false;

        // 设置raycasting参数
        cfg.raycast_range_min = 0.3;
        cfg.raycast_range_max = 10.0;
        cfg.local_update_box_d = Vec3f(20.0, 20.0, 4.0);
        cfg.esdf_local_update_box = Vec3f(10.0, 10.0, 2.0);

        // 设置unk_thresh（避免未初始化）
        cfg.unk_thresh = 0.7;

        // 调用resetMapSize()来计算half_map_size_i等参数
        cfg.resetMapSize();

        RCLCPP_INFO(this->get_logger(), "After resetMapSize: resolution=%.3f, inflation_resolution=%.3f, esdf_resolution=%.3f",
                    cfg.resolution, cfg.inflation_resolution, cfg.esdf_resolution);
        RCLCPP_INFO(this->get_logger(), "Config computed: half_map_size_i=[%d,%d,%d], inf_half_map_size_i=[%d,%d,%d]",
                    cfg.half_map_size_i.x(), cfg.half_map_size_i.y(), cfg.half_map_size_i.z(),
                    cfg.inf_half_map_size_i.x(), cfg.inf_half_map_size_i.y(), cfg.inf_half_map_size_i.z());

        rog_map_->initialize(cfg);

        RCLCPP_INFO(this->get_logger(), "ROG-Map initialized successfully!");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_.position = msg->pose.pose.position;
        current_pose_.orientation = msg->pose.pose.orientation;
        has_odom_ = true;
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!has_odom_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "No odometry received yet, skipping cloud");
            return;
        }

        // 转换点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            return;
        }

        // 构造位姿
        Pose pose;
        pose.first = Vec3f(
            current_pose_.position.x,
            current_pose_.position.y,
            current_pose_.position.z
        );
        pose.second = super_utils::Quatf(
            current_pose_.orientation.w,
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z
        );

        // 更新地图
        rog_map_->update(*cloud, pose);

        cloud_count_++;
        if (cloud_count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "Updated ROG-Map: cloud#%ld, size=%zu, pos=[%.2f, %.2f, %.2f]",
                       cloud_count_, cloud->size(),
                       pose.first.x(), pose.first.y(), pose.first.z());
        }
    }

    void publishGridMap() {
        if (!has_odom_) {
            return;
        }

        auto grid_msg = nav_msgs::msg::OccupancyGrid();
        grid_msg.header.frame_id = frame_id_;
        grid_msg.header.stamp = this->now();

        // 地图信息
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = grid_width_;
        grid_msg.info.height = grid_height_;

        // 地图原点：以当前位置为中心
        double map_size_x = grid_width_ * resolution_;
        double map_size_y = grid_height_ * resolution_;
        grid_msg.info.origin.position.x = current_pose_.position.x - map_size_x / 2.0;
        grid_msg.info.origin.position.y = current_pose_.position.y - map_size_y / 2.0;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        // 填充栅格数据
        grid_msg.data.resize(grid_width_ * grid_height_);
        int occupied_count = 0;
        int free_count = 0;
        int unknown_count = 0;

        for (int j = 0; j < grid_height_; j++) {
            for (int i = 0; i < grid_width_; i++) {
                double wx = grid_msg.info.origin.position.x + (i + 0.5) * resolution_;
                double wy = grid_msg.info.origin.position.y + (j + 0.5) * resolution_;
                Vec3f pos(wx, wy, 0.0);
                int idx = i + j * grid_width_;

                // 查询占据状态
                if (rog_map_->isOccupied(pos)) {
                    grid_msg.data[idx] = 100;
                    occupied_count++;
                } else if (rog_map_->isKnownFree(pos)) {
                    grid_msg.data[idx] = 0;
                    free_count++;
                } else {
                    grid_msg.data[idx] = -1;
                    unknown_count++;
                }
            }
        }

        grid_pub_->publish(grid_msg);

        // 统计信息
        pub_count_++;
        if (pub_count_ % 50 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "Published grid: Occ=%d, Free=%d, Unk=%d, Pos=[%.2f, %.2f]",
                       occupied_count, free_count, unknown_count,
                       current_pose_.position.x, current_pose_.position.y);
        }
    }

    // ROS接口
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ROG-Map实例
    std::shared_ptr<SimpleROGMap> rog_map_;

    // 状态
    geometry_msgs::msg::Pose current_pose_;
    bool has_odom_;
    long cloud_count_ = 0;
    int pub_count_ = 0;

    // 参数
    double resolution_;
    int grid_width_;
    int grid_height_;
    std::string frame_id_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROGMapSimpleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
