/**
 * @file rog_map_simple_node.cpp
 * @brief 简化的ROG-Map节点，使用ProbMap核心功能
 */

#define _GNU_SOURCE
#include <fenv.h>

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

        // ===== 第一步：设置 resetMapSize() 需要的基础参数 =====
        cfg.resolution = resolution;
        cfg.inflation_resolution = resolution + 0.001;
        cfg.esdf_resolution = resolution + 0.002;
        cfg.map_size_d = Vec3f(half_x * 2.0, half_y * 2.0, half_z * 2.0);

        // ===== 第二步：调用 resetMapSize() =====
        cfg.resetMapSize();

        // ===== 第三步：设置所有其他参数（在 resetMapSize 之后，避免被覆盖）=====

        // 地图滑动
        cfg.map_sliding_en = true;
        cfg.fix_map_origin = Vec3f(0, 0, 0);

        // 概率参数
        cfg.p_hit = 0.85;
        cfg.p_miss = 0.4;
        cfg.p_min = 0.12;
        cfg.p_max = 0.97;
        cfg.p_occ = 0.80;
        cfg.p_free = 0.30;

        // 手动计算 logit 值（resetMapSize 会重置这些）
        auto logit = [](double x) { return log(x / (1.0 - x)); };
        cfg.l_min = logit(cfg.p_min);
        cfg.l_max = logit(cfg.p_max);
        cfg.l_occ = logit(cfg.p_occ);
        cfg.l_free = logit(cfg.p_free);

        RCLCPP_INFO(this->get_logger(), "[DEBUG] Occupancy thresholds: l_free=%.4f, l_occ=%.4f (p_free=%.4f, p_occ=%.4f)",
                    cfg.l_free, cfg.l_occ, cfg.p_free, cfg.p_occ);

        // 虚拟地面和天花板（resetMapSize 会重置这些）
        cfg.virtual_ground_height = -0.8;
        cfg.virtual_ceil_height = 1.8;

        // Raycasting 参数
        cfg.raycasting_en = true;
        cfg.raycast_range_min = 0.3;
        cfg.raycast_range_max = 10.0;
        cfg.local_update_box_d = Vec3f(20.0, 20.0, 4.0);
        cfg.esdf_local_update_box = Vec3f(10.0, 10.0, 2.0);
        cfg.point_filt_num = 1;
        cfg.batch_update_size = 1;

        // 重新计算平方距离（因为在 resetMapSize 之后修改了 raycast_range）
        cfg.sqr_raycast_range_min = cfg.raycast_range_min * cfg.raycast_range_min;
        cfg.sqr_raycast_range_max = cfg.raycast_range_max * cfg.raycast_range_max;

        // 其他参数
        cfg.esdf_en = true;
        cfg.inflation_step = 1;
        cfg.frontier_extraction_en = false;
        cfg.unk_inflation_en = false;
        cfg.unk_thresh = 0.7;
        cfg.odom_timeout = 0.5;

        // 计算 half_local_update_box_i（需要在设置 local_update_box_d 之后）
        cfg.half_local_update_box_i.x() = std::ceil(cfg.local_update_box_d.x() / 2.0 / cfg.resolution);
        cfg.half_local_update_box_i.y() = std::ceil(cfg.local_update_box_d.y() / 2.0 / cfg.resolution);
        cfg.half_local_update_box_i.z() = std::ceil(cfg.local_update_box_d.z() / 2.0 / cfg.resolution);

        // 打印配置信息
        RCLCPP_INFO(this->get_logger(), "After resetMapSize: resolution=%.3f, inflation_resolution=%.3f, esdf_resolution=%.3f",
                    cfg.resolution, cfg.inflation_resolution, cfg.esdf_resolution);
        RCLCPP_INFO(this->get_logger(), "Config computed: half_map_size_i=[%d,%d,%d], inf_half_map_size_i=[%d,%d,%d]",
                    cfg.half_map_size_i.x(), cfg.half_map_size_i.y(), cfg.half_map_size_i.z(),
                    cfg.inf_half_map_size_i.x(), cfg.inf_half_map_size_i.y(), cfg.inf_half_map_size_i.z());
        RCLCPP_INFO(this->get_logger(), "Occupancy thresholds: l_occ=%.3f, l_free=%.3f (from p_occ=%.2f, p_free=%.2f)",
                    cfg.l_occ, cfg.l_free, cfg.p_occ, cfg.p_free);
        RCLCPP_INFO(this->get_logger(), "Virtual heights: ground=%.2f, ceil=%.2f",
                    cfg.virtual_ground_height, cfg.virtual_ceil_height);

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
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_body(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud_body);

        if (cloud_body->empty()) {
            return;
        }

        // 构造位姿 - 确保四元数归一化
        double qw = current_pose_.orientation.w;
        double qx = current_pose_.orientation.x;
        double qy = current_pose_.orientation.y;
        double qz = current_pose_.orientation.z;
        double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);

        if (norm < 1e-6) {
            RCLCPP_ERROR(this->get_logger(), "Invalid quaternion norm: %f", norm);
            return;
        }

        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;

        Pose pose;
        pose.first = Vec3f(
            current_pose_.position.x,
            current_pose_.position.y,
            current_pose_.position.z
        );
        pose.second = super_utils::Quatf(qw, qx, qy, qz);

        // 将点云从 body frame 转换到世界坐标系
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_world->reserve(cloud_body->size());

        Eigen::Quaternionf q(qw, qx, qy, qz);
        Eigen::Vector3f t(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);

        // 调试：打印前几个点的转换
        static int debug_frame_count = 0;
        if (++debug_frame_count % 50 == 0 && !cloud_body->empty()) {
            const auto& pt0 = cloud_body->points[0];
            Eigen::Vector3f p_body(pt0.x, pt0.y, pt0.z);
            Eigen::Vector3f p_world = q * p_body + t;
            RCLCPP_INFO(this->get_logger(),
                       "[DEBUG Transform] Body point: [%.2f, %.2f, %.2f] -> World point: [%.2f, %.2f, %.2f], Robot pos: [%.2f, %.2f, %.2f]",
                       pt0.x, pt0.y, pt0.z, p_world.x(), p_world.y(), p_world.z(),
                       t.x(), t.y(), t.z());
        }

        for (const auto& pt_body : *cloud_body) {
            Eigen::Vector3f p_body(pt_body.x, pt_body.y, pt_body.z);
            Eigen::Vector3f p_world = q * p_body + t;

            pcl::PointXYZI pt_world;
            pt_world.x = p_world.x();
            pt_world.y = p_world.y();
            pt_world.z = p_world.z();
            pt_world.intensity = pt_body.intensity;
            cloud_world->push_back(pt_world);
        }

        // 更新地图（使用转换后的世界坐标系点云）
        rog_map_->update(*cloud_world, pose);

        cloud_count_++;
        if (cloud_count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "Updated ROG-Map: cloud#%ld, size=%zu, pos=[%.2f, %.2f, %.2f]",
                       cloud_count_, cloud_world->size(),
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

        static int grid_debug_counter = 0;
        bool should_debug = (++grid_debug_counter % 10 == 0);

        for (int j = 0; j < grid_height_; j++) {
            for (int i = 0; i < grid_width_; i++) {
                double wx = grid_msg.info.origin.position.x + (i + 0.5) * resolution_;
                double wy = grid_msg.info.origin.position.y + (j + 0.5) * resolution_;
                // 使用机器人当前的 z 坐标，而不是固定的 0.0
                Vec3f pos(wx, wy, current_pose_.position.z);
                int idx = i + j * grid_width_;

                // 调试：采样中心点
                if (should_debug && i == grid_width_/2 && j == grid_height_/2) {
                    bool is_occ = rog_map_->isOccupied(pos);
                    bool is_free = rog_map_->isKnownFree(pos);
                    bool is_unk = rog_map_->isUnknown(pos);
                    double map_value = rog_map_->getMapValue(pos);
                    double robot_to_cell_dist = sqrt(pow(pos.x() - current_pose_.position.x, 2) +
                                                     pow(pos.y() - current_pose_.position.y, 2));
                    RCLCPP_INFO(this->get_logger(),
                               "[DEBUG Grid] Center cell: pos=[%.2f, %.2f, %.2f], mapValue=%.4f, isOcc=%d, isFree=%d, isUnk=%d, dist_from_robot=%.2f",
                               pos.x(), pos.y(), pos.z(), map_value, is_occ, is_free, is_unk, robot_to_cell_dist);
                }

                // 调试：采样几个其他位置看看是否有任何被标记为占用
                if (should_debug && i == 0 && j == 0) {
                    int sample_occupied = 0;
                    int sample_free = 0;
                    int sample_unknown = 0;
                    // 采样 10x10 的子集
                    for (int sj = 0; sj < grid_height_; sj += 10) {
                        for (int si = 0; si < grid_width_; si += 10) {
                            double swx = grid_msg.info.origin.position.x + (si + 0.5) * resolution_;
                            double swy = grid_msg.info.origin.position.y + (sj + 0.5) * resolution_;
                            Vec3f spos(swx, swy, current_pose_.position.z);
                            if (rog_map_->isOccupied(spos)) sample_occupied++;
                            else if (rog_map_->isKnownFree(spos)) sample_free++;
                            else sample_unknown++;
                        }
                    }
                    RCLCPP_INFO(this->get_logger(),
                               "[DEBUG Grid] Sampled 100 cells: occ=%d, free=%d, unk=%d",
                               sample_occupied, sample_free, sample_unknown);
                }

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

        // 调试：打印前几个非未知格子的实际值
        static int debug_count = 0;
        if (debug_count++ < 5 && (occupied_count > 0 || free_count > 0)) {
            RCLCPP_INFO(this->get_logger(), "[DEBUG] Sample map values around robot:");
            int sample_count = 0;
            for (int di = -5; di <= 5 && sample_count < 10; di++) {
                for (int dj = -5; dj <= 5 && sample_count < 10; dj++) {
                    Vec3f test_pos(current_pose_.position.x + di * resolution_,
                                   current_pose_.position.y + dj * resolution_,
                                   0.0);
                    double val = rog_map_->getMapValue(test_pos);
                    // 只打印非零值
                    if (val != 0.0) {
                        RCLCPP_INFO(this->get_logger(), "  pos[%.2f,%.2f]: val=%.3f, occ=%d, free=%d",
                                   test_pos.x(), test_pos.y(), val,
                                   rog_map_->isOccupied(test_pos), rog_map_->isKnownFree(test_pos));
                        sample_count++;
                    }
                }
            }
            if (sample_count == 0) {
                RCLCPP_WARN(this->get_logger(), "[DEBUG] All sampled values are 0.0!");
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
    // 禁用浮点异常陷阱 - 防止其他节点启用的FPE陷阱影响ROG-Map
    #ifdef __linux__
    fedisableexcept(FE_ALL_EXCEPT);
    std::cout << "[ROG-Map] Floating point exceptions disabled" << std::endl;
    #endif

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROGMapSimpleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
