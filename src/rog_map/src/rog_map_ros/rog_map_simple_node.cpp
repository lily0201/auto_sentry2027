/**
 * @file rog_map_simple_node.cpp
 * @brief 简化的ROG-Map节点，使用ProbMap核心功能
 */

#include <fenv.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rog_map/prob_map.h"
#include "rog_map/projection_layer.hpp"

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
        setUpdateTime(getSystemWalltimeNow());  // 给体素打命中时间戳(ProjectionLayer 的 obstacle_hold_time 用)
        updateProbMap(cloud, pose, pose.first);
    }

    // 绝对 z 范围 -> 3D 地图的全局 z 索引范围(裁剪到当前局部地图内)
    void scanZRange(double z_lo, double z_hi, int& z_min_id, int& z_max_id) const {
        posToGlobalIndex(z_lo, z_min_id);
        posToGlobalIndex(z_hi, z_max_id);
        if (z_min_id > z_max_id) {
            std::swap(z_min_id, z_max_id);
        }
        z_min_id = std::max(z_min_id, localMapMinIndex().z());
        z_max_id = std::min(z_max_id, localMapMaxIndex().z());
    }

    // ProjectionLayer 的按列扫描回调：统计 (gx,gy) 这一列在 [z_min_id, z_max_id] 内的占据分布。
    // 直接读原始概率，不套用 virtual ground/ceil 规则(与 ROGMap::refreshLayers 的做法一致)。
    ColumnStats scanColumn(int gx, int gy, int z_min_id, int z_max_id) const {
        ColumnStats stats;
        for (int gz = z_min_id; gz <= z_max_id; ++gz) {
            const Vec3i id_g(gx, gy, gz);
            if (!insideLocalMap(id_g)) {
                continue;
            }
            const int hash_id = getHashIndexFromGlobalIndex(id_g);
            if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size())) {
                continue;
            }
            const double prob = occupancy_buffer_[hash_id];
            const bool occupied = ProbMap::isOccupied(prob);
            const bool known_free = ProbMap::isKnownFree(prob);
            if (!occupied && !known_free) {
                continue;  // 未知体素不计入观测
            }
            ++stats.observed_count;
            stats.last_update_time = std::max(stats.last_update_time, cellLastUpdateTime(id_g));
            if (occupied) {
                ++stats.occupied_count;
                stats.occupied_z_index_min = std::min(stats.occupied_z_index_min, gz);
                stats.occupied_z_index_max = std::max(stats.occupied_z_index_max, gz);
                Vec3f pos;
                globalIndexToPos(id_g, pos);
                stats.occupied_z_min_abs = std::min(stats.occupied_z_min_abs, static_cast<double>(pos.z()));
                stats.occupied_z_max_abs = std::max(stats.occupied_z_max_abs, static_cast<double>(pos.z()));
                stats.last_hit_time = std::max(stats.last_hit_time, cellLastHitTime(id_g));
            }
        }
        return stats;
    }
};

class ROGMapSimpleNode : public rclcpp::Node {
public:
    ROGMapSimpleNode() : Node("rog_map_simple"), has_odom_(false) {
        RCLCPP_INFO(this->get_logger(), "=== ROG-Map Simple Node Starting ===");

        // 声明参数
        this->declare_parameter("cloud_topic", "/cloud_registered");
        this->declare_parameter("odom_topic", "/Odometry");
        this->declare_parameter("output_grid_topic", "/rog_map/occupancy_grid");
        this->declare_parameter("map_resolution", 0.1);
        this->declare_parameter("map_half_size_x", 6.0);   // 3D地图半宽，需大于 grid_half_size + map_sliding_threshold
        this->declare_parameter("map_half_size_y", 6.0);
        this->declare_parameter("map_half_size_z", 1.5);
        this->declare_parameter("virtual_ground_height", -1.2);  // 世界系z，需低于地面(雷达离地约0.5m)
        this->declare_parameter("virtual_ceil_height", 1.5);
        this->declare_parameter("map_sliding_threshold", 0.5);
        this->declare_parameter("raycast_range_max", 8.0);
        this->declare_parameter("grid_half_size", 5.0);    // 发布的2D栅格半宽(米)
        this->declare_parameter("proj_z_min", -0.35);      // 投影高度下限，相对雷达(需高于地面噪声)
        this->declare_parameter("proj_z_max", 0.02);       // 投影高度上限，相对雷达
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("frame_id", "map");

        // 障碍点云输出(供 Nav2 ObstacleLayer / KeepAwayFromObstacles 使用)
        // 必须是机器人底盘系：ObstacleLayer 以点云 frame 原点作为 raytrace 起点，行为节点也把 x,y 当作相对机器人的偏移
        this->declare_parameter("publish_obstacle_cloud", true);
        this->declare_parameter("obstacle_cloud_topic", "/rog_map/obstacle_cloud");
        this->declare_parameter("obstacle_cloud_frame", "base_link");
        this->declare_parameter("obstacle_cloud_spacing", 0.05);  // 点间距，应等于 costmap 分辨率
        this->declare_parameter("obstacle_cloud_z", -0.2);

        // 带真实高度的3D占据点云(仅用于可视化/调试，不接入costmap；无订阅者时不计算)
        this->declare_parameter("publish_cloud_3d", true);
        this->declare_parameter("cloud_3d_topic", "/rog_map/obstacle_cloud_3d");
        this->declare_parameter("cloud_3d_z_min", -0.9);  // 相对雷达，须在virtual_ground/ceil之内，否则查询恒为占据
        this->declare_parameter("cloud_3d_z_max", 1.2);

        // ProjectionLayer: 按 xy 列分析 3D 占据的垂直分布，区分 地面/墙/隧道(门洞)。
        // projection_layer_en: 运行该层并发布调试栅格(不改变 costmap 输出)
        // projection_use_for_output: 用该层结果替换 "固定高度带" 逻辑，生成 occupancy_grid 和障碍点云
        this->declare_parameter("projection_layer_en", false);
        this->declare_parameter("projection_use_for_output", false);
        this->declare_parameter("projection_debug_topic", "/rog_map/projection_grid");
        this->declare_parameter("layer_scan_z_min", -0.25);  // 扫描高度范围，相对雷达
        this->declare_parameter("layer_scan_z_max", 1.2);
        this->declare_parameter("layer_unknown_as_occupied", false);
        this->declare_parameter("layer_min_observed_voxels", 2);
        this->declare_parameter("layer_surface_height_delta_max", 0.10);
        this->declare_parameter("layer_wall_height_delta_min", 0.20);
        this->declare_parameter("layer_wall_occupancy_ratio_min", 0.80);
        this->declare_parameter("layer_tunnel_height_delta_min", 0.24);
        this->declare_parameter("layer_tunnel_height_delta_max", 0.40);
        this->declare_parameter("layer_tunnel_occupancy_ratio_max", 0.55);
        this->declare_parameter("layer_passable_as_free", true);
        this->declare_parameter("layer_passable_cost", 50);
        this->declare_parameter("layer_hysteresis_en", true);
        this->declare_parameter("layer_hysteresis_count", 2);
        this->declare_parameter("layer_obstacle_hold_time", 0.0);
        this->declare_parameter("layer_mask_filter_en", true);
        this->declare_parameter("layer_fill_occ_min", 5);
        this->declare_parameter("layer_denoise_occ_max", 0);

        // 获取参数
        std::string cloud_topic = this->get_parameter("cloud_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string output_topic = this->get_parameter("output_grid_topic").as_string();
        resolution_ = this->get_parameter("map_resolution").as_double();
        double half_x = this->get_parameter("map_half_size_x").as_double();
        double half_y = this->get_parameter("map_half_size_y").as_double();
        double half_z = this->get_parameter("map_half_size_z").as_double();
        double ground_h = this->get_parameter("virtual_ground_height").as_double();
        double ceil_h = this->get_parameter("virtual_ceil_height").as_double();
        double slide_thresh = this->get_parameter("map_sliding_threshold").as_double();
        double ray_max = this->get_parameter("raycast_range_max").as_double();
        double grid_half = this->get_parameter("grid_half_size").as_double();
        proj_z_min_ = this->get_parameter("proj_z_min").as_double();
        proj_z_max_ = this->get_parameter("proj_z_max").as_double();
        double pub_rate = this->get_parameter("publish_rate").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_obstacle_cloud_ = this->get_parameter("publish_obstacle_cloud").as_bool();
        std::string obstacle_topic = this->get_parameter("obstacle_cloud_topic").as_string();
        obstacle_cloud_frame_ = this->get_parameter("obstacle_cloud_frame").as_string();
        obstacle_cloud_z_ = this->get_parameter("obstacle_cloud_z").as_double();
        publish_cloud_3d_ = this->get_parameter("publish_cloud_3d").as_bool();
        std::string cloud_3d_topic = this->get_parameter("cloud_3d_topic").as_string();
        cloud_3d_z_min_ = std::max(this->get_parameter("cloud_3d_z_min").as_double(), ground_h + resolution_);
        cloud_3d_z_max_ = std::min(this->get_parameter("cloud_3d_z_max").as_double(), ceil_h - resolution_);

        projection_layer_en_ = this->get_parameter("projection_layer_en").as_bool();
        projection_use_for_output_ = projection_layer_en_ && this->get_parameter("projection_use_for_output").as_bool();
        std::string projection_debug_topic = this->get_parameter("projection_debug_topic").as_string();
        layer_scan_z_min_ = this->get_parameter("layer_scan_z_min").as_double();
        layer_scan_z_max_ = this->get_parameter("layer_scan_z_max").as_double();
        if (layer_scan_z_min_ >= layer_scan_z_max_) {
            throw std::runtime_error("layer_scan_z_min must be smaller than layer_scan_z_max");
        }
        layer_cfg_.unknown_as_occupied = this->get_parameter("layer_unknown_as_occupied").as_bool();
        layer_cfg_.min_observed_voxels = this->get_parameter("layer_min_observed_voxels").as_int();
        layer_cfg_.surface_height_delta_max = this->get_parameter("layer_surface_height_delta_max").as_double();
        layer_cfg_.wall_height_delta_min = this->get_parameter("layer_wall_height_delta_min").as_double();
        layer_cfg_.wall_occupancy_ratio_min = this->get_parameter("layer_wall_occupancy_ratio_min").as_double();
        layer_cfg_.tunnel_height_delta_min = this->get_parameter("layer_tunnel_height_delta_min").as_double();
        layer_cfg_.tunnel_height_delta_max = this->get_parameter("layer_tunnel_height_delta_max").as_double();
        layer_cfg_.tunnel_occupancy_ratio_max = this->get_parameter("layer_tunnel_occupancy_ratio_max").as_double();
        layer_cfg_.passable_as_free = this->get_parameter("layer_passable_as_free").as_bool();
        layer_cfg_.passable_cost = static_cast<uint8_t>(
            std::clamp<int64_t>(this->get_parameter("layer_passable_cost").as_int(), 0, 252));
        layer_cfg_.hysteresis_en = this->get_parameter("layer_hysteresis_en").as_bool();
        layer_cfg_.hysteresis_count = this->get_parameter("layer_hysteresis_count").as_int();
        layer_cfg_.obstacle_hold_time = this->get_parameter("layer_obstacle_hold_time").as_double();
        layer_cfg_.mask_filter_en = this->get_parameter("layer_mask_filter_en").as_bool();
        layer_cfg_.fill_occ_min = this->get_parameter("layer_fill_occ_min").as_int();
        layer_cfg_.denoise_occ_max = this->get_parameter("layer_denoise_occ_max").as_int();
        // 每个占据格按 costmap 分辨率展开成 n x n 个点，使其落到 costmap 上是连续的一片。
        // 若每格只发一个点，0.1m 间距对 0.05m 的 costmap 是互不相邻的孤立单元，会被 DenoiseLayer 当作噪声删掉。
        obstacle_sub_n_ = std::max(1, static_cast<int>(std::lround(
            resolution_ / this->get_parameter("obstacle_cloud_spacing").as_double())));

        // 栅格窗口必须完全落在3D地图内：地图中心滑动前最多偏离机器人 slide_thresh
        const double max_grid_half = std::min(half_x, half_y) - slide_thresh - 2.0 * resolution_;
        if (grid_half > max_grid_half) {
            RCLCPP_WARN(this->get_logger(),
                        "grid_half_size %.2f exceeds map limit %.2f (map_half - sliding_thresh - margin), clamped",
                        grid_half, max_grid_half);
            grid_half = max_grid_half;
        }
        // 栅格边长取偶数格，使原点可对齐到地图cell边界
        grid_width_ = 2 * static_cast<int>(std::floor(grid_half / resolution_));
        grid_height_ = grid_width_;
        if (grid_width_ <= 0) {
            throw std::runtime_error("grid size <= 0, check map_half_size / map_sliding_threshold");
        }
        if (proj_z_min_ >= proj_z_max_) {
            throw std::runtime_error("proj_z_min must be smaller than proj_z_max");
        }
        // 投影时会按 传感器z + proj_z 查询；若落在 virtual 高度之外，ProbMap 会直接返回 occupied
        if (proj_z_min_ <= ground_h || proj_z_max_ >= ceil_h) {
            RCLCPP_WARN(this->get_logger(),
                        "proj_z [%.2f, %.2f] is not strictly inside virtual range [%.2f, %.2f]; "
                        "only valid if the sensor stays near z=0",
                        proj_z_min_, proj_z_max_, ground_h, ceil_h);
        }

        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  - cloud_topic: %s", cloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  - odom_topic: %s", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  - resolution: %.2f m", resolution_);
        RCLCPP_INFO(this->get_logger(), "  - grid_size: %d x %d cells (%.1f m)", grid_width_, grid_height_,
                    grid_width_ * resolution_);
        RCLCPP_INFO(this->get_logger(), "  - proj_z: [%.2f, %.2f] relative to sensor", proj_z_min_, proj_z_max_);

        // 初始化ROG-Map
        initROGMap(resolution_, half_x, half_y, half_z, ground_h, ceil_h, slide_thresh, ray_max);

        // 订阅
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, 10,
            std::bind(&ROGMapSimpleNode::cloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&ROGMapSimpleNode::odomCallback, this, std::placeholders::_1));

        // 发布
        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic, 10);
        if (projection_layer_en_) {
            projection_layer_ = std::make_unique<ProjectionLayer>();
            projection_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(projection_debug_topic, 1);
            RCLCPP_INFO(this->get_logger(), "  - ProjectionLayer enabled (use_for_output=%d), debug grid: %s",
                        projection_use_for_output_, projection_debug_topic.c_str());
        }
        if (publish_cloud_3d_) {
            cloud_3d_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_3d_topic, 1);
        }
        if (publish_obstacle_cloud_) {
            obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic, 10);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            RCLCPP_INFO(this->get_logger(), "  - obstacle cloud: %s (frame %s, z=%.2f rel. sensor)",
                        obstacle_topic.c_str(), obstacle_cloud_frame_.c_str(), obstacle_cloud_z_);
        }

        // 定时器
        auto period_ms = static_cast<int>(1000.0 / pub_rate);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&ROGMapSimpleNode::publishGridMap, this));

        RCLCPP_INFO(this->get_logger(), "=== ROG-Map Simple Node Ready ===");
    }

private:
    void initROGMap(double resolution, double half_x, double half_y, double half_z,
                    double ground_h, double ceil_h, double slide_thresh, double ray_max) {
        rog_map_ = std::make_shared<SimpleROGMap>();

        Config cfg;

        // resetMapSize() 会根据下列输入推导 half_map_size_i / inf_half_map_size_i /
        // local_update_box / inf_virtual_*_id_g 以及规整后的 virtual 高度，
        // 因此所有输入必须在调用它之前设置好，调用之后不要再覆盖这些派生量。

        // 分辨率：膨胀图与主图同分辨率（inflation_ratio = 1）
        cfg.resolution = resolution;
        cfg.inflation_resolution = resolution;
        cfg.esdf_resolution = resolution;
        cfg.inflation_step = 1;
        cfg.unk_inflation_en = false;
        cfg.unk_inflation_step = 1;
        cfg.frontier_extraction_en = false;

        // 地图范围与虚拟地面/天花板
        cfg.map_size_d = Vec3f(half_x * 2.0, half_y * 2.0, half_z * 2.0);
        cfg.virtual_ground_height = ground_h;
        cfg.virtual_ceil_height = ceil_h;
        cfg.local_update_box_d = cfg.map_size_d;

        cfg.resetMapSize();

        // 地图滑动
        cfg.map_sliding_en = true;
        cfg.map_sliding_thresh = slide_thresh;
        cfg.fix_map_origin = Vec3f(0, 0, 0);

        // 概率参数（logit 不由 resetMapSize 计算，需手动设置）
        cfg.p_hit = 0.85;
        cfg.p_miss = 0.4;
        cfg.p_min = 0.12;
        cfg.p_max = 0.97;
        cfg.p_occ = 0.80;
        cfg.p_free = 0.30;
        auto logit = [](double x) { return log(x / (1.0 - x)); };
        cfg.l_min = logit(cfg.p_min);
        cfg.l_max = logit(cfg.p_max);
        cfg.l_occ = logit(cfg.p_occ);
        cfg.l_free = logit(cfg.p_free);
        cfg.l_hit = logit(cfg.p_hit);
        cfg.l_miss = logit(cfg.p_miss);

        // Raycasting 参数
        cfg.raycasting_en = true;
        cfg.raycast_range_min = 0.3;
        cfg.raycast_range_max = ray_max;
        cfg.sqr_raycast_range_min = cfg.raycast_range_min * cfg.raycast_range_min;
        cfg.sqr_raycast_range_max = cfg.raycast_range_max * cfg.raycast_range_max;
        cfg.point_filt_num = 1;
        cfg.batch_update_size = 1;

        // 其他参数（ESDF 未被使用，关闭以节省内存和计算）
        cfg.esdf_en = false;
        cfg.esdf_local_update_box = Vec3f(10.0, 10.0, 2.0);
        cfg.unk_thresh = 0.7;
        cfg.odom_timeout = 0.5;

        RCLCPP_INFO(this->get_logger(),
                    "ROG-Map config: res=%.3f, half_map_size_i=[%d,%d,%d], inf_half_map_size_i=[%d,%d,%d], "
                    "local_update_box_i=[%d,%d,%d]",
                    cfg.resolution,
                    cfg.half_map_size_i.x(), cfg.half_map_size_i.y(), cfg.half_map_size_i.z(),
                    cfg.inf_half_map_size_i.x(), cfg.inf_half_map_size_i.y(), cfg.inf_half_map_size_i.z(),
                    cfg.local_update_box_i.x(), cfg.local_update_box_i.y(), cfg.local_update_box_i.z());
        RCLCPP_INFO(this->get_logger(),
                    "ROG-Map map_size_d=[%.2f,%.2f,%.2f], virtual ground=%.2f, ceil=%.2f, "
                    "l_free=%.3f, l_occ=%.3f",
                    cfg.map_size_d.x(), cfg.map_size_d.y(), cfg.map_size_d.z(),
                    cfg.virtual_ground_height, cfg.virtual_ceil_height, cfg.l_free, cfg.l_occ);

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

        // Point-LIO 的 /cloud_registered 已经在世界系(map)下，无需再做坐标变换。
        // 若改订阅 /cloud_registered_body(body系)，则需要 q * p + t 转到世界系。
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud_world);

        if (cloud_world->empty()) {
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

        // 更新地图（点云与里程计同处世界系）
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

        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = grid_width_;
        grid_msg.info.height = grid_height_;

        // 原点对齐到 ROG-Map 的 cell 边界(ORIGIN_AT_CORNER: cell i 覆盖 [i*res, (i+1)*res))，
        // 使栅格 cell 与地图 cell 一一对应，避免机器人移动时边缘抖动。grid_width_ 为偶数。
        const int ix0 = static_cast<int>(std::floor(current_pose_.position.x / resolution_)) - grid_width_ / 2;
        const int iy0 = static_cast<int>(std::floor(current_pose_.position.y / resolution_)) - grid_height_ / 2;
        grid_msg.info.origin.position.x = ix0 * resolution_;
        grid_msg.info.origin.position.y = iy0 * resolution_;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        // ProjectionLayer: 窗口 = 输出栅格各边多 1 格(奇数)，起点与输出栅格重合，
        // 因此输出栅格 (i,j) 对应 layer 视图 (i,j)，索引 j*layer_w + i。
        const bool layer_ready = projection_layer_en_ && updateProjectionLayer(ix0, iy0);
        const bool use_layer = layer_ready && projection_use_for_output_;
        const int layer_w = grid_width_ + 1;

        // 沿 z 在 [sensor_z + proj_z_min, sensor_z + proj_z_max] 内取样，投影为2D
        const double z_lo = current_pose_.position.z + proj_z_min_;
        const int nz = std::max(1, static_cast<int>(std::ceil((proj_z_max_ - proj_z_min_) / resolution_)));

        grid_msg.data.resize(static_cast<size_t>(grid_width_) * grid_height_);
        int occupied_count = 0;
        int free_count = 0;
        int unknown_count = 0;

        pcl::PointCloud<pcl::PointXYZ> obstacle_map;  // 占据格展开后的点，map系
        const float obstacle_z = static_cast<float>(current_pose_.position.z + obstacle_cloud_z_);

        for (int j = 0; j < grid_height_; j++) {
            for (int i = 0; i < grid_width_; i++) {
                const double wx = grid_msg.info.origin.position.x + (i + 0.5) * resolution_;
                const double wy = grid_msg.info.origin.position.y + (j + 0.5) * resolution_;
                const int idx = i + j * grid_width_;

                bool occ = false;
                bool free = false;
                if (use_layer) {
                    const CellType type = projection_layer_->cells()[static_cast<size_t>(j) * layer_w + i].type;
                    occ = (type == CellType::OCCUPIED);
                    free = (type == CellType::FREE || type == CellType::PASSABLE);
                } else {
                    for (int k = 0; k < nz && !occ; k++) {
                        const Vec3f pos(wx, wy, z_lo + (k + 0.5) * resolution_);
                        if (rog_map_->isOccupied(pos)) {
                            occ = true;
                        } else if (rog_map_->isKnownFree(pos)) {
                            free = true;
                        }
                    }
                }

                if (occ) {
                    grid_msg.data[idx] = 100;
                    occupied_count++;
                    for (int sy = 0; sy < obstacle_sub_n_; sy++) {
                        for (int sx = 0; sx < obstacle_sub_n_; sx++) {
                            const double ox = (sx + 0.5) / obstacle_sub_n_ - 0.5;
                            const double oy = (sy + 0.5) / obstacle_sub_n_ - 0.5;
                            obstacle_map.push_back(pcl::PointXYZ(static_cast<float>(wx + ox * resolution_),
                                                                 static_cast<float>(wy + oy * resolution_),
                                                                 obstacle_z));
                        }
                    }
                } else if (free) {
                    grid_msg.data[idx] = 0;
                    free_count++;
                } else {
                    grid_msg.data[idx] = -1;
                    unknown_count++;
                }
            }
        }

        grid_pub_->publish(grid_msg);
        if (publish_obstacle_cloud_) {
            publishObstacleCloud(obstacle_map);
        }
        if (publish_cloud_3d_ && cloud_3d_pub_->get_subscription_count() > 0) {
            publishCloud3D(grid_msg.info.origin.position.x, grid_msg.info.origin.position.y);
        }
        if (layer_ready && projection_pub_->get_subscription_count() > 0) {
            publishProjectionDebug(grid_msg.info.origin.position.x, grid_msg.info.origin.position.y);
        }

        pub_count_++;
        if (pub_count_ % 50 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "Published grid: Occ=%d, Free=%d, Unk=%d, Pos=[%.2f, %.2f]",
                       occupied_count, free_count, unknown_count,
                       current_pose_.position.x, current_pose_.position.y);
        }
    }

    // 用当前 3D 占据地图更新 ProjectionLayer。成功返回 true；出错时关闭该层并报错，不影响原有输出。
    bool updateProjectionLayer(int ix0, int iy0) {
        const int w = grid_width_ + 1;   // 必须为奇数(ProjectionLayer 的要求)，grid_width_ 是偶数
        const int h = grid_height_ + 1;
        const Eigen::Vector2i min_id(ix0, iy0);
        const Eigen::Vector2d origin(ix0 * resolution_, iy0 * resolution_);  // 窗口左下角(ORIGIN_AT_CORNER)

        int z_min_id = 0;
        int z_max_id = 0;
        rog_map_->scanZRange(current_pose_.position.z + layer_scan_z_min_,
                             current_pose_.position.z + layer_scan_z_max_, z_min_id, z_max_id);
        auto scanner = [this, min_id, z_min_id, z_max_id](int mx, int my) {
            return rog_map_->scanColumn(min_id.x() + mx, min_id.y() + my, z_min_id, z_max_id);
        };

        try {
            const double now = rog_map_->getSystemWalltimeNow();
            ProjectionUpdateStats stats;
            projection_layer_->syncSlidingWindow(w, h, resolution_, min_id, origin, layer_cfg_);
            projection_layer_->updateFull(w, h, resolution_, origin, now, layer_cfg_, scanner, &stats);
            if (layer_cfg_.obstacle_hold_time > 0.0) {
                projection_layer_->advanceObstacleClearance(now, layer_cfg_);
            }
            if (++layer_log_counter_ % 50 == 0) {
                RCLCPP_INFO(this->get_logger(),
                            "ProjectionLayer: %.1f ms | thin_surface=%.0f wall=%.0f tunnel=%.0f ambiguous=%.0f "
                            "empty=%.0f insufficient=%.0f",
                            stats.update_full_time_ms, stats.thin_surface_count, stats.vertical_wall_count,
                            stats.hollow_tunnel_count, stats.ambiguous_occupied_count, stats.empty_column_count,
                            stats.insufficient_observation_count);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "ProjectionLayer update failed, disabling it: %s", e.what());
            projection_layer_en_ = false;
            projection_use_for_output_ = false;
            return false;
        }
        return projection_layer_->cells().size() == static_cast<size_t>(w) * h;
    }

    // 发布 ProjectionLayer 的分类结果(调试用，frame=map)。数值含义：
    //  -1 未知 | 0 空闲 | 20 可通行-薄表面(地面/坡面) | 70 可通行-空心隧道(门洞) | 40 可通行-其他
    //  100 障碍-竖直墙/其他 | 90 障碍-不明确
    void publishProjectionDebug(double origin_x, double origin_y) {
        const int w = grid_width_ + 1;
        const int h = grid_height_ + 1;
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.frame_id = frame_id_;
        msg.header.stamp = this->now();
        msg.info.resolution = resolution_;
        msg.info.width = w;
        msg.info.height = h;
        msg.info.origin.position.x = origin_x;
        msg.info.origin.position.y = origin_y;
        msg.info.origin.orientation.w = 1.0;
        msg.data.resize(static_cast<size_t>(w) * h);
        const auto& cells = projection_layer_->cells();
        for (size_t n = 0; n < msg.data.size(); n++) {
            const CellData& c = cells[n];
            int8_t v = -1;
            switch (c.type) {
                case CellType::FREE:
                    v = 0;
                    break;
                case CellType::PASSABLE:
                    v = (c.raw_reason == ProjectionClassReason::HOLLOW_TUNNEL) ? 70
                        : (c.raw_reason == ProjectionClassReason::THIN_SURFACE) ? 20 : 40;
                    break;
                case CellType::OCCUPIED:
                    v = (c.raw_reason == ProjectionClassReason::AMBIGUOUS_OCCUPIED) ? 90 : 100;
                    break;
                default:
                    v = -1;
                    break;
            }
            msg.data[n] = v;
        }
        projection_pub_->publish(msg);
    }

    // 发布带真实高度的占据体素中心(map系)，范围与2D栅格窗口一致，高度为 [z_min, z_max] 相对雷达。
    void publishCloud3D(double origin_x, double origin_y) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        const double z_lo = current_pose_.position.z + cloud_3d_z_min_;
        const int nz = std::max(1, static_cast<int>(std::ceil((cloud_3d_z_max_ - cloud_3d_z_min_) / resolution_)));
        for (int j = 0; j < grid_height_; j++) {
            for (int i = 0; i < grid_width_; i++) {
                const double wx = origin_x + (i + 0.5) * resolution_;
                const double wy = origin_y + (j + 0.5) * resolution_;
                for (int k = 0; k < nz; k++) {
                    const double wz = z_lo + (k + 0.5) * resolution_;
                    if (rog_map_->isOccupied(Vec3f(wx, wy, wz))) {
                        cloud.push_back(pcl::PointXYZ(static_cast<float>(wx), static_cast<float>(wy),
                                                      static_cast<float>(wz)));
                    }
                }
            }
        }
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = this->now();
        cloud_3d_pub_->publish(msg);
    }

    // 把 map 系的障碍点转到底盘系并发布。
    void publishObstacleCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud_map) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            // p_base = T * p_map；取最新的 TF，并用它的时间戳，保证下游按同一时刻查 TF 能查到
            tf = tf_buffer_->lookupTransform(obstacle_cloud_frame_, frame_id_, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Skip obstacle cloud, TF %s <- %s unavailable: %s",
                                 obstacle_cloud_frame_.c_str(), frame_id_.c_str(), ex.what());
            return;
        }

        const auto& r = tf.transform.rotation;
        const auto& tr = tf.transform.translation;
        const Eigen::Quaternionf q(static_cast<float>(r.w), static_cast<float>(r.x),
                                   static_cast<float>(r.y), static_cast<float>(r.z));
        const Eigen::Vector3f t(static_cast<float>(tr.x), static_cast<float>(tr.y), static_cast<float>(tr.z));

        pcl::PointCloud<pcl::PointXYZ> cloud_base;
        cloud_base.reserve(cloud_map.size());
        for (const auto& p : cloud_map) {
            const Eigen::Vector3f pb = q.normalized() * Eigen::Vector3f(p.x, p.y, p.z) + t;
            cloud_base.push_back(pcl::PointXYZ(pb.x(), pb.y(), pb.z()));
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud_base, msg);
        msg.header.frame_id = obstacle_cloud_frame_;
        msg.header.stamp = tf.header.stamp;
        obstacle_pub_->publish(msg);  // 空点云也发布，表示当前无障碍
    }

    // ROS接口
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_3d_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr projection_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROG-Map实例
    std::shared_ptr<SimpleROGMap> rog_map_;

    // 状态
    geometry_msgs::msg::Pose current_pose_;
    bool has_odom_;
    long cloud_count_ = 0;
    int pub_count_ = 0;

    // 参数
    double resolution_;
    double proj_z_min_;
    double proj_z_max_;
    int grid_width_;
    int grid_height_;
    std::string frame_id_;
    bool publish_obstacle_cloud_{true};
    std::string obstacle_cloud_frame_;
    double obstacle_cloud_z_{-0.2};
    int obstacle_sub_n_{1};
    bool publish_cloud_3d_{true};
    std::unique_ptr<ProjectionLayer> projection_layer_;
    ProjectionLayerConfig layer_cfg_;
    bool projection_layer_en_{false};
    bool projection_use_for_output_{false};
    double layer_scan_z_min_{-0.25};
    double layer_scan_z_max_{1.2};
    int layer_log_counter_{0};
    double cloud_3d_z_min_{-0.9};
    double cloud_3d_z_max_{1.2};
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
