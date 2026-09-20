//
// Created by elsa on 25-2-19.
//

#ifndef POINTCLOUD_MERGE_HPP
#define POINTCLOUD_MERGE_HPP

#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int32.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <vector>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include"livox_ros_driver2/msg/custom_msg.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

/**
 * @brief 点云合并节点，用于同步和合并两个激光雷达的点云数据
 *
 * 该节点实现以下功能：
 * - 订阅两个激光雷达的点云数据并进行精确时间同步
 * - 根据雷达状态标志(lidar_flag)决定是否合并点云
 * - 应用体素网格滤波减少点云数据量
 * - 发布合并后的点云数据
 */
class LidarMergeNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数，初始化点云合并节点
     * @param node_name 节点名称
     * @note 声明并获取ROS参数，初始化订阅器、发布器和同步器
     */
    explicit LidarMergeNode(std::string node_name);

    /**
     * @brief 点云合并函数
     * @param lidar_in 输入点云消息
     * @param lidar_out 输出点云消息
     * @note 将输入点云中的点复制到输出点云消息
     */
    void point_merge(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_in,
                     livox_ros_driver2::msg::CustomMsg::SharedPtr& lidar_out);

    /**
     * @brief 激光雷达数据同步回调函数
     * @param lidar_1_pc 雷达1点云消息
     * @param lidar_2_pc 雷达2点云消息
     * @note 当双雷达正常工作时合并点云数据并发布
     */
    void lidarCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_1_pc,
                       const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_2_pc);

    /**
     * @brief 雷达状态标志回调函数
     * @param msg 雷达状态标志消息
     * @note 更新当前雷达状态标志，决定是否合并点云
     */
    void lidarFlagCallback(const std_msgs::msg::Int32::SharedPtr msg);

private:
    /** @brief 雷达状态标志 (0:双雷达正常, 1:仅雷达1正常, 2:仅雷达2正常, 3:双雷达故障) */
    int lidar_flag;
    /** @brief 存储点云数据的缓存 */
    std::vector<livox_ros_driver2::msg::CustomMsg::ConstPtr> livox_data;
    /** @brief 是否使用体素网格滤波标志 */
    bool use_voxel_grid_filter_;
    /** @brief 体素网格滤波器叶大小 */
    double voxel_leaf_size_;
    /** @brief 雷达1点云的体素网格滤波器 */
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_1;
    /** @brief 雷达2点云的体素网格滤波器 */
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_2;
    /** @brief 雷达1点云话题名称 */
    std::string lidar_topic_1_;
    /** @brief 雷达2点云话题名称 */
    std::string lidar_topic_2_;
    /** @brief 输出点云话题名称 */
    std::string OutPut_lidar_topic_;
    /** @brief 主雷达坐标系ID */
    std::string main_livox_frame_id_;

    /** @brief 合并后的点云发布器 */
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr output_lidar_pub_;

    /** @brief 时间同步策略类型定义 */
    typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver2::msg::CustomMsg,
                                                            livox_ros_driver2::msg::CustomMsg> exact_policy_lidar;
    /** @brief 时间同步器类型定义 */
    typedef message_filters::Synchronizer<exact_policy_lidar> exact_synchronizer_lidar;

    /** @brief 雷达1点云订阅器 */
    message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar_subscriber_1;
    /** @brief 雷达2点云订阅器 */
    message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar_subscriber_2;

    /** @brief 雷达状态标志订阅器 */
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lidar_flag_subscriber_;

    /** @brief 点云同步器实例 */
    std::shared_ptr<exact_synchronizer_lidar> lidar_sync_;
};

#endif //POINTCLOUD_MERGE_HPP
