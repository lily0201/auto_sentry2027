//
// Created by elsa on 25-2-19.
//

#include "pointcloud_merge/pointcloud_merge.hpp"

LidarMergeNode::LidarMergeNode(std::string node_name)
    : Node("pointcloud_merge_node")
{
    RCLCPP_INFO(this->get_logger(), "%s节点初始化.", node_name.c_str());
    // 声明参数
    this->declare_parameter("lidar_topic_1", "/livox/lidar_192_168_1_187");
    this->declare_parameter("lidar_topic_2", "/livox/lidar_192_168_1_104");
    this->declare_parameter("OutPut_lidar_topic", "/livox/lidar");
    this->declare_parameter("use_voxel_grid_filter", true);
    this->declare_parameter("voxel_leaf_size", 0.2);
    this->declare_parameter("main_livox_frame_id", "livox_192_168_1_104");

    lidar_flag = 0;
    this->get_parameter("lidar_topic_1", lidar_topic_1_);
    this->get_parameter("lidar_topic_2", lidar_topic_2_);
    this->get_parameter("OutPut_lidar_topic", OutPut_lidar_topic_);
    this->get_parameter("use_voxel_grid_filter", use_voxel_grid_filter_);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
    this->get_parameter("main_livox_frame_id", main_livox_frame_id_);

    voxel_grid_filter_1.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid_filter_2.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

    //精确时间同步
    lidar_subscriber_1.subscribe(this, lidar_topic_1_, rmw_qos_profile_default);
    lidar_subscriber_2.subscribe(this, lidar_topic_2_, rmw_qos_profile_default);

    lidar_sync_ = std::make_shared<exact_synchronizer_lidar>(exact_policy_lidar(10),
                                                             lidar_subscriber_1, lidar_subscriber_2);
    lidar_sync_->registerCallback(std::bind(&LidarMergeNode::lidarCallback, this, _1, _2));
    lidar_sync_->setMaxIntervalDuration(std::chrono::milliseconds(1));
    // 订阅lidar_flag
    lidar_flag_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("lidar_flag", 10,
                                                                             std::bind(
                                                                                 &LidarMergeNode::lidarFlagCallback,
                                                                                 this, _1));

    output_lidar_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(OutPut_lidar_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "%s节点初始化成功.", node_name.c_str());
}
void LidarMergeNode::point_merge(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_in,
                                       livox_ros_driver2::msg::CustomMsg::SharedPtr& lidar_out)
{
    for (auto& point : lidar_in->points)
    {
        //注释内容都为测试计算时间用
        // auto begin=rclcpp::Clock().now();
        // int32_t begin_second=begin.nanoseconds();

        livox_ros_driver2::msg::CustomPoint point_msg;
        // auto tf=rclcpp::Clock().now();
        // int32_t tf_second=tf.nanoseconds();
        // RCLCPP_INFO(this->get_logger(), "mat use %d",tf_second-begin_second,"s");

        point_msg.x = point.x;
        point_msg.y = point.y;
        point_msg.z = point.z;
        point_msg.reflectivity = point.reflectivity;
        point_msg.tag = point.tag;
        point_msg.line = point.line;
        point_msg.offset_time = point.offset_time;
        // auto filter=rclcpp::Clock().now();
        // int32_t filter_second=filter.nanoseconds();
        // RCLCPP_INFO(this->get_logger(), "give value use %d",filter_second-tf_second,"s");

        lidar_out->points.push_back(point_msg);
        // auto end=rclcpp::Clock().now();
        // int32_t end_second=end.nanoseconds();
        // RCLCPP_INFO(this->get_logger(), "pushback use %d",end_second-filter_second,"s");
    }
}
void LidarMergeNode::lidarCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_1_pc,
                                   const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_2_pc)
{
    if (lidar_flag == 0)
    { // TODO 优化，这里是否可以只进行一次merge
        livox_ros_driver2::msg::CustomMsg::SharedPtr output_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
        output_msg->points.reserve(lidar_1_pc->point_num + lidar_2_pc->point_num);
        point_merge(lidar_1_pc, output_msg);
        point_merge(lidar_2_pc, output_msg);

        // auto tf=rclcpp::Clock().now();
        // double tf_second=tf.seconds();
        // RCLCPP_INFO(this->get_logger(), "transform finsh use %f",tf_second-begin_second,"s");
        output_msg->header.stamp = lidar_2_pc->header.stamp;
        output_msg->header.frame_id = main_livox_frame_id_;
        output_msg->timebase = lidar_2_pc->timebase;
        output_msg->point_num = output_msg->points.size();
        output_msg->lidar_id = lidar_2_pc->lidar_id;
        output_msg->rsvd = lidar_2_pc->rsvd;
        output_lidar_pub_->publish(*output_msg);
    }
}
void LidarMergeNode::lidarFlagCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "receive lidar_flag: %d", msg->data);
    lidar_flag = msg->data;
}
