//
// Created by elsa on 25-3-15.
//

#include "robot_behavior_tree/plugins/action/follow_aim.hpp"

namespace nav2_behavior_tree
{
    FollowAimAction::FollowAimAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        global_frame_("map"),
        robot_base_frame_("odom_yaw"),
        distance(2.5)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
        if (!node_->has_parameter("transform_tolerance"))
        {
            node_->declare_parameter("transform_tolerance", 0.15);
        }
        node_->get_parameter("transform_tolerance", transform_tolerance_);

        truncate_path_info_pub_ = node_->create_publisher<rm_interfaces::msg::TruncatePathInfo>(
            "/robot/truncated_path_info", 10);

        config().blackboard->get<double>("enemy_position_x", received_enemy_position_.pose.position.x);
        config().blackboard->get<double>("enemy_position_y", received_enemy_position_.pose.position.y);
        config().blackboard->get<double>("enemy_position_x", last_received_enemy_position_.pose.position.x);
        config().blackboard->get<double>("enemy_position_y", last_received_enemy_position_.pose.position.y);
    }

    BT::NodeStatus FollowAimAction::tick()
    {
        //获取map->lidar_base_link的坐标系变换
        if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_,
                                       transform_tolerance_))
        {
            RCLCPP_ERROR(node_->get_logger(), "follow_aim: 没有获取到TF变换");
            return BT::NodeStatus::FAILURE;
        }

        //获取敌人坐标
        config().blackboard->get<double>("enemy_position_x", received_enemy_position_.pose.position.x);
        config().blackboard->get<double>("enemy_position_y", received_enemy_position_.pose.position.y);

        // RCLCPP_INFO(node_->get_logger(), "???????????????received_enemy_position_: %lf, %lf",
        //     received_enemy_position_.pose.position.x, received_enemy_position_.pose.position.y);
        // RCLCPP_INFO(node_->get_logger(), "???!!!!!!!!!!!!last_received_enemy_position_: %lf, %lf",
        //     last_received_enemy_position_.pose.position.x, last_received_enemy_position_.pose.position.y);

        // 两次目标点距离不是太近时才更新新的目标点
        if((received_enemy_position_.pose.position.x - last_received_enemy_position_.pose.position.x) *
            (received_enemy_position_.pose.position.x - last_received_enemy_position_.pose.position.x) +
            (received_enemy_position_.pose.position.y - last_received_enemy_position_.pose.position.y) *
            (received_enemy_position_.pose.position.y - last_received_enemy_position_.pose.position.y) >= 0.2)
        {
            last_received_enemy_position_.pose.position.x = received_enemy_position_.pose.position.x;
            last_received_enemy_position_.pose.position.y = received_enemy_position_.pose.position.y;

            geometry_msgs::msg::TransformStamped odom_to_lidar_base_link =
                tf_->lookupTransform(global_frame_, robot_base_frame_,  rclcpp::Time(), rclcpp::Duration::from_seconds(0.5));
            tf2::doTransform(received_enemy_position_, received_enemy_position_,odom_to_lidar_base_link);
            // 自瞄世界系，只有平移没有旋转，转换到map系
            aim_goal_.header.stamp = node_->get_clock()->now();
            aim_goal_.header.frame_id = "map";
            aim_goal_.pose.position.x = received_enemy_position_.pose.position.x;
            aim_goal_.pose.position.y = received_enemy_position_.pose.position.y;
            aim_goal_.pose.position.z = 0.0;
            aim_goal_.pose.orientation.x = 0.0;
            aim_goal_.pose.orientation.y = 0.0;
            aim_goal_.pose.orientation.z = 0.0;
            aim_goal_.pose.orientation.w = 1.0;

            setOutput<geometry_msgs::msg::PoseStamped>("goal", aim_goal_);
            config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal",aim_goal_); //设定追击目标位置

            // RCLCPP_INFO(node_->get_logger(), "follow_aim 追击目标点: %lf, %lf",
            //     aim_goal_.pose.position.x, aim_goal_.pose.position.y);

            distance = 2.0;
            truncate_path_info_msg_.distance = distance;
            truncate_path_info_msg_.dafu = false;
            truncate_path_info_pub_->publish(truncate_path_info_msg_);
            RCLCPP_INFO(node_->get_logger(), "`````````!!!``````````follow_aim: truncate distance: %lf", distance); //设定路径裁剪长度
        }
        setOutput<geometry_msgs::msg::PoseStamped>("goal", aim_goal_);
        config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal",aim_goal_); //设定追击目标位置

        RCLCPP_INFO(node_->get_logger(), "follow_aim 追击目标点: %lf, %lf",
            aim_goal_.pose.position.x, aim_goal_.pose.position.y);
        config().blackboard->set<double>("enemy_position_x", received_enemy_position_.pose.position.x);
        config().blackboard->set<double>("enemy_position_y", received_enemy_position_.pose.position.y);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::FollowAimAction>("FollowAim");
}