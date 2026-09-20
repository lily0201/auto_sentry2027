#include <string>

#include "robot_behavior_tree/plugins/action/update_goal.hpp"

namespace nav2_behavior_tree
{
    UpdateGoalAction::UpdateGoalAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          position_x(0.0),
          position_y(0.0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        getInput("position_x",position_x);
        getInput("position_y",position_y);

        config().blackboard->get("if_dafu", if_dafu);
        config().blackboard->get("dafu_yaw_angle", dafu_yaw_angle);

        truncate_path_info_pub_ = node_->create_publisher<rm_interfaces::msg::TruncatePathInfo>(
            "/robot/truncated_path_info", 10);
    }

    BT::NodeStatus UpdateGoalAction::tick()
    {
        getInput("position_x",position_x);
        getInput("position_y",position_y);
        config().blackboard->get("if_dafu", if_dafu);
        config().blackboard->get("dafu_yaw_angle", dafu_yaw_angle);

        pose.header.stamp = node_->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = position_x;
        pose.pose.position.y = position_y;
        pose.pose.position.z = 0.0;

        if(if_dafu) //打符模式下需要控制特定目标点转向
        {
            // 生成基于yaw角度的旋转四元数（围绕Z轴）
            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, dafu_yaw_angle); // 只有绕Z轴旋转，pitch和roll都为0

            pose.pose.orientation.x = quat.x();
            pose.pose.orientation.y = quat.y();
            pose.pose.orientation.z = quat.z();
            pose.pose.orientation.w = quat.w();
            RCLCPP_INFO(node_->get_logger(), "打符模式pose.orientation: %lf, %lf, %lf, %lf",
                quat.x(), quat.y(), quat.z(), quat.w());
        }
        else
        {
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
        }

        setOutput<geometry_msgs::msg::PoseStamped>("goal", pose);
        config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal", pose);

        distance = 0.0;
        truncate_path_info_msg_.distance = distance;
        truncate_path_info_msg_.dafu = if_dafu;
        truncate_path_info_msg_.final_pose.pose.orientation.x = pose.pose.orientation.x;
        truncate_path_info_msg_.final_pose.pose.orientation.y = pose.pose.orientation.y;
        truncate_path_info_msg_.final_pose.pose.orientation.z = pose.pose.orientation.z;
        truncate_path_info_msg_.final_pose.pose.orientation.w = pose.pose.orientation.w;
        truncate_path_info_pub_->publish(truncate_path_info_msg_);

        RCLCPP_INFO(node_->get_logger(), "~~~~~~~~~~~~~~~update_goal: truncate distance: %lf", distance); //设定路径裁剪长度为0
        RCLCPP_INFO(node_->get_logger(), "~~~~~~~~~~~~~~~update_goal:更新目标点: x: %lf, y: %lf", position_x, position_y);

        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::UpdateGoalAction>("UpdateGoal");
}
