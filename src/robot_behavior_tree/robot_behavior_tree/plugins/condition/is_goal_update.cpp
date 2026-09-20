#include <string>

#include "robot_behavior_tree/plugins/condition/is_goal_update.hpp"

namespace nav2_behavior_tree
{
    IsGoalUpdateCondition::IsGoalUpdateCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal", last_goal);
    }

    BT::NodeStatus IsGoalUpdateCondition::tick()
    {
        getInput("goal", current_goal);
        if (fabs(current_goal.pose.position.x - last_goal.pose.position.x) > 0.001 || fabs(
            current_goal.pose.position.y - last_goal.pose.position.y) > 0.001)
        {
            RCLCPP_INFO(node_->get_logger(), "前一次目标点: %lf, %lf", last_goal.pose.position.x, last_goal.pose.position.y);
            RCLCPP_INFO(node_->get_logger(), "目标点已更新，新的目标点: %lf, %lf", current_goal.pose.position.x,
                        current_goal.pose.position.y);
            last_goal = current_goal;
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "目标点未更新，当前目标点: %lf, %lf",
            last_goal.pose.position.x, current_goal.pose.position.y);
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsGoalUpdateCondition>("IsGoalUpdate");
}
