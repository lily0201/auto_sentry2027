#include <string>

#include "robot_behavior_tree/plugins/condition/if_robot_stuck.hpp"

namespace nav2_behavior_tree
{
    IfRobotStuckCondition::IfRobotStuckCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        average_robot_pos_x_(0),
        average_robot_pos_y_(0),
        robot_pos_x_(0),
        robot_pos_y_(0),
        stuck_threshold_distance_(0),
        stuck_cnt_(0),
        stuck_threshold_cnt_(0)
    {
        getInput("stuck_threshold_distance", stuck_threshold_distance_);
        getInput("stuck_threshold_cnt", stuck_threshold_cnt_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    BT::NodeStatus IfRobotStuckCondition::tick()
    {
        config().blackboard->get<double>("robot_pos_x", robot_pos_x_);
        config().blackboard->get<double>("robot_pos_y", robot_pos_y_);
        // RCLCPP_INFO(node_->get_logger(), "++++++++++++++++++++++++++++++robot_pos_x_ = %lf, robot_pos_y_ = %lf",robot_pos_x_,robot_pos_y_);
        // RCLCPP_INFO(node_->get_logger(), "++++++++++++++++++++++++++++++average_robot_pos_x_ = %lf, average_robot_pos_y_ = %lf",average_robot_pos_x_,average_robot_pos_y_);
        // RCLCPP_INFO(node_->get_logger(), "++++++++++++++++++++++++++++++stuck_threshold_distance_ = %lf",abs(robot_pos_x_-average_robot_pos_x_)+abs(robot_pos_y_-average_robot_pos_y_));
        if(abs(robot_pos_x_-average_robot_pos_x_)+abs(robot_pos_y_-average_robot_pos_y_) > stuck_threshold_distance_)
        {
            // RCLCPP_INFO(node_->get_logger(), "++++++++++++++++++++++++++++++正常移动中。。。");
            average_robot_pos_x_ = robot_pos_x_;
            average_robot_pos_y_ = robot_pos_y_;
            stuck_cnt_=0;
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            stuck_cnt_++;
            // RCLCPP_INFO(node_->get_logger(), "++++++++++++++++++++++++++++++stuck_cnt_ = %d",stuck_cnt_);
            average_robot_pos_x_ = 0.1*robot_pos_x_+0.9*average_robot_pos_x_;
            average_robot_pos_y_ = 0.1*robot_pos_y_+0.9*average_robot_pos_y_;
            if(stuck_cnt_ > stuck_threshold_cnt_)
            {
                RCLCPP_INFO(node_->get_logger(), "+++++++++++机器人卡住");
                stuck_cnt_ = stuck_threshold_cnt_;
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfRobotStuckCondition>("IfRobotStuck");
}
