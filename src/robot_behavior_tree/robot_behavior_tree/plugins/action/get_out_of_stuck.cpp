#include <string>

#include "robot_behavior_tree/plugins/action/get_out_of_stuck.hpp"

namespace nav2_behavior_tree
{

    // 取消所有导航目标
    void GetOutOfStuckAction::cancel_all_goals()
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), "导航动作服务器未启动");
            return;
        }

        // 发送取消所有目标的请求（无需目标ID）
        auto future = action_client_->async_cancel_all_goals();

        // // 可选：处理取消结果（异步或同步）
        // auto result = future.get();
        // if (result == rclcpp::action::CancelAllGoalsResult::SUCCESS)
        // {
        //     RCLCPP_INFO(this->get_logger(), "成功取消所有导航目标");
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(), "取消目标失败");
        // }
    }
    GetOutOfStuckAction::GetOutOfStuckAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          is_force_move_complete_(true),
          count(0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        // 创建导航动作客户端
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "/navigate_to_pose"); // 导航动作的默认话题
    }

BT::NodeStatus GetOutOfStuckAction::tick()
{
    config().blackboard->get<bool>("is_force_move_complete", is_force_move_complete_);
    cancel_all_goals();
    if (is_force_move_complete_)
    {
        count++;
        if (count == 3)
        {
            config().blackboard->set<bool>("is_force_move_backward", true);
            RCLCPP_INFO(node_->get_logger(), "GetOutOfStuckAction: is_force_move_backward");
        }
        else if (count == 1)
        {
            config().blackboard->set<bool>("is_force_move_left", true);
            RCLCPP_INFO(node_->get_logger(), "GetOutOfStuckAction: is_force_move_left");
        }
        else if (count == 2)
        {
            config().blackboard->set<bool>("is_force_move_backward", true);
            RCLCPP_INFO(node_->get_logger(), "GetOutOfStuckAction: is_force_move_backward");
        }
        else if (count == 0)
        {
            config().blackboard->set<bool>("is_force_move_right", true);
            RCLCPP_INFO(node_->get_logger(), "GetOutOfStuckAction: is_force_move_right");
        }
        else if (count == 4)
        {
            config().blackboard->set<bool>("is_force_move_backward", true);
            RCLCPP_INFO(node_->get_logger(), "GetOutOfStuckAction: is_force_move_backward");
        }
        else if (count == 5)
        {
            config().blackboard->set<bool>("is_force_move_forward", true);
            RCLCPP_INFO(node_->get_logger(), "GetOutOfStuckAction: is_force_move_forward");
        }
        else
        {
            count = 0;
            config().blackboard->set<bool>("is_force_move_backward", true);
            RCLCPP_INFO(node_->get_logger(), "GetOutOfStuckAction: is_force_move_backward");
        }
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "is_force_move_complete:%d", is_force_move_complete_);
    }
    return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::GetOutOfStuckAction>("GetOutOfStuck");
}
