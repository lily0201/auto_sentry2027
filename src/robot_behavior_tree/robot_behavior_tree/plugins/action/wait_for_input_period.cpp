//
// Created by elsa on 25-4-29.
//

#include <string>

#include "robot_behavior_tree/plugins/action/wait_for_input_period.hpp"
namespace nav2_behavior_tree
{
    WaitForInputPeriodAction::WaitForInputPeriodAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          count(0), wait_time(0.0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        // 创建导航动作客户端
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "/navigate_to_pose"); // 导航动作的默认话题

        config().blackboard->get<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
        getInput("wait_time", wait_time);

        count_size = wait_time * 1000 / bt_loop_duration_.count();
    }

    BT::NodeStatus WaitForInputPeriodAction::tick()
    {
        config().blackboard->get<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
        getInput("wait_time", wait_time);
        count_size = wait_time * 1000 / bt_loop_duration_.count();

        if(count >= count_size)
        {
            count = 0;
            if_in_mode_changing = false;
            config().blackboard->set<bool>("if_in_mode_changing", if_in_mode_changing);
            RCLCPP_INFO(node_->get_logger(), "等待时间结束，等待%lf秒", wait_time);
        }
        else {
            if_in_mode_changing = true;
            config().blackboard->set<bool>("if_in_mode_changing", if_in_mode_changing);
            cancel_all_goals();
        }
        count++;
        return BT::NodeStatus::SUCCESS;
    }

    // 取消所有导航目标
    void WaitForInputPeriodAction::cancel_all_goals()
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

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::WaitForInputPeriodAction>("WaitForInputPeriod");
}