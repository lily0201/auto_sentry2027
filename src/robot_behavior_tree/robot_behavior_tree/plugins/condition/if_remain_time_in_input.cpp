#include <string>

#include "robot_behavior_tree/plugins/condition/if_remain_time_in_input.hpp"

namespace nav2_behavior_tree
{

    IfRemainTimeInInputCondition::IfRemainTimeInInputCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          high_time(420),
          low_time(300),
          is_in_inputtime(false)
    {
        getInput("high_time", high_time);
        getInput("low_time", low_time);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        remaintime_sub_= node_->create_subscription<rm_interfaces::msg::Gamestatus>(
            "/robot/gamestatus",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IfRemainTimeInInputCondition::remaintimeCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus IfRemainTimeInInputCondition::tick()
    {
        getInput("high_time", high_time);
        getInput("low_time", low_time);
        callback_group_executor_.spin_some();
        if (is_in_inputtime)
        {
            RCLCPP_INFO(node_->get_logger(), "剩余时间在 %d ~ %d 之间", high_time, low_time);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    void IfRemainTimeInInputCondition::remaintimeCallback(rm_interfaces::msg::Gamestatus::SharedPtr msg)
    {
        if(msg->stage_remain_time <= high_time && msg->stage_remain_time > low_time)
        {
            is_in_inputtime = true;
        }
        else 
            is_in_inputtime = false;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfRemainTimeInInputCondition>("IfRemainTimeInInput");
}
