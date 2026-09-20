#include <string>

#include "robot_behavior_tree/plugins/action/send_decision.hpp"

namespace nav2_behavior_tree
{

    SendDecisionAction::SendDecisionAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        sentry_blood(0)
    {
        config().blackboard->get<bool>("is_we_are_blue",is_we_are_blue_);
        config().blackboard->get<bool>("select_force_no_resurrection", select_force_no_resurrection);
        config().blackboard->get<uint32_t>("resurrection",resurrection);
        config().blackboard->get<uint32_t>("buy_bullet_at_recovery",buy_bullet_at_recovery);
        config().blackboard->get<uint32_t>("buy_bullet_remote_number", buy_bullet_remote_number);
        config().blackboard->get<uint32_t>("buy_blood_number", buy_blood_number);

        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_);
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        hp_sub_ = node_->create_subscription<rm_interfaces::msg::Hp>(
            "/robot/hp",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&SendDecisionAction::hpCallback, this, std::placeholders::_1),
            sub_option);
        decision_pub_ = node_->create_publisher<rm_interfaces::msg::Decision>("/robot/decision", 10);
    }

    BT::NodeStatus SendDecisionAction::tick()
    {
        config().blackboard->get<bool>("is_we_are_blue",is_we_are_blue_);
        config().blackboard->get<bool>("select_force_no_resurrection", select_force_no_resurrection);
        config().blackboard->get<uint32_t>("resurrection",resurrection);
        config().blackboard->get<uint32_t>("buy_bullet_at_recovery", buy_bullet_at_recovery);
        config().blackboard->get<uint32_t>("buy_bullet_remote_number", buy_bullet_remote_number);
        config().blackboard->get<uint32_t>("buy_blood_number", buy_blood_number);
        callback_group_executor_.spin_some();

        rm_interfaces::msg::Decision decision;
        if(sentry_blood <= 0 && !select_force_no_resurrection)
        {
            if (resurrection == 0)
            {
                decision.resurrection = 0;
                decision.immediate_resurrection = 0;
                RCLCPP_INFO(node_->get_logger(), "send_decision:哨兵不复活");
            }
            else if (resurrection == 1)
            {
                decision.resurrection = 0;
                decision.immediate_resurrection = 1;
                RCLCPP_INFO(node_->get_logger(), "send_decision:哨兵选择立即复活");
            }
            else if (resurrection == 2)
            {
                decision.resurrection = 1;
                decision.immediate_resurrection = 0;
                RCLCPP_INFO(node_->get_logger(), "send_decision:哨兵选择读条复活");
            }
            else
            {
                decision.resurrection = 1;
                decision.immediate_resurrection = 1;
                RCLCPP_INFO(node_->get_logger(), "send_decision:哨兵选择两种复活方式");
            }
        }
        else
        {
            decision.resurrection = 0;
            decision.immediate_resurrection = 0;
        }

        if(is_we_are_blue_){
            decision.sender_id = 107;
        }
        else{
            decision.sender_id = 7;
        }
        decision.buy_bullet_at_recovery = buy_bullet_at_recovery;
        RCLCPP_INFO(node_->get_logger(), "send_decision:在补给区内购买弹量: %d", buy_bullet_at_recovery);
        decision.buy_bullet_outside = buy_bullet_remote_number;
        RCLCPP_INFO(node_->get_logger(), "send_decision:当前远程购买弹量次数: %d", buy_bullet_remote_number);
        decision.buy_blood = buy_blood_number;
        RCLCPP_INFO(node_->get_logger(), "send_decision:当前远程买血量次数: %d", buy_blood_number);
        decision_pub_->publish(decision);

        return BT::NodeStatus::SUCCESS;
    }

    void SendDecisionAction::hpCallback(rm_interfaces::msg::Hp::SharedPtr msg)
    {
        if(is_we_are_blue_)
            sentry_blood = msg->blue_7_robot_hp;
        else
            sentry_blood = msg->red_7_robot_hp;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SendDecisionAction>("SendDecision");
}
