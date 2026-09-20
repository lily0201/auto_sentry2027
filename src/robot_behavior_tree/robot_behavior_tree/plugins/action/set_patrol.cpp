#include <string>

#include "robot_behavior_tree/plugins/action/set_patrol.hpp"
namespace nav2_behavior_tree
{
    SetPatrolAction::SetPatrolAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          max_pitch_(0.0),
          min_pitch_(0.0),
          spin(0),
          mode(0),
          patrol(0),
          robot_aim(1)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        sentryinfo_sub_ = node_->create_subscription<rm_interfaces::msg::Sentryinfo>(
            "/robot/sentryinfo",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&SetPatrolAction::sentryinfoCallback, this, std::placeholders::_1),
            sub_option);
        hurtdata_sub_ = node_->create_subscription<rm_interfaces::msg::HurtData>(
            "/robot/hurtdata",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&SetPatrolAction::hurtdataCallback, this, std::placeholders::_1),
            sub_option);

        getInput("max_pitch",max_pitch_);
        getInput("min_pitch",min_pitch_);
        getInput("mode",mode);
        getInput("spin",spin);
        getInput("patrol",patrol);
        getInput("robot_aim",robot_aim);
    }

    BT::NodeStatus SetPatrolAction::tick()
    {
        callback_group_executor_.spin_some();

        getInput("max_pitch",max_pitch_);
        getInput("min_pitch",min_pitch_);
        getInput("mode",mode);
        getInput("spin",spin);
        getInput("patrol",patrol);
        getInput("robot_aim",robot_aim);

        if (is_hurt || !is_out_fight) { //受伤强制小陀螺
            mode = 1;
            spin = 1;
            RCLCPP_INFO(node_->get_logger(), "set_patrol: 受伤强制小陀螺");
        }

        config().blackboard->set<float>("max_pitch",max_pitch_);
        config().blackboard->set<float>("min_pitch",min_pitch_);
        config().blackboard->set<int>("mode",mode);
        config().blackboard->set<int>("spin",spin);
        config().blackboard->set<int>("patrol",patrol);
        config().blackboard->set<int>("robot_aim",robot_aim);

        RCLCPP_INFO(node_->get_logger(), "setting mode: %d, spin: %d, patrol: %d, robot_aim: %d",
            mode, spin, patrol, robot_aim);

        return BT::NodeStatus::SUCCESS;
    }

    void SetPatrolAction::sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg) {
        is_out_fight = msg->if_out_fight;
    }

    void SetPatrolAction::hurtdataCallback(rm_interfaces::msg::HurtData::SharedPtr msg) {
        if(msg->armor_id == 6 && msg->hp_deduction_reason == 6){
            is_hurt = false;
        }
        else {
            is_hurt = true;
            RCLCPP_INFO(node_->get_logger(), "机器人处于受伤状态");
        }
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetPatrolAction>("SetPatrol");
}