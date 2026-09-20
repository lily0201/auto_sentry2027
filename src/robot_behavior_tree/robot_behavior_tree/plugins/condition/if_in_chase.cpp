//
// Created by elsa on 25-7-6.
//

#include <string>

#include "robot_behavior_tree/plugins/condition/if_in_chase.hpp"

namespace nav2_behavior_tree
{

    IfInChaseCondition::IfInChaseCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        is_in_chase(false), if_can_chase(false), sentry_blood(400)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        enemy_chasing_sub_ = node_->create_subscription<rm_interfaces::msg::EnemyChasing>(
            "/robot/enemy_chasing",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IfInChaseCondition::EnemyChasingCallback, this, std::placeholders::_1),
            sub_option);
        hp_sub_ = node_->create_subscription<rm_interfaces::msg::Hp>(
            "/robot/hp",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IfInChaseCondition::HpCallback, this, std::placeholders::_1),
            sub_option);

        config().blackboard->set<bool>("is_in_chase", is_in_chase);
        config().blackboard->get<double>("chasing_higher_limit", chasing_higher_limit);
        config().blackboard->get<double>("chasing_lower_limit", chasing_lower_limit);
        config().blackboard->get("if_dafu", if_dafu);
        config().blackboard->get("is_we_are_blue", is_we_are_blue);
    }

    BT::NodeStatus IfInChaseCondition::tick()
    {
        config().blackboard->get("if_dafu", if_dafu);
        callback_group_executor_.spin_some();

        //排除存在高低差的情况
        if(if_can_chase && aim_goal_.pose.position.z > chasing_lower_limit &&
            aim_goal_.pose.position.z < chasing_higher_limit)
            is_in_chase = true;
        else
            is_in_chase = false;

        //打符模式下禁用追击
        if(if_dafu)
            is_in_chase = false;

        //血量少于280不追击
        if (sentry_blood <= 280)
            is_in_chase = false;

        if (is_in_chase)
        {
            RCLCPP_INFO(node_->get_logger(), "机器人在追击状态");
            config().blackboard->set<bool>("is_in_chase", is_in_chase);
            config().blackboard->set<double>("enemy_position_x", aim_goal_.pose.position.x);
            config().blackboard->set<double>("enemy_position_y", aim_goal_.pose.position.y);
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "机器人不在追击状态");
        //敌人坐标清零
        // config().blackboard->set<double>("enemy_position_x", 0.0);
        // config().blackboard->set<double>("enemy_position_y", 0.0);
        config().blackboard->set<bool>("is_in_chase", is_in_chase);

        return BT::NodeStatus::FAILURE;
    }

    void IfInChaseCondition::EnemyChasingCallback(rm_interfaces::msg::EnemyChasing::SharedPtr msg)
    {
        if_can_chase = msg->is_in_chase;
        aim_goal_.pose.position.x = msg->aim_goal.pose.position.x;
        aim_goal_.pose.position.y = msg->aim_goal.pose.position.y;
        aim_goal_.pose.position.z = msg->aim_goal.pose.position.z;
    }

    void IfInChaseCondition::HpCallback(rm_interfaces::msg::Hp::SharedPtr msg) {
        if (is_we_are_blue)
            sentry_blood = msg->blue_7_robot_hp;
        else
            sentry_blood = msg->red_7_robot_hp;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfInChaseCondition>("IfInChase");
}
