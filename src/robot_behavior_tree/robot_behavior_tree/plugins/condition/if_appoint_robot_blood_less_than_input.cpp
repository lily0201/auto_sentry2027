#include <string>

#include "robot_behavior_tree/plugins/condition/if_appoint_robot_blood_less_than_input.hpp"

namespace nav2_behavior_tree
{

    IfAppointRobotBloodLessThanInputCondition::IfAppointRobotBloodLessThanInputCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          is_we_are_blue_(true),
          is_our(0),
          is_health_low_(false)
    {
        is_we_are_blue_ = config().blackboard->get<bool>("is_we_are_blue");
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        robot_hp_sub_ = node_->create_subscription<rm_interfaces::msg::Hp>(
            "/robot/hp",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IfAppointRobotBloodLessThanInputCondition::robothpCallback, this, std::placeholders::_1),
            sub_option);
        getInput("is_our",is_our);
        getInput("RobotName",robotname);
        getInput("Health_threshold",health_threshold);
    }

    BT::NodeStatus IfAppointRobotBloodLessThanInputCondition::tick()
    {
        getInput("is_our",is_our);
        getInput("RobotName",robotname);
        getInput("Health_threshold",health_threshold);
        callback_group_executor_.spin_some();
        if(is_health_low_){
            if(is_our == 0)
                RCLCPP_INFO(node_->get_logger(), "对方%d号生命值低于%d", robotname, health_threshold);
            else
                RCLCPP_INFO(node_->get_logger(), "我方%d号生命值低于%d", robotname, health_threshold);
            return BT::NodeStatus::SUCCESS;
        }
        if(is_our == 0)
                RCLCPP_INFO(node_->get_logger(), "对方%d号生命值高于%d", robotname, health_threshold);
            else
                RCLCPP_INFO(node_->get_logger(), "我方%d号生命值高于%d", robotname, health_threshold);
        return BT::NodeStatus::FAILURE;
    }

    void IfAppointRobotBloodLessThanInputCondition::robothpCallback(rm_interfaces::msg::Hp::SharedPtr msg)
    {
        if(is_we_are_blue_ == true)
        {
            if(is_our == 0)
            {
                if(robotname == 1)
                {
                    is_health_low_ = msg->red_1_robot_hp <= health_threshold;
                }
                if(robotname == 2)
                {
                    is_health_low_ = msg->red_2_robot_hp <= health_threshold;
                }
                if(robotname == 3)
                {
                    is_health_low_ = msg->red_3_robot_hp <= health_threshold;
                }
                if(robotname == 4)
                {
                    is_health_low_ = msg->red_4_robot_hp <= health_threshold;
                }
                if(robotname == 7)
                {
                    is_health_low_ = msg->red_7_robot_hp <= health_threshold;
                }
                if(robotname == 8)
                {
                    is_health_low_ = msg->red_outpost_hp <= health_threshold;
                }
                if(robotname == 9)
                {
                    is_health_low_ = msg->red_base_hp <= health_threshold;
                }
            }
            else
            {
                if(robotname == 1)
                {
                    is_health_low_ = msg->blue_1_robot_hp <= health_threshold;
                }
                if(robotname == 2)
                {
                    is_health_low_ = msg->blue_2_robot_hp <= health_threshold;
                }
                if(robotname == 3)
                {
                    is_health_low_ = msg->blue_3_robot_hp <= health_threshold;
                }
                if(robotname == 4)
                {
                    is_health_low_ = msg->blue_4_robot_hp <= health_threshold;
                }
                if(robotname == 7)
                {
                    is_health_low_ = msg->blue_7_robot_hp <= health_threshold;
                }
                if(robotname == 8)
                {
                    is_health_low_ = msg->blue_outpost_hp <= health_threshold;
                }
                if(robotname == 9)
                {
                    is_health_low_ = msg->blue_base_hp <= health_threshold;
                }
            }
        }
        else
        {
            if(is_our == 1)
            {
                if(robotname == 1)
                {
                    is_health_low_ = msg->red_1_robot_hp <= health_threshold;
                }
                if(robotname == 2)
                {
                    is_health_low_ = msg->red_2_robot_hp <= health_threshold;
                }
                if(robotname == 3)
                {
                    is_health_low_ = msg->red_3_robot_hp <= health_threshold;
                }
                if(robotname == 4)
                {
                    is_health_low_ = msg->red_4_robot_hp <= health_threshold;
                }
                if(robotname == 7)
                {
                    is_health_low_ = msg->red_7_robot_hp <= health_threshold;
                }
                if(robotname == 8)
                {
                    is_health_low_ = msg->red_outpost_hp <= health_threshold;
                }
                if(robotname == 9)
                {
                    is_health_low_ = msg->red_base_hp <= health_threshold;
                }
            }
            else
            {
                if(robotname == 1)
                {
                    is_health_low_ = msg->blue_1_robot_hp <= health_threshold;
                }
                if(robotname == 2)
                {
                    is_health_low_ = msg->blue_2_robot_hp <= health_threshold;
                }
                if(robotname == 3)
                {
                    is_health_low_ = msg->blue_3_robot_hp <= health_threshold;
                }
                if(robotname == 4)
                {
                    is_health_low_ = msg->blue_4_robot_hp <= health_threshold;
                }
                if(robotname == 7)
                {
                    is_health_low_ = msg->blue_7_robot_hp <= health_threshold;
                }
                if(robotname == 8)
                {
                    is_health_low_ = msg->blue_outpost_hp <= health_threshold;
                }
                if(robotname == 9)
                {
                    is_health_low_ = msg->blue_base_hp <= health_threshold;
                }
            }
        }
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfAppointRobotBloodLessThanInputCondition>("IfAppointRobotBloodLessThanInput");
}
