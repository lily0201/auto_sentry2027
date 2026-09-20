#include <string>

#include "robot_behavior_tree/plugins/condition/if_appoint_robot_in_target_area.hpp"

namespace nav2_behavior_tree
{

    IfAppointRobotInTargetAreaCondition::IfAppointRobotInTargetAreaCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          is_in_target_area_(false),
          is_our(0), robotname(0), position_x(0.0), position_y(0.0), range_x(0.0), range_y(0.0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        robotposition_sub_ = node_->create_subscription<rm_interfaces::msg::Robotposition>(
            "/robot/robotposition",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IfAppointRobotInTargetAreaCondition::robotpositionCallback, this, std::placeholders::_1),
            sub_option);

        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue_);
        getInput("is_our",is_our);
        getInput("RobotName",robotname);
        getInput("position_x",position_x);
        getInput("position_y",position_y);
        getInput("range_x",range_x);
        getInput("range_y",range_y);
    }

    BT::NodeStatus IfAppointRobotInTargetAreaCondition::tick()
    {
        getInput("is_our",is_our);
        getInput("RobotName",robotname);
        getInput("position_x",position_x);
        getInput("position_y",position_y);
        getInput("range_x",range_x);
        getInput("range_y",range_y);
        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue_);

        callback_group_executor_.spin_some();

        if(is_in_target_area_)
        {
            if(is_our)
                RCLCPP_INFO(node_->get_logger(), "我方%d号机器人在目标点(%lf, %lf)附近", robotname, position_x, position_y);
            else
                RCLCPP_INFO(node_->get_logger(), "敌方%d号机器人在目标点(%lf, %lf)附近", robotname, position_x, position_y);

            return BT::NodeStatus::SUCCESS;
        }
        if(is_our)
            RCLCPP_INFO(node_->get_logger(), "我方%d号机器人不在目标点(%lf, %lf)附近", robotname, position_x, position_y);
        else
            RCLCPP_INFO(node_->get_logger(), "敌方%d号机器人不在目标点(%lf, %lf)附近", robotname, position_x, position_y);

        return BT::NodeStatus::FAILURE;
    }

    void IfAppointRobotInTargetAreaCondition::robotpositionCallback(rm_interfaces::msg::Robotposition::SharedPtr msg)
    {
        if (is_our) //通过裁判系统只能读到己方机器人的坐标
        {
            if (robotname == 1)
            {
                if (abs(msg->hero_x - position_x) < range_x && abs(msg->hero_y - position_y) < range_y)
                    is_in_target_area_ = true;
                else
                    is_in_target_area_ = false;
            }
            else if(robotname == 2)
            {
                if (abs(msg->engineer_x - position_x) < range_x && abs(msg->engineer_y - position_y) < range_y)
                    is_in_target_area_ = true;
                else
                    is_in_target_area_ = false;
            }
            else if(robotname == 3)
            {
                if (abs(msg->standard_3_x - position_x) < range_x && abs(msg->standard_3_y - position_y) < range_y)
                    is_in_target_area_ = true;
                else
                    is_in_target_area_ = false;
            }
            else if(robotname == 4)
            {
                if (abs(msg->standard_4_x - position_x) < range_x && abs(msg->standard_4_y - position_y) < range_y)
                    is_in_target_area_ = true;
                else
                    is_in_target_area_ = false;
            }
        }
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfAppointRobotInTargetAreaCondition>("IfAppointRobotInTargetArea");
}
