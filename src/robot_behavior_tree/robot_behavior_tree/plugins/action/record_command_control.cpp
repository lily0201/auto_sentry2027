//
// Created by elsa on 25-4-28.
//

#include <string>

#include "robot_behavior_tree/plugins/action/record_command_control.hpp"

namespace nav2_behavior_tree
{
    RecordCommandControlAction::RecordCommandControlAction(
        const std::string& action_name,
        const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(action_name, conf),
          cmd_keyboard(0), in_command(false), select_force_no_resurrection(false), blackboard_cmd_keyboard(""),
          count(0), if_force_move(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        mapcommand_sub_ = node_->create_subscription<rm_interfaces::msg::Mapcommand>(
            "/robot/mapcommand",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RecordCommandControlAction::mapcommandCallback, this, std::placeholders::_1),
            sub_option);

        bt_loop_duration_ = config().blackboard->get<std::chrono::milliseconds>("bt_loop_duration");
        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);
        config().blackboard->get<bool>("select_if_go_fort", select_if_go_fort);
        config().blackboard->get<bool>("select_force_no_resurrection", select_force_no_resurrection);
        config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", last_goal);

        count_size = 2 * 1000 / bt_loop_duration_.count(); // 2s
    }

    BT::NodeStatus RecordCommandControlAction::tick()
    {
        config().blackboard->get<bool>("is_we_are_blue", is_we_are_blue);
        config().blackboard->get<bool>("select_if_go_fort", select_if_go_fort);
        config().blackboard->get<bool>("select_force_no_resurrection", select_force_no_resurrection);
        config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", last_goal);

        //给current_goal赋初始值为last_goal
        current_goal.header.stamp = node_->now();
        current_goal.header.frame_id = "map";
        current_goal.pose.position.x = last_goal.pose.position.x;
        current_goal.pose.position.y = last_goal.pose.position.y;
        current_goal.pose.position.z = 0.0;
        current_goal.pose.orientation.x = 0.0;
        current_goal.pose.orientation.y = 0.0;
        current_goal.pose.orientation.z = 0.0;
        current_goal.pose.orientation.w = 1.0;

        //获取云台手发来的新消息并做处理
        callback_group_executor_.spin_some();

        if (if_force_move) {
            count++;
            if (count >= count_size) {
                count = 0;
                if_force_move = false;
                in_command = false;
                RCLCPP_INFO(node_->get_logger(), "强制移动结束");
            }
        }

        config().blackboard->set<std::string>("cmd_keyboard", blackboard_cmd_keyboard);
        config().blackboard->set<bool>("in_command", in_command);
        config().blackboard->set<bool>("select_if_go_fort", select_if_go_fort);
        config().blackboard->set<bool>("select_force_no_resurrection", select_force_no_resurrection);

        RCLCPP_INFO(node_->get_logger(), "record command control successfully");

        return BT::NodeStatus::SUCCESS;
    }

    void RecordCommandControlAction::mapcommandCallback(rm_interfaces::msg::Mapcommand::SharedPtr msg)
    {
        mapcommand_.cmd_keyboard = msg->cmd_keyboard;
        mapcommand_.target_position_x = msg->target_position_x;
        mapcommand_.target_position_y = msg->target_position_y;

        //重复判断
        if (fabs(last_goal.pose.position.x - mapcommand_.target_position_x) > 0.01 ||
            fabs(last_goal.pose.position.y == mapcommand_.target_position_y) > 0.01)
        {
            //根据云台手发送的按键消息选择不同的执行任务
            if (mapcommand_.cmd_keyboard == 'H' || mapcommand_.cmd_keyboard == 'h')
            {
                blackboard_cmd_keyboard = "H";
                select_if_go_fort = !select_if_go_fort;
                in_command = false;
                if (select_if_go_fort)
                    RCLCPP_INFO(node_->get_logger(), "当前选择上堡垒");
                else
                    RCLCPP_INFO(node_->get_logger(), "当前选择不上堡垒");
            }
            else if (mapcommand_.cmd_keyboard == 'T' || mapcommand_.cmd_keyboard == 't')
            {
                blackboard_cmd_keyboard = "T";
                select_force_no_resurrection = true;
                in_command = false;
                RCLCPP_INFO(node_->get_logger(), "当前选择强制不复活");
            }
            else if (mapcommand_.cmd_keyboard == 'G' || mapcommand_.cmd_keyboard == 'g')
            {
                blackboard_cmd_keyboard = "G";
                in_command = true;
                RCLCPP_INFO(node_->get_logger(), "哨兵一键回家补血");
            }
            else if (mapcommand_.cmd_keyboard == 0 && mapcommand_.target_position_x != 0
                && mapcommand_.target_position_y != 0)
            {
                blackboard_cmd_keyboard = "0";
                in_command = true;
                current_goal.header.stamp = node_->now();
                current_goal.pose.position.x = mapcommand_.target_position_y - 6.93; //赛前减去启动原点
                current_goal.pose.position.y = 24.6 - mapcommand_.target_position_x;
                setOutput<geometry_msgs::msg::PoseStamped>("goal", current_goal);

                RCLCPP_INFO(node_->get_logger(), "****************更新云台手指定目标点: x: %lf, y: %lf******************",
                            current_goal.pose.position.x, current_goal.pose.position.y);
            }
            else if (mapcommand_.cmd_keyboard == 'W' || mapcommand_.cmd_keyboard == 'w' ||
                mapcommand_.cmd_keyboard == 'A' || mapcommand_.cmd_keyboard == 'a' ||
                mapcommand_.cmd_keyboard == 'S' || mapcommand_.cmd_keyboard == 's' ||
                mapcommand_.cmd_keyboard == 'D' || mapcommand_.cmd_keyboard == 'd')
            {
                blackboard_cmd_keyboard = "W";
                in_command = true;
                if_force_move = true;
                RCLCPP_INFO(node_->get_logger(), "*****************哨兵强制移动中，cmd_keyboard: %hhu****************",
                    mapcommand_.cmd_keyboard);
            }
            else if (mapcommand_.cmd_keyboard == 'Y' || mapcommand_.cmd_keyboard == 'y')
            {
                blackboard_cmd_keyboard = "Y";
                in_command = false;
                RCLCPP_INFO(node_->get_logger(), "强制退出云台手控制模式");
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "**************云台手未发送指令/指令错误***************");
            }
            cmd_keyboard = mapcommand_.cmd_keyboard;
        }
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RecordCommandControlAction>("RecordCommandControl");
}
