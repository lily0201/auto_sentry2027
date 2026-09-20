//
// Created by elsa on 25-4-28.
//

#ifndef RECORD_COMMAND_CONTROL_HPP
#define RECORD_COMMAND_CONTROL_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/mapcommand.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 记录云台手控制状态节点，记录来自裁判系统的云台手消息，并执行以下的相应动作：
     * @brief 按键G/g -- 切换哨兵上堡垒模式，默认为上堡垒，每发送一次请求会切换一次
     * @brief 按键T/t -- 切换哨兵是否选择强制不复活，默认为可以复活，选择强制不复活则在该局剩余时间中哨兵都！不！会！再！复！活！
     * @brief 按键B/b -- 哨兵一键回家补血，补血完成后继续刚才正在执行的任务
     * @brief 按键Y/y -- 强制退出云台手控制模式
     * @brief 按键WASD/wasd -- 强制前后左右移动
     * @brief 无按键信息，默认指定目标点导航
     * @brief 由于0x0303消息裁判系统会持续发送，因此需要做重复判断，避免卡死在云台手控制模式内
     */
    class RecordCommandControlAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        RecordCommandControlAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        RecordCommandControlAction() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
            };
        }

    private:
        /**
         * @brief Callback function 接收裁判系统发来的云台手指令消息
         * @param msg Shared pointer to rm_interfaces::msg::Mapcommand message
         */
        void mapcommandCallback(rm_interfaces::msg::Mapcommand::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Mapcommand>::SharedPtr mapcommand_sub_;

        ///接收裁判系统发来的云台手指令消息
        rm_interfaces::msg::Mapcommand mapcommand_;
        ///当前云台手指定目标点
        geometry_msgs::msg::PoseStamped current_goal;
        ///当前存在"goal"blackboard中的上一次目标点
        geometry_msgs::msg::PoseStamped last_goal;

        ///是否是蓝方
        bool is_we_are_blue;
        ///是否在云台手控制状态，只在云台手指定目标点/一键回家补血这种长时间指令时生效
        bool in_command;
        ///云台手选择哨兵是否上堡垒
        bool select_if_go_fort;
        ///云台手选择哨兵是否强制不复活
        bool select_force_no_resurrection;
        ///云台手发送的按键消息，没有则为0
        uint8_t cmd_keyboard;
        ///存在黑板中的按键消息
        std::string blackboard_cmd_keyboard;
        ///机器人无敌判断 & 买血是否成功计数时间阈值
        int count_size;
        int count;
        bool if_force_move;
        ///行为树每次循环需要的时间
        std::chrono::milliseconds bt_loop_duration_;
    };

} // namespace nav2_behavior_tree

#endif //RECORD_COMMAND_CONTROL_HPP
