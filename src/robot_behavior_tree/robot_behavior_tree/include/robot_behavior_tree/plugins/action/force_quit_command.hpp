//
// Created by elsa on 25-5-15.
//

#ifndef FORCE_QUIT_COMMAND_HPP
#define FORCE_QUIT_COMMAND_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 强制退出云台手控制模式
     */
    class ForceQuitCommandAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        ForceQuitCommandAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        ForceQuitCommandAction() = delete;

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
            return {};
        }

    private:
        rclcpp::Node::SharedPtr node_;

        ///是否在云台手控制状态，只在云台手指定目标点/一键回家补血这种长时间指令时生效
        bool in_command;
    };

} // namespace nav2_behavior_tree

#endif //FORCE_QUIT_COMMAND_HPP
