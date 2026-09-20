//
// Created by elsa on 25-7-20.
//

#ifndef SET_AIM_MODE_HPP
#define SET_AIM_MODE_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 切换当前导航控制的模式，并根据不同的控制mode切换路径规划用到的参数
     */
    class SetAimModeAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SetAimModeAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SetAimModeAction() = delete;

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
                BT::InputPort<int>("robot_aim", "robot aim mode"),
            };
        }

    private:
        rclcpp::Node::SharedPtr node_;

        int robot_aim_;
    };

} // namespace nav2_behavior_tree

#endif //SET_AIM_MODE_HPP
