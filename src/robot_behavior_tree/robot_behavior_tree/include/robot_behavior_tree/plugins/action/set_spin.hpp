//
// Created by elsa on 25-5-24.
//

#ifndef SET_SPIN_HPP
#define SET_SPIN_HPP

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
    class SetSpinAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SetSpinAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SetSpinAction() = delete;

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
                BT::InputPort<int>("spin", "spin"),
            };
        }

    private:
        rclcpp::Node::SharedPtr node_;

        int spin_;
    };

} // namespace nav2_behavior_tree

#endif //SET_SPIN_HPP
