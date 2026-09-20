#ifndef PRINTF_HPP_
#define PRINTF_HPP_
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 打印相关信息，有时起到一个占位的作用
     */
    class PrintfAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        PrintfAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        PrintfAction() = delete;

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
                BT::InputPort<std::string>("txt", "txt"),
            };
         }

    private:
        rclcpp::Node::SharedPtr node_;
        ///要打印的信息
        std::string txt;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
