#ifndef BUY_BLOOD_HPP_
#define BUY_BLOOD_HPP_
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/sentryinfo.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 远程买血节点，当满足以下两个条件时会向裁判系统发送买血请求：
     * @brief 1.不在等待买血状态；2.当前买血总次数与裁判系统次数同步
     * @brief 将当前远程买血总次数+1后发送给裁判系统表示发送买血请求
     */
    class BuyBloodAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        BuyBloodAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        BuyBloodAction() = delete;

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
        /**
         * @brief Callback function 接收裁判系统传回来的sentry_info消息
         * @param msg Shared pointer to rm_interfaces::msg::Sentryinfo message
         */
        void sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Sentryinfo>::SharedPtr sentryinfo_sub_;

        ///当前买血总次数，将该次数+1后发送给裁判系统表示发送买血请求
        uint32_t buy_blood_number;
        ///当前是否已经请求买血在等待中
        bool if_waiting_buy_blood;
        ///接收裁判系统传回来的sentry_info消息
        rm_interfaces::msg::Sentryinfo received_sentryinfo_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
