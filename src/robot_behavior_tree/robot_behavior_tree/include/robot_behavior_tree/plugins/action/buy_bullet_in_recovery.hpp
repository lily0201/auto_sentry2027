#ifndef BUY_BULLET_IN_RECOVERY_HPP_
#define BUY_BULLET_IN_RECOVERY_HPP_
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/sentryinfo.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 补给区买弹节点，当满足以下两个条件时会向裁判系统发送补给区买弹请求：
     * @brief 1.不在等待补给区买弹状态；2.当前补给区买弹总弹量与裁判系统次数同步
     * @brief 将当前补给区买弹总弹量 + 本次需要购买的弹量后发送给裁判系统表示发送补给区买弹请求
     */
    class BuyBulletInRecoveryAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        BuyBulletInRecoveryAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        BuyBulletInRecoveryAction() = delete;

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
                BT::InputPort<uint32_t>("number_of_bullet", "number_of_bullet to buy"),//本次需要购买的弹量
            };
         };

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

        ///本次需要购买的弹量
        uint32_t number_of_bullet;
        ///当前补给区买弹总弹量，在补给区领取的免费发弹量不计入
        uint32_t buy_bullet_at_recovery;
        ///当前是否已经请求补给区买弹在等待中
        bool if_waiting_buy_bullet_at_recovery;
        ///接收裁判系统传回来的sentry_info消息
        rm_interfaces::msg::Sentryinfo received_sentryinfo_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
