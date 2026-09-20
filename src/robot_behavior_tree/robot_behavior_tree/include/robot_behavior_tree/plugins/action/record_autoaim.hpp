//
// Created by elsa on 25-5-1.
//

#ifndef RECORD_AUTOAIM_HPP
#define RECORD_AUTOAIM_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/autoaim_strategy.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 将行为树中的相关信息打包发给另一个节点
     *
     */
    class RecordAutoaimAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        RecordAutoaimAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        RecordAutoaimAction() = delete;


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
        void get_blackboard_value();

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<rm_interfaces::msg::AutoaimStrategy>::SharedPtr autoaim_strategy_pub_;
        rm_interfaces::msg::AutoaimStrategy autoaim_strategy_;

        bool is_we_are_blue;
        /* 0-6依次为英雄、工程、步兵3、步兵4、哨兵、前哨站、基地 */
        /* 白名单中的数据 */
        uint8_t whitelist[7];
        int enemy_blood[7]; //敌方机器人血量
        bool enemy_invincible[7]; //敌方是否处于无敌状态
    };

} // namespace nav2_behavior_tree

#endif //RECORD_AUTOAIM_HPP
