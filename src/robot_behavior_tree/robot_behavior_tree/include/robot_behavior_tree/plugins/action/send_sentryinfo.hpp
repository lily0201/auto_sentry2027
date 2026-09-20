#ifndef SEND_SENTRTINFO_HPP_
#define SEND_SENTRTINFO_HPP_

#include <string>
#include <memory>
#include <mutex>

#include <locale>
#include <codecvt>
#include <cstring> // 用于 std::memcpy
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/info.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree {
    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SendSentryInfoAction : public BT::SyncActionNode {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SendSentryInfoAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SendSentryInfoAction() = delete;

        std::u16string utf8_to_utf16(const std::string &utf8_str);

        std::string utf16_to_utf8(const std::u16string &utf16_str);

        // 将 UTF-16 编码的 std::u16string 转换为 uint8_t 数组
        void utf16_to_uint8_array(const std::u16string &utf16_str, std::array<uint8_t, 30> &user_data);

        // 将 uint8_t 数组转换为 UTF-16 编码的 std::u16string
        std::u16string uint8_array_to_utf16(const std::array<uint8_t, 30> &user_data);

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts() {
            return {};
        }

    private:
        uint16_t receiver_id;
        uint16_t sender_id;
        std::string info;
        //std::array<uint8_t, 30> user_data = {0};
        //std::array<uint8_t, 30> data_temp = {0};
        rclcpp::Publisher<rm_interfaces::msg::Info>::SharedPtr info_pub_;
        rclcpp::Node::SharedPtr node_;
    };
} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
