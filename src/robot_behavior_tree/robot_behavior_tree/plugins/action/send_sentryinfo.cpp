#include <string>

#include "robot_behavior_tree/plugins/action/send_sentryinfo.hpp"

namespace nav2_behavior_tree
{

    SendSentryInfoAction::SendSentryInfoAction(
         const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        config().blackboard->get<uint16_t>("sender_id",sender_id);
        config().blackboard->get<uint16_t>("receiver_id",receiver_id);
        config().blackboard->get<std::string>("info",info);
        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_ );
        info_pub_ = node_->create_publisher<rm_interfaces::msg::Info>("/robot/info", 10);
    }

    BT::NodeStatus SendSentryInfoAction::tick()
    {
        config().blackboard->get<uint16_t>("sender_id",sender_id);
        config().blackboard->get<uint16_t>("receiver_id",receiver_id);
        config().blackboard->get<std::string>("info",info);
        rm_interfaces::msg::Info info_msg;
        RCLCPP_INFO(node_->get_logger(), "**************************************");
        RCLCPP_INFO(node_->get_logger(), "* SendSentryInfoAction: %s *",info.c_str());
        RCLCPP_INFO(node_->get_logger(), "**************************************");

        // 将 std::string 转换为 UTF-16 编码的 std::u16string
        std::u16string utf16_str = utf8_to_utf16(info);

        // 创建一个 std::array<uint8_t, 30>
        std::array<uint8_t, 30> user_data;

        // 将 UTF-16 编码的 std::u16string 转换为 uint8_t 数组
        utf16_to_uint8_array(utf16_str, user_data);

        // 验证复制结果（逐个输出 std::array<uint8_t, 30> 中的数据）
        // std::cout << "user_data: ";
        // for (size_t i = 0; i < user_data.size(); ++i) {
        //     std::cout << std::hex << static_cast<int>(user_data[i]) << " ";
        // }
        // std::cout << std::endl;

        //只给云台手发
        info_msg.sender_id = sender_id;
        info_msg.receiver_id = receiver_id;
        info_msg.user_data = user_data;
        info_pub_->publish(info_msg);

        // 将 uint8_t 数组转换回 UTF-16 编码的 std::u16string
        //std::u16string recovered_utf16_str = uint8_array_to_utf16(user_data);

        // 将 UTF-16 编码的 std::u16string 转换回 std::string (UTF-8)
        //std::string recovered_str = utf16_to_utf8(recovered_utf16_str);

        // 输出转换回来的字符串
        //std::cout << "Recovered string: " << recovered_str << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

    std::u16string utf8_to_utf16(const std::string& utf8_str) {
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> convert;
    return convert.from_bytes(utf8_str);
}

// 将 UTF-16 编码的 std::u16string 转换为 std::string (UTF-8)
    std::u16string SendSentryInfoAction::utf8_to_utf16(const std::string& utf8_str) {
        std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> convert;
        return convert.from_bytes(utf8_str);
    }

    std::string SendSentryInfoAction::utf16_to_utf8(const std::u16string& utf16_str) {
        std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> convert;
        return convert.to_bytes(utf16_str);
    }

    // 将 UTF-16 编码的 std::u16string 转换为 uint8_t 数组
    void SendSentryInfoAction::utf16_to_uint8_array(const std::u16string& utf16_str, std::array<uint8_t, 30>& user_data) {
        std::memset(user_data.data(), 0, user_data.size());
        std::memcpy(user_data.data(), utf16_str.c_str(), std::min(user_data.size(), utf16_str.size() * sizeof(char16_t)));
    }

    // 将 uint8_t 数组转换为 UTF-16 编码的 std::u16string
    std::u16string SendSentryInfoAction::uint8_array_to_utf16(const std::array<uint8_t, 30>& user_data) {
        return std::u16string(reinterpret_cast<const char16_t*>(user_data.data()), user_data.size() / sizeof(char16_t));
    }


} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SendSentryInfoAction>("SendSentryInfo");
}
