#include <string>

#include "robot_behavior_tree/plugins/action/buy_blood.hpp"

namespace nav2_behavior_tree
{
    BuyBloodAction::BuyBloodAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        sentryinfo_sub_ = node_->create_subscription<rm_interfaces::msg::Sentryinfo>(
            "/robot/sentryinfo",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&BuyBloodAction::sentryinfoCallback, this, std::placeholders::_1),
            sub_option);

        config().blackboard->get<uint32_t>("buy_blood_number", buy_blood_number);
        config().blackboard->get<bool>("if_waiting_buy_blood", if_waiting_buy_blood);
    }

    BT::NodeStatus BuyBloodAction::tick()
    {
        //更新裁判系统数据，获取黑板中的数据
        callback_group_executor_.spin_some();
        config().blackboard->get<uint32_t>("buy_blood_number", buy_blood_number);
        config().blackboard->get<bool>("if_waiting_buy_blood", if_waiting_buy_blood);
        RCLCPP_INFO(node_->get_logger(), "买血判断 buy_blood_number:%d, 裁判系统数据: %d", buy_blood_number,
            received_sentryinfo_.number_of_get_blood);

        //发送远程买血请求/等待买血判断是否成功
        if(received_sentryinfo_.number_of_get_blood == buy_blood_number && !if_waiting_buy_blood)
        {
            buy_blood_number++;
            config().blackboard->set<uint32_t>("buy_blood_number", buy_blood_number); //存到黑板中等到send_decision时一起发送
            RCLCPP_INFO(node_->get_logger(), "发送远程买血请求");
            if_waiting_buy_blood = true; //当前在等待买血状态
        }
        else if(buy_blood_number - received_sentryinfo_.number_of_get_blood == 1)
        {
            RCLCPP_INFO(node_->get_logger(), "远程买血请求已发送，等待中！");
            if_waiting_buy_blood = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "远程买血请求错误，等待与裁判系统同步！");
            if_waiting_buy_blood = false;
        }
        config().blackboard->set<bool>("if_waiting_buy_blood", if_waiting_buy_blood);

        return BT::NodeStatus::SUCCESS;
    }

    void BuyBloodAction::sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg)
    {
        received_sentryinfo_.number_of_get_blood = msg->number_of_get_blood; //获取来自裁判系统的买血总次数，用于同步判断
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::BuyBloodAction>("BuyBlood");
}