#include <string>

#include "robot_behavior_tree/plugins/action/buy_bullet.hpp"
namespace nav2_behavior_tree
{
    BuyBulletAction::BuyBulletAction(
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
            std::bind(&BuyBulletAction::sentryinfoCallback, this, std::placeholders::_1),
            sub_option);

        config().blackboard->get<uint32_t>("buy_bullet_remote_number", buy_bullet_remote_number);
        config().blackboard->get<bool>("if_waiting_buy_bullet_remote", if_waiting_buy_bullet_remote);
    }

    BT::NodeStatus BuyBulletAction::tick()
    {
        //更新裁判系统数据，获取黑板中的数据
        config().blackboard->get<uint32_t>("buy_bullet_remote_number", buy_bullet_remote_number);
        config().blackboard->get<bool>("if_waiting_buy_bullet_remote", if_waiting_buy_bullet_remote);
        callback_group_executor_.spin_some();
        RCLCPP_INFO(node_->get_logger(), "远程买弹判断 buy_bullet_remote_number:%d, 裁判系统数据: %d", buy_bullet_remote_number,
            received_sentryinfo_.number_of_get_ammunition);

        //发送远程买弹请求/等待远程买弹判断是否成功
        if(received_sentryinfo_.number_of_get_ammunition == buy_bullet_remote_number && !if_waiting_buy_bullet_remote)
        {
            buy_bullet_remote_number++;
            config().blackboard->set<uint32_t>("buy_bullet_remote_number", buy_bullet_remote_number);
            RCLCPP_INFO(node_->get_logger(), "发送远程买弹请求");
            if_waiting_buy_bullet_remote = true;//当前在等待远程买弹状态
        }
        else if(buy_bullet_remote_number - received_sentryinfo_.number_of_get_ammunition == 1)
        {
            RCLCPP_INFO(node_->get_logger(), "等待远程买弹请求处理中");
            if_waiting_buy_bullet_remote = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "远程买弹请求错误，等待同步");
            if_waiting_buy_bullet_remote = false;
        }
        config().blackboard->set<bool>("if_waiting_buy_bullet_remote", if_waiting_buy_bullet_remote);

        return BT::NodeStatus::SUCCESS;
    }

    void BuyBulletAction::sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg)
    {
        received_sentryinfo_.number_of_get_ammunition = msg->number_of_get_ammunition; //获取来自裁判系统的远程买弹总次数，用于同步判断
        RCLCPP_INFO(node_->get_logger(), "裁判系统远程买弹次数: %d", received_sentryinfo_.number_of_get_ammunition);
    }
} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::BuyBulletAction>("BuyBullet");
}