#include <string>

#include "robot_behavior_tree/plugins/action/random_move.hpp"

namespace nav2_behavior_tree
{

    RandomMoveAction::RandomMoveAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        select_time(1.0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        getInput("select_time",select_time);
        getInput("random_position_x_1",random_position_x[0]);
        getInput("random_position_y_1",random_position_y[0]);
        getInput("random_position_x_2",random_position_x[1]);
        getInput("random_position_y_2",random_position_y[1]);
        getInput("random_position_x_3",random_position_x[2]);
        getInput("random_position_y_3",random_position_y[2]);
        getInput("random_position_x_4",random_position_x[3]);
        getInput("random_position_y_4",random_position_y[3]);

        bt_loop_duration_ = config().blackboard->get<std::chrono::milliseconds>("bt_loop_duration");
        count_size_ = select_time * 1000 / bt_loop_duration_.count();
    }

    BT::NodeStatus RandomMoveAction::tick()
    {
        static int count = count_size_;
        // 创建一个随机数生成器
        std::random_device rd;  // 用于生成种子
        std::mt19937 gen(rd()); // Mersenne Twister 19937引擎初始化为rd()的值

        // 创建一个范围为1到4的均匀分布
        std::uniform_int_distribution<> dis(0, 3);

        if(count == count_size_){
            // 生成一个随机数
            static int last_random_number = 0;
            int random_number = dis(gen);
            if(last_random_number == random_number){
                random_number = dis(gen);
            }
            last_random_number = random_number;
            RCLCPP_INFO(node_->get_logger(), "随机选取的数字是: %d", random_number);
            pose.header.stamp = node_->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = random_position_x[random_number]; //指定目标点
            pose.pose.position.y = random_position_y[random_number];
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            setOutput<geometry_msgs::msg::PoseStamped>("randomgoal",pose);
            RCLCPP_INFO(node_->get_logger(), "随机选取的目标点为: %lf, %lf", pose.pose.position.x, pose.pose.position.y);
            count = 0;
        }
        count++;
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RandomMoveAction>("RandomMove");
}
