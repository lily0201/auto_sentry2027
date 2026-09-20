#include <string>

#include "robot_behavior_tree/plugins/action/routine_patrol.hpp"

namespace nav2_behavior_tree
{
    RoutinePatrolAction::RoutinePatrolAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          select_time(1.0), target_position_index(0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        truncate_path_info_pub_ = node_->create_publisher<rm_interfaces::msg::TruncatePathInfo>(
            "/robot/truncated_path_info", 10);

        getInput("select_time",select_time);
        getInput("routine_position_x_1",routine_position_x[0]);
        getInput("routine_position_y_1",routine_position_y[0]);
        getInput("routine_position_x_2",routine_position_x[1]);
        getInput("routine_position_y_2",routine_position_y[1]);

        bt_loop_duration_ = config().blackboard->get<std::chrono::milliseconds>("bt_loop_duration");
        count_size_ = select_time * 1000 / bt_loop_duration_.count();
        pose.header.frame_id = "map";
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
    }

    BT::NodeStatus RoutinePatrolAction::tick()
    {
        static int count = count_size_;
        
        if(count >= count_size_){
            
            count = 0;
            if(!target_position_index)
                target_position_index++;
            else
                target_position_index--;
        }
        pose.header.stamp = node_->now();
        pose.pose.position.x = routine_position_x[target_position_index];
        pose.pose.position.y = routine_position_y[target_position_index];
        RCLCPP_INFO(node_->get_logger(), "-------------------------当前为第 %d 个巡逻目标点: %lf, %lf count:%d -------------------",
            target_position_index, pose.pose.position.x, pose.pose.position.y,count);
        setOutput<geometry_msgs::msg::PoseStamped>("routine_goal", pose);
        count++;

        //更新裁剪路径相关信息
        truncate_path_info_msg_.distance = 0.0;
        truncate_path_info_msg_.dafu = false;
        truncate_path_info_pub_->publish(truncate_path_info_msg_);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
 {
     factory.registerNodeType<nav2_behavior_tree::RoutinePatrolAction>("RoutinePatrol");
 }
