#include <string>

#include "robot_behavior_tree/plugins/action/force_move.hpp"

namespace nav2_behavior_tree
{

    ForceMoveAction::ForceMoveAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        is_force_move_complete(true),
        force_move_time_(1.0),
        force_v_x(0.0),
        force_v_y(0.0)
    {
        config().blackboard->get<rclcpp::Node::SharedPtr>("node",node_ );
        force_v_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        getInput("force_move_time", force_move_time_);
        bt_loop_duration_ = config().blackboard->get<std::chrono::milliseconds>("bt_loop_duration");
        count_size_ = force_move_time_ * 10000 / bt_loop_duration_.count();    
    }

    BT::NodeStatus ForceMoveAction::tick()
    {  
        static int count = count_size_;
        if(config().blackboard->get<bool>("is_force_move_forward")){
            config().blackboard->set<bool>("is_force_move_forward",false);
            is_force_move_complete = false;
            force_v_x = 1.0;
            force_v_y = 0.0;
            count = 0;
            RCLCPP_INFO(node_->get_logger(), "强制向前移动");
        }
        else if (config().blackboard->get<bool>("is_force_move_backward")){
            config().blackboard->set<bool>("is_force_move_backward",false);
            is_force_move_complete = false;
            force_v_x = -1.0;
            force_v_y = 0.0;
            count = 0;
            RCLCPP_INFO(node_->get_logger(), "强制向后移动");
        }
        else if (config().blackboard->get<bool>("is_force_move_left")){
            config().blackboard->set<bool>("is_force_move_left",false);
            is_force_move_complete = false;
            force_v_x = 0.0;
            force_v_y = 1.0;
            count = 0;
            RCLCPP_INFO(node_->get_logger(), "强制向左移动");
        }
        else if (config().blackboard->get<bool>("is_force_move_right")){
            config().blackboard->set<bool>("is_force_move_right",false);
            is_force_move_complete = false;
            force_v_x = 0.0;
            force_v_y = -1.0;
            count = 0;
            RCLCPP_INFO(node_->get_logger(), "强制向右移动");
        }
        //RCLCPP_INFO(node_->get_logger(), count_size_);
        if(count < count_size_){
            if(count >= count_size_-2){
                geometry_msgs::msg::Twist force_v;
                force_v.linear.x = 0.0;
                force_v.linear.y = 0.0;
                force_v_pub_->publish(force_v);
                RCLCPP_INFO(node_->get_logger(), "强制移动停止");
                count ++;       
            }
            else {
                geometry_msgs::msg::Twist force_v;
                force_v.linear.x = force_v_x;
                force_v.linear.y = force_v_y;
                force_v_pub_->publish(force_v);
                RCLCPP_INFO(node_->get_logger(), "强制移动中");
                count ++;
            }
            config().blackboard->set<bool>("is_force_move_complete",is_force_move_complete);
            return BT::NodeStatus::SUCCESS;
            // return BT::NodeStatus::RUNNING;
        }
        else {
            is_force_move_complete = true;
            RCLCPP_INFO(node_->get_logger(), "强制移动结束");
            config().blackboard->set<bool>("is_force_move_complete",is_force_move_complete);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::ForceMoveAction>("ForceMove");
}
