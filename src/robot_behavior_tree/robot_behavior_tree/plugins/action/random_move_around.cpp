#include <string>

#include "robot_behavior_tree/plugins/action/random_move_around.hpp"

namespace nav2_behavior_tree
{

    RandomMoveAroundAction::RandomMoveAroundAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          select_time(1.0),
          robot_position_x(0.0),
          robot_position_y(0.0),
          global_frame_("map"),
          robot_base_frame_("base_link")
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        bt_loop_duration_ = config().blackboard->get<std::chrono::milliseconds>("bt_loop_duration");
        tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

        if (!node_->has_parameter("transform_tolerance"))
        {
            node_->declare_parameter("transform_tolerance", 0.15);
        }
        node_->get_parameter("transform_tolerance", transform_tolerance_);

        getInput("select_time",select_time);
        getInput("moving_distance",moving_distance);
        count_size_ = select_time * 1000 / bt_loop_duration_.count();
    }

    BT::NodeStatus RandomMoveAroundAction::tick()
    {
        getInput("moving_distance",moving_distance);
        //获取当前tf变换
        geometry_msgs::msg::PoseStamped current_pose;
        if (!nav2_util::getCurrentPose(
            current_pose, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
        {
            RCLCPP_INFO(node_->get_logger(), "random_move_around:没有获取到TF变换");
            return BT::NodeStatus::FAILURE;
        }
        robot_position_x = current_pose.pose.position.x;
        robot_position_y = current_pose.pose.position.y;

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
            RCLCPP_INFO(node_->get_logger(), "random_move_around:随机选取的数字是: %d", random_number);
            pose.header.stamp = node_->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = robot_position_x + ((random_number==0) - (random_number==1)) * moving_distance; //前后
            pose.pose.position.y = robot_position_y + ((random_number==2) - (random_number==3)) * moving_distance; //左右
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            setOutput<geometry_msgs::msg::PoseStamped>("goal",pose);
            RCLCPP_INFO(node_->get_logger(), "random_move_around:随机选取的目标点为: %lf, %lf",
                pose.pose.position.x, pose.pose.position.y);
            count = 0;
        }
        count++;
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RandomMoveAroundAction>("RandomMoveAround");
}
