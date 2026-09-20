//
// Created by elsa on 25-5-15.
//

#include <string>

#include "robot_behavior_tree/plugins/action/record_passing_tunnel_index.hpp"

namespace nav2_behavior_tree
{
    RecordPassingTunnelIndexAction::RecordPassingTunnelIndexAction(
        const std::string& action_name,
        const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(action_name, conf),
          tunnel_index_(0), if_in_tunnel_(false), if_last_in_tunnel_(false), if_in_mode_changing(false),
          global_frame_("map"), transform_tolerance_(0.1)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        lidarstation_sub_= node_->create_subscription<rm_interfaces::msg::Lidarstation>(
            "/robot/lidarstation", rclcpp::SystemDefaultsQoS(),
            std::bind(&RecordPassingTunnelIndexAction::lidarstationCallback, this, std::placeholders::_1),
            sub_option);
        tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

        lidarstation_ = std::make_shared<rm_interfaces::msg::Lidarstation>();

        config().blackboard->get<int>("mode", mode);
        if (mode == 0)
            robot_base_frame_ = "base_link";
        else
            robot_base_frame_ = "lidar_base_link";

        config().blackboard->get<std::vector<double>>("red_outpost_tunnel_x", red_outpost_tunnel_x_);
        config().blackboard->get<std::vector<double>>("red_outpost_tunnel_y", red_outpost_tunnel_y_);
        config().blackboard->get<std::vector<double>>("red_banana_tunnel_x", red_banana_tunnel_x_);
        config().blackboard->get<std::vector<double>>("red_banana_tunnel_y", red_banana_tunnel_y_);
        config().blackboard->get<std::vector<double>>("blue_outpost_tunnel_x", blue_outpost_tunnel_x_);
        config().blackboard->get<std::vector<double>>("blue_outpost_tunnel_y", blue_outpost_tunnel_y_);
        config().blackboard->get<std::vector<double>>("blue_banana_tunnel_x", blue_banana_tunnel_x_);
        config().blackboard->get<std::vector<double>>("blue_banana_tunnel_y", blue_banana_tunnel_y_);
        RCLCPP_INFO(node_->get_logger(), "红方前哨狗洞四个范围端点: (%lf, %lf), (%lf, %lf), (%lf, %lf), (%lf, %lf)",
            red_outpost_tunnel_x_[0], red_outpost_tunnel_y_[0], red_outpost_tunnel_x_[1], red_outpost_tunnel_y_[1],
            red_outpost_tunnel_x_[2], red_outpost_tunnel_y_[2], red_outpost_tunnel_x_[3], red_outpost_tunnel_y_[3]);
        RCLCPP_INFO(node_->get_logger(), "红方沟槽狗洞六个范围端点: (%lf, %lf), (%lf, %lf), (%lf, %lf), (%lf, %lf)",
            red_banana_tunnel_x_[0], red_banana_tunnel_y_[0], red_banana_tunnel_x_[1], red_banana_tunnel_y_[1],
            red_banana_tunnel_x_[2], red_banana_tunnel_y_[2], red_banana_tunnel_x_[3], red_banana_tunnel_y_[3]);
        RCLCPP_INFO(node_->get_logger(), "蓝方前哨狗洞四个范围端点: (%lf, %lf), (%lf, %lf), (%lf, %lf), (%lf, %lf)",
            blue_outpost_tunnel_x_[0], blue_outpost_tunnel_y_[0], blue_outpost_tunnel_x_[1], blue_outpost_tunnel_y_[1],
            blue_outpost_tunnel_x_[2], blue_outpost_tunnel_y_[2], blue_outpost_tunnel_x_[3], blue_outpost_tunnel_y_[3]);
        RCLCPP_INFO(node_->get_logger(), "蓝方沟槽狗洞六个范围端点: (%lf, %lf), (%lf, %lf), (%lf, %lf), (%lf, %lf)",
            blue_banana_tunnel_x_[0], blue_banana_tunnel_y_[0], blue_banana_tunnel_x_[1], blue_banana_tunnel_y_[1],
            blue_banana_tunnel_x_[2], blue_banana_tunnel_y_[2], blue_banana_tunnel_x_[3], blue_banana_tunnel_y_[3]);

        red_outpost_tunnel.resize(4);
        red_banana_tunnel.resize(6);
        blue_outpost_tunnel.resize(4);
        blue_banana_tunnel.resize(6);
        dangerous_tunnel.resize(4);

        for (int i = 0; i < 4; i++) {
            red_outpost_tunnel[i].x = red_outpost_tunnel_x_[i];
            red_outpost_tunnel[i].y = red_outpost_tunnel_y_[i];
            blue_outpost_tunnel[i].x = blue_outpost_tunnel_x_[i];
            blue_outpost_tunnel[i].y = blue_outpost_tunnel_y_[i];
        }
        for (int i = 0; i < 6; i++) {
            red_banana_tunnel[i].x = red_banana_tunnel_x_[i];
            red_banana_tunnel[i].y = red_banana_tunnel_y_[i];
            blue_banana_tunnel[i].x = blue_banana_tunnel_x_[i];
            blue_banana_tunnel[i].y = blue_banana_tunnel_y_[i];
        }
        for(int i = 0; i < 4; i++)
            dangerous_tunnel[i] = false;
    }

    BT::NodeStatus RecordPassingTunnelIndexAction::tick()
    {
        /* 根据自身tf坐标判断自己要路过哪个狗洞 */
        geometry_msgs::msg::PoseStamped current_pose;
        if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
        {
            RCLCPP_INFO(node_->get_logger(), "没有获取到TF变换");
            return BT::NodeStatus::FAILURE;
        }

        Point current_position;
        current_position.x = current_pose.pose.position.x;
        current_position.y = current_pose.pose.position.y;
        RCLCPP_INFO(node_->get_logger(), "current_position: (%lf, %lf)", current_position.x, current_position.y);

        if (IsPointInPolygon(current_position, red_outpost_tunnel)) {
            tunnel_index_ = 1;
            if_in_tunnel_ = true;
        }
        else if (IsPointInPolygon(current_position, red_banana_tunnel)) {
            tunnel_index_ = 2;
            if_in_tunnel_ = true;
        }
        else if (IsPointInPolygon(current_position, blue_outpost_tunnel)) {
            tunnel_index_ = 3;
            if_in_tunnel_ = true;
        }
        else if (IsPointInPolygon(current_position, blue_banana_tunnel)) {
            tunnel_index_ = 4;
            if_in_tunnel_ = true;
        }
        else {
            tunnel_index_ = 0;
            if_in_tunnel_ = false;
            RCLCPP_INFO(node_->get_logger(), "哨兵不在狗洞内");
        }
        config().blackboard->set<int>("tunnel_index", tunnel_index_);
        config().blackboard->set<bool>("if_in_tunnel", if_in_tunnel_);
        RCLCPP_INFO(node_->get_logger(), "当前导航路线经过狗洞 %d", tunnel_index_);

        // if (if_in_tunnel_ != if_last_in_tunnel_) {
        //     if (!if_last_in_tunnel_ && if_in_tunnel_) {
        //         if_in_mode_changing = true; // 进入切换模式的等待状态
        //     }
        //     else {
        //         if_in_mode_changing = false;
        //     }
        //     if_last_in_tunnel_ = if_in_tunnel_;
        // }
        // else {
        //     config().blackboard->get<bool>("if_in_mode_changing", if_in_mode_changing);
        // }
        // RCLCPP_INFO(node_->get_logger(), "!!!!!!!!!!!!!if_in_tunnel_: %d", if_in_tunnel_);
        // RCLCPP_INFO(node_->get_logger(), "!!!!!!!!!!!!if_last_in_tunnel_: %d", if_last_in_tunnel_);
        //
        // RCLCPP_INFO(node_->get_logger(), "record_tunnel: !!!!!!!!!!!!!!!if_in_mode_changing: %d", if_in_mode_changing);
        // config().blackboard->set<bool>("if_in_mode_changing", if_in_mode_changing);

        // callback_group_executor_.spin_some();

        config().blackboard->set<std::vector<bool>>("dangerous_tunnel", dangerous_tunnel);

        return BT::NodeStatus::SUCCESS;
    }

    bool RecordPassingTunnelIndexAction::IsPointInPolygon(const Point& vtPoint, const std::vector<Point>& vecPoints)
    {
        bool bResult = false; //判断结果（true；点落在多边形内；false:点未落在多边形内）
        int nSize = vecPoints.size();
        int pre = nSize - 1; //nSize -1 是多边形的最后一个顶点
        for (int cur = 0; cur < nSize; ++cur)
        {
            //判断点是否在线段的两侧
            if ((vecPoints[cur].x < vtPoint.x && vecPoints[pre].x >= vtPoint.x)
                || (vecPoints[pre].x < vtPoint.x && vecPoints[cur].x >= vtPoint.x))
            {
                //根据两点式方程计算出过点P且平行于X轴的直线与线段的交点，两点式方程：
                // y = p1.y + (p.x - p1.x) * (p2.y - p1.y) / (p2.x - p1.x)
                double line_p_y = vecPoints[cur].y + (vtPoint.x - vecPoints[cur].x) * (vecPoints[pre].y - vecPoints[cur].y)
                                    /  (vecPoints[pre].x - vecPoints[cur].x);
                if (line_p_y > vtPoint.y){
                    bResult = !bResult;
                }
                else if(line_p_y == vtPoint.y){
                    return true;
                }
            }
            //进行下一线段判断
            pre = cur;
        }
        return bResult;
    }

    void RecordPassingTunnelIndexAction::lidarstationCallback(const rm_interfaces::msg::Lidarstation::ConstPtr& msg)
    {
        lidarstation_->attack_enhance = msg->attack_enhance;
        for(int i = 0; i < 5; i++)
        {
            lidarstation_->lidarpositions[i].x = msg->lidarpositions[i].x;
            lidarstation_->lidarpositions[i].y = msg->lidarpositions[i].y;
            lidarstation_->lidarpositions[i].z = msg->lidarpositions[i].z;
        }

        for(int i = 0; i < 4; i++)
            dangerous_tunnel[i] = false;

        /* 根据雷达站传来的数据判断四个狗洞是否有人堵 */
        for(int i = 0; i < 5; i++)
        {
            Point enemyPoint;
            enemyPoint.x = lidarstation_->lidarpositions[i].x;
            enemyPoint.y = lidarstation_->lidarpositions[i].y;

            if (IsPointInPolygon(enemyPoint, red_outpost_tunnel)) {
                dangerous_tunnel[0] = true;
                RCLCPP_INFO(node_->get_logger(), "敌方 %d 号机器人在红方前哨狗洞内", i);
            }
            else if (IsPointInPolygon(enemyPoint, red_banana_tunnel)) {
                dangerous_tunnel[1] = true;
                RCLCPP_INFO(node_->get_logger(), "敌方 %d 号机器人在红方沟槽狗洞内", i);
            }
            else if (IsPointInPolygon(enemyPoint, blue_outpost_tunnel)) {
                dangerous_tunnel[2] = true;
                RCLCPP_INFO(node_->get_logger(), "敌方 %d 号机器人在蓝方前哨狗洞内", i);
            }
            else if (IsPointInPolygon(enemyPoint, blue_banana_tunnel)) {
                dangerous_tunnel[3] = true;
                RCLCPP_INFO(node_->get_logger(), "敌方 %d 号机器人在蓝方沟槽狗洞内", i);
            }
        }
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RecordPassingTunnelIndexAction>("RecordPassingTunnelIndex");
}
