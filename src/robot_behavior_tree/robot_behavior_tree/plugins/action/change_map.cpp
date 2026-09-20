//
// Created by elsa on 24-12-28.
//

#include "robot_behavior_tree/plugins/action/change_map.hpp"
namespace nav2_behavior_tree
{
    ChangeMapAction::ChangeMapAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
      	map_yaml_(""),
      	map_image_("")
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  // 可以根据需要选择适合的策略
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);  // 选择适合的持久性策略
        map_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
    }

    BT::NodeStatus ChangeMapAction::tick()
    {
       	getInput("map_yaml",map_yaml_);
        getInput("map_image",map_image_);
        publish_map(map_yaml_, map_image_);
        return BT::NodeStatus::SUCCESS;
    }

    void ChangeMapAction::publish_map(const std::string &yaml_file, const std::string &map_file)
    {
        // 解析YAML文件
        YAML::Node map_info = YAML::LoadFile(yaml_file);
        std::string image_file = map_info["image"].as<std::string>();
        double resolution = map_info["resolution"].as<double>(); //设定地图分辨率
        std::vector<double> origin = map_info["origin"].as<std::vector<double>>(); //设定地图原点
        RCLCPP_INFO(node_->get_logger(), "image_file: %s", image_file.c_str());
        RCLCPP_INFO(node_->get_logger(), "resolution: %lf", resolution);
        RCLCPP_INFO(node_->get_logger(), "origin: %lf, %lf", origin[1], origin[2]);

        // 读取图片文件
        cv::Mat map_image = cv::imread(map_file, cv::IMREAD_GRAYSCALE);
        if (map_image.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load map image: %s", image_file.c_str());
            return;
        }
        cv::flip(map_image, map_image, 0);  // 图片上下翻转

        // 创建栅格地图OccupancyGrid消息
        nav_msgs::msg::OccupancyGrid map_msg;
        rclcpp::Clock clock(RCL_SYSTEM_TIME);
        map_msg.info.map_load_time = clock.now();
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = clock.now();
        map_msg.info.resolution = resolution;
        map_msg.info.width = map_image.cols;
        map_msg.info.height = map_image.rows;
        RCLCPP_INFO(node_->get_logger(), "map_image.cols: %d", map_image.cols);
        RCLCPP_INFO(node_->get_logger(), "map_image.rows: %d", map_image.rows);
        map_msg.info.origin.position.x = origin[0];
        map_msg.info.origin.position.y = origin[1];
        map_msg.info.origin.position.z = origin[2];
        map_msg.info.origin.orientation.w = 1.0;  // 假设地图无旋转

        // 将图片数据转换为OccupancyGrid数据
        map_msg.data.resize(map_msg.info.width * map_msg.info.height);
        for (int i = 0; i < map_image.rows; i++) {
            for (int j = 0; j < map_image.cols; j++) {
                int index = i * map_image.cols + j;
                uint8_t pixel_value = map_image.at<uint8_t>(i, j);
                if (pixel_value >= 100) {
                    map_msg.data[index] = 0;  // 空闲区域
                } else {
                    map_msg.data[index] = 100;  // 占用区域
                }
            }
        }

        // 发布地图
        map_publisher_->publish(map_msg);
        RCLCPP_INFO(node_->get_logger(), "------------map published---------");
    }


} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::ChangeMapAction>("ChangeMap");
}