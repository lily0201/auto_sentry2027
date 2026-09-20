#include "pointcloud_merge/pointcloud_merge.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<LidarMergeNode> node = std::make_shared<LidarMergeNode>("pointcloud_merge_node");
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}