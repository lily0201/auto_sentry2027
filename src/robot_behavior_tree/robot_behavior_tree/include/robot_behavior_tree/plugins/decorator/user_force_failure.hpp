#ifndef USER_FORCE_FAILURE_HPP_
#define USER_FORCE_FAILURE_HPP_

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{
/**
 * @brief The ForceSuccessNode returns always SUCCESS or RUNNING.
 */
class UserForceFailureDecorator : public BT::DecoratorNode
{
public:
  UserForceFailureDecorator(const std::string& name);

private:
   BT::NodeStatus tick() override;
};

//------------ implementation ----------------------------
}  // namespace BT
#endif