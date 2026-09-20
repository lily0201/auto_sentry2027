#ifndef USER_FORCE_SUCCESS_HPP_
#define USER_FORCE_SUCCESS_HPP_

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{
/**
 * @brief The ForceSuccessNode returns always SUCCESS or RUNNING.
 */
class UserForceSuccessDecorator : public BT::DecoratorNode
{
public:
  UserForceSuccessDecorator(const std::string& name) ;

private:
   BT::NodeStatus tick() override;
};

//------------ implementation ----------------------------
}  // namespace BT
#endif