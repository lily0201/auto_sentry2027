cmake_minimum_required(VERSION 3.8)
project(robot_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        include
)

set(dependencies
        rclcpp
        nav2_behavior_tree
        behaviortree_cpp_v3
        robot_msgs
        rm_interfaces
        geometry_msgs
        yaml_cpp_vendor
        OpenCV
)

add_library(nav2_keep_away_from_obstacles_action_bt_node SHARED plugins/action/keep_away_from_obstacles_action.cpp)
list(APPEND plugin_libs nav2_keep_away_from_obstacles_action_bt_node)

add_library(nav2_record_robot_blood_action_bt_node SHARED plugins/action/record_robot_blood.cpp)
list(APPEND plugin_libs nav2_record_robot_blood_action_bt_node)

add_library(nav2_record_robot_status_action_bt_node SHARED plugins/action/record_robot_status.cpp)
list(APPEND plugin_libs nav2_record_robot_status_action_bt_node)

add_library(nav2_buy_blood_action_bt_node SHARED plugins/action/buy_blood.cpp)
list(APPEND plugin_libs nav2_buy_blood_action_bt_node)

add_library(nav2_buy_bullet_action_bt_node SHARED plugins/action/buy_bullet.cpp)
list(APPEND plugin_libs nav2_buy_bullet_action_bt_node)

add_library(nav2_send_patrol_action_bt_node SHARED plugins/action/send_patrol.cpp)
list(APPEND plugin_libs nav2_send_patrol_action_bt_node)

add_library(nav2_select_resurrection_action_bt_node SHARED plugins/action/select_resurrection.cpp)
list(APPEND plugin_libs nav2_select_resurrection_action_bt_node)

add_library(nav2_buy_bullet_in_recovery_action_bt_node SHARED plugins/action/buy_bullet_in_recovery.cpp)
list(APPEND plugin_libs nav2_buy_bullet_in_recovery_action_bt_node)

add_library(nav2_send_decision_action_bt_node SHARED plugins/action/send_decision.cpp)
list(APPEND plugin_libs nav2_send_decision_action_bt_node)

add_library(nav2_printf_action_bt_node SHARED plugins/action/printf.cpp)
list(APPEND plugin_libs nav2_printf_action_bt_node)

add_library(nav2_set_patrol_action_bt_node SHARED plugins/action/set_patrol.cpp)
list(APPEND plugin_libs nav2_set_patrol_action_bt_node)

add_library(nav2_set_sentryinfo_action_bt_node SHARED plugins/action/set_sentryinfo.cpp)
list(APPEND plugin_libs nav2_set_sentryinfo_action_bt_node)

add_library(nav2_send_sentryinfo_action_bt_node SHARED plugins/action/send_sentryinfo.cpp)
list(APPEND plugin_libs nav2_send_sentryinfo_action_bt_node)

add_library(nav2_set_whitelist_action_bt_node SHARED plugins/action/set_whitelist.cpp)
list(APPEND plugin_libs nav2_set_whitelist_action_bt_node)

add_library(nav2_send_whitelist_action_bt_node SHARED plugins/action/send_whitelist.cpp)
list(APPEND plugin_libs nav2_send_whitelist_action_bt_node)

add_library(nav2_set_blackboard_action_bt_node SHARED plugins/action/set_blackboard.cpp)
list(APPEND plugin_libs nav2_set_blackboard_action_bt_node)

add_library(nav2_random_move_action_bt_node SHARED plugins/action/random_move.cpp)
list(APPEND plugin_libs nav2_random_move_action_bt_node)

add_library(nav2_random_move_around_action_bt_node SHARED plugins/action/random_move_around.cpp)
list(APPEND plugin_libs nav2_random_move_around_action_bt_node)

add_library(nav2_update_goal_action_bt_node SHARED plugins/action/update_goal.cpp)
list(APPEND plugin_libs nav2_update_goal_action_bt_node)

add_library(nav2_send_to_autoaim_action_bt_node SHARED plugins/action/send_to_autoaim.cpp)
list(APPEND plugin_libs nav2_send_to_autoaim_action_bt_node)

add_library(nav2_change_mode_action_bt_node SHARED plugins/action/change_mode.cpp)
list(APPEND plugin_libs nav2_change_mode_action_bt_node)

add_library(nav2_follow_aim_action_bt_node SHARED plugins/action/follow_aim.cpp)
list(APPEND plugin_libs nav2_follow_aim_action_bt_node)

add_library(nav2_force_move_action_bt_node SHARED plugins/action/force_move.cpp)
list(APPEND plugin_libs nav2_force_move_action_bt_node)

add_library(nav2_routine_patrol_action_bt_node SHARED plugins/action/routine_patrol.cpp)
list(APPEND plugin_libs nav2_routine_patrol_action_bt_node)

add_library(nav2_record_passing_tunnel_index_action_bt_node SHARED plugins/action/record_passing_tunnel_index.cpp)
list(APPEND plugin_libs nav2_record_passing_tunnel_index_action_bt_node)

add_library(nav2_record_command_control_action_bt_node SHARED plugins/action/record_command_control.cpp)
list(APPEND plugin_libs nav2_record_command_control_action_bt_node)

add_library(nav2_change_map_action_bt_node SHARED plugins/action/change_map.cpp)
list(APPEND plugin_libs nav2_change_map_action_bt_node)

add_library(nav2_wait_for_input_period_action_bt_node SHARED plugins/action/wait_for_input_period.cpp)
list(APPEND plugin_libs nav2_wait_for_input_period_action_bt_node)

add_library(nav2_record_autoaim_action_bt_node SHARED plugins/action/record_autoaim.cpp)
list(APPEND plugin_libs nav2_record_autoaim_action_bt_node)

add_library(nav2_get_out_of_stuck_action_bt_node SHARED plugins/action/get_out_of_stuck.cpp)
list(APPEND plugin_libs nav2_get_out_of_stuck_action_bt_node)

add_library(nav2_truncate_path_action_bt_node SHARED plugins/action/truncate_path.cpp)
list(APPEND plugin_libs nav2_truncate_path_action_bt_node)

add_library(nav2_force_quit_command_action_bt_node SHARED plugins/action/force_quit_command.cpp)
list(APPEND plugin_libs nav2_force_quit_command_action_bt_node)

add_library(nav2_change_patrol_pitch_action_bt_node SHARED plugins/action/change_patrol_pitch.cpp)
list(APPEND plugin_libs nav2_change_patrol_pitch_action_bt_node)

add_library(nav2_set_gimbal_patrol_action_bt_node SHARED plugins/action/set_gimbal_patrol.cpp)
list(APPEND plugin_libs nav2_set_gimbal_patrol_action_bt_node)

add_library(nav2_set_spin_action_bt_node SHARED plugins/action/set_spin.cpp)
list(APPEND plugin_libs nav2_set_spin_action_bt_node)

add_library(nav2_set_aim_mode_action_bt_node SHARED plugins/action/set_aim_mode.cpp)
list(APPEND plugin_libs nav2_set_aim_mode_action_bt_node)

add_library(nav2_update_goal_dafu_action_bt_node SHARED plugins/action/update_goal_dafu.cpp)
list(APPEND plugin_libs nav2_update_goal_dafu_action_bt_node)

add_library(nav2_if_coin_more_than_input_condition_bt_node SHARED plugins/condition/if_coin_more_than_input.cpp)
list(APPEND plugin_libs nav2_if_coin_more_than_input_condition_bt_node)

add_library(nav2_is_rfid_in_recovery_condition_bt_node SHARED plugins/condition/is_rfid_in_recovery.cpp)
list(APPEND plugin_libs nav2_is_rfid_in_recovery_condition_bt_node)

add_library(nav2_is_rfid_in_fort_condition_bt_node SHARED plugins/condition/is_rfid_in_fort.cpp)
list(APPEND plugin_libs nav2_is_rfid_in_fort_condition_bt_node)

add_library(nav2_is_rfid_in_center_condition_bt_node SHARED plugins/condition/is_rfid_in_center.cpp)
list(APPEND plugin_libs nav2_is_rfid_in_center_condition_bt_node)

add_library(nav2_if_robot_stuck_condition_bt_node SHARED plugins/condition/if_robot_stuck.cpp)
list(APPEND plugin_libs nav2_if_robot_stuck_condition_bt_node)

add_library(nav2_if_allowance_shoot_less_than_input_condition_bt_node SHARED plugins/condition/if_allowance_shoot_less_than_input.cpp)
list(APPEND plugin_libs nav2_if_allowance_shoot_less_than_input_condition_bt_node)

add_library(nav2_is_out_fight_condition_bt_node SHARED plugins/condition/is_out_fight.cpp)
list(APPEND plugin_libs nav2_is_out_fight_condition_bt_node)

add_library(nav2_wait_for_game_start_condition_bt_node SHARED plugins/condition/wait_for_game_start.cpp)
list(APPEND plugin_libs nav2_wait_for_game_start_condition_bt_node)

add_library(nav2_if_in_command_condition_bt_node SHARED plugins/condition/if_in_command.cpp)
list(APPEND plugin_libs nav2_if_in_command_condition_bt_node)

add_library(nav2_if_cancel_resurrection_condition_bt_node SHARED plugins/condition/if_cancel_resurrection.cpp)
list(APPEND plugin_libs nav2_if_cancel_resurrection_condition_bt_node)

add_library(nav2_if_remain_time_in_input_condition_bt_node SHARED plugins/condition/if_remain_time_in_input.cpp)
list(APPEND plugin_libs nav2_if_remain_time_in_input_condition_bt_node)

add_library(nav2_if_appoint_robot_blood_less_than_input_condition_bt_node SHARED plugins/condition/if_appoint_robot_blood_less_than_input.cpp)
list(APPEND plugin_libs nav2_if_appoint_robot_blood_less_than_input_condition_bt_node)

add_library(nav2_if_in_ramp_condition_bt_node SHARED plugins/condition/if_in_ramp.cpp)
list(APPEND plugin_libs nav2_if_in_ramp_condition_bt_node)

add_library(nav2_is_sentry_hurt_condition_bt_node SHARED plugins/condition/is_sentry_hurt.cpp)
list(APPEND plugin_libs nav2_is_sentry_hurt_condition_bt_node)

add_library(nav2_is_robot_shoot_condition_bt_node SHARED plugins/condition/is_robot_shoot.cpp)
list(APPEND plugin_libs nav2_is_robot_shoot_condition_bt_node)

add_library(nav2_is_goal_update_condition_bt_node SHARED plugins/condition/is_goal_update.cpp)
list(APPEND plugin_libs nav2_is_goal_update_condition_bt_node)

add_library(nav2_is_tunnel_dangerous_condition_bt_node SHARED plugins/condition/is_tunnel_dangerous.cpp)
list(APPEND plugin_libs nav2_is_tunnel_dangerous_condition_bt_node)

add_library(nav2_is_our_dart_hit_condition_bt_node SHARED plugins/condition/is_our_dart_hit.cpp)
list(APPEND plugin_libs nav2_is_our_dart_hit_condition_bt_node)

add_library(nav2_is_enemy_dart_hit_condition_bt_node SHARED plugins/condition/is_enemy_dart_hit.cpp)
list(APPEND plugin_libs nav2_is_enemy_dart_hit_condition_bt_node)

add_library(nav2_is_robot_in_weak_condition_bt_node SHARED plugins/condition/is_robot_in_weak.cpp)
list(APPEND plugin_libs nav2_is_robot_in_weak_condition_bt_node)

add_library(nav2_if_recovery_bullet_less_than_input_condition_bt_node SHARED plugins/condition/if_recovery_bullet_less_than_input.cpp)
list(APPEND plugin_libs nav2_if_recovery_bullet_less_than_input_condition_bt_node)

add_library(nav2_if_appoint_robot_in_target_area_condition_bt_node SHARED plugins/condition/if_appoint_robot_in_target_area.cpp)
list(APPEND plugin_libs nav2_if_appoint_robot_in_target_area_condition_bt_node)

add_library(nav2_if_select_go_fort_condition_bt_node SHARED plugins/condition/if_select_go_fort.cpp)
list(APPEND plugin_libs nav2_if_select_go_fort_condition_bt_node)

add_library(nav2_if_mode_change_condition_bt_node SHARED plugins/condition/if_mode_change.cpp)
list(APPEND plugin_libs nav2_if_mode_change_condition_bt_node)

add_library(nav2_is_goal_reached_condition_bt_node SHARED plugins/condition/is_goal_reached.cpp)
list(APPEND plugin_libs nav2_is_goal_reached_condition_bt_node)

add_library(nav2_if_robot_referee_offline_condition_bt_node SHARED plugins/condition/if_robot_referee_offline.cpp)
list(APPEND plugin_libs nav2_if_robot_referee_offline_condition_bt_node)

add_library(nav2_if_in_chase_condition_bt_node SHARED plugins/condition/if_in_chase.cpp)
list(APPEND plugin_libs nav2_if_in_chase_condition_bt_node)

add_library(nav2_if_in_dafu_status_condition_bt_node SHARED plugins/condition/if_in_dafu_status.cpp)
list(APPEND plugin_libs nav2_if_in_dafu_status_condition_bt_node)

add_library(nav2_user_force_success_decorator_bt_node SHARED plugins/decorator/user_force_success.cpp)
list(APPEND plugin_libs nav2_user_force_success_decorator_bt_node)

add_library(nav2_user_force_failure_decorator_bt_node SHARED plugins/decorator/user_force_failure.cpp)
list(APPEND plugin_libs nav2_user_force_failure_decorator_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)
ament_export_dependencies(${dependencies})


ament_package()
