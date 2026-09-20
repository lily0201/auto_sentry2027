cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Transmit/Action.msg"
        "msg/Transmit/Decision.msg"
        "msg/Transmit/Map.msg"
        "msg/Transmit/Info.msg"
        "msg/Transmit/Velocity.msg"
        "msg/Transmit/SelfPosition.msg"
        "msg/Transmit/Whitelist.msg"
        "msg/Receive/Buff.msg"
        "msg/Receive/HurtData.msg"
        "msg/Receive/Dartinfo.msg"
        "msg/Receive/Event.msg"
        "msg/Receive/Gamestatus.msg"
        "msg/Receive/Hp.msg"
        "msg/Receive/Interaction.msg"
        "msg/Receive/Mapcommand.msg"
        "msg/Receive/Projectileallowance.msg"
        "msg/Receive/Rfidstatus.msg"
        "msg/Receive/Robotp.msg"
        "msg/Receive/Robotposition.msg"
        "msg/Receive/Robotstatus.msg"
        "msg/Receive/Sentryinfo.msg"
        "msg/Receive/Lidarposition.msg"
        "msg/Receive/Lidarstation.msg"
        "msg/Receive/AngleError.msg"
        "msg/Receive/EnemyPosition.msg"
        "msg/Receive/EnemyPositions.msg"
        "msg/Receive/AutoaimStrategy.msg"
        "msg/Receive/EnemyChasing.msg"
        "msg/Receive/TruncatePathInfo.msg"
        "srv/EnemyStrategist.srv"
        DEPENDENCIES std_msgs geometry_msgs
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

ament_package()
