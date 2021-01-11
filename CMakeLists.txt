cmake_minimum_required(VERSION 3.5)
project(servo_mgr)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(i2c_pwm REQUIRED)
find_package(rosidl_cmake REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  srvs/ConfigurePCA9685.srv
  srvs/ConfigureServo.srv
  srvs/TestServo.srv
  msgs/ServoControl.msg
  LIBRARY_NAME ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED
  src/PwmServo.cpp
  src/ServoManager.cpp
  src/ServoManagerNode.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "i2c_pwm"
  "rclcpp"
  "rclcpp_components"
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "servo_mgr::ServoManagerNode"
)

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(TARGETS
  ${PROJECT_NAME}
  INCLUDES DESTINATION include
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

rosidl_target_interfaces(${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  "rosidl_typesupport_cpp"
)

ament_target_dependencies(${PROJECT_NAME}
  i2c_pwm
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  # Include linting tests
  find_package(ament_cmake_cpplint)
  ament_cpplint(FILTERS "-whitespace/line_length")

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
    ament_cmake_cppcheck
  )

  ament_lint_auto_find_test_dependencies()

  include_directories(src)

  ament_add_gtest(${PROJECT_NAME}_test
    test/PwmServoTest.cpp
  )

  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    i2c_pwm
  )

  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )

endif()


ament_package()
