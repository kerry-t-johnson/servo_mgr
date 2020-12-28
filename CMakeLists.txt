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
find_package(i2c_pwm REQUIRED)
find_package(rosidl_cmake REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  srvs/ConfigurePCA9685.srv
  srvs/ConfigureServo.srv
  msgs/ServoControl.msg
  LIBRARY_NAME ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}_library SHARED
  src/PwmServo.cpp
)

ament_target_dependencies(${PROJECT_NAME}_library
  "i2c_pwm"
  "rclcpp"
)

ament_export_targets(export_${PROJECT_NAME}_library HAS_LIBRARY_TARGET)

install(TARGETS
  ${PROJECT_NAME}_library
  EXPORT export_${PROJECT_NAME}_library
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

target_include_directories(${PROJECT_NAME}_library
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(${PROJECT_NAME}
  src/ServoManager.cpp
  src/ServoManagerNode.cpp
  src/main.cpp
)

rosidl_target_interfaces(${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  "rosidl_typesupport_cpp"
)

ament_target_dependencies(${PROJECT_NAME}
  i2c_pwm
  rclcpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_library
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
    ${PROJECT_NAME}_library
  )

endif()


ament_package()
