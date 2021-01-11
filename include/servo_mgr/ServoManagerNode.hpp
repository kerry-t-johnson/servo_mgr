// Copyright 2020 Kerry Johnson
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef SERVOMANAGERNODE_HPP_
#define SERVOMANAGERNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "servo_mgr/msg/servo_control.hpp"
#include "servo_mgr/srv/configure_pca9685.hpp"
#include "servo_mgr/srv/configure_servo.hpp"
#include "servo_mgr/srv/test_servo.hpp"
#include "servo_mgr/ServoManagerVisibilityControl.hpp"

class ServoManager;

namespace servo_mgr {

class ServoManagerNode: public rclcpp::Node
{
public:
  SERVO_MGR_PUBLIC
  explicit ServoManagerNode(const rclcpp::NodeOptions &options);
  virtual ~ServoManagerNode();

private:
  void onConfigurePCA9685(const servo_mgr::srv::ConfigurePCA9685::Request::SharedPtr request,
                          servo_mgr::srv::ConfigurePCA9685::Response::SharedPtr response);

  void onConfigureServo(const servo_mgr::srv::ConfigureServo::Request::SharedPtr request,
                        servo_mgr::srv::ConfigureServo::Response::SharedPtr response);

  void onTestServo(const servo_mgr::srv::TestServo::Request::SharedPtr request,
                   servo_mgr::srv::TestServo::Response::SharedPtr response);

  void onServoControl(const servo_mgr::msg::ServoControl::SharedPtr msg);
  void onServoControlAbsolute(const servo_mgr::msg::ServoControl::SharedPtr msg);

  rclcpp::Service<servo_mgr::srv::ConfigurePCA9685>::SharedPtr configurePca9685Svc_;
  rclcpp::Service<servo_mgr::srv::ConfigureServo>::SharedPtr configureServoSvc_;
  rclcpp::Service<servo_mgr::srv::TestServo>::SharedPtr testServoSvc_;
  rclcpp::Subscription<servo_mgr::msg::ServoControl>::SharedPtr servoControlSubscriber_;
  rclcpp::Subscription<servo_mgr::msg::ServoControl>::SharedPtr servoControlAbsoluteSubscriber_;
  std::unique_ptr<ServoManager> impl_;
};

}  // namespace servo_mgr

#endif  // SERVOMANAGERNODE_HPP_
