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
#include "ServoManagerNode.hpp"

#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "i2c_pwm/Pca9685.hpp"
#include "ServoManager.hpp"

ServoManagerNode::ServoManagerNode(rclcpp::NodeOptions options)
  : rclcpp::Node("ServoManager", options), impl_(new ServoManager)
{
  declare_parameter<bool>("use_mock_pca9685", false);

  configurePca9685Svc_ = create_service<servo_mgr::srv::ConfigurePCA9685>("configure_pca9685",
                                                                          std::bind(&ServoManagerNode::onConfigurePCA9685,
                                                                                    this,
                                                                                    std::placeholders::_1,
                                                                                    std::placeholders::_2));

  configureServoSvc_ = create_service<servo_mgr::srv::ConfigureServo>("configure_servo",
                                                                      std::bind(&ServoManagerNode::onConfigureServo,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

  testServoSvc_ = create_service<servo_mgr::srv::TestServo>("test_servo",
                                                            std::bind(&ServoManagerNode::onTestServo,
                                                                      this,
                                                                      std::placeholders::_1,
                                                                      std::placeholders::_2));

  servoControlSubscriber_ = create_subscription<servo_mgr::msg::ServoControl>("servo_control",
                                                                              10,
                                                                              std::bind(&ServoManagerNode::onServoControl,
                                                                                        this,
                                                                                        std::placeholders::_1));

  servoControlAbsoluteSubscriber_ = create_subscription<
    servo_mgr::msg::ServoControl>("servo_control_absolute",
                                  10,
                                  std::bind(&ServoManagerNode::onServoControlAbsolute,
                                            this,
                                            std::placeholders::_1));
}

ServoManagerNode::~ServoManagerNode()
{
}

void ServoManagerNode::onConfigurePCA9685(const std::shared_ptr<
                                            servo_mgr::srv::ConfigurePCA9685::Request> request,
                                          std::shared_ptr<
                                            servo_mgr::srv::ConfigurePCA9685::Response> response)
{
  try {
    const bool mockModeParam = get_parameter("use_mock_pca9685").get_value<bool>();

    if (mockModeParam) {
      RCLCPP_DEBUG(get_logger(), "Servo Manager is using a mock PCA9685");
    }

    i2c_pwm::Pca9685::setMockMode(mockModeParam);

    impl_->configurePca9685(request->id,
                            request->device_file,
                            request->address,
                            request->auto_initialize);
  } catch (const std::exception &ex) {
    response->successful = false;
    response->message = ex.what();
  }
}

void ServoManagerNode::onConfigureServo(const std::shared_ptr<
                                          servo_mgr::srv::ConfigureServo::Request> request,
                                        std::shared_ptr<
                                          servo_mgr::srv::ConfigureServo::Response> response)
{
  try {
    impl_->configureServo(request->id,
                          request->center,
                          request->range,
                          request->invert_direction,
                          request->default_value);
  } catch (const std::exception &ex) {
    response->successful = false;
    response->message = ex.what();
  }
}

void ServoManagerNode::onTestServo(const servo_mgr::srv::TestServo::Request::SharedPtr request,
                                   servo_mgr::srv::TestServo::Response::SharedPtr)
{
  // Convert delay_millis to nanos:
  const struct timespec SERVO_TEST_DELAY = { 0, request->delay_millis * 1000000L };

  for (float i = 0; i < 2 * M_PI * request->num_tests;
    i += request->step_size) {
    const float value = ::sin(i);
    impl_->setServo(request->id, value);
    ::nanosleep(&SERVO_TEST_DELAY, NULL);
  }

  impl_->resetServo(request->id);
}

void ServoManagerNode::onServoControl(const servo_mgr::msg::ServoControl::SharedPtr msg)
{
  try {
    impl_->setServo(msg->servo_id, msg->value);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(get_logger(), "Error in servo_control: %s", ex.what());
  }
}

void ServoManagerNode::onServoControlAbsolute(const servo_mgr::msg::ServoControl::SharedPtr msg)
{
  try {
    impl_->setServo(msg->servo_id, static_cast<uint16_t>(msg->value));
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(get_logger(),
                 "Error in servo_control_absolute: %s",
                 ex.what());
  }
}
