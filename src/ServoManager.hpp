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
#ifndef SERVOMANAGER_HPP_
#define SERVOMANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "i2c_pwm/Pca9685.hpp"

class PwmServo;

class ServoManager
{
public:
  ServoManager();
  virtual ~ServoManager();

  void setAll(uint16_t value);
  void setAll(float value);

  void configurePca9685(uint8_t id,
                        const std::string &deviceFile,
                        int address,
                        bool autoInitialize = true);
  void configureServo(uint16_t id,
                      uint16_t center = i2c_pwm::Pca9685::MAX_VALUE / 2,
                      uint16_t range = i2c_pwm::Pca9685::MAX_VALUE,
                      bool invertDirection = false);
  void setServo(uint16_t id, uint16_t data);
  void setServo(uint16_t id, float data);

private:
  typedef std::vector<std::shared_ptr<i2c_pwm::Pca9685>> pca_board_collection;
  typedef std::map<uint16_t, std::shared_ptr<PwmServo>> servo_collection;

  pca_board_collection pcaBoards_;
  servo_collection servos_;
  rclcpp::Logger logger_;
};

#endif  // SERVOMANAGER_HPP_
