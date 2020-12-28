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
#include "ServoManager.hpp"
#include <memory>
#include <string>
#include "i2c_pwm/Pca9685.hpp"
#include "PwmServo.hpp"

namespace {
const uint8_t MAX_BOARDS = 62;
}  // namespace

ServoManager::ServoManager()
  : pcaBoards_(MAX_BOARDS), logger_(rclcpp::get_logger("ServoManager"))
{
}

ServoManager::~ServoManager()
{
}

void ServoManager::configurePca9685(uint8_t id,
                                    const std::string &deviceFile,
                                    int address,
                                    bool autoInitialize)
{
  if (pcaBoards_[id]) {
    std::ostringstream ostr;
    ostr << "PCA9685 board " << id << " already exists";
    throw std::runtime_error(ostr.str());
  }

  std::shared_ptr<i2c_pwm::Pca9685> pcaBoard = i2c_pwm::Pca9685::create(deviceFile,
                                                                        address,
                                                                        autoInitialize);
  pcaBoards_[id] = pcaBoard;

  RCLCPP_INFO(logger_, "Created PCA9685 board at slot %d", id);
}

void ServoManager::configureServo(uint16_t id,
                                  uint16_t center,
                                  uint16_t range,
                                  bool invertDirection)
{
  const uint8_t boardId = id / i2c_pwm::Pca9685::NUM_CHANNELS;

  std::shared_ptr<i2c_pwm::Pca9685> pcaBoard = pcaBoards_[boardId];
  if (!pcaBoard) {
    std::ostringstream ostr;
    ostr << "PCA9685 board " << boardId << " does not exist";
    throw std::runtime_error(ostr.str());
  }

  servo_collection::const_iterator findIter = servos_.find(id);
  std::shared_ptr<PwmServo> servo;

  if (findIter != servos_.end()) {
    servo = findIter->second;
  } else {
    servo = std::shared_ptr<PwmServo>(new PwmServo(pcaBoard, id));
    servos_[id] = servo;

    RCLCPP_INFO(logger_,
                "Created Servo %d (Board: %d, Channel: %d)",
                id,
                boardId,
                id % i2c_pwm::Pca9685::NUM_CHANNELS);
  }

  servo->configure(center, range, invertDirection);
}

void ServoManager::setAll(uint16_t value)
{
  for (auto iter = servos_.begin(); iter != servos_.end(); ++iter) {
    iter->second->set(value);
  }
}

void ServoManager::setAll(float value)
{
  for (auto iter = servos_.begin(); iter != servos_.end(); ++iter) {
    iter->second->set(value);
  }
}

void ServoManager::setServo(uint16_t id, uint16_t data)
{
  servo_collection::const_iterator iter = servos_.find(id);

  if (iter != servos_.end()) {
    iter->second->set(data);
  } else {
    std::ostringstream ostr;
    ostr << "Servo " << id << " does not exist";
    throw std::runtime_error(ostr.str());
  }
}

void ServoManager::setServo(uint16_t id, float data)
{
  servo_collection::const_iterator iter = servos_.find(id);

  if (iter != servos_.end()) {
    iter->second->set(data);
  } else {
    std::ostringstream ostr;
    ostr << "Servo " << id << " does not exist";
    throw std::runtime_error(ostr.str());
  }
}

