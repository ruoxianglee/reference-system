// Copyright 2021 Apex.AI, Inc.
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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
#include <chrono>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/msg_types.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/sample_management.hpp"
#include "std_msgs/msg/float32.hpp"

namespace nodes
{
namespace rclcpp_system
{

class Sensor : public rclcpp::Node
{
public:
  explicit Sensor(const SensorSettings & settings)
  : Node(settings.node_name)
  {
    publisher_ = this->create_publisher<message_t>(settings.topic_name, 1);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback();});

    cycle_time_ = settings.cycle_time;
    cycle_time_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "modified_cycle_time",
        1,
        std::bind(&Sensor::modified_cycle_time_callback, this, std::placeholders::_1)
    );
  }

private:
  void timer_callback()
  {
    uint64_t timestamp = now_as_int();
    auto message = publisher_->borrow_loaned_message();
    message.get().size = 0;

    set_sample(
      this->get_name(), sequence_number_++, 0, timestamp,
      message.get());

    publisher_->publish(std::move(message));
  }

  void change_timer_period(std::chrono::nanoseconds new_period) {
      timer_->cancel();
      cycle_time_ = new_period;
      timer_ = this->create_wall_timer(
          cycle_time_,
          std::bind(&Sensor::timer_callback, this)
      );
  }

  void modified_cycle_time_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
      auto timeInMs = static_cast<int>(msg->data);
      std::cout << "Changing time frequency to: " << timeInMs << std::endl;
      change_timer_period(std::chrono::milliseconds(timeInMs));
  }

private:
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cycle_time_sub_;
  std::chrono::nanoseconds cycle_time_;
  uint32_t sequence_number_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
