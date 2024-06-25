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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__NEW_TRANSFORM_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__NEW_TRANSFORM_HPP_
#include <chrono>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/msg_types.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"

namespace nodes
{
namespace rclcpp_system
{

class NewTransform : public rclcpp::Node
{
public:
  explicit NewTransform(const NewTransformSettings & settings)
  : Node(settings.node_name),
    number_crunch_limit_(settings.number_crunch_limit),
    dynamic_workload_(settings.dynamic_workload),
    start_time_(this->now())
  {
    subscription_ = this->create_subscription<message_t>(
      settings.input_topic, 1,
      [this](const message_t::SharedPtr msg) {input_callback(msg);});
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 1);

    if(dynamic_workload_)
    {
      std::cout << "Transform node has dynamic workload." << std::endl;
    }
  }

private:
  void dynamic_workloads()
  {
    // Calculate elapsed time
    auto elapsed_time = this->now() - start_time_;
    int64_t elapsed_seconds = elapsed_time.seconds();

    // Determine sleep time based on elapsed time
    uint64_t sleep_time_ms = pre_sleep_time_ms_;
    int super_period = 60;
    int scaled_elapsed_seconds = elapsed_seconds % super_period;
    
    if ((scaled_elapsed_seconds >=0) && (scaled_elapsed_seconds <=30)) {
      sleep_time_ms = 80;
    } 
    else if ((scaled_elapsed_seconds >30) && (scaled_elapsed_seconds <=60)) {
      sleep_time_ms = 120;
    }

    if (sleep_time_ms != pre_sleep_time_ms_)
      std::cout << "Transform workload is set to " << sleep_time_ms << " ms." << std::endl;

    pre_sleep_time_ms_ = sleep_time_ms;

    // Sleep for the determined amount of time
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
  }

  void input_callback(const message_t::SharedPtr input_message)
  {
    uint64_t timestamp = now_as_int();
    // auto number_cruncher_result = number_cruncher(number_crunch_limit_);

    if(dynamic_workload_){
      dynamic_workloads();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    auto output_message = publisher_->borrow_loaned_message();
    output_message.get().size = 0;
    merge_history_into_sample(output_message.get(), input_message);

    uint32_t missed_samples = get_missed_samples_and_update_seq_nr(
      input_message, input_sequence_number_);

    std::string node_name = this->get_name();
    if (node_name == "PointsTransformerFront")
    {
      std::cout << "PointsTransformerFront message ID: " << input_sequence_number_ << std::endl;
    }

    set_sample(
      this->get_name(), sequence_number_++, missed_samples, timestamp,
      output_message.get());

    // use result so that it is not optimizied away by some clever compiler
    output_message.get().data[0] = number_cruncher_result;
    publisher_->publish(std::move(output_message));

    if (node_name == "ObjectCollisionEstimator")
    {
      // hot path latency
      // uint64_t hot_path_latency_in_ns = 0;
      // uint64_t root_timestamp = 0;
      // const auto settings = SampleManagementSettings::get();
      // std::cout << "sample size: " << sample->size << std::endl;
      // for (uint64_t i = 0; i < sample->size; ++i) {
      //   uint64_t idx = sample->size - i - 1;
      //   std::string current_node_name(
      //     reinterpret_cast<const char *>(sample->stats[idx].node_name.data()));

      //   if (settings.is_hot_path_root(current_node_name)) {
      //     root_timestamp = std::max(root_timestamp, sample->stats[idx].timestamp);
      //     std::cout << "Got root timestamp " << root_timestamp << std::endl;
      //   }
      // }
      // hot_path_latency_in_ns = timestamp - root_timestamp;

      // hot path drops
      // uint64_t hot_path_drops = 0;
      // for (uint64_t i = 0; i < sample->size; ++i) {
      //   uint64_t idx = sample->size - i - 1;
      //   std::string current_node_name(
      //     reinterpret_cast<const char *>(sample->stats[idx].node_name.data()));
      //   if (settings.is_hot_path_node(current_node_name) > 0) {
      //     hot_path_drops += sample->stats[idx].dropped_samples;
      //     std::cout << "Got message drops " << hot_path_drops << std::endl;
      //   }
      // }
      // hot_path_drops += missed_samples;
      print_statistic_infomation(this->get_name(), missed_samples, input_message, timestamp);
    }
  }

private:
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::Subscription<message_t>::SharedPtr subscription_;
  uint64_t number_crunch_limit_;
  uint32_t sequence_number_ = 0;
  uint32_t input_sequence_number_ = 0;
  bool dynamic_workload_;
  rclcpp::Time start_time_;
  uint64_t pre_sleep_time_ms_ = 80;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__NEW_TRANSFORM_HPP_
