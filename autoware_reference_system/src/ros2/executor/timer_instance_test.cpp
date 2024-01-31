
#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/type/rclcpp_system.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"
#include <chrono>

using time_t = std::chrono::nanoseconds;
using milliseconds = std::chrono::milliseconds;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::vector<std::shared_ptr<typename SystemType::NodeBaseType>> nodes;
  
  nodes.emplace_back(std::make_shared<MinimalSubscriber>());
  nodes.emplace_back(std::make_shared<MinimalPublisher>());

  rclcpp::executors::SingleThreadedExecutor executor;
  for (auto & node : nodes) {
    executor.add_node(node);
  }
  executor.spin();

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}