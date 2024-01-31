
#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/type/rclcpp_system.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"
#include <chrono>

// using time_t = std::chrono::nanoseconds;
// using milliseconds = std::chrono::milliseconds;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // std::vector<std::shared_ptr<typename SystemType::NodeBaseType>> nodes;
  
  auto pub_node = std::make_shared<MinimalPublisher>();
  auto sub_node = std::make_shared<MinimalSubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(pub_node);
  executor.add_node(sub_node);
  
  executor.spin();

  rclcpp::shutdown();

  return 0;
}