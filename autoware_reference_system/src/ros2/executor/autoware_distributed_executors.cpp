// Copyright 2021 Robert Bosch GmbH
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

#include <memory>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/type/rclcpp_system.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"
#include "autoware_reference_system/priorities.hpp"

void set_rt_properties(int prio, int cpu)
{
  struct sched_param sched_param = { 0 };
  sched_param.sched_priority = prio;
  sched_setscheduler(0, SCHED_FIFO, &sched_param);

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu, &cpuset);

  sched_setaffinity(0, sizeof(cpuset), &cpuset);
}

void cpu_dummy_task() {
    // while (true) {
    //     volatile unsigned long long sum = 0;
    //     for (unsigned long long i = 0; i < 1000000000; ++i) {
    //         sum += i;
    //     }
    // }
    
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_real_distribution<> dis(0, 50);
    // double jitter = dis(gen);
    // usleep(150000 - jitter*1000); // 150 ms - jitter

    // usleep(150000);
    sleep_randomly(80,5);
}

int executor_thread_prio = 98;
int dummy_task_prio = 99;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  auto nodes_vec = create_autoware_simplied_nodes<RclcppSystem, TimeConfig>();
  using NodeMap = std::unordered_map<std::string,
      std::shared_ptr<RclcppSystem::NodeBaseType>>;

  NodeMap nodes;
  for (const auto & node : nodes_vec) {
    nodes.emplace(node->get_name(), node);
    std::cout << node->get_name() << "\n";
  }

  rclcpp::executors::SingleThreadedExecutor
    timer_exe,
    tranformer_exe,
    filter_exe,
    detector_exe,
    estimator_exe;

  std::set<std::string> timer_node = {"FrontLidarDriver"};
  std::set<std::string> tranformer_node = {"PointsTransformerFront"};
  std::set<std::string> filter_node = {"RayGroundFilter"};
  std::set<std::string> detector_node = {"EuclideanClusterDetector"};
  std::set<std::string> estimator_node = {"ObjectCollisionEstimator"};

  for (const auto & node : timer_node) {
    timer_exe.add_node(nodes.at(node));
  }
  for (const auto & node : tranformer_node) {
    tranformer_exe.add_node(nodes.at(node));
  }
  for (const auto & node : filter_node) {
    std::cout << node << "\n";
    filter_exe.add_node(nodes.at(node));
  }
  for (const auto & node : detector_node) {
    detector_exe.add_node(nodes.at(node));
  }
  for (const auto & node : estimator_node) {
    estimator_exe.add_node(nodes.at(node));
  }

  int core_ids[5] = {3, 4, 5, 6, 7};

  // Launch multiple dummy tasks
  // int num_of_dummy_tasks = 1;  // Adjust for more contention
  // std::vector<std::thread> dummy_threads;
  // for (int i = 0; i < num_of_dummy_tasks; ++i) {
  //     dummy_threads.emplace_back([core_ids[i+1]]() {
  //         set_rt_properties(dummy_task_prio, core_ids[i+1]);
  //         cpu_dummy_task();
  //     });
  // }

  std::thread timer_thread {[&]() {
      set_rt_properties(executor_thread_prio, core_ids[0]);
      std::cout << "Thread timer is running on CPU: " << sched_getcpu() << std::endl;
      timer_exe.spin();
    }};
  std::thread tranformer_thread {[&]() {
      set_rt_properties(executor_thread_prio, core_ids[1]);
      std::cout << "Thread tranformer is running on CPU: " << sched_getcpu() << std::endl;
      tranformer_exe.spin();
    }};
  std::thread filter_thread {[&]() {
      set_rt_properties(executor_thread_prio, core_ids[2]);
      std::cout << "Thread filter is running on CPU: " << sched_getcpu() << std::endl;
      filter_exe.spin();
    }};
  std::thread detector_thread {[&]() {
      set_rt_properties(executor_thread_prio, core_ids[3]);
      std::cout << "Thread detector is running on CPU: " << sched_getcpu() << std::endl;
      detector_exe.spin();
    }};
  std::thread estimator_thread {[&]() {
      set_rt_properties(executor_thread_prio, core_ids[4]);
      std::cout << "Thread estimator is running on CPU: " << sched_getcpu() << std::endl;
      estimator_exe.spin();
    }};

  timer_thread.join();
  tranformer_thread.join();
  filter_thread.join();
  detector_thread.join();
  estimator_thread.join();

  // Optionally, keep the main thread alive
  // for (auto& thread : dummy_threads) {
  //     thread.join();
  // }

  rclcpp::shutdown();
}
