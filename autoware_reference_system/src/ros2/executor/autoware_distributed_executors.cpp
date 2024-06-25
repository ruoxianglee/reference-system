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
#include <iostream>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/system/type/rclcpp_system.hpp"
#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"
#include "autoware_reference_system/priorities.hpp"

int executor_thread_prio = 0; //nice value
int dummy_task_prio = -10;

void set_rt_properties(int prio, int cpu)
{
  // Method 1: set scheduling policy and priority
  // struct sched_param sched_param = { 0 };
  // sched_param.sched_priority = prio;
  // sched_setscheduler(0, SCHED_OTHER, &sched_param);
    
  // Method 2: set nice value
  // pid_t tid = syscall(SYS_gettid);
  // int ret = setpriority(PRIO_PROCESS, tid, prio);
  // if (ret == 0) {
  //     std::cout << "Nice value of thread " << tid << " set to " << prio << std::endl;
  // } else {
  //     perror("setpriority");
  // }

  // Bind thread to specific CPU core
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu, &cpuset);

  sched_setaffinity(0, sizeof(cpuset), &cpuset);
}

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

  // rclcpp::executors::SingleThreadedExecutor
  //   timer_exe,
  //   tranformer_exe,
  //   filter_exe,
  //   detector_exe,
  //   estimator_exe;
  auto timer_exe = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto tranformer_exe = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto filter_exe = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto detector_exe = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto estimator_exe = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  std::set<std::string> timer_node = {"FrontLidarDriver"};
  std::set<std::string> tranformer_node = {"PointsTransformerFront"};
  std::set<std::string> filter_node = {"RayGroundFilter"};
  std::set<std::string> detector_node = {"EuclideanClusterDetector"};
  std::set<std::string> estimator_node = {"ObjectCollisionEstimator"};

  for (const auto & node : timer_node) {
    timer_exe->add_node(nodes.at(node));
  }
  for (const auto & node : tranformer_node) {
    tranformer_exe->add_node(nodes.at(node));
  }
  for (const auto & node : filter_node) {
    tranformer_exe->add_node(nodes.at(node));
    // filter_exe->add_node(nodes.at(node));
  }
  for (const auto & node : detector_node) {
    tranformer_exe->add_node(nodes.at(node));
    // detector_exe->add_node(nodes.at(node));
  }
  for (const auto & node : estimator_node) {
    estimator_exe->add_node(nodes.at(node));
  }

  int core_ids[5] = {3, 4, 5, 6, 7};

  // std::vector<std::thread> thread_pool;
  // std::vector<std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>> executors = {
  //     timer_exe,
  //     tranformer_exe,
  //     filter_exe,
  //     detector_exe,
  //     estimator_exe};

  // for (int i = 0; i < 3; ++i)
  // {
  //   thread_pool.emplace_back([&, i]()
  //                            {
  //     set_rt_properties(executor_thread_prio, core_ids[i]);
  //     std::cout << "Thread " << i << " is running on CPU: " << sched_getcpu() << std::endl;
  //     while (rclcpp::ok()) {
  //       for (size_t j = i; j < executors.size(); j += 3) {
  //         executors[j]->spin_some(std::chrono::milliseconds(0));
  //       }
  //     } });
  // }

  // for (auto &thread : thread_pool)
  // {
  //   thread.join();
  // }

  std::thread timer_thread {[&]() {
      set_rt_properties(executor_thread_prio, core_ids[0]);
      std::cout << "Thread timer is running on CPU: " << sched_getcpu() << std::endl;
      timer_exe->spin();
    }};
  std::thread tranformer_thread {[&]() {
      set_rt_properties(executor_thread_prio, core_ids[1]);
      std::cout << "Thread tranformer is running on CPU: " << sched_getcpu() << std::endl;
      tranformer_exe->spin();
    }};
  // std::thread filter_thread {[&]() {
  //     set_rt_properties(executor_thread_prio, core_ids[2]);
  //     std::cout << "Thread filter is running on CPU: " << sched_getcpu() << std::endl;
  //     filter_exe->spin();
  //   }};
  // std::thread detector_thread {[&]() {
  //     set_rt_properties(executor_thread_prio, core_ids[3]);
  //     std::cout << "Thread detector is running on CPU: " << sched_getcpu() << std::endl;
  //     detector_exe->spin();
  //   }};
  std::thread estimator_thread {[&]() {
      set_rt_properties(executor_thread_prio, core_ids[4]);
      std::cout << "Thread estimator is running on CPU: " << sched_getcpu() << std::endl;
      estimator_exe->spin();
    }};

  timer_thread.join();
  tranformer_thread.join();
  // filter_thread.join();
  detector_thread.join();
  estimator_thread.join();

  rclcpp::shutdown();
}
