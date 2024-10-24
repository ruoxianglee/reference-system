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
#ifndef REFERENCE_SYSTEM__NUMBER_CRUNCHER_HPP_
#define REFERENCE_SYSTEM__NUMBER_CRUNCHER_HPP_

#include <chrono>
#include <cmath>
#include <vector>
#include <thread>

#include<unistd.h>                
#include <random>
// prevents the compiler from optimizing access to value.
// Taken from http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2016/p0412r0.html
template<typename Tp>
inline void escape(Tp const & value)
{
  asm volatile ("" : : "g" (value) : "memory");
}

void sleep_randomly(double mean, double stddev) {
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(mean, stddev);

    // Generate a random sleep duration
    double sleepTime = dist(gen);

    // Ensure that the sleep time is non-negative
    if (sleepTime < 0) {
        sleepTime = 0;
    }

    // Convert the sleep time to milliseconds
    auto sleepDuration = std::chrono::milliseconds(static_cast<int>(sleepTime));

    // Sleep for the generated duration
    std::this_thread::sleep_for(sleepDuration);
}

// Computes an expensive function (count number of primes below maximum_number)
// This serves as a scalable dummy-workload for the various nodes.
static inline int64_t number_cruncher(const uint64_t maximum_number)
{
  // Test 2: T ms - jitter (jitter: uniform real distribution 0-80)
  // std::random_device rd;
  // std::mt19937 gen(rd());
  // std::uniform_real_distribution<> dis(0, 50);
  // double jitter = dis(gen);
  // usleep(150000 - jitter*1000); // 150 ms - jitter

  // switch (maximum_number) {
  //   case 1:
  //     usleep(80000); // 80ms
  //     return 0;
  //   case 2:
  //     usleep(80000); // 150ms
  //     return 0;
  //   case 3:
  //     usleep(80000); // 80ms
  //     return 0;
  //   default:
  //     break;
  // }

  int64_t number_of_primes = 0;
  uint64_t initial_value = 2;
  // edge case where max number is too low
  if (maximum_number <= initial_value) {
    return 2;
  }
  for (uint64_t i = initial_value; i <= maximum_number; ++i) {
    bool is_prime = true;
    for (uint64_t n = initial_value; n < i; ++n) {
      if (i % n == 0) {
        is_prime = false;
        break;
      }
    }
    escape(is_prime);
    if (is_prime) {
      // number_of_primes cannot overflow since there are fewer than 2**63
      // primes in [0, 2**64).
      ++number_of_primes;
    }
  }
  return number_of_primes;
}

// Returns the time (in ms of wall-clock time) needed to compute number_cruncher(maximum_number)
static inline long double get_crunch_time_in_ms(const uint64_t maximum_number)
{
  auto start = std::chrono::system_clock::now();
  number_cruncher(maximum_number);
  auto stop = std::chrono::system_clock::now();

  using milliseconds_ld = std::chrono::duration<long double, std::milli>;
  return milliseconds_ld(stop - start).count();
}

#endif  // REFERENCE_SYSTEM__NUMBER_CRUNCHER_HPP_
