#include "safety_monitor.h"
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <thread>

#define CHECK(condition) do { if (!(condition)) { \
  std::cerr << "Failed at line " << __LINE__ << ": " #condition << '\n'; \
  std::exit(1); } } while (false)

int main()
{
  using Fault = SafetyMonitor::Fault;
  // No valid pose at startup: preserve the existing no-auto-land behavior.
  SafetyMonitor startup(0.2, 0.05, 3);
  CHECK(startup.fault(100.0) == Fault::None);
  CHECK(startup.accept(100.0, 0.0));

  // Source age consumes the deadline, even if callbacks keep publishing.
  SafetyMonitor delayed(0.2, 0.05, 3);
  CHECK(delayed.accept(1.0, 0.15));
  CHECK(delayed.accept(1.02, 0.169)); // newer callback, almost equally old measurement
  CHECK(delayed.fault(1.04) == Fault::None);
  CHECK(delayed.fault(1.06) == Fault::Timeout);
  CHECK(!delayed.accept(1.06, 0.21));
  CHECK(delayed.fault(1.06) == Fault::Timeout);
  CHECK(delayed.accept(1.07, 0.0));
  CHECK(delayed.fault(1.08) == Fault::None);

  // An isolated bad pose does not trigger the fast failure path.
  SafetyMonitor burst(0.2, 0.05, 3);
  CHECK(burst.accept(2.0, 0.0));
  burst.reject(2.01);
  CHECK(burst.fault(2.08) == Fault::None);
  CHECK(burst.accept(2.09, 0.0));
  burst.reject(2.10);
  burst.reject(2.11);
  burst.reject(2.12);
  CHECK(burst.fault(2.14) == Fault::None);
  CHECK(burst.fault(2.16) == Fault::InvalidSequence);
  CHECK(burst.accept(2.17, 0.0));
  CHECK(burst.fault(2.18) == Fault::None);

  CHECK(burst.fault(2.40) == Fault::Timeout);

  // Separate reader remains usable while producer does not supply new poses.
  SafetyMonitor concurrent(0.2, 0.05, 3);
  CHECK(concurrent.accept(3.0, 0.0));
  std::atomic<bool> saw_timeout(false);
  std::thread watchdog([&] {
    saw_timeout.store(concurrent.fault(3.21) == Fault::Timeout);
  });
  watchdog.join();
  CHECK(saw_timeout.load());
  std::thread producer([&] {
    for (int i = 0; i < 10000; ++i) concurrent.accept(4.0 + i * 0.005, 0.0);
  });
  std::thread reader([&] {
    for (int i = 0; i < 10000; ++i) concurrent.fault(4.0 + i * 0.005);
  });
  producer.join(); reader.join();
  CHECK(concurrent.fault(54.3) == Fault::Timeout);
  std::cout << "PASS: startup, source freshness, isolated/burst errors, concurrency\n";
}
