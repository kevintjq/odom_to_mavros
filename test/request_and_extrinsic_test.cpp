#include "bounded_process.h"
#include "extrinsic_validation.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>

#define CHECK(x) do { if (!(x)) { std::cerr << "Failed line " << __LINE__ << ": " #x << '\n'; std::exit(1); } } while(false)

BoundedProcess::Result finish(BoundedProcess& process, double now)
{
  for (int i = 0; i < 1000; ++i) {
    const auto result = process.poll(now);
    if (result != BoundedProcess::Result::Running) return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cerr << "Child failed to terminate/reap\n";
  std::exit(1);
}

int main()
{
  using R = BoundedProcess::Result;
  BoundedProcess worker;
  CHECK(worker.poll(0) == R::Idle);
  CHECK(worker.start("/definitely/not/a/program", {}, 0, .3) != 0);
  CHECK(!worker.running());
  CHECK(worker.start("/bin/true", {}, 0, .3) == 0);
  CHECK(finish(worker, 0) == R::Success);
  CHECK(worker.start("/bin/false", {}, 0, .3) == 0);
  CHECK(finish(worker, 0) == R::Failed);
  // Simulate a permanently blocked service with a long-running child.
  for (int i = 0; i < 20; ++i) {
    CHECK(worker.start("/bin/sleep", {"60"}, 1, .3) == 0);
    CHECK(worker.start("/bin/true", {}, 1, .3) == EBUSY);
    CHECK(worker.poll(1.1) == R::Running);
    CHECK(finish(worker, 1.31) == R::TimedOut);
    CHECK(!worker.running());
    CHECK(worker.start("/bin/true", {}, 2, .3) == 0);
    CHECK(finish(worker, 2) == R::Success);
  }
  // No zombies after retries or destruction of an active request.
  { BoundedProcess pending; CHECK(pending.start("/bin/sleep", {"60"}, 0, .3) == 0); }
  int status = 0;
  CHECK(waitpid(-1, &status, WNOHANG) == -1 && errno == ECHILD);

  const auto deg = validatedExtrinsic({{1, -2, 3, 90, -180, 45}}, "deg");
  CHECK(std::abs(deg[3] - std::acos(-1.0) / 2) < 1e-12);
  CHECK(deg[0] == 1 && deg[1] == -2 && deg[2] == 3);
  for (const std::string unit : {"rad", "radian", "radians", "deg", "degree", "degrees"})
    CHECK(std::isfinite(validatedExtrinsic({{0, 0, 0, 0, 0, 0}}, unit)[5]));
  for (const std::string unit : {"", "degg", "RAD"}) {
    bool threw = false;
    try { validatedExtrinsic({{0, 0, 0, 0, 0, 0}}, unit); }
    catch (const std::runtime_error&) { threw = true; }
    CHECK(threw);
  }
  for (unsigned axis = 0; axis < 6; ++axis) {
    for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity()}) {
      std::array<double, 6> input = {{0, 0, 0, 0, 0, 0}};
      input[axis] = bad;
      bool threw = false;
      try { validatedExtrinsic(input, "rad"); }
      catch (const std::runtime_error&) { threw = true; }
      CHECK(threw);
    }
  }
  std::cout << "PASS: blocked worker timeout/retry/reaping; spawn/error/success; strict extrinsic validation\n";
}
