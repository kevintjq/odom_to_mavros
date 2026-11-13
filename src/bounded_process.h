#ifndef ODOM_TO_MAVROS_BOUNDED_PROCESS_H
#define ODOM_TO_MAVROS_BOUNDED_PROCESS_H

#include <cerrno>
#include <csignal>
#include <spawn.h>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

extern char** environ;

// Linux/POSIX single-owner worker. Poll never waits for a running child.
// Unlike detached threads, timed-out workers are killed and reaped before reuse.
class BoundedProcess
{
public:
  enum class Result { Idle, Running, Success, Failed, TimedOut };
  BoundedProcess() = default;
  BoundedProcess(const BoundedProcess&) = delete;
  BoundedProcess& operator=(const BoundedProcess&) = delete;
  ~BoundedProcess()
  {
    if (pid_ <= 0) return;
    ::kill(pid_, SIGKILL);
    int status;
    while (::waitpid(pid_, &status, 0) < 0 && errno == EINTR) {}
  }

  bool running() const { return pid_ > 0; }
  int start(const std::string& executable, std::vector<std::string> args,
            double now, double timeout)
  {
    if (running()) return EBUSY;
    args.insert(args.begin(), executable);
    std::vector<char*> argv;
    for (auto& arg : args) argv.push_back(&arg[0]);
    argv.push_back(nullptr);
    pid_t child = -1;
    // posix_spawn avoids running C++/ROS code after fork in a multithreaded node.
    const int error = ::posix_spawn(&child, executable.c_str(), nullptr, nullptr,
                                    argv.data(), environ);
    if (error != 0) return error;
    pid_ = child;
    started_ = now;
    timeout_ = timeout;
    timed_out_ = false;
    return 0;
  }

  Result poll(double now)
  {
    if (!running()) return Result::Idle;
    int status = 0;
    pid_t reaped = ::waitpid(pid_, &status, WNOHANG);
    if (reaped == 0) {
      if (!timed_out_ && now - started_ >= timeout_) {
        ::kill(pid_, SIGKILL);
        timed_out_ = true;
      }
      return Result::Running; // Reap on a subsequent tick; never wait here.
    }
    if (reaped < 0 && errno != ECHILD) return Result::Running;
    pid_ = -1;
    if (timed_out_) return Result::TimedOut;
    return reaped > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0
        ? Result::Success : Result::Failed;
  }

private:
  pid_t pid_ = -1;
  double started_ = 0.0;
  double timeout_ = 0.0;
  bool timed_out_ = false;
};
#endif
