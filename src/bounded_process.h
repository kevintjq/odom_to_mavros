#ifndef ODOM_TO_MAVROS_BOUNDED_PROCESS_H
#define ODOM_TO_MAVROS_BOUNDED_PROCESS_H

#include <cerrno>
#include <csignal>
#include <spawn.h>
#include <string>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

extern char** environ;

// One child at a time, with optional prewarming over a private socket:
// child sends 'R' after initialization, then waits for the parent's 'L'.
// All polling is nonblocking; a timed-out child is killed and reaped before reuse.
class BoundedProcess
{
public:
  enum class Result { Idle, Running, Ready, Success, Failed, TimedOut };
  BoundedProcess() = default;
  BoundedProcess(const BoundedProcess&) = delete;
  BoundedProcess& operator=(const BoundedProcess&) = delete;
  ~BoundedProcess()
  {
    closeControl();
    if (pid_ <= 0) return;
    ::kill(pid_, SIGKILL);
    int status;
    while (::waitpid(pid_, &status, 0) < 0 && errno == EINTR) {}
  }

  bool running() const { return pid_ > 0; }
  bool ready() const { return ready_; }

  int start(const std::string& executable, std::vector<std::string> args,
            double now, double timeout, bool prepare_only = false)
  {
    if (running()) return EBUSY;
    args.insert(args.begin(), executable);
    std::vector<char*> argv;
    for (auto& arg : args) argv.push_back(&arg[0]);
    argv.push_back(nullptr);

    int sockets[2] = {-1, -1};
    posix_spawn_file_actions_t actions;
    if (prepare_only) {
      if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sockets) != 0) return errno;
      int error = ::posix_spawn_file_actions_init(&actions);
      if (error == 0) {
        error = ::posix_spawn_file_actions_addclose(&actions, sockets[0]);
        if (error == 0) error = ::posix_spawn_file_actions_adddup2(&actions, sockets[1], STDIN_FILENO);
        if (error == 0) error = ::posix_spawn_file_actions_addclose(&actions, sockets[1]);
        if (error != 0) ::posix_spawn_file_actions_destroy(&actions);
      }
      if (error != 0) { ::close(sockets[0]); ::close(sockets[1]); return error; }
    }

    pid_t child = -1;
    // Avoid executing C++/ROS code between fork and exec in a multithreaded node.
    const int error = ::posix_spawn(&child, executable.c_str(),
        prepare_only ? &actions : nullptr, nullptr, argv.data(), environ);
    if (prepare_only) {
      ::posix_spawn_file_actions_destroy(&actions);
      ::close(sockets[1]);
    }
    if (error != 0) { if (sockets[0] >= 0) ::close(sockets[0]); return error; }
    pid_ = child;
    control_ = sockets[0];
    started_ = now;
    timeout_ = timeout;
    timed_out_ = false;
    preparing_ = prepare_only;
    ready_ = false;
    return 0;
  }

  int send(double now, double timeout)
  {
    if (!running() || !ready_) return EINVAL;
    if (::send(control_, "L", 1, MSG_NOSIGNAL | MSG_DONTWAIT) != 1) return errno;
    closeControl();
    ready_ = false;
    preparing_ = false;
    started_ = now;
    timeout_ = timeout;
    return 0;
  }

  Result poll(double now)
  {
    if (!running()) return Result::Idle;
    int status = 0;
    const pid_t reaped = ::waitpid(pid_, &status, WNOHANG);
    if (reaped == 0) {
      if (preparing_ && !ready_ && !timed_out_) {
        char message = 0;
        const ssize_t count = ::recv(control_, &message, 1, MSG_DONTWAIT);
        if (count == 1 && message == 'R') ready_ = true;
        else if (count == 0 || count == 1 ||
                 (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
          ::kill(pid_, SIGKILL); // Broken readiness protocol; reap before retrying.
      }
      if (!ready_ && !timed_out_ && now - started_ >= timeout_) {
        ::kill(pid_, SIGKILL);
        timed_out_ = true;
      }
      return ready_ ? Result::Ready : Result::Running;
    }
    if (reaped < 0 && errno != ECHILD) return Result::Running;
    pid_ = -1;
    ready_ = false;
    closeControl();
    if (timed_out_) return Result::TimedOut;
    return reaped > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0
        ? Result::Success : Result::Failed;
  }

private:
  void closeControl() { if (control_ >= 0) { ::close(control_); control_ = -1; } }
  pid_t pid_ = -1;
  int control_ = -1;
  double started_ = 0.0;
  double timeout_ = 0.0;
  bool timed_out_ = false;
  bool preparing_ = false;
  bool ready_ = false;
};
#endif
