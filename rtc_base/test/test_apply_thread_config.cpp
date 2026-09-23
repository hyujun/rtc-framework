// ── ApplyThreadConfig: the thread is named even when the config is refused ──
//
// `ps -L`, `/proc/<pid>/task/*/comm` and verify_rt_runtime.sh all find RT
// threads BY NAME. ApplyThreadConfig used to set the name last, after the
// affinity and scheduler calls, and to return early when either failed — so on
// a host without RT permission (EPERM) every thread kept its creator's name,
// and the one thread whose configuration failed was precisely the one no tool
// could identify. Naming needs no privilege, so it now happens first.
//
// The failure path is forced by lowering this process's soft RLIMIT_RTPRIO to
// 0 for the duration of the case (always permitted, restored afterwards): an
// unprivileged SCHED_FIFO request then fails with EPERM. A process with
// CAP_SYS_NICE (root) bypasses the limit; the case then records that the
// refusal path was NOT exercised instead of reporting a vacuous pass as a
// result about that path.

#include "rtc_base/threading/thread_utils.hpp"

#include <gtest/gtest.h>
#include <pthread.h>
#include <sys/resource.h>

#include <string>
#include <thread>

namespace rtc {
namespace {

TEST(ApplyThreadConfig, NamesTheThreadEvenWhenTheSchedulerRefuses) {
  rlimit saved{};
  ASSERT_EQ(getrlimit(RLIMIT_RTPRIO, &saved), 0);
  rlimit lowered = saved;
  lowered.rlim_cur = 0;
  ASSERT_EQ(setrlimit(RLIMIT_RTPRIO, &lowered), 0);

  bool applied = true;
  std::string name_seen;
  std::thread probe([&] {
    applied = ApplyThreadConfig(ThreadConfig{-1, SCHED_FIFO, 10, 0, "name_probe"});
    char buf[16] = {};
    pthread_getname_np(pthread_self(), buf, sizeof(buf));
    name_seen = buf;
  });
  probe.join();
  ASSERT_EQ(setrlimit(RLIMIT_RTPRIO, &saved), 0);

  RecordProperty("scheduler_path",
                 applied ? "accepted (privileged host: refusal NOT_EVALUATED)" : "refused (EPERM)");
  EXPECT_EQ(name_seen, "name_probe")
      << "a thread whose config was " << (applied ? "accepted" : "refused")
      << " kept its creator's name";
}

TEST(ApplyThreadConfig, AnAcceptedConfigIsStillNamed) {
  std::string name_seen;
  bool applied = false;
  std::thread probe([&] {
    applied = ApplyThreadConfig(ThreadConfig{-1, SCHED_OTHER, 0, 0, "name_ok"});
    char buf[16] = {};
    pthread_getname_np(pthread_self(), buf, sizeof(buf));
    name_seen = buf;
  });
  probe.join();
  EXPECT_TRUE(applied);
  EXPECT_EQ(name_seen, "name_ok");
}

}  // namespace
}  // namespace rtc
