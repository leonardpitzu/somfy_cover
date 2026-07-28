// Host-side tests for the RX-sync animator used by the io-homecontrol cover.
//
// This is the dead-reckoning logic that keeps the Home Assistant entity in sync
// when someone drives the motor from a physical remote. It has no ESPHome
// dependencies by design, so it compiles and runs standalone here.
//
// The equivalent RTS behaviour is covered end-to-end in test_rts_cover.cpp; the
// RTS cover deliberately keeps its own inlined copy so that the daily-driver
// path is never disturbed by changes made for io-homecontrol.

#include "../../components/somfy/rx_sync_animator.h"

#include <cstdio>

using esphome::somfy::RxSyncAnimator;
using esphome::somfy::RxSyncUpdate;

static int g_checks = 0;
static int g_passed = 0;

static void check(bool ok, const char *name) {
  g_checks++;
  if (ok) {
    g_passed++;
    printf("  PASS  %s\n", name);
  } else {
    printf("  FAIL  %s\n", name);
  }
}

static void check_close(float actual, float expected, float tol, const char *name) {
  const bool ok = std::fabs(actual - expected) <= tol;
  g_checks++;
  if (ok) {
    g_passed++;
    printf("  PASS  %s\n", name);
  } else {
    printf("  FAIL  %s (got %.4f, want %.4f +/- %.4f)\n", name, actual, expected, tol);
  }
}

namespace {

constexpr uint32_t TRAVEL_MS = 10000;

/// Run the animator forward at ~50 Hz, as the ESPHome main loop would.
/// Returns the last update produced.
RxSyncUpdate run_for(RxSyncAnimator &anim, uint32_t &now_ms, uint32_t duration_ms, int *publishes = nullptr) {
  RxSyncUpdate last{0.0f, false, false};
  for (uint32_t elapsed = 0; elapsed < duration_ms && anim.active(); elapsed += 20) {
    now_ms += 20;
    last = anim.update(now_ms, TRAVEL_MS);
    if (publishes != nullptr && last.publish)
      (*publishes)++;
  }
  return last;
}

}  // namespace

/// A closed cover opening must reach the open end stop after a full travel.
static void test_full_open() {
  printf("Full open from closed\n");

  uint32_t now = 1000;
  RxSyncAnimator anim;
  anim.start(/*opening=*/true, 0.0f, now);

  check(anim.active(), "animator is active after start");
  check(anim.opening(), "direction is reported as opening");

  RxSyncUpdate mid = run_for(anim, now, 5000);
  check_close(mid.position, 0.5f, 0.05f, "half open after half the travel time");
  check(!mid.finished, "not finished half way");

  RxSyncUpdate end = run_for(anim, now, 5500);
  check_close(end.position, 1.0f, 0.001f, "lands exactly on the open end stop");
  check(end.finished, "reports finished");
  check(!anim.active(), "animator deactivates itself when finished");
}

/// Closing mirrors opening.
static void test_full_close() {
  printf("Full close from open\n");

  uint32_t now = 1000;
  RxSyncAnimator anim;
  anim.start(/*opening=*/false, 1.0f, now);

  check(!anim.opening(), "direction is reported as closing");

  RxSyncUpdate mid = run_for(anim, now, 5000);
  check_close(mid.position, 0.5f, 0.05f, "half closed after half the travel time");

  RxSyncUpdate end = run_for(anim, now, 5500);
  check_close(end.position, 0.0f, 0.001f, "lands exactly on the closed end stop");
  check(end.finished, "reports finished");
}

/// Only the remaining distance takes time -- a half-open cover finishes opening
/// in half the configured duration, not the full one.
static void test_partial_travel_is_proportional() {
  printf("Partial travel scales with remaining distance\n");

  uint32_t now = 1000;
  RxSyncAnimator anim;
  anim.start(/*opening=*/true, 0.5f, now);

  RxSyncUpdate quarter = run_for(anim, now, 2500);
  check_close(quarter.position, 0.75f, 0.05f, "three quarters open after 2.5 s");
  check(anim.active(), "still travelling at 2.5 s");

  RxSyncUpdate end = run_for(anim, now, 3000);
  check(end.finished, "finished after ~5 s, not ~10 s");
  check_close(end.position, 1.0f, 0.001f, "reached the open end stop");
}

/// Starting from the end stop the cover is already there: finish immediately
/// rather than dividing by a zero-length duration.
static void test_zero_distance_finishes_immediately() {
  printf("Zero remaining distance\n");

  uint32_t now = 1000;
  RxSyncAnimator anim;
  anim.start(/*opening=*/true, 1.0f, now);

  RxSyncUpdate update = anim.update(now + 20, TRAVEL_MS);
  check(update.finished, "finishes on the first update");
  check_close(update.position, 1.0f, 0.001f, "stays on the open end stop");
  check(!anim.active(), "animator is no longer active");
}

/// stop() freezes the cover where it is, as a MY/STOP press from the remote does.
static void test_stop_freezes_position() {
  printf("Stop mid-travel\n");

  uint32_t now = 1000;
  RxSyncAnimator anim;
  anim.start(/*opening=*/true, 0.0f, now);

  RxSyncUpdate mid = run_for(anim, now, 3000);
  const float frozen = mid.position;
  check(frozen > 0.1f && frozen < 0.9f, "cover is part way open");

  anim.stop();
  check(!anim.active(), "stop() deactivates the animator");
}

/// Publishes must be throttled so a moving cover does not flood the API.
static void test_publishes_are_throttled() {
  printf("Publish throttling\n");

  uint32_t now = 1000;
  RxSyncAnimator anim;
  anim.start(/*opening=*/true, 0.0f, now);

  int publishes = 0;
  run_for(anim, now, 5000, &publishes);  // 250 loop iterations at 50 Hz

  check(publishes > 0, "publishes something while travelling");
  check(publishes <= 5000 / RxSyncAnimator::PUBLISH_INTERVAL_MS + 1, "at most one publish per interval");
}

/// A second start() must fully reset the animation, including publish throttling.
static void test_restart_resets_state() {
  printf("Restart resets state\n");

  uint32_t now = 1000;
  RxSyncAnimator anim;
  anim.start(/*opening=*/true, 0.0f, now);
  run_for(anim, now, 3000);

  anim.start(/*opening=*/false, 1.0f, now);
  check(!anim.opening(), "direction is updated on restart");

  RxSyncUpdate first = anim.update(now + 20, TRAVEL_MS);
  check(first.publish, "publish throttle is reset, so the first update publishes");
  check_close(first.position, 1.0f, 0.01f, "starts from the new start position");
}

int main() {
  printf("Somfy RX-sync animator tests\n\n");

  test_full_open();
  printf("\n");
  test_full_close();
  printf("\n");
  test_partial_travel_is_proportional();
  printf("\n");
  test_zero_distance_finishes_immediately();
  printf("\n");
  test_stop_freezes_position();
  printf("\n");
  test_publishes_are_throttled();
  printf("\n");
  test_restart_resets_state();

  printf("\n%d/%d checks passed\n", g_passed, g_checks);
  return g_passed == g_checks ? 0 : 1;
}
