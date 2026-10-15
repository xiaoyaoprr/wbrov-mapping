#include "map_frame_lifecycle.hpp"

#include <iostream>
#include <stdexcept>

namespace
{
void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}
}

int main()
{
  using namespace zed_dual_stream;

  MapFrameLifecycle before(750, 8);
  before.queue(1, 1, 1000);
  require(before.resolve_valid(1, 1749) == OdomReceiptDisposition::kOnTimeValidReady,
    "deadline minus one is on time");
  before.begin_publish(1);
  require(before.commit_publish(1, 1750).state == MapFrameState::kPublished,
    "on-time receipt can commit once");

  MapFrameLifecycle exact(750, 8);
  exact.queue(2, 2, 2000);
  require(exact.resolve_valid(2, 2750) == OdomReceiptDisposition::kOnTimeValidReady,
    "exact deadline is inclusive");

  MapFrameLifecycle late(750, 8);
  late.queue(3, 3, 3000);
  require(late.resolve_valid(3, 3751) == OdomReceiptDisposition::kLateValidCausedTimeout,
    "deadline plus one is timeout and never ready");
  require(!late.active(), "late receipt finalizes timeout");

  MapFrameLifecycle poll_then_late(750, 8);
  poll_then_late.queue(30, 30, 3000);
  require(poll_then_late.expire(3751).has_value(), "poll creates the timeout terminal");
  require(poll_then_late.resolve_valid(30, 3760) ==
    OdomReceiptDisposition::kLateValidAfterTimeout,
    "late receipt after poll timeout is auxiliary and cannot create another terminal");

  MapFrameLifecycle invalid_first(750, 8);
  invalid_first.queue(4, 4, 4000);
  require(invalid_first.resolve_invalid(4, 4010) ==
    OdomReceiptDisposition::kInvalidPendingDropped, "invalid pending is terminal");
  require(invalid_first.resolve_valid(4, 4020) ==
    OdomReceiptDisposition::kValidAfterInvalidConflict,
    "invalid then valid remains poisoned");

  MapFrameLifecycle invalid_exact(750, 8);
  invalid_exact.queue(40, 40, 4000);
  require(invalid_exact.resolve_invalid(40, 4750) ==
    OdomReceiptDisposition::kInvalidPendingDropped,
    "invalid receipt at exact deadline remains an invalid verdict");

  MapFrameLifecycle invalid_late(750, 8);
  invalid_late.queue(41, 41, 4000);
  require(invalid_late.resolve_invalid(41, 4751) ==
    OdomReceiptDisposition::kLateInvalidCausedTimeout,
    "invalid receipt after deadline is classified as timeout");
  require(!invalid_late.active(), "late invalid receipt finalizes timeout");

  MapFrameLifecycle valid_first(750, 8);
  valid_first.queue(5, 5, 5000);
  require(valid_first.resolve_valid(5, 5010) ==
    OdomReceiptDisposition::kOnTimeValidReady, "valid prepares but does not publish itself");
  require(valid_first.resolve_invalid(5, 5020) ==
    OdomReceiptDisposition::kInvalidBeforePublishConflict,
    "valid then invalid before publish is dropped");
  require(!valid_first.active(), "conflicting verdict cannot remain publishable");

  MapFrameLifecycle expired(750, 8);
  expired.queue(6, 6, 6000);
  require(!expired.expire(6750), "poll at exact deadline does not expire");
  require(expired.expire(6751).has_value(), "poll after deadline expires");

  MapFrameLifecycle failed(750, 8);
  failed.queue(7, 7, 7000);
  failed.resolve_valid(7, 7010);
  failed.begin_publish(7);
  require(failed.fail_publish(7, 7020).state == MapFrameState::kPublishFailed,
    "publish exception has one explicit terminal");
  require(!failed.shutdown_censor(7030), "failed publish cannot gain second terminal");

  MapFrameLifecycle censored(750, 8);
  censored.queue(8, 8, 8000);
  require(censored.shutdown_censor(8010)->state == MapFrameState::kShutdownCensored,
    "pending frame is right-censored once");

  std::cout << "MAP_FRAME_LIFECYCLE_TEST_OK\n";
}
