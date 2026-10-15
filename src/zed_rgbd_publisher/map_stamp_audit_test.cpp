#include "map_stamp_audit.hpp"

#include <iostream>
#include <stdexcept>

namespace
{
void require(bool value, const char *message)
{
  if (!value) {
    throw std::runtime_error(message);
  }
}
}  // namespace

int main()
{
  using namespace zed_dual_stream;
  MapStampAuditLedger ledger(3);
  require(!ledger.queue(100, 10), "first queue has no eviction");
  require(ledger.observe_valid(100, 20) == MapStampObservation::kExactValidPending,
    "valid pending acknowledges once");
  require(ledger.observe_invalid(100, 21) == MapStampObservation::kInvalidAfterValidConflict,
    "valid then invalid is an explicit conflict");

  ledger.queue(200, 30);
  require(ledger.timeout(200, 780), "pending stamp times out");
  require(ledger.observe_valid(200, 810) == MapStampObservation::kLateValidAfterTimeout,
    "late valid retains timed-out map association");

  ledger.queue(300, 40);
  require(ledger.observe_invalid(300, 41) == MapStampObservation::kInvalidPending,
    "invalid pending becomes an irreversible drop");
  require(ledger.observe_valid(300, 42) == MapStampObservation::kValidAfterInvalidConflict,
    "invalid then valid is conflict poisoned and cannot acknowledge");

  const auto evicted = ledger.queue(400, 50);
  require(evicted && evicted->stamp_ns == 100 &&
    evicted->state == MapStampState::kAcknowledged,
    "bounded audit ledger exposes tombstone eviction");
  require(ledger.shutdown_censor(400, 60), "pending map is right-censored on shutdown");
  require(ledger.observe_valid(400, 61) == MapStampObservation::kLateValidAfterShutdown,
    "late valid after shutdown remains associated but cannot publish");
  require(ledger.observe_valid(999, 70) == MapStampObservation::kUnknownStamp,
    "ordinary non-map odom stamp is outside map ledger");

  std::cout << "MAP_STAMP_AUDIT_TEST_OK\n";
  return 0;
}
