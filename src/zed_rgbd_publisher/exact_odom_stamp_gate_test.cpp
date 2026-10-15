#include "exact_odom_stamp_gate.hpp"

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
  zed_dual_stream::ExactOdomStampGate after_queue(750000000);
  after_queue.queue(100, 0);
  require(after_queue.pending() && !after_queue.ready(), "queued frame waits for odom");
  require(after_queue.queued_steady_ns() == 0, "queue timestamp is observable for ack wait audit");
  after_queue.acknowledge(99);
  require(!after_queue.ready(), "different odom stamp cannot release frame");
  after_queue.acknowledge(100);
  require(after_queue.ready(), "exact odom stamp releases frame");
  after_queue.consume();
  require(!after_queue.pending(), "consume clears pending frame");

  zed_dual_stream::ExactOdomStampGate before_queue(750000000);
  before_queue.acknowledge(200);
  before_queue.queue(200, 10);
  require(before_queue.ready(), "recent exact odom can precede map queue");

  zed_dual_stream::ExactOdomStampGate expiry(750000000);
  expiry.queue(300, 100);
  require(!expiry.expire(750000100), "frame cannot expire early");
  require(expiry.expire(750000101), "unacknowledged frame expires after timeout");
  require(!expiry.pending(), "expiry clears pending frame");

  zed_dual_stream::ExactOdomStampGate cancellation(750000000);
  cancellation.queue(55, 100);
  require(!cancellation.cancel(56), "different stamp cannot cancel pending map");
  require(cancellation.pending(), "failed cancellation preserves pending map");
  require(cancellation.cancel(55), "matching invalid odom cancels pending map");
  require(!cancellation.pending(), "successful cancellation clears pending map");

  bool overlap_rejected = false;
  try {
    zed_dual_stream::ExactOdomStampGate overlap(750000000);
    overlap.queue(1, 0);
    overlap.queue(2, 1);
  } catch (const std::logic_error &) {
    overlap_rejected = true;
  }
  require(overlap_rejected, "overlapping pending frames are rejected");

  std::cout << "EXACT_ODOM_STAMP_GATE_TEST_OK\n";
}
