#include "async_diagnostics.hpp"
#include <cassert>
#include <future>
#include <string>
#include <vector>
int main() {
  std::promise<void> entered, release;
  auto release_future=release.get_future().share();
  std::vector<std::string> delivered;
  AsyncDiagnostics logger(2, [&](const auto &r) {
    if(delivered.empty()) {entered.set_value(); release_future.wait();}
    delivered.emplace_back(r.message.data());
  });
  logger.log(20,"camera","first %d",1);
  entered.get_future().wait();
  auto producer=std::async(std::launch::async,[&] {
    logger.log(20,"camera","second");
    logger.log(20,"camera","third");
    logger.log(20,"camera","overflow");
  });
  // The sink remains deliberately blocked until the producer has finished.
  assert(producer.wait_for(std::chrono::seconds(1))==std::future_status::ready);
  assert(logger.counts().rejected==1);
  release.set_value(); producer.get();
  assert(!logger.close());
  assert((delivered==std::vector<std::string>{"first 1","second","third"}));
  assert(logger.counts().accepted==3 && logger.counts().emitted==3);
  AsyncDiagnostics normal(8,[](const auto &r) {assert(r.enqueued_steady_ns>0);});
  normal.log(30,"camera","valid"); assert(normal.close());
  AsyncDiagnostics oversized(8,[](const auto &) {});
  oversized.log(20,"camera","%s",std::string(3000,'x').c_str());
  assert(!oversized.close() && oversized.counts().rejected==1);
  AsyncDiagnostics broken(8,[](const auto &) {throw std::runtime_error("sink failure");});
  broken.log(40,"camera","error");
  assert(!broken.close() && broken.counts().sink_failed);
}
