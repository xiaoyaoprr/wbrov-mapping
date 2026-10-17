#include "bounded_async_writer.hpp"
#include <cassert>
#include <chrono>
#include <future>
#include <vector>

int main() {
  using namespace std::chrono_literals;
  {
    // Block the disk sink deliberately; accepted capture records must still
    // enqueue promptly, then be drained in order when disk service returns.
    std::promise<void> entered,release;
    auto released=release.get_future().share();std::vector<int> written;
    BoundedAsyncWriter<int> writer(8,[&](int i) {
      if(i==1){entered.set_value();released.wait();}written.push_back(i);
    });
    writer.append(1);entered.get_future().wait();
    auto producer=std::async(std::launch::async,[&]{writer.append(2);writer.append(3);});
    const auto status=producer.wait_for(200ms);
    release.set_value();producer.get();writer.close();
    assert(status==std::future_status::ready);
    assert((written==std::vector<int>{1,2,3}));
    bool rejected=false;try {writer.append(4);}catch(const std::runtime_error &){rejected=true;}
    assert(rejected);
  }
  {
    // A saturated queue fails explicitly, retaining every previously accepted
    // record. Never drop the oldest frame or let memory grow without a bound.
    std::promise<void> entered,release;auto released=release.get_future().share();
    std::vector<int> written;
    BoundedAsyncWriter<int> writer(2,[&](int i) {
      if(i==1){entered.set_value();released.wait();}written.push_back(i);
    });
    writer.append(1);entered.get_future().wait();writer.append(2);writer.append(3);
    bool rejected=false;try {writer.append(4);}catch(const std::runtime_error &){rejected=true;}
    release.set_value();writer.close();
    assert(rejected && (written==std::vector<int>{1,2,3}));
  }
  {
    BoundedAsyncWriter<int> writer(2,[](int){throw std::runtime_error("injected disk failure");});
    writer.append(1);bool failed=false;
    try {writer.close();}catch(const std::runtime_error &){failed=true;}
    assert(failed);failed=false;
    try {writer.append(2);}catch(const std::runtime_error &){failed=true;}
    assert(failed);
  }
  {BoundedAsyncWriter<int> empty(1,[](int){assert(false);});empty.close();}
}
