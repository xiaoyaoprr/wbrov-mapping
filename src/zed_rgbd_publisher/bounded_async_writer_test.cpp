#include "bounded_async_writer.hpp"
#include "recording_journal_limits.hpp"
#include <cassert>
#include <chrono>
#include <future>
#include <vector>

int main() {
  using namespace std::chrono_literals;
  {
    // A stalled metadata sink must retain the 30 Hz * 28 s field burst,
    // including its headroom, and close must preserve exact ordered records.
    std::promise<void> entered,release;auto released=release.get_future().share();
    std::vector<std::string> written,expected;
    BoundedAsyncWriter<std::string> journal(wbrov_recording::journal_capacity,[&](const std::string &row) {
      if(written.empty()){entered.set_value();released.wait();}written.push_back(row);
    });
    expected.push_back("source=0\n");journal.append(expected.back());entered.get_future().wait();
    bool accepted=true;
    try {for(int i=1;i<=1800;++i){expected.push_back("source="+std::to_string(i)+"\n");journal.append(expected.back());}}
    catch(...){accepted=false;}
    const auto pending=journal.snapshot();release.set_value();journal.close();
    assert(accepted && pending.highWater==1800 && written==expected);
    const auto closed=journal.snapshot();assert(closed.accepted==1801 && closed.completed==1801 && closed.queued==0);
  }
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
  {
    std::promise<void> entered,release;auto released=release.get_future().share();std::vector<int> written;
    BoundedAsyncWriter<int> writer(1,[&](int i){if(i==1){entered.set_value();released.wait();}written.push_back(i);});
    writer.append(1);entered.get_future().wait();writer.append(2);
    auto producer=std::async(std::launch::async,[&]{return writer.append_wait(3,0ms);});
    assert(producer.wait_for(50ms)==std::future_status::timeout);
    release.set_value();assert(producer.get());writer.close();assert((written==std::vector<int>{1,2,3}));
    const auto s=writer.snapshot();assert(s.accepted==3 && s.completed==3 && s.queued==0 && s.highWater==1);
    assert(s.waits==1 && s.waitMs>=0 && s.maxWaitMs==s.waitMs);
  }
  {
    std::promise<void> entered,release;auto released=release.get_future().share();
    BoundedAsyncWriter<int> writer(1,[&](int i){if(i==1){entered.set_value();released.wait();}});
    writer.append(1);entered.get_future().wait();writer.append(2);bool timed=false;
    try{writer.append_wait(3,20ms);}catch(const std::runtime_error &e){timed=std::string(e.what()).find("timeout")!=std::string::npos;}
    release.set_value();writer.close();assert(timed);
  }
  {
    std::promise<void> entered,fail;auto failed=fail.get_future().share();
    BoundedAsyncWriter<int> writer(1,[&](int){entered.set_value();failed.wait();throw std::runtime_error("disk failed");});
    writer.append(1);entered.get_future().wait();writer.append(2);
    auto producer=std::async(std::launch::async,[&]{try{writer.append_wait(3,2s);}catch(const std::runtime_error &e){return std::string(e.what())=="disk failed";}return false;});
    assert(producer.wait_for(50ms)==std::future_status::timeout);fail.set_value();assert(producer.get());
    try{writer.close();}catch(const std::runtime_error &){}
  }
}
