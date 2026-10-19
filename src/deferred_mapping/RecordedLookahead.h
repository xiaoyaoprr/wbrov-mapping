#pragma once
#include <map>
#include <string>
#include <stdexcept>
#include <utility>

// One sealed segment of speculative odometry, never a map-write receipt.
// The actual command must match its entire pinned identity before reuse.
template<class Measurement> class RecordedLookahead {
public:
    bool ready() const {return ready_;}
    void begin(const std::string &identity) {
        if(ready_ || identity.empty())throw std::runtime_error("lookahead already occupied or identity missing");
        identity_=identity;ready_=true;
    }
    void require(const std::string &identity) const {
        if(!ready_ || identity!=identity_)throw std::runtime_error("lookahead input identity changed");
    }
    void add(long long source,Measurement value) {
        if(!ready_ || values_.size()>=64 || !values_.emplace(source,std::move(value)).second)
            throw std::runtime_error("invalid or excessive lookahead measurements");
    }
    Measurement take(long long source) {
        const auto it=values_.find(source);
        if(!ready_ || it==values_.end())throw std::runtime_error("lookahead measurement missing");
        auto value=std::move(it->second);values_.erase(it);return value;
    }
    void finish() {
        if(!ready_ || !values_.empty())throw std::runtime_error("unconsumed lookahead measurements");
        ready_=false;identity_.clear();
    }
private:
    bool ready_=false;std::string identity_;std::map<long long,Measurement> values_;
};
