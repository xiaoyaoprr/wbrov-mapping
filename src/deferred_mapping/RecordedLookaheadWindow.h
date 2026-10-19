#pragma once
#include "RecordedLookahead.h"
#include <deque>

// A bounded sequence of speculative measurements from sealed recordings.
// Only the front segment may be consumed; exact manifest identity is checked
// again when its real command arrives. Preparing a segment never ACKs it.
template<class Measurement> class RecordedLookaheadWindow {
public:
    explicit RecordedLookaheadWindow(std::size_t capacity=3):capacity_(capacity) {
        if(capacity_<1 || capacity_>3)throw std::runtime_error("invalid lookahead window");
    }
    bool ready() const{return !segments_.empty();}
    std::size_t size() const{return segments_.size();}
    bool contains(int sequence) const {
        for(const auto &entry:segments_)if(entry.sequence==sequence)return true;
        return false;
    }
    void require(int sequence,const std::string &identity) const {
        for(const auto &entry:segments_)if(entry.sequence==sequence){entry.values.require(identity);return;}
        throw std::runtime_error("unprepared lookahead segment");
    }
    void requireFront(int sequence,const std::string &identity) const {
        if(!ready() || segments_.front().sequence!=sequence)throw std::runtime_error("lookahead consumption out of order");
        segments_.front().values.require(identity);
    }
    void begin(int after,int sequence,const std::string &identity) {
        if(segments_.size()>=capacity_ || sequence!=(ready()?segments_.back().sequence+1:after+1))
            throw std::runtime_error("lookahead window has gap or exceeds bound");
        Segment entry;entry.sequence=sequence;entry.values.begin(identity);segments_.push_back(std::move(entry));
    }
    void add(int sequence,long long source,Measurement value) {
        if(!ready() || segments_.back().sequence!=sequence)throw std::runtime_error("lookahead add out of order");
        segments_.back().values.add(source,std::move(value));
    }
    Measurement take(int sequence,long long source) {
        if(!ready() || segments_.front().sequence!=sequence)throw std::runtime_error("lookahead take out of order");
        return segments_.front().values.take(source);
    }
    void finish(int sequence) {
        if(!ready() || segments_.front().sequence!=sequence)throw std::runtime_error("lookahead finish out of order");
        segments_.front().values.finish();segments_.pop_front();
    }
private:
    struct Segment {int sequence=0;RecordedLookahead<Measurement> values;};
    std::deque<Segment> segments_;std::size_t capacity_;
};
