#pragma once
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>

namespace wbrov_spool {
struct IoObservation {
    bool schedulerAvailable=false,memoryAvailable=false,pressureAvailable=false;
    std::uint64_t runtimeNs=0,runqueueNs=0,dirtyKiB=0,writebackKiB=0,ioSomeUs=0,ioFullUs=0;
};
inline std::uint64_t counterDelta(std::uint64_t before,std::uint64_t after,bool available) {
    return available && after>=before?after-before:0;
}
// Called by the writer thread, not the capture thread. These are explanatory
// observations, never gating signals. runqueue wait is not total I/O wait.
inline IoObservation observeIo() {
    IoObservation o;std::ifstream scheduler("/proc/thread-self/schedstat");
    o.schedulerAvailable=bool(scheduler>>o.runtimeNs>>o.runqueueNs);
    std::ifstream memory("/proc/meminfo");std::string line;unsigned fields=0;
    while(std::getline(memory,line)) {
        std::istringstream in(line);std::string key;std::uint64_t value;
        if(in>>key>>value) {
            if(key=="Dirty:"){o.dirtyKiB=value;++fields;}
            if(key=="Writeback:"){o.writebackKiB=value;++fields;}
        }
    }
    o.memoryAvailable=fields==2;
    std::ifstream pressure("/proc/pressure/io");fields=0;
    while(std::getline(pressure,line)) {
        std::istringstream in(line);std::string kind,part;in>>kind;
        while(in>>part)if(part.rfind("total=",0)==0) {
            std::uint64_t value=0;std::istringstream number(part.substr(6));
            if(number>>value) {
                if(kind=="some"){o.ioSomeUs=value;++fields;}
                if(kind=="full"){o.ioFullUs=value;++fields;}
            }
        }
    }
    o.pressureAvailable=fields==2;return o;
}
}
