#pragma once
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <unistd.h>

namespace wbrov_spool {
inline void syncFile(const std::filesystem::path &p) {
    const int fd=::open(p.c_str(),O_RDONLY);
    if(fd<0)throw std::runtime_error("spool sync open failed: "+p.string());
    const int rc=::fsync(fd);::close(fd);
    if(rc)throw std::runtime_error("spool fsync failed: "+p.string());
}
inline void writeText(const std::filesystem::path &p,const std::string &text) {
    std::ofstream out(p);out<<text;out.flush();
    if(!out)throw std::runtime_error("spool text write failed: "+p.string());
    out.close();if(!out)throw std::runtime_error("spool text close failed: "+p.string());
}
inline void atomicText(const std::filesystem::path &p,const std::string &text,bool durable=true) {
    const auto tmp=p.string()+".tmp";writeText(tmp,text);
    if(durable)syncFile(tmp);
    std::filesystem::rename(tmp,p);
    if(durable)syncFile(p.parent_path());
}
struct CommitTimes {double dataSyncMs=0,dataRenameMs=0,receiptWriteMs=0,receiptSyncMs=0,receiptRenameMs=0,directorySyncMs=0;};
// The status/high-water mark may advance ONLY after this function returns.
// Data and receipt are both fsynced, then one directory fsync commits both
// renames. A visible receipt alone is not a committed segment. The hook is for
// deterministic interruption tests; production supplies no hook.
inline CommitTimes commitSegment(const std::filesystem::path &partial,
        const std::filesystem::path &data,const std::filesystem::path &receipt,
        const std::string &text,const std::function<void(const char *)> &before={}) {
    if(partial.parent_path()!=data.parent_path() || data.parent_path()!=receipt.parent_path())
        throw std::runtime_error("segment commit must stay in one directory");
    using Clock=std::chrono::steady_clock;CommitTimes t;
    const auto step=[&](const char *name,double &ms,const auto &operation){
        if(before)before(name);
        const auto begin=Clock::now();operation();
        ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
    };
    const auto tmp=receipt.string()+".tmp";
    step("data_sync",t.dataSyncMs,[&]{syncFile(partial);});
    step("data_rename",t.dataRenameMs,[&]{std::filesystem::rename(partial,data);});
    step("receipt_write",t.receiptWriteMs,[&]{writeText(tmp,text);});
    step("receipt_sync",t.receiptSyncMs,[&]{syncFile(tmp);});
    step("receipt_rename",t.receiptRenameMs,[&]{std::filesystem::rename(tmp,receipt);});
    step("directory_sync",t.directorySyncMs,[&]{syncFile(data.parent_path());});
    return t;
}
}
