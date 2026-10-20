#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <time.h>
#include <unistd.h>

// OFFLINE TEST ONLY. Preload into the standalone paced writer, never into the
// camera/SDK or mapping runtime. One exact disposable output file is delayed;
// the actual fsync still runs and its return value is preserved.
extern "C" int fsync(int fd) {
    using Sync=int(*)(int);static const auto real=reinterpret_cast<Sync>(dlsym(RTLD_NEXT,"fsync"));
    if(!real){errno=ENOSYS;return -1;}
    const char *target=std::getenv("WBROV_TEST_STALL_PATH");
    char descriptor[64],path[4096];std::snprintf(descriptor,sizeof(descriptor),"/proc/self/fd/%d",fd);
    const auto size=::readlink(descriptor,path,sizeof(path)-1);
    if(target && size>0){path[size]=0;static std::atomic_flag used=ATOMIC_FLAG_INIT;
        if(std::strcmp(path,target)==0 && !used.test_and_set()){
            std::fprintf(stderr,"[OFFLINE_TEST] injecting one 14000 ms fsync delay: %s\n",path);
            timespec remaining{14,0};while(::nanosleep(&remaining,&remaining) && errno==EINTR){}
        }
    }
    return real(fd);
}
