// OFFLINE DIAGNOSTIC ONLY. Never link into, or preload for, a robot entry.
// Hold rtflann's entropy and C rand streams per thread while comparing identical
// recorded observations. OpenCV's existing per-lane seed remains unchanged.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
static std::atomic<unsigned long long> entropyCalls{0}, randCalls{0};
static thread_local unsigned int entropyState=81018, randState=1;
extern "C" unsigned int fixedRandomDevice(void *) asm("_ZNSt13random_device9_M_getvalEv");
extern "C" unsigned int fixedRandomDevice(void *) {
    ++entropyCalls;
    entropyState=1664525u*entropyState+1013904223u;
    return entropyState;
}
extern "C" int rand() noexcept {++randCalls;return rand_r(&randState);}
extern "C" void srand(unsigned int seed) noexcept {randState=seed;}
__attribute__((constructor)) static void requireOfflineDiagnostic() {
    if(!std::getenv("WBROV_OFFLINE_RANDOM_DIAGNOSTIC"))std::abort();
}
__attribute__((destructor)) static void report() {
    std::fprintf(stderr,"WBROV_RANDOM_DIAGNOSTIC entropy=%llu rand=%llu\n",entropyCalls.load(),randCalls.load());
}
