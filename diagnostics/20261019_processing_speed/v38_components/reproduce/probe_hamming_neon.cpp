#include "rtflann/flann.hpp"
#include <arm_neon.h>
#include <array>
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <random>
#include <string>
#include <vector>

// Isolated distance-functor experiment. Preserve the existing rtflann Hamming
// complete-word semantics, candidate set, search order, distances and ties.
// This prototype does not modify Core, descriptor extraction or match gates.
struct CandidateHamming {
    using ElementType=unsigned char;using ResultType=unsigned int;
    template<class A,class B> ResultType operator()(A a,B b,std::size_t size,ResultType=0) const {
#ifdef FLANN_PLATFORM_64_BIT
        size=(size/8)*8;
#else
        size=(size/4)*4;
#endif
        unsigned result=0;std::size_t i=0;
        for(;i+16<=size;i+=16)result+=vaddlvq_u8(vcntq_u8(veorq_u8(vld1q_u8(a+i),vld1q_u8(b+i))));
        for(;i<size;++i)result+=__builtin_popcount(unsigned(a[i]^b[i]));
        return result;
    }
};

static std::size_t rounded(std::size_t size) {
#ifdef FLANN_PLATFORM_64_BIT
    return size/8*8;
#else
    return size/4*4;
#endif
}
static unsigned reference(const unsigned char *a,const unsigned char *b,std::size_t n) {
    unsigned result=0;
    for(std::size_t i=0;i<rounded(n);++i)for(unsigned bit=0;bit<8;++bit)result+=((a[i]^b[i])>>bit)&1;
    return result;
}
static void edgeTests() {
    alignas(16) std::array<unsigned char,304> a{},b{};std::mt19937 random(81019);
    for(int turn=0;turn<32;++turn) {
        for(auto &x:a)x=static_cast<unsigned char>(random());
        for(auto &x:b)x=static_cast<unsigned char>(random());
        for(std::size_t n=0;n<=257;++n) {
            assert(rtflann::Hamming<unsigned char>()(a.data(),b.data(),n)==reference(a.data(),b.data(),n));
            for(unsigned offset=0;offset<32;++offset)
                assert(CandidateHamming()(a.data()+offset,b.data()+31-offset,n)==reference(a.data()+offset,b.data()+31-offset,n));
        }
    }
    a.fill(0);b.fill(255);
    for(std::size_t n=0;n<=257;++n)assert(CandidateHamming()(a.data(),b.data(),n)==rounded(n)*8);
}
template<class Index> auto search(Index &index,rtflann::Matrix<unsigned char> &query) {
    std::vector<std::size_t> ids(query.rows*2);std::vector<unsigned> distances(query.rows*2);
    rtflann::Matrix<std::size_t> output(ids.data(),query.rows,2);
    rtflann::Matrix<unsigned> dists(distances.data(),query.rows,2);
    index.knnSearch(query,output,dists,2,rtflann::SearchParams(32,0,true));
    return std::make_pair(ids,distances);
}
static void indexTests(const std::string &prefix) {
    for(std::size_t width:{32,64}) {
        std::mt19937 random(20261019);std::vector<unsigned char> data(12000*width),queryData(1000*width);
        for(auto &x:data)x=static_cast<unsigned char>(random());
        for(std::size_t i=17;i<12000;i+=17)std::memcpy(data.data()+i*width,data.data()+(i-1)*width,width);
        for(std::size_t i=0;i<1000;++i) {
            std::memcpy(queryData.data()+i*width,data.data()+((i*43)%12000)*width,width);
            queryData[i*width+i%width]^=1;
        }
        rtflann::Matrix<unsigned char> training(data.data(),12000,width),queries(queryData.data(),1000,width);
        rtflann::Index<rtflann::Hamming<unsigned char>> original(training,rtflann::LshIndexParams(12,20,2));original.buildIndex();
        const std::string saved=prefix+std::to_string(width)+".index";original.save(saved);
        rtflann::Index<rtflann::Hamming<unsigned char>> control(training,rtflann::SavedIndexParams(saved));
        rtflann::Index<CandidateHamming> candidate(training,rtflann::SavedIndexParams(saved));
        const auto expected=search(original,queries);assert(search(control,queries)==expected);assert(search(candidate,queries)==expected);
        for(int round=0;round<6;++round) {
            double before=0,after=0;
            auto run=[&](bool useCandidate) {
                const auto start=std::chrono::steady_clock::now();
                for(int repeat=0;repeat<6;++repeat)assert((useCandidate?search(candidate,queries):search(control,queries))==expected);
                const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
                (useCandidate?after:before)=ms;
            };
            run(round%2);run(!(round%2));
            std::cout<<"{\"width\":"<<width<<",\"round\":"<<round<<",\"queries\":6000,\"before_ms\":"<<before<<",\"after_ms\":"<<after<<",\"indices_and_distances_equal\":true}"<<std::endl;
        }
    }
}
int main(int argc,char **argv) {
    edgeTests();std::cout<<"edge and unaligned/tail tests passed"<<std::endl;
    if(argc==2)indexTests(argv[1]);
    return 0;
}
