#include "rtflann/flann.hpp"
#include <arm_neon.h>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

// Isolated generic L1 experiment. Preserve the scalar grouping, early-exit
// boundaries and partial-distance function; never reassociate the reduction.
struct CandidateL1 {
    using is_kdtree_distance=bool;using ElementType=float;using ResultType=float;
    template<class A,class B> float operator()(A a,B b,std::size_t size,float worst=-1) const {
        if constexpr(!(std::is_pointer_v<A> && std::is_pointer_v<B> && std::is_same_v<std::remove_cv_t<std::remove_pointer_t<A>>,float> && std::is_same_v<std::remove_cv_t<std::remove_pointer_t<B>>,float>))return rtflann::L1<float>()(a,b,size,worst);
        else {
            float result=0;std::size_t i=0;
            for(;i+4<=size;i+=4) {
                const auto av=vld1q_f32(a+i),bv=vld1q_f32(b+i);
                const auto exponent=vdupq_n_u32(0x7f800000u);
                const auto special=vorrq_u32(vceqq_u32(vandq_u32(vreinterpretq_u32_f32(av),exponent),exponent),
                                             vceqq_u32(vandq_u32(vreinterpretq_u32_f32(bv),exponent),exponent));
                // Keep NaN payloads and infinity arithmetic on the unchanged
                // complete scalar path, before evaluating any vector subtraction.
                if(vmaxvq_u32(special))return rtflann::L1<float>()(a,b,size,worst);
                const auto d=vabsq_f32(vsubq_f32(av,bv));
                result+=vgetq_lane_f32(d,0)+vgetq_lane_f32(d,1)+vgetq_lane_f32(d,2)+vgetq_lane_f32(d,3);
                if(worst>0 && result>worst)return result;
            }
            for(;i<size;++i) {
                std::uint32_t ab=0,bb=0;std::memcpy(&ab,a+i,4);std::memcpy(&bb,b+i,4);
                if((ab&0x7f800000u)==0x7f800000u || (bb&0x7f800000u)==0x7f800000u)
                    return rtflann::L1<float>()(a,b,size,worst);
                result+=std::abs(a[i]-b[i]);
            }
            return result;
        }
    }
    template<class A,class B> float accum_dist(const A &a,const B &b,int) const {return std::abs(a-b);}
};
static bool same(float a,float b){return std::memcmp(&a,&b,4)==0;}
static void edgeTests() {
    alignas(16) std::array<float,552> a{},b{};std::mt19937 random(20261019);
    const std::array<float,9> limits{-1,0,.0001f,1,4,16,64,256,std::numeric_limits<float>::infinity()};
    for(unsigned round=0;round<9;++round) {
        for(std::size_t i=0;i<a.size();++i) {
            if(round<3){a[i]=float(random()%256);b[i]=float(random()%256);}
            else {a[i]=std::ldexp(float(std::int32_t(random())),int(random()%60)-40);b[i]=std::ldexp(float(std::int32_t(random())),int(random()%60)-40);}
        }
        for(unsigned offset=0;offset<4;++offset)for(std::size_t n=0;n<=521;++n)for(float bound:limits) {
            const auto original=rtflann::L1<float>()(a.data()+8+offset,b.data()+12-offset,n,bound);
            assert(same(original,CandidateL1()(a.data()+8+offset,b.data()+12-offset,n,bound)));
        }
    }
    const std::array<std::uint32_t,10> finiteBits{0,0x80000000,1,0x80000001,0x007fffff,0x807fffff,0x00800000,0x80800000,0x7f7fffff,0xff7fffff};
    for(std::size_t i=0;i<a.size();++i){std::memcpy(&a[i],&finiteBits[i%10],4);std::memcpy(&b[i],&finiteBits[(i+3)%10],4);}
    for(unsigned offset=0;offset<4;++offset)for(std::size_t n=0;n<=521;++n)for(float bound:limits)
        assert(same(rtflann::L1<float>()(a.data()+8+offset,b.data()+12-offset,n,bound),CandidateL1()(a.data()+8+offset,b.data()+12-offset,n,bound)));
    const std::array<std::uint32_t,8> bits{0,0x80000000,0x7fc00013,0x7fa00015,0x7f800000,0xff800000,0x7f7fffff,0xff7fffff};
    for(std::size_t i=0;i<a.size();++i){std::memcpy(&a[i],&bits[i%8],4);std::memcpy(&b[i],&bits[(i+3)%8],4);}
    for(unsigned offset=0;offset<4;++offset)for(std::size_t n=0;n<=521;++n)for(float bound:limits)
        assert(same(rtflann::L1<float>()(a.data()+8+offset,b.data()+12-offset,n,bound),CandidateL1()(a.data()+8+offset,b.data()+12-offset,n,bound)));
    std::array<double,552> wide{};
    for(std::size_t i=0;i<wide.size();++i){a[i]=float(i%7)*.125f;wide[i]=double(i%13)*.1875;}
    for(std::size_t n=0;n<=521;++n)for(float bound:limits)
        assert(same(rtflann::L1<float>()(a.data()+8,wide.data()+12,n,bound),CandidateL1()(a.data()+8,wide.data()+12,n,bound)));
    std::vector<float> iteratorData(a.begin(),a.end());
    for(std::size_t n=0;n<=521;++n)
        assert(same(rtflann::L1<float>()(iteratorData.begin()+8,a.data()+12,n),CandidateL1()(iteratorData.begin()+8,a.data()+12,n)));
    std::cout<<"L1 distance bits, scalar reduction order, all tails, vector alignment and early exits equal"<<std::endl;
}
template<class Index> auto search(Index &index,rtflann::Matrix<float> &queries) {
    std::vector<std::size_t> ids(queries.rows*2);std::vector<float> distances(queries.rows*2);
    rtflann::Matrix<std::size_t> indexOut(ids.data(),queries.rows,2);rtflann::Matrix<float> distanceOut(distances.data(),queries.rows,2);
    index.knnSearch(queries,indexOut,distanceOut,2,rtflann::SearchParams(32,0,true));
    return std::make_pair(ids,distances);
}
static void indexTests(const std::string &prefix) {
    for(int kind=0;kind<2;++kind) {
        const std::size_t width=kind?64:256;std::mt19937 random(81019);
        std::vector<float> data(12000*width),query(1000*width);
        for(auto &x:data)x=kind?float(random()%10000)*.01f:float(random()%2);
        for(std::size_t i=17;i<12000;i+=17)std::memcpy(data.data()+i*width,data.data()+(i-1)*width,width*4);
        for(std::size_t i=0;i<1000;++i){std::memcpy(query.data()+i*width,data.data()+((i*43)%12000)*width,width*4);query[i*width+i%width]+=kind?.001f:1.f;}
        rtflann::Matrix<float> training(data.data(),12000,width),queries(query.data(),1000,width);
        rtflann::Index<rtflann::L1<float>> original(training,rtflann::KDTreeIndexParams(4));original.buildIndex();
        const auto saved=prefix+std::to_string(kind)+".index";original.save(saved);
        rtflann::Index<rtflann::L1<float>> control(training,rtflann::SavedIndexParams(saved));
        rtflann::Index<CandidateL1> candidate(training,rtflann::SavedIndexParams(saved));
        const auto expected=search(original,queries);assert(search(control,queries)==expected);assert(search(candidate,queries)==expected);
        for(int round=0;round<6;++round) {
            double before=0,after=0;
            auto run=[&](bool useCandidate) {
                const auto began=std::chrono::steady_clock::now();
                for(int repeat=0;repeat<6;++repeat)assert((useCandidate?search(candidate,queries):search(control,queries))==expected);
                const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();(useCandidate?after:before)=ms;
            };
            run(round%2);run(!(round%2));
            std::cout<<"{\"kind\":"<<kind<<",\"width\":"<<width<<",\"round\":"<<round<<",\"queries\":6000,\"before_ms\":"<<before<<",\"after_ms\":"<<after<<",\"indices_and_distances_equal\":true}"<<std::endl;
        }
    }
}
int main(int argc,char **argv){edgeTests();if(argc==2)indexTests(argv[1]);}
