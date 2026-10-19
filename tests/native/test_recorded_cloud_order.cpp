#include "RecordedCloudPly.h"
#include <cassert>
#include <sstream>
#include <random>
#include <climits>
static void referencePly(std::ostream &out,const RecordedCloudCache::Cloud &cloud) {
    std::map<RecordedCloudCache::Key,RecordedCloudCache::Point> ordered;
    for(const auto &entry:cloud)ordered.insert(entry);
    out<<"ply\nformat binary_little_endian 1.0\ncomment historical verified map; not current localization\nelement vertex "<<ordered.size()
       <<"\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    for(const auto &entry:ordered) {
        const auto &p=entry.second;
        out.write(reinterpret_cast<const char*>(&p.x),4);out.write(reinterpret_cast<const char*>(&p.y),4);out.write(reinterpret_cast<const char*>(&p.z),4);
        out.put(p.r);out.put(p.g);out.put(p.b);
    }
}
void compare(const RecordedCloudCache::Cloud &cloud) {
    std::ostringstream before(std::ios::binary),after(std::ios::binary);
    referencePly(before,cloud);writeRecordedCloudPly(after,cloud);assert(before.str()==after.str());
}
int main() {
    compare({});RecordedCloudCache::Cloud cloud;
    cloud.emplace(RecordedCloudCache::Key{-1,0,0},RecordedCloudCache::Point{-0.0f,0,0,1,2,3});compare(cloud);
    std::mt19937 rng(81019);
    for(int i=0;i<120000;++i)cloud.emplace(RecordedCloudCache::Key{int(rng()%1000)-500,int(rng()%1000)-500,int(rng()%50)},RecordedCloudCache::Point{float(i),float(-i),1,static_cast<unsigned char>(i),2,3});
    compare(cloud); // signed offsets, duplicates, many full write batches
    for(const auto &key: {RecordedCloudCache::Key{INT_MIN,INT_MIN,INT_MIN},RecordedCloudCache::Key{INT_MAX,INT_MAX,INT_MAX}})cloud.emplace(key,RecordedCloudCache::Point{0,0,0,4,5,6});
    compare(cloud); // full 96-bit tuple fallback
    for(int y:{0,1}) {
        RecordedCloudCache::Cloud edge;
        edge.emplace(RecordedCloudCache::Key{INT_MIN,0,0},RecordedCloudCache::Point{1,2,3,4,5,6});
        edge.emplace(RecordedCloudCache::Key{INT_MAX,INT_MAX,y},RecordedCloudCache::Point{2,3,4,5,6,7});
        compare(edge); // exactly 63 bits then 64 bits
    }
}
