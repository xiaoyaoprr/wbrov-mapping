#include "RecordedOrderedPrefetch.h"
#include "RecordedCloudAssembly.h"
#include "RecordedCloudSamples.h"
#include "RecordedCloudPly.h"
#include <cassert>
#include <chrono>
#include <sstream>

using Cloud=RecordedCloudCache::Cloud;
static std::string ply(const Cloud &cloud) {
    std::ostringstream out(std::ios::binary);writeRecordedCloudPly(out,cloud);return out.str();
}
template<class Function> static void fails(Function f) {
    bool failed=false;try{f();}catch(const std::exception&){failed=true;}assert(failed);
}
int main() {
    const auto owner=std::this_thread::get_id();std::vector<int> prepared;
    auto make=[&](int id)->std::function<int()> {
        assert(std::this_thread::get_id()==owner);prepared.push_back(id);
        return [id]{if(id==1)std::this_thread::sleep_for(std::chrono::milliseconds(10));return id;};
    };
    fails([&]{RecordedOrderedPrefetch<int> bad({2,1},make);});
    fails([&]{RecordedOrderedPrefetch<int> bad({1,1},make);});
    fails([&]{RecordedOrderedPrefetch<int> bad({},make,0);});
    RecordedOrderedPrefetch<int> tiny({1,2,3},make);
    assert(!tiny.parallel() && prepared.empty() && !tiny.prepared(2));tiny.finish();
    {
        RecordedOrderedPrefetch<int> jobs({1,3,5,7},make);
        assert(jobs.parallel() && prepared==std::vector<int>({1,3}));
        fails([&]{jobs.finish();});fails([&]{jobs.take(3);});
        for(int id:{1,3,5,7}) {assert(!jobs.prepared(id-1));assert(jobs.take(id)==id);}
        jobs.finish();assert(prepared==std::vector<int>({1,3,5,7}));
    }
    // Exceptions from preparation or worker jobs unwind with no borrowed
    // inputs or joinable threads. Remaining queued jobs must be drained.
    auto failure=[](int id)->std::function<int()> {
        return [id]{if(id==2)throw std::runtime_error("worker");return id;};
    };
    fails([&]{RecordedOrderedPrefetch<int> jobs({1,2,3,4},failure);assert(jobs.take(1)==1);jobs.take(2);});
    fails([&]{RecordedOrderedPrefetch<int> jobs({1,2,3,4},[&](int id) {
        if(id==2)throw std::runtime_error("preparation");return make(id);
    });});

    RecordedCloudCache cache(1);RecordedCloudSamples samples(0);RecordedCloudAssembly assembly;
    RecordedCloudAssembly::Inputs inputs;RecordedCloudCache::Pose pose{};
    for(int id=1;id<=7;++id)inputs[id]={pose,"source"+std::to_string(id)};
    auto node=[](int id) {return Cloud{{{0,0,0},{float(id),0,0,static_cast<unsigned char>(id),2,3}}};};
    // ID 4 is initially cached but inserting ID 1 evicts it. Its synchronous
    // fallback must not consume the already prepared result for ID 5.
    cache.update(4,pose,inputs.at(4).input,[&]{return node(4);});
    assert(cache.matches(4,pose,inputs.at(4).input));
    std::unique_ptr<RecordedOrderedPrefetch<Cloud>> jobs;
    std::vector<int> planned,consumed,synchronous;
    auto prepareNode=[&](int id)->std::function<Cloud()> {
        assert(std::this_thread::get_id()==owner);
        const auto &sample=samples.get(id,inputs.at(id).input,[&] {
            RecordedCloudSamples::Data d;d.points.push_back(node(id).begin()->second);return d;
        });
        return [owned=sample.points] {Cloud c;for(const auto &p:owned)c.emplace(RecordedCloudCache::Key{0,0,0},p);return c;};
    };
    auto prepare=[&](const std::vector<int> &required) {
        planned=required;std::vector<int> missing;
        for(int id:required)if(!cache.matches(id,inputs.at(id).pose,inputs.at(id).input))missing.push_back(id);
        jobs.reset(new RecordedOrderedPrefetch<Cloud>(std::move(missing),prepareNode));
    };
    auto build=[&](int id)->const Cloud& {
        consumed.push_back(id);
        cache.update(id,inputs.at(id).pose,inputs.at(id).input,[&] {
            if(jobs->prepared(id))return jobs->take(id);
            synchronous.push_back(id);return prepareNode(id)();
        });return cache.points(id);
    };
    auto finish=[&]{jobs->finish();};
    auto compare=[&] {
        Cloud reference;for(const auto &n:inputs){auto c=node(n.first);reference.insert(c.begin(),c.end());}
        assert(ply(reference)==ply(assembly.points()));
    };
    assert(assembly.updatePrepared(inputs,prepare,build,finish));compare();
    assert(planned==std::vector<int>({1,2,3,4,5,6,7}));assert(consumed==planned);
    assert(synchronous==std::vector<int>({4})); // transient sample storage copied before next prepare
    inputs[8]={pose,"new"};assembly.updatePrepared(inputs,prepare,build,finish);compare();
    assert(planned==std::vector<int>({8}));
    assert(!assembly.updatePrepared(inputs,prepare,build,finish));
    inputs[0]={pose,"lower"};assembly.updatePrepared(inputs,prepare,build,finish);compare();
    assert(planned.size()==9 && planned.front()==0);
    inputs.erase(0);assembly.updatePrepared(inputs,prepare,build,finish);compare();assert(planned.size()==8);
    inputs[2].pose[3]=1;assembly.updatePrepared(inputs,prepare,build,finish);compare();assert(planned.size()==8);
    inputs[3].input="new pixels";assembly.updatePrepared(inputs,prepare,build,finish);compare();assert(planned.size()==8);

    const auto before=ply(assembly.points());auto committed=inputs;
    inputs[9]={pose,"unpublished"};
    fails([&]{assembly.updatePrepared(inputs,prepare,build,[&]{finish();throw std::runtime_error("finish");});});
    assert(ply(assembly.points())==before && !assembly.updatePrepared(committed,prepare,build,finish));
    fails([&]{assembly.updatePrepared(inputs,prepare,[&](int)->const Cloud&{throw std::runtime_error("build");},finish);});
    assert(ply(assembly.points())==before);
    jobs.reset(); // drains tasks even though building/publishing failed
    assert(assembly.updatePrepared(inputs,prepare,build,finish));compare();
}
