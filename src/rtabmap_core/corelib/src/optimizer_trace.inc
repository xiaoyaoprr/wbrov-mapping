// Private diagnostic build only. No graph or optimizer values are modified.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <stdexcept>
namespace {
using TracePoses = std::map<int, rtabmap::Transform>;
using TraceLinks = std::multimap<int, rtabmap::Link>;
void traceOptimizer(unsigned long call, const char * phase, int rootId,
                    const TracePoses & poses, const TraceLinks & links,
                    const rtabmap::Optimizer * optimizer) {
    const char * path = std::getenv("WBROV_PRIVATE_OPTIMIZER_TRACE");
    if(!path) throw std::runtime_error("missing private trace directory");
    const std::filesystem::path dir(path);
    if(!std::filesystem::is_directory(dir)) throw std::runtime_error("trace directory absent");
    auto errors = rtabmap::graph::computeMaxGraphErrors(poses, links, false);
    int gravity=0;for(const auto &e:links)if(e.second.type()==rtabmap::Link::kGravity)++gravity;
    std::ofstream f(dir/"calls.csv",std::ios::app);
    f<<std::setprecision(17)<<call<<','<<phase<<','<<rootId<<','<<poses.size()<<','<<links.size()<<','<<gravity
     <<','<<optimizer->iterations()<<','<<optimizer->gravitySigma()<<','<<optimizer->priorsIgnored()
     <<','<<optimizer->isSlam2d()<<','<<errors.linear<<','<<errors.linearRatio
     <<','<<errors.linearLink.from()<<','<<errors.linearLink.to()<<','<<errors.angular<<','<<errors.angularRatio
     <<','<<errors.angularLink.from()<<','<<errors.angularLink.to();
    if(poses.count(rootId))for(int i=0;i<12;++i)f<<','<<poses.at(rootId)[i];
    f<<'\n';f.close();if(!f)throw std::runtime_error("trace write failed");
    const int last=poses.empty()?0:poses.rbegin()->first;
    if(last==124||last==125||last==137||last==757||last==758||last==759||last==760||last==825||last==1195){
        const std::string name=std::to_string(call)+"_"+phase;
        const auto pd=dir/(name+"_poses.csv"),ed=dir/(name+"_links.csv");
        if(std::filesystem::exists(pd)||std::filesystem::exists(ed))throw std::runtime_error("trace collision");
        std::ofstream p(pd),e(ed);p<<std::setprecision(17);e<<std::setprecision(17);
        for(const auto&v:poses){p<<v.first;for(int i=0;i<12;++i)p<<','<<v.second[i];p<<'\n';}
        for(const auto&v:links){const auto&l=v.second;e<<l.from()<<','<<l.to()<<','<<int(l.type());
          for(int i=0;i<12;++i)e<<','<<l.transform()[i];for(int i=0;i<36;++i)e<<','<<l.infMatrix().at<double>(i/6,i%6);e<<'\n';}
        p.close();e.close();if(!p||!e)throw std::runtime_error("trace detail write failed");
    }
}
unsigned long traceNextCall(){static unsigned long n=0;return ++n;}
} // namespace
