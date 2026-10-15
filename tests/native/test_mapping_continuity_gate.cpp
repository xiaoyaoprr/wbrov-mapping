#include <rtabmap_slam/MappingContinuityGate.h>
#include <cassert>
#include <future>
#include <iostream>
#include <limits>
using Gate=rtabmap_slam::MappingContinuityGate;
constexpr long long T=1000000000LL;
constexpr long long D=100000000LL;
Gate::Evidence visual() { return {false,true,false,true,true,true}; }
Gate::Evidence depth() { return {false,false,true,true,true,true}; }
void observe(Gate & g,long long t,Gate::Evidence e=visual()) { assert(g.observe(t,t,t,e)); }
void stable(Gate & g,long long t) { observe(g,t);observe(g,t+D);observe(g,t+2*D); }
bool confirm(Gate & g,long long t,Gate::Epoch e=0,bool reg=true) {return g.coreValidated(g.ticket(t),e,reg,true,t,t);}
void normal(Gate &g) {stable(g,T);assert(confirm(g,T+2*D));assert(g.mainMapAllowed(0,T+2*D,T+2*D));}
void mapObservations(Gate &g,long long start,Gate::Epoch epoch=0,Gate::Evidence e=visual()) {
    for(unsigned i=0;i<5;++i) {
        const auto t=start+static_cast<long long>(i)*150000000LL;observe(g,t,e);
        const bool done=g.observeRegisteredMap(g.ticket(t),epoch,true,0.001,0.1,0.001,0.1,t,t);
        assert(done==(i==4));
        if(i<4)assert(!g.processAllowed(true,t,t,t));
    }
}
int main() {
    {
        Gate g;normal(g);const auto serial=g.localContinuitySerial();
        g.rejectAlignment("bad global candidate");assert(g.localContinuitySerial()==serial);
        stable(g,T+3*D);assert(g.localGeometryReady(T+5*D,T+5*D,T+5*D));
        assert(g.processAllowed(false,T+5*D,T+5*D,T+5*D));
        assert(!g.mainMapAllowed(0,T+5*D,T+5*D));
        observe(g,T+6*D,Gate::Evidence{});assert(g.localContinuitySerial()>serial);
        assert(!g.processAllowed(false,T+6*D,T+6*D,T+6*D));
        stable(g,T+7*D);assert(g.localGeometryReady(T+9*D,T+9*D,T+9*D));
    }
    {
        Gate g;assert(!g.mainMapAllowed(0,T,T));observe(g,T);
        assert(!g.processAllowed(true,T,T,T));assert(!g.processAllowed(false,T,T,T));
        assert(!confirm(g,T));observe(g,T+D);assert(!confirm(g,T+D));
        observe(g,T+2*D);assert(confirm(g,T+2*D));
        assert(g.snapshot(T+2*D,T+2*D).state==Gate::State::NORMAL);
        assert(!g.mainMapAllowed(1,T+2*D,T+2*D));
    }
    {
        Gate g;normal(g);const auto t=g.ticket(T+2*D);
        assert(!g.observe(T+2*D,T+4*D,T+4*D,visual()));
        assert(!g.observe(T+D,T+4*D,T+4*D,visual()));
        assert(g.mainMapAllowed(0,T+5*D,T+5*D));
        assert(!g.mainMapAllowed(0,T+5*D+1,T+5*D));
        g.tick(T+6*D,T+6*D);assert(g.snapshot(T+6*D,T+6*D).state==Gate::State::PAUSED);
        assert(!g.coreValidated(t,0,true,true,T+6*D,T+6*D));
    }
    {
        Gate g;normal(g);auto inFlight=g.ticket(T+2*D);const auto writtenSerial=g.localContinuitySerial();
        observe(g,T+3*D,Gate::Evidence{});assert(!g.mainMapAllowed(0,T+3*D,T+3*D));
        assert(g.localContinuitySerial()!=writtenSerial); // in-flight work must retain its pre-process serial
        assert(!g.processAllowed(true,T+3*D,T+3*D,T+3*D));
        observe(g,T+4*D);assert(!g.coreValidated(inFlight,0,true,true,T+4*D,T+4*D));
        assert(!confirm(g,T+4*D));observe(g,T+5*D);assert(!confirm(g,T+5*D));
        observe(g,T+6*D);assert(!confirm(g,T+6*D)); // no Tag; registered epoch still requires direct map observations
        assert(g.snapshot(T+6*D,T+6*D).observingMainMap);
        assert(!g.mainMapAllowed(0,T+6*D,T+6*D));
        mapObservations(g,T+7*D);
        assert(g.mainMapAllowed(0,T+13*D,T+13*D));
    }
    {
        Gate g;normal(g);observe(g,T+3*D);
        auto old=g.ticket(T+3*D);assert(!g.coreValidated(old,1,false,true,T+3*D,T+3*D));
        assert(!g.mainMapAllowed(0,T+3*D,T+3*D));
        assert(!g.coreValidated(old,1,true,true,T+3*D,T+3*D));
        observe(g,T+4*D);assert(!confirm(g,T+4*D,1,true));
        observe(g,T+5*D);assert(!confirm(g,T+5*D,1,false)); // tracking is not registration
        assert(!confirm(g,T+5*D,1,true)); // fresh registration starts read-only observation period
        mapObservations(g,T+6*D,1);
    }
    {
        Gate g;normal(g);observe(g,T+3*D,depth());
        assert(g.snapshot(T+3*D,T+3*D).state==Gate::State::DEGRADED);
        assert(g.mainMapAllowed(0,T+3*D,T+3*D));
        auto weakVisual=visual();weakVisual.recoveryGeometry=false;
        observe(g,T+35*D/10,weakVisual);
        assert(g.snapshot(T+35*D/10,T+35*D/10).state==Gate::State::DEGRADED);
        auto imuOnly=depth();imuOnly.depthGeometry=false;observe(g,T+4*D,imuOnly);
        assert(g.snapshot(T+4*D,T+4*D).state==Gate::State::PAUSED);
        assert(!g.mainMapAllowed(0,T+4*D,T+4*D));
    }
    {
        Gate g;normal(g);observe(g,T+3*D,depth());
        observe(g,T+4*D);assert(g.snapshot(T+4*D,T+4*D).state==Gate::State::DEGRADED);
        observe(g,T+5*D);assert(g.snapshot(T+5*D,T+5*D).state==Gate::State::DEGRADED);
        observe(g,T+6*D);assert(g.snapshot(T+6*D,T+6*D).state==Gate::State::NORMAL);
    }
    {
        Gate g;normal(g);assert(g.observe(T+3*D,T+3*D,T+7*D,visual()));
        assert(!confirm(g,T+3*D));assert(g.snapshot(T+7*D,T+7*D).state==Gate::State::PAUSED);
        Gate h;normal(h);assert(h.observe(T+3*D,T+3*D,T+3*D-1,visual()));assert(!confirm(h,T+3*D));
        Gate gap;normal(gap);observe(gap,T+7*D);assert(!confirm(gap,T+7*D));
    }
    {
        Gate g;normal(g);auto old=g.ticket(T+2*D);g.reset();
        stable(g,D);assert(!g.coreValidated(old,0,true,true,3*D,3*D));assert(confirm(g,3*D));
        g.invalidate("suspected jump");assert(!g.mainMapAllowed(0,3*D,3*D));
    }
    {
        Gate g;stable(g,T);assert(!g.coreValidated(g.ticket(T+2*D),0,true,false,T+2*D,T+2*D));
        assert(!g.coreValidated(g.ticket(T-D),0,true,true,T+2*D,T+2*D));
        assert(confirm(g,T+2*D));int sent=0;
        assert(g.withFreshMainPose(T+2*D,T+2*D,[&]{++sent;}));g.invalidate();
        assert(!g.withFreshMainPose(T+2*D,T+2*D,[&]{++sent;}));assert(sent==1);
    }
    {
        std::array<double,36> c{};assert(!Gate::covarianceHealthy(c));
        for(unsigned i=0;i<6;++i) {c[7*i]=0.001;}
        assert(Gate::covarianceHealthy(c));
        c[1]=c[6]=0.002;assert(!Gate::covarianceHealthy(c));
        c[1]=c[6]=0;c[0]=std::numeric_limits<double>::quiet_NaN();assert(!Gate::covarianceHealthy(c));
    }
    {
        std::vector<std::array<double,5>> p;
        for(unsigned y=0;y<4;++y)for(unsigned x=0;x<5;++x)
            p.push_back({{static_cast<double>(x)*80,static_cast<double>(y)*60,
                static_cast<double>(x)*0.1,static_cast<double>(y)*0.1,1.0}});
        assert(Gate::visualSupportHealthy(p,{{0,320,0,180}}));
        auto cluster=p;for(auto & a:cluster) {a[0]=100+a[0]/20;a[1]=80+a[1]/20;}
        assert(!Gate::visualSupportHealthy(cluster,{{0,384,0,240}})); // feature bbox must never become normalization frame
        for(auto & a:p) {a[1]=a[0]/2;a[3]=a[2]/2;}
        assert(!Gate::visualSupportHealthy(p,{{0,320,0,180}}));
    }
    {
        Gate g;normal(g);observe(g,T+3*D,Gate::Evidence{});stable(g,T+4*D);
        assert(g.beginMapObservation(0,true,T+6*D,T+6*D));
        observe(g,T+7*D);
        assert(!g.observeRegisteredMap(g.ticket(T+7*D),0,true,0.051,0.0,0.0,0.0,T+7*D,T+7*D));
        assert(g.snapshot(T+7*D,T+7*D).state==Gate::State::PAUSED);
        assert(!g.processAllowed(true,T+7*D,T+7*D,T+7*D));
    }
    {
        Gate g;normal(g);auto t=g.ticket(T+2*D);
        observe(g,T+3*D);observe(g,T+4*D);observe(g,T+5*D);observe(g,T+6*D);
        assert(g.recoveryCommitAllowed(t,0,T+6*D,T+6*D)); // long solve while genuinely fresh observations continue
        observe(g,T+7*D,Gate::Evidence{});
        assert(!g.recoveryCommitAllowed(t,0,T+7*D,T+7*D));
    }
    {
        Gate g;normal(g);observe(g,T+3*D);
        assert(!g.coreValidated(g.ticket(T+3*D),1,false,true,T+3*D,T+3*D));
        observe(g,T+4*D);observe(g,T+5*D);
        const auto prepared=g.ticket(T+5*D);
        assert(g.beginCandidateObservation(prepared,1,T+5*D,T+5*D));
        assert(!g.snapshot(T+5*D,T+5*D).registered);
        assert(!g.processAllowed(false,T+5*D,T+5*D,T+5*D));
        assert(!g.candidateCommitReady(prepared,1,T+5*D,T+5*D));
        assert(!g.coreValidated(prepared,1,true,true,T+5*D,T+5*D)); // registration flag cannot bypass pending K
        mapObservations(g,T+6*D,1);
        const auto end=T+12*D;
        assert(!g.mainMapAllowed(1,end,end));
        assert(!g.processAllowed(false,end,end,end));
        assert(g.candidateCommitReady(prepared,1,end,end));
        assert(!g.candidateCommitted(prepared,1,false,end,end));
        assert(g.candidateCommitted(prepared,1,true,end,end));
        assert(g.mainMapAllowed(1,end,end));
        assert(!g.candidateCommitted(prepared,1,true,end,end)); // unique final outcome
    }
    {
        Gate g;normal(g);observe(g,T+3*D);
        assert(!g.coreValidated(g.ticket(T+3*D),1,false,true,T+3*D,T+3*D));
        observe(g,T+4*D);observe(g,T+5*D);const auto prepared=g.ticket(T+5*D);
        assert(g.beginCandidateObservation(prepared,1,T+5*D,T+5*D));
        observe(g,T+6*D);
        assert(!g.observeRegisteredMap(g.ticket(T+6*D),1,false,0,0,0,0,T+6*D,T+6*D));
        assert(!g.candidateCommitReady(prepared,1,T+6*D,T+6*D));
        assert(!g.candidateCommitted(prepared,1,true,T+6*D,T+6*D));
        assert(!g.mainMapAllowed(1,T+6*D,T+6*D));
    }
    {
        Gate g;normal(g);observe(g,T+3*D);
        assert(!g.coreValidated(g.ticket(T+3*D),1,false,true,T+3*D,T+3*D));
        observe(g,T+4*D);observe(g,T+5*D);const auto prepared=g.ticket(T+5*D);
        assert(g.beginCandidateObservation(prepared,1,T+5*D,T+5*D));mapObservations(g,T+6*D,1);
        observe(g,T+13*D,Gate::Evidence{});
        assert(!g.candidateCommitReady(prepared,1,T+13*D,T+13*D));
        assert(!g.candidateCommitted(prepared,1,true,T+13*D,T+13*D));
    }
    std::cout<<"PASS: bootstrap, measured hysteresis, unique/fresh source, observation before candidate commit, failed K/loss invalidate commit, unique final commit, same-epoch no-Tag, depth/IMU boundaries\n";
}
