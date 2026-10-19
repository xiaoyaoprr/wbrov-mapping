#include <rtabmap_slam/MappingContinuityGate.h>
#include <cassert>
#include <future>
#include <iostream>
#include <limits>
using Gate=rtabmap_slam::MappingContinuityGate;
constexpr long long T=1000000000LL;
constexpr long long D=100000000LL;
Gate::Evidence visual() { Gate::Evidence e;e.lost=false;e.visualGeometry=true;
    e.finiteMotion=e.covarianceValid=e.recoveryGeometry=e.covarianceFinitePD=e.dimensionsValid=e.measuredRegistration=true;return e; }
Gate::Evidence depth() {auto e=visual();e.visualGeometry=false;e.depthGeometry=true;return e;}
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
        auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;
        observe(g,T+3*D,weak);stable(g,T+4*D);
        assert(g.beginMapObservation(0,true,T+6*D,T+6*D));
        observe(g,T+7*D);
        const auto ticket=g.ticket(T+7*D);
        assert(g.waitForRegisteredMapObservation(ticket,0,T+7*D,T+7*D));
        assert(!g.mainMapAllowed(0,T+7*D,T+7*D) && g.localContinuitySerial()==serial);
        assert(g.ticket(T+7*D).generation==ticket.generation);
        assert(!g.waitForRegisteredMapObservation(ticket,0,T+7*D,T+7*D)); // no repeated evidence
        mapObservations(g,T+8*D);
        assert(g.mainMapAllowed(0,T+14*D,T+14*D));
        assert(g.localContinuitySerial()==serial);
    }
    {
        Gate g;normal(g);auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;
        observe(g,T+3*D,weak);stable(g,T+4*D);
        assert(g.beginMapObservation(0,true,T+6*D,T+6*D));
        const auto serial=g.localContinuitySerial();
        for(int i=7;i<=63;++i) {
            observe(g,T+i*D);
            assert(g.waitForRegisteredMapObservation(g.ticket(T+i*D),0,T+i*D,T+i*D));
            assert(!g.mainMapAllowed(0,T+i*D,T+i*D));
        }
        g.tick(T+63*D+1,T+63*D+1);
        assert(g.localContinuitySerial()>serial); // misses never slide six-second budget
    }
    {
        Gate g;stable(g,T);assert(!confirm(g,T+2*D,0,false));
        const auto serial=g.localContinuitySerial();
        auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;
        observe(g,T+3*D,weak);
        assert(g.snapshot(T+3*D,T+3*D).qualityPending);
        assert(g.localContinuitySerial()==serial);
        stable(g,T+4*D);
        assert(g.beginMapObservation(0,false,T+6*D,T+6*D));
        assert(g.snapshot(T+6*D,T+6*D).observingLocal);
        mapObservations(g,T+7*D);
        assert(!g.mainMapAllowed(0,T+13*D,T+13*D));
        assert(g.processAllowed(false,T+13*D,T+13*D,T+13*D));
        assert(g.localContinuitySerial()==serial);
    }
    {
        // 250 ms transport + 100 ms inter-arrival is not a measurement stall.
        // The source pose is nevertheless too old for navigation publication.
        Gate g;
        for(int i=0;i<3;++i)assert(g.observe(T+i*D,T+i*D+250000000LL,T+i*D+250000000LL,visual()));
        const auto source=T+2*D,arrival=source+250000000LL;
        assert(g.coreValidated(g.ticket(source),0,true,true,arrival,arrival));
        g.tick(arrival+D,arrival+D);
        assert(g.mainMapAllowed(0,arrival+D,arrival+D));
        int sent=0;assert(!g.withFreshMainPose(arrival+D,arrival+D,[&]{++sent;}));assert(sent==0);
        const auto generation=g.ticket(source).generation;
        g.tick(arrival+4*D,arrival+4*D);
        assert(g.ticket(source).generation==generation);
        assert(!g.recoveryCommitAllowed(g.ticket(source),0,arrival+4*D,arrival+4*D));
        observe(g,source+7*D,Gate::Evidence{});
        assert(g.ticket(source).generation!=generation);
    }
    {
        // A later weak episode must not retroactively expire the first one.
        // Every episode remains bounded, and all observations restart after it.
        Gate g;normal(g);const auto serial=g.localContinuitySerial();
        auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;
        observe(g,T+3*D,weak);stable(g,T+4*D);
        for(int i=7;i<=12;++i)observe(g,T+i*D);
        const auto beforeWeak=g.ticket(T+12*D);
        observe(g,T+13*D,weak);
        assert(g.snapshot(T+13*D,T+13*D).qualityPending);
        assert(g.localContinuitySerial()==serial);
        assert(!g.recoveryCommitAllowed(beforeWeak,0,T+13*D,T+13*D));
        stable(g,T+14*D);assert(g.beginMapObservation(0,true,T+16*D,T+16*D));
        mapObservations(g,T+17*D);
        assert(g.mainMapAllowed(0,T+23*D,T+23*D));
        assert(g.localContinuitySerial()==serial);
    }
    {
        Gate g;normal(g);const auto serial=g.localContinuitySerial();
        auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;
        observe(g,T+3*D,weak);stable(g,T+4*D);
        for(int i=7;i<=12;++i)observe(g,T+i*D);
        for(int i=13;i<=19;++i)observe(g,T+i*D,weak);
        assert(g.localContinuitySerial()>serial);
        assert(!g.snapshot(T+19*D,T+19*D).qualityPending);
    }
    {
        Gate g;normal(g);auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;
        const auto before=g.snapshot(T+2*D,T+2*D);
        observe(g,T+6*D,weak);const auto after=g.snapshot(T+6*D,T+6*D);
        assert(after.cutCount==before.cutCount+1);
        assert(after.lastCutReason=="unverified measurement gap");
        assert(after.sourceNs==T+6*D && !after.qualityPending && !after.mainMapWriteAllowed);
    }
    {
        Gate g;normal(g);const auto serial=g.localContinuitySerial();const auto old=g.ticket(T+2*D);
        auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;weak.qualityReason="cells=4<6";
        observe(g,T+3*D,weak);
        auto state=g.snapshot(T+3*D,T+3*D);
        assert(state.qualityPending && state.qualityTriggerReason=="cells=4<6");
        assert(g.localContinuitySerial()==serial && !g.mainMapAllowed(0,T+3*D,T+3*D));
        assert(!g.processAllowed(false,T+3*D,T+3*D,T+3*D));
        for(int i=4;i<=9;++i) {
            observe(g,T+i*D);assert(!g.resumeContinuousReceipt(0,true,T+i*D,T+i*D,T+i*D));
        }
        assert(!g.coreValidated(old,0,true,true,T+9*D,T+9*D));
        assert(g.beginMapObservation(0,true,T+9*D,T+9*D));
        mapObservations(g,T+10*D);
        assert(g.mainMapAllowed(0,T+16*D,T+16*D));
        assert(g.localContinuitySerial()==serial);
    }
    for(int scenario=0;scenario<7;++scenario) {
        Gate g;normal(g);const auto serial=g.localContinuitySerial();
        auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;weak.qualityReason="cells=4<6";
        if(scenario==0)weak.lost=true;
        if(scenario==1)weak.finiteMotion=false;
        if(scenario==2)weak.covarianceFinitePD=false;
        if(scenario==3)weak.dimensionsValid=false;
        if(scenario==4)weak.measuredRegistration=false; // IMU-only/numeric-only is never a bridge
        observe(g,T+3*D,weak);
        if(scenario==5) {for(int i=4;i<=9;++i)observe(g,T+i*D,weak);} // no sliding weak budget
        if(scenario==6) {stable(g,T+4*D);assert(g.beginMapObservation(0,true,T+6*D,T+6*D));
            observe(g,T+7*D);assert(!g.observeRegisteredMap(g.ticket(T+7*D),0,false,0,0,0,0,T+7*D,T+7*D));}
        assert(g.localContinuitySerial()>serial);
        const auto state=g.snapshot(T+9*D,T+9*D);
        assert(!state.qualityPending && !state.mainMapWriteAllowed && !state.lastCutReason.empty());
        assert(state.lastCutQualityReason=="cells=4<6" || scenario==6);
    }
    {
        Gate g;normal(g);const auto serial=g.localContinuitySerial();
        auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;
        observe(g,T+3*D,weak);stable(g,T+4*D);
        for(int i=7;i<=63;++i)observe(g,T+i*D);
        assert(g.localContinuitySerial()==serial);
        // No observer/timer scheduling can extend the six-second deadline.
        assert(g.beginMapObservation(0,true,T+63*D,T+63*D));
        assert(!g.observeRegisteredMap(g.ticket(T+63*D),0,true,0,0,0,0,T+63*D+1,T+63*D+1));
        assert(g.localContinuitySerial()>serial);
    }
    {
        Gate g;normal(g);auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;
        observe(g,T+3*D,weak);const auto serial=g.localContinuitySerial();
        g.tick(T+8*D+1,T+8*D+1);
        assert(g.localContinuitySerial()>serial && !g.snapshot(T+8*D+1,T+8*D+1).qualityPending);
    }
    for(int scenario=0;scenario<8;++scenario) {
        Gate g;normal(g);const auto serial=g.localContinuitySerial();const auto ticket=g.ticket(T+2*D);
        auto e=visual();long long gap=350000000LL;e.measuredIntervalNs=gap;
        if(scenario==1)e.measuredIntervalNs=0; // initialization/reset or no evidence
        if(scenario==2)e.measuredIntervalNs=50000000LL; // missing DDS reports
        if(scenario==3){gap=750000001LL;e.measuredIntervalNs=gap;}
        if(scenario==4)e.recoveryGeometry=false;
        if(scenario==5)e.lost=true;
        if(scenario==6){observe(g,T+3*D,Gate::Evidence{});}
        if(scenario==7){gap=750000000LL;e.measuredIntervalNs=gap;}
        const auto t=(scenario==6?T+3*D:T+2*D)+gap;
        g.tick(t,t);observe(g,t,e);
        const bool bridged=scenario==0 || scenario==7;
        assert((g.localContinuitySerial()==serial)==bridged);
        assert(!g.mainMapAllowed(0,t,t));
        assert(!g.coreValidated(ticket,0,true,true,t,t));
        for(int i=1;i<=6;++i) {
            const auto fresh=t+i*D;observe(g,fresh);
            const bool resumed=g.resumeContinuousReceipt(0,true,fresh,fresh,fresh);
            if(i<5 || !bridged)assert(!resumed);
        }
        assert(g.mainMapAllowed(0,t+6*D,t+6*D)==bridged);
    }

    {
        // A late delivery with an unbroken measured source sequence pauses the
        // map but must not force Core to create an unrelated coordinate frame.
        Gate g;normal(g);const auto serial=g.localContinuitySerial();
        const auto inFlight=g.ticket(T+2*D);
        g.tick(T+51*D/10,T+51*D/10);
        assert(g.localContinuitySerial()==serial);
        assert(!g.mainMapAllowed(0,T+51*D/10,T+51*D/10));
        assert(!g.processAllowed(false,T+2*D,T+51*D/10,T+51*D/10));
        assert(!g.coreValidated(inFlight,0,true,true,T+51*D/10,T+51*D/10));
        for(int i=3;i<=5;++i) {
            const auto source=T+i*D,arrival=source+22*D/10;
            assert(g.observe(source,arrival,arrival,visual()));
        }
        assert(g.localContinuitySerial()==serial);
        const auto now=T+72*D/10;
        assert(!g.mainMapAllowed(0,now,now));
        assert(!g.processAllowed(true,T+5*D,now,now));
        assert(!g.beginMapObservation(0,true,now,now)); // no two-old-keyframe deadlock on a one-node map
        for(unsigned i=0;i<3;++i) {
            const auto source=T+6*D+static_cast<long long>(i)*D;
            const auto arrival=source+22*D/10;
            assert(g.observe(source,arrival,arrival,visual()));
            assert(!g.resumeContinuousReceipt(1,true,source,arrival,arrival));
            assert(!g.resumeContinuousReceipt(0,false,source,arrival,arrival));
            assert(g.resumeContinuousReceipt(0,true,source,arrival,arrival)==(i==2));
            if(i<2)assert(!g.mainMapAllowed(0,arrival,arrival));
        }
        assert(g.mainMapAllowed(0,T+102*D/10,T+102*D/10));
        assert(!g.coreValidated(inFlight,0,true,true,T+102*D/10,T+102*D/10));
        assert(g.processAllowed(true,T+8*D,T+102*D/10,T+102*D/10));
    }
    {
        Gate g;stable(g,T);assert(!confirm(g,T+2*D,0,false));
        const auto serial=g.localContinuitySerial();
        assert(g.observe(T+3*D,T+7*D,T+7*D,visual())); // late, still same source chain
        assert(g.localContinuitySerial()==serial);
        assert(!g.processAllowed(false,T+3*D,T+7*D,T+7*D));
        assert(g.observe(T+4*D,T+71*D/10,T+71*D/10,visual()));
        for(int i=5;i<=7;++i) {
            const auto source=T+i*D,arrival=source+22*D/10;
            assert(g.observe(source,arrival,arrival,visual()));
        }
        assert(g.localContinuitySerial()==serial);
        assert(g.processAllowed(false,T+7*D,T+92*D/10,T+92*D/10));
        assert(!g.mainMapAllowed(0,T+92*D/10,T+92*D/10));
        assert(!g.resumeContinuousReceipt(0,true,T+7*D,T+92*D/10,T+92*D/10));
    }
    for(int cause=0;cause<6;++cause) {
        Gate g;normal(g);g.tick(T+51*D/10,T+51*D/10);
        // Every genuine coordinate/quality change cancels receipt-only resume.
        if(cause==0)observe(g,T+6*D); // actual source gap
        if(cause==1)g.observe(T+3*D,T+51*D/10,T+51*D/10,Gate::Evidence{});
        if(cause==2)g.invalidate("reset or jump");
        if(cause==3)g.rejectAlignment("rejected geometry");
        if(cause==4)g.observe(T+3*D,T+51*D/10,T+3*D-1,visual());
        if(cause==5)g.coreValidated(g.ticket(T+2*D),1,false,true,T+51*D/10,T+51*D/10);
        const int first=cause==0?7:((cause==1 || cause==4)?4:3);
        for(int i=first;i<=first+7;++i) {
            const auto source=T+i*D,arrival=source+22*D/10;
            assert(g.observe(source,arrival,arrival,visual()));
        }
        const auto end=T+(first+7)*D,arrival=end+22*D/10;
        assert(!g.resumeContinuousReceipt(cause==5?1:0,true,end,arrival,arrival));
        assert(!g.mainMapAllowed(cause==5?1:0,arrival,arrival));
    }
    {
        // No timer or receipt-age change may hide a true measurement gap/loss.
        Gate gap;normal(gap);const auto serial=gap.localContinuitySerial();
        gap.tick(T+6*D,T+6*D);observe(gap,T+7*D);
        assert(gap.localContinuitySerial()>serial);
        assert(!gap.processAllowed(false,T+7*D,T+7*D,T+7*D));
        Gate lost;normal(lost);const auto lostSerial=lost.localContinuitySerial();
        assert(lost.observe(T+3*D,T+7*D,T+7*D,Gate::Evidence{}));
        assert(lost.localContinuitySerial()>lostSerial);
        Gate future;normal(future);const auto futureSerial=future.localContinuitySerial();
        assert(future.observe(T+3*D,T+3*D,T+3*D-1,visual()));
        assert(future.localContinuitySerial()>futureSerial);
    }
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
    {
        // A prepared proposal survives a bounded weak episode, but restarts
        // independent observations. Commit must resolve the window completely.
        Gate g;normal(g);observe(g,T+3*D);
        assert(!g.coreValidated(g.ticket(T+3*D),1,false,true,T+3*D,T+3*D));
        observe(g,T+4*D);observe(g,T+5*D);const auto prepared=g.ticket(T+5*D);
        assert(g.beginCandidateObservation(prepared,1,T+5*D,T+5*D));
        auto weak=visual();weak.visualGeometry=weak.recoveryGeometry=false;
        observe(g,T+6*D,weak);
        assert(g.snapshot(T+6*D,T+6*D).qualityPending);
        assert(g.ticket(T+6*D).generation==prepared.generation);
        assert(!g.candidateCommitReady(prepared,1,T+6*D,T+6*D));
        stable(g,T+7*D);mapObservations(g,T+10*D,1);
        const auto end=T+16*D;
        assert(g.candidateCommitReady(prepared,1,end,end));
        assert(g.candidateCommitted(prepared,1,true,end,end));
        assert(!g.snapshot(end,end).qualityPending);
        assert(g.processAllowed(true,end,end,end));
        const auto serial=g.localContinuitySerial();
        for(int i=17;i<=75;++i)observe(g,T+i*D);
        assert(g.localContinuitySerial()==serial && g.mainMapAllowed(1,T+75*D,T+75*D));
    }
    {
        // Hardware trace: each independent old-map solve took about 2 s.
        // The original 6 s admission ticket expired after only 2/5 solves,
        // although fresh authenticated odometry continued without a cut.
        Gate g;normal(g);observe(g,T+3*D);
        assert(!g.coreValidated(g.ticket(T+3*D),1,false,true,T+3*D,T+3*D));
        observe(g,T+4*D);observe(g,T+5*D);
        const auto prepared=g.ticket(T+5*D);long long now=T+5*D;
        assert(g.beginCandidateObservation(prepared,1,now,now));
        for(unsigned i=0;i<5;++i) {
            observe(g,now+D);now+=D;const auto frame=g.ticket(now);
            assert(g.observationInputAllowed(frame,1,now,now));
            for(int j=0;j<20;++j){now+=D;observe(g,now);g.tick(now,now);}
            assert(g.recoveryCommitAllowed(prepared,1,now,now));
            if(i==4)assert(!g.recoveryCommitAllowed(g.ticket(now),1,now,now)); // cannot replace original identity
            assert(g.observeRegisteredMap(frame,1,true,0,0,0,0,now,now)==(i==4));
            assert(!g.mainMapAllowed(1,now,now));
        }
        assert(now-prepared.frameNs>6*T);
        assert(g.candidateCommitReady(prepared,1,now,now));
        assert(!g.beginCandidateObservation(prepared,1,now,now)); // duplicate prepare cannot renew life
        for(int j=0;j<20;++j){now+=D;observe(g,now);g.tick(now,now);}
        assert(g.recoveryCommitAllowed(prepared,1,now,now));
        assert(g.candidateCommitted(prepared,1,true,now,now));
        assert(g.mainMapAllowed(1,now,now));
    }
    {
        Gate g;normal(g);observe(g,T+3*D);
        assert(!g.coreValidated(g.ticket(T+3*D),1,false,true,T+3*D,T+3*D));
        observe(g,T+4*D);observe(g,T+5*D);
        const auto prepared=g.ticket(T+5*D);long long now=T+5*D;
        assert(g.beginCandidateObservation(prepared,1,now,now));
        // Fresh packets alone must never renew a stalled solve.
        for(int j=0;j<61;++j){now+=D;observe(g,now);g.tick(now,now);}
        assert(!g.recoveryCommitAllowed(prepared,1,now,now));
        assert(!g.candidateCommitReady(prepared,1,now,now));
        assert(!g.candidateCommitted(prepared,1,true,now,now));
        assert(!g.mainMapAllowed(1,now,now));
        assert(!g.beginCandidateObservation(prepared,1,now,now));
    }
    {
        Gate g;normal(g);observe(g,T+3*D);
        assert(!g.coreValidated(g.ticket(T+3*D),1,false,true,T+3*D,T+3*D));
        observe(g,T+4*D);observe(g,T+5*D);const auto prepared=g.ticket(T+5*D);
        assert(g.beginCandidateObservation(prepared,1,T+5*D,T+5*D));mapObservations(g,T+6*D,1);
        const auto end=T+12*D;assert(g.candidateCommitReady(prepared,1,end,end));
        assert(!g.recoveryCommitAllowed(prepared,1,end+4*D,end+4*D));
        assert(!g.candidateCommitted(prepared,1,true,end+4*D,end+4*D)); // current stream still must be fresh
    }
    std::cout<<"PASS: bootstrap, measured hysteresis, unique/fresh source, observation before candidate commit, failed K/loss invalidate commit, bounded slow verification, stalled solve/current freshness rejection, unique final commit, same-epoch no-Tag, depth/IMU boundaries\n";
}
