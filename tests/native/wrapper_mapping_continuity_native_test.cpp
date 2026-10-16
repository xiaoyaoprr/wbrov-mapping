#include <rtabmap_slam/CoreWrapper.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>

// Adapter-level synthetic test through the actual linked Wrapper. Explicit
// instantiation accesses private members without adding production test hooks.
// This test does NOT claim end-to-end DDS source authentication or water proof.
template<typename Tag,typename Tag::type Member> struct Access {
    friend typename Tag::type member(Tag) {return Member;}
};
using Wrapper=rtabmap_slam::CoreWrapper;
using Gate=rtabmap_slam::MappingContinuityGate;
struct Observe {using type=bool(Wrapper::*)(const rtabmap_msgs::msg::OdomInfo &);friend type member(Observe);};
template struct Access<Observe,&Wrapper::observeResetQuarantineOdomInfo>;
struct GateMember {using type=Gate Wrapper::*;friend type member(GateMember);};
template struct Access<GateMember,&Wrapper::mappingContinuityGate_>;
struct PubGate {using type=bool(Wrapper::*)()const;friend type member(PubGate);};
template struct Access<PubGate,&Wrapper::globalMapPublicationAllowed>;
struct Core {using type=rtabmap::Rtabmap Wrapper::*;friend type member(Core);};
template struct Access<Core,&Wrapper::rtabmap_>;
struct Status {using type=void(Wrapper::*)();friend type member(Status);};
template struct Access<Status,&Wrapper::publishMappingContinuityStatus>;
void require(bool condition,const char * why) {if(!condition)throw std::runtime_error(why);}
long long steady() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}

std::vector<rclcpp::Parameter> parameters(const std::string & database)
{
	const std::string zeroSha(64, '0');
	const std::string sessionToken =
			"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
	return {
		rclcpp::Parameter("database_path", database),
		rclcpp::Parameter("is_rtabmap_paused", false),
		rclcpp::Parameter("publish_tf", false),
		rclcpp::Parameter("wait_for_transform", 0.0),
		rclcpp::Parameter("frame_id", "base_link"),
		rclcpp::Parameter("subscribe_sensor_data", false),
		rclcpp::Parameter("reset_quarantine_enabled", true),
		rclcpp::Parameter(
				"reset_quarantine_expected_odom_info_node",
				"private_test_odom_source"),
		rclcpp::Parameter("reset_quarantine_expected_odom_namespace", "/"),
		rclcpp::Parameter(
				"reset_quarantine_expected_sensor_node",
				"private_test_odom_source"),
		rclcpp::Parameter("reset_quarantine_expected_sensor_namespace", "/"),
		rclcpp::Parameter("reset_quarantine_lost_count", 8),
		rclcpp::Parameter("reset_quarantine_odom_info_max_age_sec", 5.0),
		rclcpp::Parameter("reset_quarantine_max_unannounced_translation", 0.5),
		rclcpp::Parameter("reset_quarantine_max_unannounced_rotation_deg", 30.0),
		rclcpp::Parameter("safe_relocalization_enabled", true),
		rclcpp::Parameter("safe_relocalization_start_new_map_on_first_update", false),
		rclcpp::Parameter("safe_relocalization_database_sha256", zeroSha),
		rclcpp::Parameter("safe_relocalization_session_token", sessionToken),
		rclcpp::Parameter("Mem/IncrementalMemory", std::string("true")),
		rclcpp::Parameter("Mem/InitWMWithAllNodes", std::string("true")),
		rclcpp::Parameter("Optimizer/LandmarksIgnored", std::string("true")),
		rclcpp::Parameter("Optimizer/PriorsIgnored", std::string("true")),
		rclcpp::Parameter("Rtabmap/LoopGPS", std::string("false")),
		rclcpp::Parameter("Reg/Force3DoF", std::string("false")),
		rclcpp::Parameter("Mem/ReduceGraph", std::string("false")),
		rclcpp::Parameter("Mem/RehearsalIdUpdatedToNewOne", std::string("false")),
		rclcpp::Parameter("Mem/RehearsalSimilarity", std::string("1")),
		rclcpp::Parameter("Rtabmap/TimeThr", std::string("0")),
		rclcpp::Parameter("Rtabmap/MemoryThr", std::string("0")),
		rclcpp::Parameter("RGBD/OptimizeMaxErrorRepairRadius", std::string("0")),
		rclcpp::Parameter("Optimizer/Iterations", std::string("20"))};
}



rtabmap_msgs::msg::OdomInfo sample(Wrapper & w,bool support=true) {
    rtabmap_msgs::msg::OdomInfo m;m.header.stamp=w.now();m.header.frame_id="odom";
    m.lost=false;m.type=0;m.inliers=30;m.matches=30;m.transform.rotation.w=1.0;
    rtabmap_msgs::msg::CameraModels models;rtabmap_msgs::msg::CameraModel model;
    model.camera_info.width=384;model.camera_info.height=240;model.local_transform.rotation.w=1.0;
    models.models.push_back(model);m.local_bundle_models.push_back(models);m.local_bundle_ids.push_back(1);
    geometry_msgs::msg::Pose modelPose;modelPose.orientation.w=1.0;m.local_bundle_poses.push_back(modelPose);
    for(unsigned i=0;i<6;++i)m.covariance[7*i]=0.001;
    if(support)for(unsigned y=0;y<5;++y)for(unsigned x=0;x<6;++x) {
        const auto id=static_cast<int>(y*6+x+1);
        rtabmap_msgs::msg::KeyPoint p;p.pt.x=static_cast<float>(x*60);p.pt.y=static_cast<float>(y*50);
        rtabmap_msgs::msg::Point3f xyz;xyz.x=static_cast<float>(x)*0.1f;xyz.y=static_cast<float>(y)*0.1f;xyz.z=1.0f;
        m.words_keys.push_back(id);m.words_values.push_back(p);m.word_inliers.push_back(id);
        m.local_map_keys.push_back(id);m.local_map_values.push_back(xyz);
    }
    return m;
}
int main(int argc,char ** argv) {
    rclcpp::init(argc,argv);
    try {
        require(argc==2,"pass a new private empty fixture directory");
        std::filesystem::path root(argv[1]);require(!std::filesystem::exists(root),"fixture path already exists");
        std::filesystem::create_directories(root);
        rclcpp::NodeOptions options;options.parameter_overrides(parameters((root/"continuity.db").string()));
        auto w=std::make_shared<Wrapper>(options);auto & g=w.get()->*member(GateMember{});
        auto & core=w.get()->*member(Core{});const auto graphSize=core.getLocalOptimizedPoses().size();
        auto observe=[&](const rtabmap_msgs::msg::OdomInfo & m){return (w.get()->*member(Observe{}))(m);};
        auto bad=sample(*w,false);bad.inliers=200;bad.matches=200;
        require(observe(bad),"unique numeric-only sample refused by arrival observer");
        require(g.snapshot(steady(),w->now().nanoseconds()).state==Gate::State::PAUSED,"count-only visual evidence admitted");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));auto wrongSize=sample(*w);
        wrongSize.local_bundle_models.front().models.front().camera_info.width=960;
        require(observe(wrongSize),"wrong-size measurement arrival rejected unexpectedly");
        require(g.snapshot(steady(),w->now().nanoseconds()).state==Gate::State::PAUSED,"high-map dimensions silently treated as VO dimensions");
        rtabmap_msgs::msg::OdomInfo good;
        for(unsigned i=0;i<3;++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(110));good=sample(*w);require(observe(good),"support sample not observed");
        }
        const auto ns=rclcpp::Time(good.header.stamp).nanoseconds();
        require(g.processAllowed(true,ns,steady(),w->now().nanoseconds()),"measured visual support never becomes stable");
        require(!(w.get()->*member(PubGate{}))(),"tracking alone opened public map");
        require(g.coreValidated(g.ticket(ns),core.getResetEpoch(),true,true,steady(),w->now().nanoseconds()),"Core registration did not permit recovery");
        require(g.snapshot(steady(),w->now().nanoseconds()).state==Gate::State::NORMAL,"visual state not NORMAL");
        auto oldTicket=g.ticket(ns);
        // Expiration must revoke publication without inventing a frontend
        // coordinate reset. Model a delayed delivery whose source is continuous.
        const auto continuousSerial=g.localContinuitySerial();
        std::this_thread::sleep_for(std::chrono::milliseconds(310));
        rclcpp::spin_some(w); // Run the Wrapper's actual 100 ms expiry timer.
        require(!g.mainMapAllowed(core.getResetEpoch(),steady(),w->now().nanoseconds()),"expired evidence kept main-map writes open");
        require(g.localContinuitySerial()==continuousSerial,"receipt timer invented a local coordinate discontinuity");
        auto delayed=sample(*w);delayed.header.stamp=rclcpp::Time(ns+183000000LL);
        require(observe(delayed),"continuous delayed source was not observed");
        require(g.localContinuitySerial()==continuousSerial,"continuous delayed source split the local graph");
        require(!g.coreValidated(oldTicket,core.getResetEpoch(),true,true,steady(),w->now().nanoseconds()),"pre-expiration ticket reopened main map");
        require(!g.beginMapObservation(core.getResetEpoch(),true,steady(),w->now().nanoseconds()),"delivery stall incorrectly demanded two old keyframes");
        for(unsigned i=0;i<6;++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(110));good=sample(*w);
            require(observe(good),"continuous fresh recovery sample ignored");
        }
        const auto resumedNs=rclcpp::Time(good.header.stamp).nanoseconds();
        require(g.resumeContinuousReceipt(core.getResetEpoch(),true,resumedNs,steady(),w->now().nanoseconds()),"unbroken same-frame delivery recovery stayed locked");
        require(g.mainMapAllowed(core.getResetEpoch(),steady(),w->now().nanoseconds()),"fresh same-frame recovery did not reopen writes");
        const auto beforeGap=g.localContinuitySerial();
        std::this_thread::sleep_for(std::chrono::milliseconds(360));rclcpp::spin_some(w);
        auto bridged=sample(*w);const auto bridgedNs=rclcpp::Time(bridged.header.stamp).nanoseconds();
        bridged.interval=static_cast<float>(static_cast<double>(bridgedNs-resumedNs)/1e9);
        require(observe(bridged),"measured long interval ignored");
        require(g.localContinuitySerial()==beforeGap,"Wrapper discarded the exact F2M measured interval");
        require(!g.mainMapAllowed(core.getResetEpoch(),steady(),w->now().nanoseconds()),"measured interval bypassed observation pause");
        for(unsigned i=0;i<6;++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(110));good=sample(*w);require(observe(good),"post-gap sample ignored");
        }
        require(g.resumeContinuousReceipt(core.getResetEpoch(),true,rclcpp::Time(good.header.stamp).nanoseconds(),steady(),w->now().nanoseconds()),"measured gap failed to resume after observation");
        // Real mappingEvidence(): counts remain good, but spatial support
        // collapses. This must pause without authorizing a receipt-only bridge.
        const auto qualitySerial=g.localContinuitySerial();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));auto clustered=sample(*w);
        for(auto & word:clustered.words_values){word.pt.x=100+word.pt.x/30;word.pt.y=80+word.pt.y/30;}
        require(observe(clustered),"clustered measured frame ignored");
        auto quality=g.snapshot(steady(),w->now().nanoseconds());
        require(quality.qualityPending && quality.covarianceFinitePD && quality.dimensionsValid,"bounded geometry pause not entered");
        require(quality.qualityTriggerReason.find("cells=")!=std::string::npos,"actual quality cause missing");
        require(g.localContinuitySerial()==qualitySerial,"single weak measured frame split map before verification");
        for(unsigned i=0;i<6;++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(110));good=sample(*w);require(observe(good),"quality recovery packet ignored");
        }
        require(!g.resumeContinuousReceipt(core.getResetEpoch(),true,rclcpp::Time(good.header.stamp).nanoseconds(),steady(),w->now().nanoseconds()),"unverified weak interval used receipt-only resume");
        require(!g.mainMapAllowed(core.getResetEpoch(),steady(),w->now().nanoseconds()),"healthy packets alone authorized map writes");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));auto lost=sample(*w);lost.lost=true;
        require(observe(lost),"lost packet ignored");
        require(!g.coreValidated(oldTicket,core.getResetEpoch(),true,true,steady(),w->now().nanoseconds()),"in-flight preloss result rearmed gate");
        require(!(w.get()->*member(PubGate{}))(),"loss left public map gate open");
        require(!observe(good),"out-of-order good packet refreshed state");
        for(unsigned i=0;i<3;++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(110));good=sample(*w);
            good.inliers=good.matches=0;good.word_inliers.clear();good.icp_correspondences=120;
            good.icp_inliers_ratio=0.7f;good.icp_structural_complexity=0.01f;good.icp_structural_distribution=0.75f;
            require(observe(good),"depth sample ignored");
        }
        const auto depthNs=rclcpp::Time(good.header.stamp).nanoseconds();
        require(!g.coreValidated(g.ticket(depthNs),core.getResetEpoch(),true,true,steady(),w->now().nanoseconds()),"registration skipped the main-map observation period");
        const auto depthState=g.snapshot(steady(),w->now().nanoseconds());
        require(depthState.observingMainMap && depthState.depthGeometry && !depthState.mainMapWriteAllowed,"depth recovery bypassed read-only observations");
        require(!g.observeRegisteredMap(g.ticket(depthNs),core.getResetEpoch(),false,0,0,0,0,steady(),w->now().nanoseconds()),"missing actual main-map verification accepted");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));auto imuOnly=sample(*w,false);
        imuOnly.inliers=imuOnly.matches=0;require(observe(imuOnly),"IMU-only packet ignored");
        require(g.snapshot(steady(),w->now().nanoseconds()).state==Gate::State::PAUSED,"IMU-only opened mapping");
        require(core.getLocalOptimizedPoses().size()==graphSize,"adapter-only evidence mutated graph");
        (w.get()->*member(Status{}))();
        std::cout<<"PASS native linked Wrapper adapter: full-frame support/dimension binding, numeric-only reject, hysteresis, bootstrap, receipt expiry preserves local continuity but revokes writes, registration observation period, loss/old ticket, depth evidence, IMU-only PAUSED, no graph writes\n";
        w.reset();rclcpp::shutdown();return 0;
    }catch(const std::exception & e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
