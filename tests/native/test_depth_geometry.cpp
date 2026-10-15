#include "DepthGeometryRecovery.h"
#include <iostream>
#include <stdexcept>

using namespace wbrov_depth;
static int checks=0;
static void check(bool condition,const char *name) {
    if(!condition) throw std::runtime_error(name);
    ++checks; std::cout<<"PASS "<<name<<"\n";
}
static Frame render(const Pose &worldCamera,double stamp,bool plane=false) {
    Frame f;f.fx=145;f.fy=145;f.cx=79.5;f.cy=59.5;f.stamp=stamp;
    f.depth=cv::Mat(120,160,CV_32FC1,cv::Scalar(0));
    for(int y=0;y<f.depth.rows;++y) for(int x=0;x<f.depth.cols;++x) {
        const Eigen::Vector3d dir=worldCamera.linear()*Eigen::Vector3d((x-f.cx)/f.fx,(y-f.cy)/f.fy,1);
        const Eigen::Vector3d origin=worldCamera.translation();
        double hit=100;
        if(dir.z()>1e-8) {double t=(0.8-origin.z())/dir.z();if(t>0)hit=std::min(hit,t);}
        if(!plane) {
            for(int axis=0;axis<2;++axis) for(int sign : {-1,1}) {
                if(std::abs(dir[axis])<1e-8)continue;
                double t=(sign*(axis==0?0.25:0.18)-origin[axis])/dir[axis];
                if(t>0)hit=std::min(hit,t);
            }
        }
        if(hit<2)f.depth.at<float>(y,x)=float(hit);
    }
    return f;
}
static Pose motion(double x,double y,double z,double yaw=0) {
    Pose p=Pose::Identity();p.translation()=Eigen::Vector3d(x,y,z);
    p.linear()=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitY()).toRotationMatrix();return p;
}
int main() {
    try {
        Config cfg;const Pose identity=Pose::Identity();
        const Pose known=motion(0.012,-0.006,0.009,0.012);
        const Frame a=render(identity,1.0), b=render(known,1.1);
        Result good=evaluate(a,b,identity,cfg,true,Eigen::Vector3d::UnitZ(),known.linear().transpose()*Eigen::Vector3d::UnitZ());
        if(!good.accepted)std::cerr<<"positive rejected: "<<good.reason<<"\n";
        check(good.accepted,"real_depth_corner_geometry_accepted");
        check((good.targetFromSource.translation()-known.translation()).norm()<0.004,"transform_direction_translation");
        check(angle(good.targetFromSource.linear().transpose()*known.linear())<0.01,"transform_direction_rotation");
        check(good.correspondences>=cfg.minimumCorrespondences && good.hessianRatio>=cfg.minimumHessianRatio,"actual_correspondences_and_full_rank_information");
        check(Eigen::LLT<Matrix6>(good.covariance).info()==Eigen::Success,"positive_definite_covariance");
        check(!good.inertialTranslation,"acceleration_integration_not_claimed");
        Frame noDepth=b;noDepth.depth=cv::Mat();
        check(!evaluate(a,noDepth,known,cfg,true,Eigen::Vector3d::UnitZ(),known.linear().transpose()*Eigen::Vector3d::UnitZ()).accepted,"imu_only_is_lost");
        Frame nan=b.clone();nan.depth.setTo(std::numeric_limits<float>::quiet_NaN());
        check(!evaluate(a,nan,known,cfg).accepted,"nan_depth_rejected");
        const Frame pa=render(identity,1.0,true),pb=render(known,1.1,true);
        check(!evaluate(pa,pb,known,cfg).accepted,"single_plane_tangent_translation_unobservable");
        Frame late=b;late.stamp=1.6;
        check(!evaluate(a,late,known,cfg).accepted,"fixed_anchor_timeout");
        Frame duplicate=b;duplicate.stamp=1.0;
        check(!evaluate(a,duplicate,known,cfg).accepted,"duplicate_source_stamp_rejected");
        Frame changed=b;changed.fx+=1;
        check(!evaluate(a,changed,known,cfg).accepted,"camera_intrinsics_change_rejected");
        Eigen::Matrix3d badImu=Eigen::AngleAxisd(0.2,Eigen::Vector3d::UnitX()).toRotationMatrix();
        check(!evaluate(a,b,known,cfg,true,Eigen::Vector3d::UnitZ(),badImu.transpose()*Eigen::Vector3d::UnitZ()).accepted,"gravity_tilt_conflict_rejected");
        Pose yawMotion=Pose::Identity();
        yawMotion.linear()=Eigen::AngleAxisd(0.08,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Frame yawFrame=render(yawMotion,1.1);
        Result yawResult=evaluate(a,yawFrame,yawMotion,cfg,true,Eigen::Vector3d::UnitZ(),Eigen::Vector3d::UnitZ());
        check(yawResult.accepted && yawResult.gravityChecked,"gravity_does_not_veto_unobserved_yaw");
        Result withoutImu=evaluate(a,b,known,cfg);
        check(withoutImu.accepted && !withoutImu.gravityChecked && !withoutImu.inertialTranslation,
              "depth_without_imu_is_not_mislabeled_inertial_fusion");
        Frame sparse=b.clone();sparse.depth.colRange(0,130).setTo(0);
        check(!evaluate(a,sparse,known,cfg).accepted,"low_overlap_or_spatial_coverage_rejected");
        Config broken=cfg;broken.minimumOverlap=std::numeric_limits<double>::quiet_NaN();
        check(!evaluate(a,b,known,broken).accepted,"invalid_configuration_rejected");

        // Nonidentity camera/body extrinsic and world anchor exercise composition.
        const Pose extrinsic=motion(0.02,0.005,0.01,0.09);
        const Pose worldAnchorBody=motion(-0.005,0.003,0.001,-0.02);
        const Pose worldCurrentBody=worldAnchorBody*known;
        Frame anchor=render(worldAnchorBody*extrinsic,2.0);anchor.bodyFromCamera=extrinsic;
        Frame current=render(worldCurrentBody*extrinsic,2.1);current.bodyFromCamera=extrinsic;
        FixedVisualAnchor cache;
        check(cache.set(anchor,worldAnchorBody,true,worldAnchorBody.linear()),"visual_anchor_accepted");
        // Mutating caller memory cannot change the cloned trusted reference.
        anchor.depth.setTo(0);
        Pose recovered;Matrix6 covariance;
        Result extrinsicResult=cache.recover(current,worldAnchorBody,true,worldCurrentBody.linear(),cfg,recovered,covariance);
        if(!extrinsicResult.accepted)std::cerr<<"extrinsic rejected: "<<extrinsicResult.reason<<"\n";
        check(extrinsicResult.accepted,"cloned_anchor_supports_recovery");
        check((recovered.translation()-worldCurrentBody.translation()).norm()<0.004 &&
              angle(recovered.linear().transpose()*worldCurrentBody.linear())<0.01,"body_camera_world_composition");
        check(!cache.recover(current,recovered,true,worldCurrentBody.linear(),cfg,recovered,covariance).accepted,"cache_rejects_repeated_frame");
        for(int k=2;k<=4;++k) {
            const Pose world=worldAnchorBody*motion(0.006*k,-0.002*k,0.004*k,0.004*k);
            Frame next=render(world*extrinsic,2.0+k*0.1);next.bodyFromCamera=extrinsic;
            const Result result=cache.recover(next,recovered,true,world.linear(),cfg,recovered,covariance);
            check(result.accepted,"multiple_fallback_frames_against_fixed_anchor");
        }
        check(cache.stamp()==2.0,"depth_success_never_rolls_visual_anchor");
        Frame timeout=current;timeout.stamp=2.51;
        check(!cache.recover(timeout,recovered,true,worldCurrentBody.linear(),cfg,recovered,covariance).accepted,"successes_cannot_extend_recovery_window");
        cache.clear();current.stamp=3.0;
        check(!cache.recover(current,recovered,true,worldCurrentBody.linear(),cfg,recovered,covariance).accepted,"reset_clears_all_recovery_state");
        Frame freshAnchor=render(identity,4.0);
        check(cache.set(freshAnchor,identity,false,Eigen::Matrix3d::Identity()),"fresh_anchor_after_reset");
        Frame rewind=render(known,3.9);
        check(!cache.recover(rewind,identity,false,Eigen::Matrix3d::Identity(),cfg,recovered,covariance).accepted &&
              !cache.valid(),"time_regression_invalidates_anchor");
        check(cache.set(freshAnchor,identity,false,Eigen::Matrix3d::Identity()),"new_visual_can_reestablish_anchor");
        Frame changedCamera=render(known,4.1);changedCamera.cx+=0.1;
        check(!cache.recover(changedCamera,identity,false,Eigen::Matrix3d::Identity(),cfg,recovered,covariance).accepted &&
              !cache.valid(),"camera_contract_change_invalidates_anchor");
        std::cout<<"CHECKS "<<checks<<"\n";return 0;
    }catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<"\n";return 1;}
}
