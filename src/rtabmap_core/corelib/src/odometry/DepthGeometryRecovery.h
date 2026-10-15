#ifndef WBROV_DEPTH_GEOMETRY_RECOVERY_H
#define WBROV_DEPTH_GEOMETRY_RECOVERY_H

// Local, bounded, depth-only fallback. No acceleration integration and no ROS
// publisher. T_target_source maps source-camera points into target-camera space.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <opencv2/core.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace wbrov_depth {
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Pose = Eigen::Isometry3d;

struct Config {
    double minimumDepth = 0.20, maximumDepth = 1.0;
    double maximumAge = 0.5, maximumTranslation = 0.12;
    double maximumRotation = 12.0 * M_PI / 180.0;
    double maximumGravityTiltError = 3.0 * M_PI / 180.0;
    double maximumPointDistance = 0.05, maximumDepthEdge = 0.06;
    double maximumRms = 0.01, minimumOverlap = 0.55;
    double minimumHessianRatio = 1e-4, minimumGridCoverage = 0.5;
    double maximumRoundTripTranslation = 0.01;
    double maximumRoundTripRotation = M_PI / 180.0;
    int minimumCorrespondences = 80, iterationsPerLevel = 8;
    bool valid() const {
        const double values[] = {minimumDepth, maximumDepth, maximumAge,
            maximumTranslation, maximumRotation, maximumGravityTiltError,
            maximumPointDistance, maximumDepthEdge, maximumRms, minimumOverlap,
            minimumHessianRatio, minimumGridCoverage, maximumRoundTripTranslation,
            maximumRoundTripRotation};
        for(double v : values) if(!std::isfinite(v) || v <= 0) return false;
        return minimumDepth < maximumDepth && maximumAge <= 2.0 &&
            maximumTranslation <= 0.5 && maximumRotation <= M_PI/4 &&
            minimumOverlap <= 1 && minimumHessianRatio <= 1 &&
            minimumGridCoverage <= 1 && minimumCorrespondences >= 30 &&
            iterationsPerLevel >= 1 && iterationsPerLevel <= 30;
    }
};

struct Frame {
    cv::Mat depth;
    double fx = 0, fy = 0, cx = 0, cy = 0, stamp = 0;
    Pose bodyFromCamera = Pose::Identity();
    bool valid() const {
        return !depth.empty() && depth.rows >= 24 && depth.cols >= 32 &&
            (depth.type() == CV_32FC1 || depth.type() == CV_16UC1) &&
            std::isfinite(stamp) && stamp > 0 && std::isfinite(fx) && fx > 0 &&
            std::isfinite(fy) && fy > 0 && std::isfinite(cx) && std::isfinite(cy) &&
            cx >= 0 && cy >= 0 && cx < depth.cols && cy < depth.rows &&
            bodyFromCamera.matrix().allFinite() &&
            (bodyFromCamera.linear().transpose()*bodyFromCamera.linear()-
             Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
            std::abs(bodyFromCamera.linear().determinant()-1.0) < 1e-6;
    }
    Frame clone() const { Frame f(*this); f.depth = depth.clone(); return f; }
};

struct Result {
    bool accepted = false, inertialTranslation = false, gravityChecked = false;
    std::string reason = "not_evaluated";
    Pose targetFromSource = Pose::Identity();
    Matrix6 covariance = Matrix6::Identity()*9999.0;
    double overlap = 0, rms = std::numeric_limits<double>::infinity();
    double hessianRatio = 0, gridCoverage = 0;
    double roundTripTranslation = 0, roundTripRotation = 0;
    int correspondences = 0;
};

inline Eigen::Matrix3d skew(const Eigen::Vector3d &p) {
    Eigen::Matrix3d s;
    s << 0,-p.z(),p.y(), p.z(),0,-p.x(), -p.y(),p.x(),0;
    return s;
}
inline double angle(const Eigen::Matrix3d &r) {
    return std::acos(std::max(-1.0, std::min(1.0, (r.trace()-1.0)*0.5)));
}
inline double directionAngle(const Eigen::Vector3d &a,const Eigen::Vector3d &b) {
    return std::acos(std::max(-1.0,std::min(1.0,a.normalized().dot(b.normalized()))));
}
inline Matrix6 adjoint(const Pose &t) {
    Matrix6 a = Matrix6::Zero();
    a.block<3,3>(0,0)=t.linear(); a.block<3,3>(0,3)=skew(t.translation())*t.linear();
    a.block<3,3>(3,3)=t.linear(); return a;
}
inline Pose exponential(const Vector6 &dx) {
    Pose p = Pose::Identity();
    const Eigen::Vector3d w = dx.tail<3>();
    const double theta=w.norm();
    Eigen::Matrix3d v = Eigen::Matrix3d::Identity();
    if(theta>1e-10) {
        const Eigen::Matrix3d wx=skew(w);
        p.linear()=Eigen::AngleAxisd(theta,w/theta).toRotationMatrix();
        v += (1-std::cos(theta))/(theta*theta)*wx +
             (theta-std::sin(theta))/(theta*theta*theta)*wx*wx;
    } else { p.linear() += skew(w); v += 0.5*skew(w); }
    p.translation()=v*dx.head<3>(); return p;
}

struct Geometry {
    int width = 0, height = 0;
    std::vector<Eigen::Vector3d> point, normal;
    std::vector<unsigned char> valid;
};
inline double depthAt(const cv::Mat &d, int y, int x) {
    return d.type()==CV_32FC1 ? double(d.at<float>(y,x)) :
        double(d.at<unsigned short>(y,x))*0.001;
}
inline Geometry makeGeometry(const Frame &f, const Config &cfg) {
    Geometry g; g.width=f.depth.cols; g.height=f.depth.rows;
    const int count=g.width*g.height;
    g.point.assign(count,Eigen::Vector3d::Zero());
    g.normal.assign(count,Eigen::Vector3d::Zero()); g.valid.assign(count,0);
    for(int y=0;y<g.height;++y) for(int x=0;x<g.width;++x) {
        const double z=depthAt(f.depth,y,x); const int i=y*g.width+x;
        if(std::isfinite(z) && z>=cfg.minimumDepth && z<=cfg.maximumDepth) {
            g.point[i]=Eigen::Vector3d((x-f.cx)*z/f.fx,(y-f.cy)*z/f.fy,z);
            g.valid[i]=1;
        }
    }
    for(int y=1;y<g.height-1;++y) for(int x=1;x<g.width-1;++x) {
        const int i=y*g.width+x;
        if(!g.valid[i] || !g.valid[i-1] || !g.valid[i+1] ||
           !g.valid[i-g.width] || !g.valid[i+g.width]) continue;
        const double z=g.point[i].z();
        if(std::abs(g.point[i-1].z()-z)>cfg.maximumDepthEdge ||
           std::abs(g.point[i+1].z()-z)>cfg.maximumDepthEdge ||
           std::abs(g.point[i-g.width].z()-z)>cfg.maximumDepthEdge ||
           std::abs(g.point[i+g.width].z()-z)>cfg.maximumDepthEdge) continue;
        Eigen::Vector3d n=(g.point[i+1]-g.point[i-1]).cross(
            g.point[i+g.width]-g.point[i-g.width]);
        if(n.norm()>1e-10) g.normal[i]=n.normalized();
    }
    return g;
}
struct Equations {
    Matrix6 h = Matrix6::Zero(); Vector6 b = Vector6::Zero();
    double squaredError = 0, radiusSquared = 0;
    int count = 0, possible = 0; std::array<bool,12> cells{};
};

inline Equations equations(const Geometry &target, const Frame &targetFrame,
        const Geometry &source, const Pose &t, int stride, const Config &cfg) {
    Equations e;
    const double maxD2=cfg.maximumPointDistance*cfg.maximumPointDistance;
    for(int y=2;y<source.height-2;y+=stride) for(int x=2;x<source.width-2;x+=stride) {
        const int i=y*source.width+x;
        if(!source.valid[i] || source.normal[i].squaredNorm()<0.5) continue;
        ++e.possible;
        const Eigen::Vector3d p=t*source.point[i];
        if(!p.allFinite() || p.z()<=0) continue;
        const int px=int(std::lround(targetFrame.fx*p.x()/p.z()+targetFrame.cx));
        const int py=int(std::lround(targetFrame.fy*p.y()/p.z()+targetFrame.cy));
        if(px<2 || py<2 || px>=target.width-2 || py>=target.height-2) continue;
        // Local projective association; no assumption of constant pixel identity.
        int best=-1; double bestD=maxD2;
        for(int yy=py-1;yy<=py+1;++yy) for(int xx=px-1;xx<=px+1;++xx) {
            const int j=yy*target.width+xx;
            if(!target.valid[j] || target.normal[j].squaredNorm()<0.5) continue;
            const double d=(p-target.point[j]).squaredNorm();
            if(d<bestD) {bestD=d;best=j;}
        }
        if(best<0) continue;
        const Eigen::Vector3d &n=target.normal[best];
        if(std::abs(n.dot(t.linear()*source.normal[i]))<0.70710678) continue;
        const double residual=n.dot(p-target.point[best]);
        const double weight=std::abs(residual)<=cfg.maximumRms ? 1.0 :
            cfg.maximumRms/std::abs(residual);
        Vector6 j; j.head<3>()=n; j.tail<3>()=p.cross(n);
        e.h.noalias() += weight*j*j.transpose(); e.b.noalias() += weight*j*residual;
        e.squaredError += residual*residual; e.radiusSquared += p.squaredNorm();
        ++e.count;
        e.cells[std::min(2,y*3/source.height)*4+std::min(3,x*4/source.width)]=true;
    }
    return e;
}
inline double hessianRatio(const Equations &e) {
    if(e.count==0) return 0;
    const double radius=std::sqrt(e.radiusSquared/e.count);
    if(!std::isfinite(radius) || radius<1e-6) return 0;
    Matrix6 scale=Matrix6::Identity(); scale.bottomRightCorner<3,3>()/=radius;
    Eigen::SelfAdjointEigenSolver<Matrix6> eig(scale*e.h*scale);
    if(eig.info()!=Eigen::Success || !eig.eigenvalues().allFinite() ||
       eig.eigenvalues()[5]<=0) return 0;
    return std::max(0.0,eig.eigenvalues()[0])/eig.eigenvalues()[5];
}
inline bool optimize(const Geometry &target, const Frame &targetFrame,
        const Geometry &source, const Pose &initial, const Config &cfg,
        Pose &pose, Equations &final, std::string &reason) {
    pose=initial;
    const int strides[]={8,4,2};
    for(int stride:strides) for(int iter=0;iter<cfg.iterationsPerLevel;++iter) {
        const Equations e=equations(target,targetFrame,source,pose,stride,cfg);
        if(e.count<std::max(30,cfg.minimumCorrespondences/4)) {
            reason="insufficient_correspondences"; return false;
        }
        if(hessianRatio(e)<cfg.minimumHessianRatio) {
            reason="unobservable_geometry"; return false;
        }
        Eigen::LDLT<Matrix6> ldlt(e.h);
        if(ldlt.info()!=Eigen::Success || !ldlt.isPositive()) {
            reason="invalid_information_matrix"; return false;
        }
        const Vector6 dx=-ldlt.solve(e.b);
        if(!dx.allFinite() || dx.head<3>().norm()>cfg.maximumPointDistance ||
           dx.tail<3>().norm()>cfg.maximumRotation) {
            reason="unstable_optimizer_step"; return false;
        }
        pose=exponential(dx)*pose;
        if(dx.head<3>().norm()<1e-5 && dx.tail<3>().norm()<1e-5) break;
    }
    final=equations(target,targetFrame,source,pose,2,cfg);
    const double overlap=final.possible?double(final.count)/final.possible:0;
    const double rms=final.count?std::sqrt(final.squaredError/final.count):1e9;
    const double coverage=double(std::count(final.cells.begin(),final.cells.end(),true))/12;
    if(final.count<cfg.minimumCorrespondences) reason="insufficient_correspondences";
    else if(overlap<cfg.minimumOverlap) reason="insufficient_overlap";
    else if(coverage<cfg.minimumGridCoverage) reason="insufficient_spatial_coverage";
    else if(rms>cfg.maximumRms) reason="depth_residual_too_large";
    else if(hessianRatio(final)<cfg.minimumHessianRatio) reason="unobservable_geometry";
    else return true;
    return false;
}

inline Result evaluate(const Frame &target, const Frame &source,
        const Pose &initial, const Config &cfg,
        bool hasGravity=false,
        const Eigen::Vector3d &targetUpCamera=Eigen::Vector3d::UnitZ(),
        const Eigen::Vector3d &sourceUpCamera=Eigen::Vector3d::UnitZ()) {
    Result out;
    if(!cfg.valid()) {out.reason="invalid_configuration";return out;}
    if(!target.valid() || !source.valid()) {out.reason="invalid_depth_frame";return out;}
    if(!initial.matrix().allFinite() ||
       (initial.linear().transpose()*initial.linear()-Eigen::Matrix3d::Identity()).norm()>1e-6 ||
       std::abs(initial.linear().determinant()-1.0)>1e-6) {
        out.reason="invalid_initial_transform";return out;
    }
    if(hasGravity && (!targetUpCamera.allFinite() || !sourceUpCamera.allFinite() ||
       std::abs(targetUpCamera.norm()-1.0)>1e-6 || std::abs(sourceUpCamera.norm()-1.0)>1e-6)) {
        out.reason="invalid_gravity_direction";return out;
    }
    if(source.stamp<=target.stamp || source.stamp-target.stamp>cfg.maximumAge) {
        out.reason="anchor_expired_or_nonincreasing_stamp";return out;
    }
    if((target.bodyFromCamera.matrix()-source.bodyFromCamera.matrix()).norm()>1e-8 ||
       target.depth.size()!=source.depth.size() || std::abs(target.fx-source.fx)>1e-8 ||
       std::abs(target.fy-source.fy)>1e-8 || std::abs(target.cx-source.cx)>1e-8 ||
       std::abs(target.cy-source.cy)>1e-8) {out.reason="camera_contract_changed";return out;}
    const Geometry a=makeGeometry(target,cfg), b=makeGeometry(source,cfg);
    Pose forward,reverse; Equations ef,er;
    if(!optimize(a,target,b,initial,cfg,forward,ef,out.reason)) return out;
    // Independently optimize reverse from the original guess, not the result.
    if(!optimize(b,source,a,initial.inverse(),cfg,reverse,er,out.reason)) return out;
    const Pose round=forward*reverse;
    out.roundTripTranslation=round.translation().norm();
    out.roundTripRotation=angle(round.linear());
    if(out.roundTripTranslation>cfg.maximumRoundTripTranslation ||
       out.roundTripRotation>cfg.maximumRoundTripRotation) {
        out.reason="inconsistent_bidirectional_registration";return out;
    }
    const Pose bodyMotion=target.bodyFromCamera*forward*source.bodyFromCamera.inverse();
    if(bodyMotion.translation().norm()>cfg.maximumTranslation || angle(bodyMotion.linear())>cfg.maximumRotation) {
        out.reason="outside_local_recovery_envelope";return out;
    }
    // Accel-gravity orientation has no independently observed yaw. Compare the
    // two body-up directions only; never turn fixed IMU yaw into a yaw veto.
    if(hasGravity && directionAngle(targetUpCamera,forward.linear()*sourceUpCamera)>cfg.maximumGravityTiltError) {
        out.reason="gravity_tilt_conflict";return out;
    }
    out.correspondences=std::min(ef.count,er.count);
    out.overlap=std::min(double(ef.count)/ef.possible,double(er.count)/er.possible);
    out.rms=std::max(std::sqrt(ef.squaredError/ef.count),std::sqrt(er.squaredError/er.count));
    out.hessianRatio=std::min(hessianRatio(ef),hessianRatio(er));
    out.gridCoverage=std::min(double(std::count(ef.cells.begin(),ef.cells.end(),true))/12,
                            double(std::count(er.cells.begin(),er.cells.end(),true))/12);
    Eigen::LDLT<Matrix6> ldlt(ef.h);
    const double variance=std::max(4e-6,ef.squaredError/std::max(1,ef.count-6));
    out.covariance=variance*ldlt.solve(Matrix6::Identity());
    out.covariance.topLeftCorner<3,3>().diagonal().array()+=4e-6;
    out.covariance.bottomRightCorner<3,3>().diagonal().array()+=2.5e-5;
    out.covariance=(out.covariance+out.covariance.transpose()).eval()*0.5;
    Eigen::LLT<Matrix6> llt(out.covariance);
    if(!out.covariance.allFinite() || llt.info()!=Eigen::Success) {
        out.reason="invalid_covariance";return out;
    }
    out.targetFromSource=forward; out.accepted=true; out.reason="depth_geometry";
    out.gravityChecked=hasGravity;
    return out;
}

// Owns one cloned visual anchor. Failed/recovered depth frames never move it.
class FixedVisualAnchor {
public:
    void clear() {valid_=false;anchor_=Frame();lastStamp_=0;hasImu_=false;}
    bool set(const Frame &f,const Pose &worldBody,bool hasImu,const Eigen::Matrix3d &imu) {
        if(std::isfinite(f.stamp) && f.stamp<lastStamp_) {clear();return false;}
        if(!f.valid() || !worldBody.matrix().allFinite() || f.stamp<=lastStamp_) return false;
        anchor_=f.clone();worldBody_=worldBody;hasImu_=hasImu;imu_=imu;
        lastStamp_=f.stamp;valid_=true;return true;
    }
    Result recover(const Frame &f,const Pose &worldBodyGuess,bool hasImu,
            const Eigen::Matrix3d &imu,const Config &cfg,Pose &worldBody,Matrix6 &worldAnchorCameraCovariance) {
        Result out;
        if(!valid_) {out.reason="no_visual_anchor";return out;}
        if(std::isfinite(f.stamp) && f.stamp<lastStamp_) {
            clear();out.reason="time_regression_cleared_anchor";return out;
        }
        if(!f.valid() || f.stamp<=lastStamp_) {out.reason="invalid_or_repeated_frame";return out;}
        if((anchor_.bodyFromCamera.matrix()-f.bodyFromCamera.matrix()).norm()>1e-8 ||
           anchor_.depth.size()!=f.depth.size() || std::abs(anchor_.fx-f.fx)>1e-8 ||
           std::abs(anchor_.fy-f.fy)>1e-8 || std::abs(anchor_.cx-f.cx)>1e-8 ||
           std::abs(anchor_.cy-f.cy)>1e-8) {
            clear();out.reason="camera_contract_change_cleared_anchor";return out;
        }
        lastStamp_=f.stamp;
        Pose guess=worldBodyGuess;
        const bool useImu=hasImu_&&hasImu;
        const Eigen::Vector3d anchorUpBody=imu_.transpose()*Eigen::Vector3d::UnitZ();
        const Eigen::Vector3d currentUpBody=imu.transpose()*Eigen::Vector3d::UnitZ();
        if(useImu) {
            if(!anchorUpBody.allFinite() || !currentUpBody.allFinite() ||
               std::abs(anchorUpBody.norm()-1.0)>1e-6 || std::abs(currentUpBody.norm()-1.0)>1e-6) {
                out.reason="invalid_gravity_direction";return out;
            }
            const Eigen::Matrix3d predictedRelative=worldBody_.linear().transpose()*guess.linear();
            // Smallest tilt correction to the existing odometry guess. The yaw
            // degree of freedom stays with the geometry/odometry, not the IMU.
            const Eigen::Matrix3d tiltCorrection=Eigen::Quaterniond::FromTwoVectors(
                predictedRelative*currentUpBody,anchorUpBody).toRotationMatrix();
            guess.linear()=worldBody_.linear()*tiltCorrection*predictedRelative;
        }
        const Pose anchorWorldCamera=worldBody_*anchor_.bodyFromCamera;
        const Pose initial=anchorWorldCamera.inverse()*guess*f.bodyFromCamera;
        const Eigen::Vector3d anchorUpCamera=anchor_.bodyFromCamera.linear().transpose()*anchorUpBody;
        const Eigen::Vector3d currentUpCamera=f.bodyFromCamera.linear().transpose()*currentUpBody;
        out=evaluate(anchor_,f,initial,cfg,useImu,anchorUpCamera,currentUpCamera);
        if(out.accepted) {
            worldBody=anchorWorldCamera*out.targetFromSource*f.bodyFromCamera.inverse();
            const Matrix6 a=adjoint(anchorWorldCamera);
            worldAnchorCameraCovariance=a*out.covariance*a.transpose();
        }
        return out;
    }
    bool valid() const {return valid_;}
    double stamp() const {return valid_?anchor_.stamp:0;}
private:
    Frame anchor_; Pose worldBody_=Pose::Identity();
    Eigen::Matrix3d imu_=Eigen::Matrix3d::Identity();
    double lastStamp_=0;bool valid_=false,hasImu_=false;
};
} // namespace wbrov_depth
#endif
