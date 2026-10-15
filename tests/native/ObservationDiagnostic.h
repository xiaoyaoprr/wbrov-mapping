#pragma once
// Test-only numerical diagnostics. This never changes Core parameters/state.
#include <rtabmap/core/Registration.h>
#include <rtabmap/core/RegistrationInfo.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/core.hpp>
#include <memory>
#include <set>
#include <iostream>
#include <cmath>
#include <limits>

namespace observation_diagnostic {
using namespace rtabmap;
struct Metrics {
    int count=0,invalid=0,above=0;
    double rms=std::numeric_limits<double>::infinity(),maximum=std::numeric_limits<double>::infinity();
};
struct PairMetrics {Metrics forward,reverse;};
inline Metrics metrics(const char * direction,const Signature & a,const Signature & b,
        const Transform & t,const RegistrationInfo & info)
{
    Metrics measured;
    std::cout<<direction<<" inliers="<<info.inliers<<" backend="<<info.rejectedMsg<<" transform="<<t.prettyPrint()<<'\n';
    if(t.isNull())return measured;
    std::set<int> words(info.inliersIDs.begin(),info.inliersIDs.end());
    std::cout<<" words="<<words.size();
    for(const Signature * s:{&a,&b}) {
        std::set<int> cells;std::vector<cv::Point3d> points;
        const int width=s->sensorData().imageRaw().cols,height=s->sensorData().imageRaw().rows;
        for(int id:words) {
            const auto found=s->getWords().find(id);
            if(found==s->getWords().end())continue;
            const int index=found->second;
            if(index>=0 && index<int(s->getWordsKpts().size()) && width>0 && height>0) {
                const auto p=s->getWordsKpts()[index].pt;
                if(std::isfinite(p.x) && std::isfinite(p.y) && p.x>=0 && p.y>=0 && p.x<width && p.y<height)
                    cells.insert(std::min(2,int(3*p.x/width))+3*std::min(2,int(3*p.y/height)));
            }
            if(index>=0 && index<int(s->getWords3().size())) {
                const auto p=s->getWords3()[index];
                if(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))points.emplace_back(p.x,p.y,p.z);
            }
        }
        cv::Point3d mean(0,0,0);for(const auto & q:points)mean+=q;
        if(!points.empty())mean*=1.0/points.size();
        cv::Mat covariance=cv::Mat::zeros(3,3,CV_64F),hessian=cv::Mat::zeros(6,6,CV_64F);
        for(const auto & q:points) {
            const cv::Point3d c=q-mean;cv::Mat v=(cv::Mat_<double>(3,1)<<c.x,c.y,c.z);covariance+=v*v.t();
            cv::Mat j=(cv::Mat_<double>(3,6)<<1,0,0,0,c.z,-c.y,0,1,0,-c.z,0,c.x,0,0,1,c.y,-c.x,0);hessian+=j.t()*j;
        }
        cv::Mat ce,he;cv::eigen(covariance,ce);cv::eigen(hessian,he);
        std::cout<<" side(id="<<s->id()<<",image="<<width<<'x'<<height<<",cells="<<cells.size()<<",points="<<points.size()
            <<",span="<<(points.empty()?0:std::sqrt(ce.at<double>(1)/points.size()))
            <<",hessian="<<(he.at<double>(0)>0?he.at<double>(5)/he.at<double>(0):0)<<')';
    }
    std::cout<<'\n';
    if(info.covariance.rows==6 && info.covariance.cols==6 && info.covariance.type()==CV_64FC1) {
        cv::Mat eigen;cv::eigen(info.covariance,eigen);
        std::cout<<" covariance finite="<<cv::checkRange(info.covariance)<<" symmetry="<<cv::norm(info.covariance-info.covariance.t(),cv::NORM_INF)
            <<" min="<<eigen.at<double>(5)<<" max="<<eigen.at<double>(0)<<" condition="<<eigen.at<double>(0)/eigen.at<double>(5)<<'\n';
    } else std::cout<<" covariance wrong dimensions/type\n";
    double sum=0,maximum=0;int count=0,bad=0,above=0;
    for(int word:words) {
        const auto x=a.getWords().equal_range(word),y=b.getWords().equal_range(word);
        if(std::distance(x.first,x.second)!=1 || std::distance(y.first,y.second)!=1){++bad;continue;}
        const int i=x.first->second,j=y.first->second;
        if(i<0 || j<0 || i>=int(a.getWords3().size()) || j>=int(b.getWords3().size())){++bad;continue;}
        const auto q=a.getWords3()[i],r=b.getWords3()[j];
        const cv::Point3d transformed(t.r11()*r.x+t.r12()*r.y+t.r13()*r.z+t.x(),
            t.r21()*r.x+t.r22()*r.y+t.r23()*r.z+t.y(),t.r31()*r.x+t.r32()*r.y+t.r33()*r.z+t.z());
        const double error=cv::norm(cv::Point3d(q.x,q.y,q.z)-transformed);
        if(!std::isfinite(error)){++bad;continue;}
        sum+=error*error;maximum=std::max(maximum,error);++count;if(error>.025)++above;
    }
    std::cout<<" metric count="<<count<<" invalid="<<bad<<" rms="<<(count?std::sqrt(sum/count):0)<<" max="<<maximum<<" above_025="<<above<<'\n';
    measured.count=count;measured.invalid=bad;measured.above=above;
    if(count){measured.rms=std::sqrt(sum/count);measured.maximum=maximum;}
    return measured;
}
inline PairMetrics pair(SensorData current,SensorData old,ParametersMap parameters,bool tight)
{
    current.uncompressData();old.uncompressData();
    current.setFeatures({}, {},cv::Mat());old.setFeatures({}, {},cv::Mat());
    int configured=Parameters::defaultVisMinInliers(),minimum=25;
    Parameters::parse(parameters,Parameters::kVisMinInliers(),configured);
    Parameters::parse(parameters,Parameters::kRtabmapCrossMapLoopMinInliers(),minimum);
    parameters[Parameters::kRegStrategy()]="0";parameters[Parameters::kRegForce3DoF()]="false";
    parameters[Parameters::kVisEstimationType()]="0";parameters[Parameters::kVisMinInliers()]=std::to_string(std::max(25,std::max(configured,minimum)));
    float threshold=Parameters::defaultVisInlierDistance();Parameters::parse(parameters,Parameters::kVisInlierDistance(),threshold);
    if(tight){threshold=std::min(.025f,threshold);parameters[Parameters::kVisInlierDistance()]=std::to_string(threshold);}
    std::cout<<"PAIR current="<<current.id()<<" old="<<old.id()<<" ransac_threshold="<<threshold<<" tight="<<tight<<'\n';
    Signature a(current),b(old),reverseB(old),reverseA(current);
    std::unique_ptr<Registration> f(Registration::create(parameters)),r(Registration::create(parameters));
    RegistrationInfo fi,ri;const Transform ft=f->computeTransformationMod(a,b,Transform(),&fi),rt=r->computeTransformationMod(reverseB,reverseA,Transform(),&ri);
    PairMetrics measured;measured.forward=metrics("forward",a,b,ft,fi);measured.reverse=metrics("reverse",reverseB,reverseA,rt,ri);
    if(!ft.isNull() && !rt.isNull())std::cout<<" direction_distance="<<ft.getDistance(rt.inverse())<<" direction_angle_deg="<<ft.getAngle(rt.inverse())*180.0/3.141592653589793<<'\n';
    return measured;
}
}
