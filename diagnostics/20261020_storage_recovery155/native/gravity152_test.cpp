#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Registration.h>
#include <rtabmap/core/RegistrationInfo.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core.hpp>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <set>
#include <stdexcept>
using namespace rtabmap;
bool finitePoint(const cv::Point3f & point)
{
	return std::isfinite(point.x) &&
			std::isfinite(point.y) &&
			std::isfinite(point.z) &&
			(point.x != 0.0f || point.y != 0.0f || point.z != 0.0f);
}

bool finiteTransform(const Transform & transform)
{
	if(transform.isNull())
	{
		return false;
	}
	for(int i=0; i<transform.size(); ++i)
	{
		if(!std::isfinite(transform[i]))
		{
			return false;
		}
	}
	return true;
}

bool signatureImageSize(const Signature & signature, int & width, int & height)
{
	width = signature.sensorData().imageRaw().cols;
	height = signature.sensorData().imageRaw().rows;
	if(width > 0 && height > 0)
	{
		return true;
	}

	width = 0;
	height = 0;
	const std::vector<CameraModel> & models = signature.sensorData().cameraModels();
	for(std::vector<CameraModel>::const_iterator iter=models.begin(); iter!=models.end(); ++iter)
	{
		if(iter->imageWidth() <= 0 || iter->imageHeight() <= 0)
		{
			return false;
		}
		width += iter->imageWidth();
		height = std::max(height, iter->imageHeight());
	}
	return width > 0 && height > 0;
}

int occupiedGridCells(
		const Signature & signature,
		const std::set<int> & wordIds,
		int width,
		int height)
{
	std::set<int> cells;
	const std::multimap<int, int> & words = signature.getWords();
	const std::vector<cv::KeyPoint> & keypoints = signature.getWordsKpts();
	for(std::set<int>::const_iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
	{
		std::multimap<int, int>::const_iterator wordIter = words.lower_bound(*iter);
		if(wordIter == words.end() || wordIter->first != *iter ||
				wordIter->second < 0 || wordIter->second >= (int)keypoints.size())
		{
			continue;
		}
		const cv::Point2f & point = keypoints[wordIter->second].pt;
		if(!std::isfinite(point.x) || !std::isfinite(point.y) ||
				point.x < 0.0f || point.y < 0.0f ||
				point.x >= width || point.y >= height)
		{
			continue;
		}
		int x = std::min(2, (int)(3.0f * point.x / width));
		int y = std::min(2, (int)(3.0f * point.y / height));
		cells.insert(y*3+x);
	}
	return (int)cells.size();
}

std::vector<cv::Point3f> validInlierPoints(
		const Signature & signature,
		const std::set<int> & wordIds)
{
	std::vector<cv::Point3f> output;
	const std::multimap<int, int> & words = signature.getWords();
	const std::vector<cv::Point3f> & points = signature.getWords3();
	output.reserve(wordIds.size());
	for(std::set<int>::const_iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
	{
		std::multimap<int, int>::const_iterator wordIter = words.lower_bound(*iter);
		if(wordIter != words.end() && wordIter->first == *iter &&
				wordIter->second >= 0 && wordIter->second < (int)points.size() &&
				finitePoint(points[wordIter->second]))
		{
			output.push_back(points[wordIter->second]);
		}
	}
	return output;
}

bool pointSetGeometry(
		const std::vector<cv::Point3f> & points,
		double & secondAxisStdDev,
		double & hessianRatio)
{
	if(points.size() < 6)
	{
		return false;
	}

	cv::Vec3d mean(0.0, 0.0, 0.0);
	for(std::vector<cv::Point3f>::const_iterator iter=points.begin(); iter!=points.end(); ++iter)
	{
		mean += cv::Vec3d(iter->x, iter->y, iter->z);
	}
	mean *= 1.0/(double)points.size();

	cv::Mat covariance = cv::Mat::zeros(3, 3, CV_64FC1);
	cv::Mat hessian = cv::Mat::zeros(6, 6, CV_64FC1);
	for(std::vector<cv::Point3f>::const_iterator iter=points.begin(); iter!=points.end(); ++iter)
	{
		cv::Vec3d point(iter->x-mean[0], iter->y-mean[1], iter->z-mean[2]);
		for(int row=0; row<3; ++row)
		{
			for(int col=0; col<3; ++col)
			{
				covariance.at<double>(row,col) += point[row]*point[col];
			}
		}

		cv::Mat jacobian = (cv::Mat_<double>(3,6) <<
				1.0, 0.0, 0.0, 0.0, point[2], -point[1],
				0.0, 1.0, 0.0, -point[2], 0.0, point[0],
				0.0, 0.0, 1.0, point[1], -point[0], 0.0);
		hessian += jacobian.t()*jacobian;
	}
	covariance *= 1.0/(double)points.size();

	cv::Mat covarianceEigenvalues;
	cv::Mat hessianEigenvalues;
	if(!cv::eigen(covariance, covarianceEigenvalues) ||
			!cv::eigen(hessian, hessianEigenvalues) ||
			covarianceEigenvalues.total() < 3 || hessianEigenvalues.total() < 6)
	{
		return false;
	}

	double secondVariance = covarianceEigenvalues.at<double>(1);
	double largestHessian = hessianEigenvalues.at<double>(0);
	double smallestHessian = hessianEigenvalues.at<double>(5);
	if(!std::isfinite(secondVariance) || !std::isfinite(largestHessian) ||
			!std::isfinite(smallestHessian) || secondVariance < 0.0 || largestHessian <= 0.0)
	{
		return false;
	}
	secondAxisStdDev = std::sqrt(secondVariance);
	hessianRatio = std::max(0.0, smallestHessian)/largestHessian;
	return std::isfinite(secondAxisStdDev) && std::isfinite(hessianRatio);
}

bool covarianceVariances(
		const cv::Mat & covariance,
		double & linearVariance,
		double & angularVariance)
{
	if(covariance.rows != 6 || covariance.cols != 6 || covariance.type() != CV_64FC1)
	{
		return false;
	}
	linearVariance = 0.0;
	angularVariance = 0.0;
	for(int i=0; i<6; ++i)
	{
		double value = covariance.at<double>(i,i);
		if(!std::isfinite(value) || value <= 0.0 || value >= 9999.0)
		{
			return false;
		}
		if(i < 3)
		{
			linearVariance = std::max(linearVariance, value);
		}
		else
		{
			angularVariance = std::max(angularVariance, value);
		}
	}
	return linearVariance > 0.0 && angularVariance > 0.0;
}

bool safeRigidTransform(const Transform & transform)
{
	if(!finiteTransform(transform))
	{
		return false;
	}
	const double r11=transform.r11(), r12=transform.r12(), r13=transform.r13();
	const double r21=transform.r21(), r22=transform.r22(), r23=transform.r23();
	const double r31=transform.r31(), r32=transform.r32(), r33=transform.r33();
	const double n1=r11*r11+r21*r21+r31*r31;
	const double n2=r12*r12+r22*r22+r32*r32;
	const double n3=r13*r13+r23*r23+r33*r33;
	const double d12=r11*r12+r21*r22+r31*r32;
	const double d13=r11*r13+r21*r23+r31*r33;
	const double d23=r12*r13+r22*r23+r32*r33;
	const double determinant =
			r11*(r22*r33-r23*r32)-r12*(r21*r33-r23*r31)+r13*(r21*r32-r22*r31);
	return std::fabs(n1-1.0) <= 1e-3 && std::fabs(n2-1.0) <= 1e-3 &&
			std::fabs(n3-1.0) <= 1e-3 && std::fabs(d12) <= 1e-3 &&
			std::fabs(d13) <= 1e-3 && std::fabs(d23) <= 1e-3 &&
			std::fabs(determinant-1.0) <= 1e-3;
}

cv::Point3d transformPoint(const Transform & transform, const cv::Point3d & point)
{
	return cv::Point3d(
			transform.r11()*point.x + transform.r12()*point.y + transform.r13()*point.z + transform.x(),
			transform.r21()*point.x + transform.r22()*point.y + transform.r23()*point.z + transform.y(),
			transform.r31()*point.x + transform.r32()*point.y + transform.r33()*point.z + transform.z());
}
struct RecoveryPairResult {Transform forward,reverse;};
// Metric RANSAC labels precede reprojection BA. Reclassify only that
// original support under the FINAL transform, once, for every raw recovery
// path. Original-support RMS is also required, so trimming cannot conceal
// broadly inconsistent depth. This does not add matches or refine the pose.
bool finalRecoveryMetricSupport(const Signature & from,const Signature & to,
        const Transform & transform,const RegistrationInfo & info,int minimum,
        double metricCap,std::set<int> & words,std::string & diagnostic)
{
    words.clear();
    if(!std::isfinite(metricCap) || metricCap<=0.0 || metricCap>.025 || minimum<25)
    { diagnostic="invalid_metric_support_limits"; return false; }
    const std::set<int> proposed(info.inliersIDs.begin(),info.inliersIDs.end());
    if(int(proposed.size())<minimum)
    { diagnostic="insufficient_original_unique_support"; return false; }
    double originalSquared=0.0,squared=0.0,maximum=0.0;
    for(int word:proposed) {
        const auto a=from.getWords().equal_range(word),b=to.getWords().equal_range(word);
        if(std::distance(a.first,a.second)!=1 || std::distance(b.first,b.second)!=1)
        { diagnostic="ambiguous_word_correspondence"; return false; }
        const int ai=a.first->second,bi=b.first->second;
        if(ai<0 || bi<0 || ai>=int(from.getWords3().size()) || bi>=int(to.getWords3().size()))
        { diagnostic="invalid_3d_word_index"; return false; }
        const auto & p=from.getWords3()[ai];const auto & q=to.getWords3()[bi];
        if(!finitePoint(p) || !finitePoint(q))
        { diagnostic="nonfinite_3d_correspondence"; return false; }
        const double residual=cv::norm(cv::Point3d(p.x,p.y,p.z)-
            transformPoint(transform,cv::Point3d(q.x,q.y,q.z)));
        if(!std::isfinite(residual))
        { diagnostic="nonfinite_metric_residual"; return false; }
        originalSquared+=residual*residual;
        if(residual<=metricCap) {
            words.insert(word);squared+=residual*residual;maximum=std::max(maximum,residual);
        }
    }
    const double originalRms=std::sqrt(originalSquared/proposed.size());
    const double rms=words.empty()?std::numeric_limits<double>::infinity():std::sqrt(squared/words.size());
    diagnostic=uFormat("final_metric_count=%u/%d original_count=%u original_rms=%.6f/0.018000 rms=%.6f/0.018000 max=%.6f/%.6f",
        unsigned(words.size()),minimum,unsigned(proposed.size()),originalRms,rms,maximum,metricCap);
    return int(words.size())>=minimum && originalRms<=.018 && rms<=.018 && maximum<=metricCap;
}

bool strictRecoveryPair(SensorData fromData,SensorData toData,const ParametersMap & parameters,
        double maximumCondition,RecoveryPairResult & result,std::string & reason)
{
    fromData.uncompressData();toData.uncompressData();
    if(fromData.imageRaw().empty() || toData.imageRaw().empty() || fromData.depthRaw().empty() ||
       toData.depthRaw().empty() || fromData.cameraModels().size()!=1 || toData.cameraModels().size()!=1)
    {reason="raw bidirectional RGB-D observations unavailable";return false;}
    fromData.setFeatures({}, {},cv::Mat());toData.setFeatures({}, {},cv::Mat());
    ParametersMap p=parameters;
    int minimum=25,cells=4,configured=Parameters::defaultVisMinInliers();
    float span=0.02f,hessian=0.0001f;
    Parameters::parse(parameters,Parameters::kRtabmapCrossMapLoopMinInliers(),minimum);
    Parameters::parse(parameters,Parameters::kRtabmapCrossMapLoopMinGridCells(),cells);
    Parameters::parse(parameters,Parameters::kRtabmapCrossMapLoopMin3DSpan(),span);
    Parameters::parse(parameters,Parameters::kRtabmapCrossMapLoopMinHessianRatio(),hessian);
    Parameters::parse(parameters,Parameters::kVisMinInliers(),configured);
    minimum=std::max(25,std::max(minimum,configured));cells=std::max(4,cells);
    span=std::max(.02f,span);hessian=std::max(.0001f,hessian);
    float correspondenceDistance=Parameters::defaultVisInlierDistance();
    Parameters::parse(parameters,Parameters::kVisInlierDistance(),correspondenceDistance);
    if(!std::isfinite(correspondenceDistance) || correspondenceDistance<=0)
    {reason="raw RGB-D correspondence distance is invalid";return false;}
    p[Parameters::kRegStrategy()]="0";p[Parameters::kRegForce3DoF()]="false";
    p[Parameters::kVisEstimationType()]="0";p[Parameters::kVisMinInliers()]=uNumber2Str(minimum);
    // RANSAC must classify correspondences using the same strict metric cap
    // as the posterior certificate. Its default 10cm consensus includes a few
    // compression-sensitive mismatches among hundreds of valid 2.5cm matches.
    // Tighten selection; retain every posterior residual/coverage/SPD check.
    p[Parameters::kVisInlierDistance()]=uNumber2Str(std::min(.025f,correspondenceDistance));
    // These GFTT descriptor paths were checked against raw first-pass features.
    // Prepare each image once for the pair; matching, RANSAC, refinement and
    // all posterior checks still run independently in both directions. Keep
    // other detector/matcher/optical-flow configurations on their existing path.
    int feature=Parameters::defaultVisFeatureType(),correspondences=Parameters::defaultVisCorType();
    int matcher=Parameters::defaultVisCorNNType();bool spatial=Parameters::defaultVisSSC();
    Parameters::parse(p,Parameters::kVisFeatureType(),feature);
    Parameters::parse(p,Parameters::kVisCorType(),correspondences);
    Parameters::parse(p,Parameters::kVisCorNNType(),matcher);
    Parameters::parse(p,Parameters::kVisSSC(),spatial);
    if(!spatial && correspondences==0 && (feature==6 || feature==8) && (matcher==1 || matcher==3)) {
        std::unique_ptr<Registration> preparation(Registration::create(p));
        if(!preparation || !preparation->isImageRequired())
        {reason="raw visual feature preparation unavailable";return false;}
        Signature preparedFrom(fromData),preparedTo(toData),emptyFrom,emptyTo;
        if(!preparation->computeTransformationMod(emptyFrom,preparedFrom).isNull() ||
           !preparation->computeTransformationMod(emptyTo,preparedTo).isNull())
        {reason="unpaired raw feature preparation unexpectedly produced motion";return false;}
        const auto &a=preparedFrom.sensorData();const auto &b=preparedTo.sensorData();
        fromData.setFeatures(a.keypoints(),a.keypoints3D(),a.descriptors().clone());
        toData.setFeatures(b.keypoints(),b.keypoints3D(),b.descriptors().clone());
    }
    auto validate=[&](const Signature & a,const Signature & b,const Transform & transform,const RegistrationInfo & info)->bool {
        if(!safeRigidTransform(transform) || info.inliers<minimum) return false;
        std::set<int> words;std::string diagnostic;
        if(!finalRecoveryMetricSupport(a,b,transform,info,minimum,
            std::min(.025,double(correspondenceDistance)),words,diagnostic))return false;
        for(const Signature * s:{&a,&b}) {
            int width=0,height=0;double extent=0.0,conditioning=0.0;
            const auto points=validInlierPoints(*s,words);
            if(!signatureImageSize(*s,width,height) || occupiedGridCells(*s,words,width,height)<cells ||
               int(points.size())<minimum || !pointSetGeometry(points,extent,conditioning) ||
               extent<span || conditioning<hessian) return false;
        }
        double linear=0.0,angular=0.0;cv::Mat eigenvalues;
        if(!std::isfinite(maximumCondition) || maximumCondition<1 ||
           !covarianceVariances(info.covariance,linear,angular) || !cv::checkRange(info.covariance) ||
           cv::norm(info.covariance-info.covariance.t(),cv::NORM_INF)>1e-9 ||
           !cv::eigen(info.covariance,eigenvalues) || eigenvalues.at<double>(5)<=0 ||
           eigenvalues.at<double>(0)/eigenvalues.at<double>(5)>maximumCondition) return false;
        return true; // shared final metric support, coverage and covariance all passed
    };
    // Refinement may replace feature storage. Reverse matching owns its own
    // descriptors and cannot observe mutations made by the forward solve.
    SensorData reverseFrom=fromData,reverseTo=toData;
    reverseFrom.setFeatures(fromData.keypoints(),fromData.keypoints3D(),fromData.descriptors().clone());
    reverseTo.setFeatures(toData.keypoints(),toData.keypoints3D(),toData.descriptors().clone());
    Signature a(fromData),b(toData),reverseB(reverseTo),reverseA(reverseFrom);
    std::unique_ptr<Registration> forward(Registration::create(p)),reverse(Registration::create(p));
    if(!forward || !reverse || !forward->isImageRequired() || !reverse->isImageRequired())
    {reason="raw visual registration unavailable";return false;}
    RegistrationInfo f,r;
    result.forward=forward->computeTransformationMod(a,b,Transform(),&f);
    result.reverse=reverse->computeTransformationMod(reverseB,reverseA,Transform(),&r);
    const bool forwardValid=validate(a,b,result.forward,f),reverseValid=validate(reverseB,reverseA,result.reverse,r);
    if(!forwardValid || !reverseValid ||
       result.forward.getDistance(result.reverse.inverse())>.025 ||
       result.forward.getAngle(result.reverse.inverse())>2.0*M_PI/180.0)
    {reason=uFormat("raw bidirectional RGB-D check failed (f=%d,r=%d,finliers=%d,rinliers=%d,fbackend=%s,rbackend=%s)",
        forwardValid?1:0,reverseValid?1:0,f.inliers,r.inliers,f.rejectedMsg.c_str(),r.rejectedMsg.c_str());return false;}
    return true;
}


// Fit translation and yaw from metric correspondences in gravity-aligned
// local/main frames. Does not reuse an inverted solve as reverse evidence.
// Caller must still validate full raw registration, both directions, measured
// gravity coverage, three independent bridges and every shadow/holdout gate.
bool gravityAlignedMetricFit(const Signature & from,const Signature & to,
        const Transform & fromPose,const Transform & toPose,
        const Transform & raw,const RegistrationInfo & info,int minimum,
        double metricCap,Transform & measured,std::string & reason)
{
    measured.setNull();
    if(!safeRigidTransform(fromPose) || !safeRigidTransform(toPose) || !safeRigidTransform(raw))
    {reason="invalid gravity-aligned fit frame";return false;}
    std::set<int> words;
    if(!finalRecoveryMetricSupport(from,to,raw,info,minimum,metricCap,words,reason))return false;
    std::vector<cv::Point3d> a,b;
    cv::Point3d ca(0,0,0),cb(0,0,0);
    for(int word:words) {
        const auto pa=from.getWords3().at(from.getWords().find(word)->second);
        const auto pb=to.getWords3().at(to.getWords().find(word)->second);
        a.push_back(transformPoint(fromPose,cv::Point3d(pa.x,pa.y,pa.z)));
        b.push_back(transformPoint(toPose,cv::Point3d(pb.x,pb.y,pb.z)));
        ca+=a.back();cb+=b.back();
    }
    ca*=1.0/a.size();cb*=1.0/b.size();
    double dot=0,cross=0;
    for(std::size_t i=0;i<a.size();++i) {
        const auto p=a[i]-ca,q=b[i]-cb;
        dot+=p.x*q.x+p.y*q.y;cross+=q.x*p.y-q.y*p.x;
    }
    if(!std::isfinite(dot) || !std::isfinite(cross) || std::hypot(dot,cross)<1e-12)
    {reason="gravity-aligned fit has no observable yaw";return false;}
    const double yaw=std::atan2(cross,dot),c=std::cos(yaw),s=std::sin(yaw);
    const cv::Point3d t=ca-cv::Point3d(c*cb.x-s*cb.y,s*cb.x+c*cb.y,cb.z);
    const Transform world(float(t.x),float(t.y),float(t.z),0.0f,0.0f,float(yaw));
    measured=fromPose.inverse()*world*toPose;
    if(!safeRigidTransform(measured) || measured.getDistance(raw)>.025 ||
       measured.getAngle(raw)>2.0*M_PI/180.0)
    {reason="gravity-aligned fit disagrees with raw six-axis registration";return false;}
    // Recheck the complete original support RMS, final metric support count,
    // and the original maximum error cap under the constrained solution.
    return finalRecoveryMetricSupport(from,to,measured,info,minimum,metricCap,words,reason);
}

void expect(bool value,const char *name){if(!value)throw std::runtime_error(name);}
int main()try {
    ULogger::setLevel(ULogger::kError);
    const Transform truth(.03f,-.02f,.01f,0,0,.2f),raw(.03f,-.02f,.01f,0,.01f,.2f);
    auto test=[&](int mode,Transform fromPose,Transform toPose,Transform & fit)->bool {
        std::vector<cv::Point3f> a,b;std::multimap<int,int> words;RegistrationInfo info;
        for(int i=0;i<100;++i) {
            cv::Point3d q=(mode==2)?cv::Point3d(0,0,.4+i*.001):cv::Point3d((i%10)*.01-.045,(i/10)*.01-.045,.4+((i*7)%5)*.01);
            auto p=transformPoint(truth,q);if(mode==4 && i<30)p.x+=.08;
            a.emplace_back(p.x,p.y,p.z);b.emplace_back(q.x,q.y,q.z);words.emplace(i,i);
            if(mode!=3 || i<24)info.inliersIDs.push_back(i);
        }
        Signature sa,sb;sa.setWords(words,{},a,cv::Mat());sb.setWords(words,{},b,cv::Mat());
        std::string why;
        const bool ok=gravityAlignedMetricFit(sa,sb,fromPose,toPose,mode==2?truth:raw,info,25,.025,fit,why);
        std::cout<<"CASE "<<mode<<" accepted="<<ok<<" reason="<<why<<std::endl;return ok;
    };
    Transform fit;const auto identity=Transform::getIdentity();
    expect(test(0,identity,identity,fit),"known constrained fit rejected");
    expect(fit.getDistance(truth)<1e-5 && fit.getAngle(truth)<1e-4,"known pose not recovered");
    expect(test(1,Transform(1,2,3,0,0,-.4),Transform(-2,1,2,0,0,.6),fit),"world yaw/translation gauge changed metric solution");
    expect(fit.getDistance(truth)<1e-5 && fit.getAngle(truth)<1e-4,"world gauge changed result");
    expect(!test(2,identity,identity,fit),"unobservable yaw accepted");
    expect(!test(3,identity,identity,fit),"24 inliers accepted");
    expect(!test(4,identity,identity,fit),"inconsistent depth accepted");
    expect(!test(5,Transform(0,0,0,.1f,0,0),identity,fit),"gravity versus raw contradiction accepted");
    expect(!test(6,Transform(),identity,fit),"missing pose accepted");
    std::cout<<"PASS 7 gravity-aligned metric cases"<<std::endl;return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
