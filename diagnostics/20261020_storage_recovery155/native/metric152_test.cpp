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


int main() {
    auto verify=[](int bad,double error,double cap,bool expected,int expectedCount,
            bool ambiguous=false,bool nonfinite=false) {
        Signature a,b;std::multimap<int,int> words;
        std::vector<cv::KeyPoint> keypoints(100);std::vector<cv::Point3f> p,q;
        RegistrationInfo info;info.inliers=100;
        for(int i=0;i<100;++i) {
            words.emplace(i+1,i);info.inliersIDs.push_back(i+1);
            p.emplace_back(.01f*(i%10),.01f*(i/10),.7f);
            q.push_back(p.back());if(i<bad)q.back().x+=error;
        }
        auto other=words;
        if(ambiguous){other.erase(2);other.emplace(1,1);}
        if(nonfinite)q[0].z=std::numeric_limits<float>::quiet_NaN();
        a.setWords(words,keypoints,p,cv::Mat());b.setWords(other,keypoints,q,cv::Mat());
        std::set<int> selected;std::string why;
        bool ok=finalRecoveryMetricSupport(a,b,Transform::getIdentity(),info,25,cap,selected,why);
        if(ok!=expected || (expected && int(selected.size())!=expectedCount))
            throw std::runtime_error(why);
    };
    verify(0,0,.025,true,100);
    verify(4,.032,.025,true,96); // few stale BA labels, original RMS remains small
    verify(50,.04,.025,false,0); // broad inconsistency cannot be trimmed away
    verify(76,.02,.019,false,0); // fewer than 25 final supporters
    verify(1,.021,.02,true,99); // honor a tighter configured cap
    verify(0,0,.025,false,0,true); // duplicate IDs cannot create support
    verify(0,0,.025,false,0,false,true);
    verify(0,0,.03,false,0); // no caller may relax the hard metric cap
    std::cout<<"PASS 8 final-metric-support cases"<<std::endl;
}
