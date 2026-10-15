/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Version.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/Optimizer.h"
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/Signature.h"

#include "rtabmap/core/EpipolarGeometry.h"

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"

#include "rtabmap/core/DBDriver.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/BayesFilter.h"
#include "rtabmap/core/Compression.h"
#include "rtabmap/core/Registration.h"
#include "rtabmap/core/RegistrationInfo.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UProcessInfo.h>

#ifdef RTABMAP_PYTHON
#include "rtabmap/core/PythonInterface.h"
#endif

#ifdef RTABMAP_MRPT
// Used for odometry error propagation
#include <mrpt/poses/CPose3DPDFGaussian.h>
#endif

#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/TextureMesh.h>

#include <stdlib.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <set>

#define LOG_F "LogF.txt"
#define LOG_I "LogI.txt"

#define GRAPH_FILE_NAME "Graph.dot"


//
//
//
// =======================================================
// MAIN LOOP, see method "void Rtabmap::process();" below.
// =======================================================
//
//
//

#include "optimizer_trace.inc"

namespace rtabmap
{

namespace
{

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

double featureJaccard(const std::set<int> & a, const std::set<int> & b)
{
	if(a.empty() && b.empty())
	{
		return 1.0;
	}
	size_t intersection = 0;
	std::set<int>::const_iterator aIter=a.begin();
	std::set<int>::const_iterator bIter=b.begin();
	while(aIter!=a.end() && bIter!=b.end())
	{
		if(*aIter < *bIter)
		{
			++aIter;
		}
		else if(*bIter < *aIter)
		{
			++bIter;
		}
		else
		{
			++intersection;
			++aIter;
			++bIter;
		}
	}
	return (double)intersection/(double)(a.size()+b.size()-intersection);
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

bool safeInformationMatrix(
		const cv::Mat & information,
		double maxCondition,
        std::string & reason,
        bool boundProposalInformationScale = true)
{
    // The absolute 1e6 ceiling limits new bridge proposals. Internally stored
    // odometry may legitimately exceed it (for example covariance 3e-8).
    // Existing graph constraints keep their exact weights, SPD/conditioning
    // checks and every subsequent optimization/deformation check.
    // Only the three internal graph-validation call sites opt out of this
    // proposal-only scale ceiling; proposal callers retain the default.
	if(information.rows != 6 || information.cols != 6 ||
			information.type() != CV_64FC1 || !std::isfinite(maxCondition) ||
			maxCondition < 1.0)
	{
		reason = "information matrix must be 6x6 CV_64F";
		return false;
	}
	for(int row=0; row<6; ++row)
	{
		for(int col=0; col<6; ++col)
		{
			const double value = information.at<double>(row,col);
			if(!std::isfinite(value) ||
					std::fabs(value-information.at<double>(col,row)) > 1e-9)
			{
				reason = "information matrix is non-finite or asymmetric";
				return false;
			}
		}
	}
	cv::Mat eigenvalues;
	if(!cv::eigen(information, eigenvalues) || eigenvalues.total() != 6)
	{
		reason = "information matrix eigendecomposition failed";
		return false;
	}
	const double largest = eigenvalues.at<double>(0);
	const double smallest = eigenvalues.at<double>(5);
	if(!std::isfinite(largest) || !std::isfinite(smallest) ||
			smallest <= 1e-9 || (boundProposalInformationScale && largest > 1000000.0) ||
			largest/smallest > maxCondition)
	{
		reason = "information matrix is not SPD or is ill-conditioned";
		return false;
	}
	return true;
}

bool safeRigidTransform(const Transform & transform);

#include "safe_gravity_residual.inc"

double safeTiltError(const Transform & a, const Transform & b)
{
	// Compare each pose's local up axis in the common/world frame. This removes
	// yaw without Euler decomposition and stays well-defined near +/-90 pitch.
	const double ax = a.r13();
	const double ay = a.r23();
	const double az = a.r33();
	const double bx = b.r13();
	const double by = b.r23();
	const double bz = b.r33();
	const double denominator = std::sqrt((ax*ax+ay*ay+az*az)*(bx*bx+by*by+bz*bz));
	if(!std::isfinite(denominator) || denominator <= 1e-12)
	{
		return std::numeric_limits<double>::infinity();
	}
	const double cosine = std::max(-1.0, std::min(1.0,
			(ax*bx+ay*by+az*bz)/denominator));
	return std::acos(cosine);
}

bool sameConstraintMeasurement(const Link & a, const Link & b)
{
	if(a.from() != b.from() || a.to() != b.to() || a.type() != b.type() ||
			!safeRigidTransform(a.transform()) || !safeRigidTransform(b.transform()) ||
			a.transform().getDistance(b.transform()) > 1e-6 ||
			a.transform().getAngle(b.transform()) > 1e-6 ||
			a.infMatrix().rows != 6 || a.infMatrix().cols != 6 ||
			b.infMatrix().rows != 6 || b.infMatrix().cols != 6 ||
			a.infMatrix().type() != CV_64FC1 || b.infMatrix().type() != CV_64FC1)
	{
		return false;
	}
	return cv::norm(a.infMatrix(), b.infMatrix(), cv::NORM_INF) <= 1e-6;
}

bool sameConstraintMeasurementUndirected(const Link & a, const Link & b)
{
	if(sameConstraintMeasurement(a, b))
	{
		return true;
	}
	// Memory::getMetricConstraints() may canonicalize an already committed
	// edge to the opposite endpoint order.  Compare the exact inverse rather
	// than weakening any transform, information, endpoint or type threshold.
	return a.from() == b.to() && a.to() == b.from() && a.type() == b.type() &&
			sameConstraintMeasurement(a, b.inverse());
}

bool containsConstraintMeasurementUndirected(
		const std::multimap<int, Link> & constraints,
		const Link & expected)
{
	const int endpointKeys[2] = {expected.from(), expected.to()};
	for(unsigned int i=0; i<2; ++i)
	{
		const std::pair<std::multimap<int, Link>::const_iterator,
				std::multimap<int, Link>::const_iterator> range =
				constraints.equal_range(endpointKeys[i]);
		for(std::multimap<int, Link>::const_iterator iter=range.first;
				iter!=range.second; ++iter)
		{
			if(sameConstraintMeasurementUndirected(expected, iter->second))
			{
				return true;
			}
		}
		if(expected.from() == expected.to())
		{
			break;
		}
	}
	return false;
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

double depthMeters(const cv::Mat & depth, int y, int x)
{
	if(depth.type() == CV_32FC1)
	{
		return depth.at<float>(y, x);
	}
	if(depth.type() == CV_16UC1)
	{
		return double(depth.at<unsigned short>(y, x))/1000.0;
	}
	return std::numeric_limits<double>::quiet_NaN();
}

bool fitDepthPlane(std::vector<cv::Point3d> points, cv::Point3d & center,
		cv::Vec3d & normal, double & rms)
{
	if(points.size() < 40)
	{
		return false;
	}
	for(int iteration=0; iteration<3; ++iteration)
	{
		center = cv::Point3d(0.0, 0.0, 0.0);
		for(std::vector<cv::Point3d>::const_iterator point=points.begin(); point!=points.end(); ++point)
		{
			center += *point;
		}
		center *= 1.0/double(points.size());
		cv::Mat covariance = cv::Mat::zeros(3, 3, CV_64FC1);
		for(std::vector<cv::Point3d>::const_iterator point=points.begin(); point!=points.end(); ++point)
		{
			const cv::Vec3d delta(point->x-center.x, point->y-center.y, point->z-center.z);
			for(int row=0; row<3; ++row)
			{
				for(int column=0; column<3; ++column)
				{
					covariance.at<double>(row,column) += delta[row]*delta[column];
				}
			}
		}
		cv::Mat eigenvalues;
		cv::Mat eigenvectors;
		if(!cv::eigen(covariance, eigenvalues, eigenvectors))
		{
			return false;
		}
		normal = cv::Vec3d(eigenvectors.at<double>(2,0),
				eigenvectors.at<double>(2,1), eigenvectors.at<double>(2,2));
		const double norm = cv::norm(normal);
		if(!std::isfinite(norm) || norm <= 1e-9)
		{
			return false;
		}
		normal *= 1.0/norm;
		std::vector<double> residuals;
		residuals.reserve(points.size());
		for(std::vector<cv::Point3d>::const_iterator point=points.begin(); point!=points.end(); ++point)
		{
			residuals.push_back(std::fabs(normal.dot(cv::Vec3d(
					point->x-center.x, point->y-center.y, point->z-center.z))));
		}
		std::vector<double> sorted = residuals;
		std::nth_element(sorted.begin(), sorted.begin()+sorted.size()/2, sorted.end());
		const double median = sorted[sorted.size()/2];
		const double threshold = std::max(0.003, std::min(0.008, 3.0*median));
		if(iteration < 2)
		{
			std::vector<cv::Point3d> inliers;
			for(size_t i=0; i<points.size(); ++i)
			{
				if(residuals[i] <= threshold)
				{
					inliers.push_back(points[i]);
				}
			}
			if(inliers.size() < 40)
			{
				return false;
			}
			points.swap(inliers);
		}
		else
		{
			double squared = 0.0;
			for(std::vector<double>::const_iterator residual=residuals.begin(); residual!=residuals.end(); ++residual)
			{
				squared += (*residual)*(*residual);
			}
			rms = std::sqrt(squared/double(residuals.size()));
		}
	}
	return std::isfinite(rms) && rms <= 0.003;
}

bool depthTagCorners(const SensorData & data,
		const AcceptedTagCornerObservation & observation,
		std::array<cv::Point3d, 4> & bodyCorners, std::string & reason)
{
	if(data.cameraModels().size() != 1 || !data.cameraModels().front().isValidForProjection())
	{
		reason = "corner observation signature lacks one valid camera";
		return false;
	}
	cv::Mat image;
	cv::Mat depth;
	data.uncompressDataConst(&image, &depth);
	const CameraModel & camera = data.cameraModels().front();
	if(depth.empty() || (depth.type() != CV_32FC1 && depth.type() != CV_16UC1) ||
			depth.cols != camera.imageWidth() || depth.rows != camera.imageHeight())
	{
		reason = uFormat("corner observation signature lacks matching metric depth "
				"(signature=%d depth_empty=%d depth_type=%d depth=%dx%d camera=%dx%d compressed_bytes=%d)",
				observation.signatureId, depth.empty()?1:0, depth.empty()?-1:depth.type(),
				depth.cols, depth.rows, camera.imageWidth(), camera.imageHeight(),
				(int)data.depthOrRightCompressed().total());
		return false;
	}
	std::vector<cv::Point2f> polygon;
	double minimumX=double(depth.cols), minimumY=double(depth.rows), maximumX=0.0, maximumY=0.0;
	for(int corner=0; corner<4; ++corner)
	{
		const double x = observation.cornersXY[corner*2];
		const double y = observation.cornersXY[corner*2+1];
		if(!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || y < 0.0 ||
				x >= depth.cols || y >= depth.rows)
		{
			reason = "corner observation lies outside the accepted image";
			return false;
		}
		polygon.push_back(cv::Point2f((float)x, (float)y));
		minimumX=std::min(minimumX,x); maximumX=std::max(maximumX,x);
		minimumY=std::min(minimumY,y); maximumY=std::max(maximumY,y);
	}
	std::vector<cv::Point3d> planePoints;
	const int x0=std::max(0, int(std::floor(minimumX)));
	const int x1=std::min(depth.cols-1, int(std::ceil(maximumX)));
	const int y0=std::max(0, int(std::floor(minimumY)));
	const int y1=std::min(depth.rows-1, int(std::ceil(maximumY)));
	for(int y=y0; y<=y1; ++y)
	{
		for(int x=x0; x<=x1; ++x)
		{
			if(cv::pointPolygonTest(polygon, cv::Point2f(float(x)+0.5f, float(y)+0.5f), false) < 0.0)
			{
				continue;
			}
			const double z = depthMeters(depth, y, x);
			if(std::isfinite(z) && z >= 0.20 && z <= 0.80)
			{
				planePoints.push_back(cv::Point3d(
						(double(x)-camera.cx())/camera.fx()*z,
						(double(y)-camera.cy())/camera.fy()*z, z));
			}
		}
	}
	cv::Point3d center;
	cv::Vec3d normal;
	double rms = 0.0;
	if(!fitDepthPlane(planePoints, center, normal, rms))
	{
		reason = "Tag quadrilateral has no robust depth plane within 0.20-0.80 m";
		return false;
	}
	const double planeOffset = normal.dot(cv::Vec3d(center.x, center.y, center.z));
	const Transform & bodyFromCamera = camera.localTransform();
	double sideLengths[4] = {0.0,0.0,0.0,0.0};
	std::array<cv::Point3d, 4> cameraCorners;
	for(int corner=0; corner<4; ++corner)
	{
		const double x=observation.cornersXY[corner*2];
		const double y=observation.cornersXY[corner*2+1];
		const cv::Vec3d ray((x-camera.cx())/camera.fx(), (y-camera.cy())/camera.fy(), 1.0);
		const double denominator = normal.dot(ray);
		if(!std::isfinite(denominator) || std::fabs(denominator) < 0.1)
		{
			reason = "Tag corner ray is nearly parallel to its depth plane";
			return false;
		}
		const double scale = planeOffset/denominator;
		cameraCorners[corner]=cv::Point3d(ray[0]*scale,ray[1]*scale,ray[2]*scale);
		if(!std::isfinite(scale) || scale < 0.20 || scale > 0.80)
		{
			reason = "Tag corner plane intersection is outside 0.20-0.80 m";
			return false;
		}
		bodyCorners[corner]=transformPoint(bodyFromCamera,cameraCorners[corner]);
	}
	for(int corner=0; corner<4; ++corner)
	{
		sideLengths[corner]=cv::norm(cameraCorners[(corner+1)%4]-cameraCorners[corner]);
	}
	const double meanSide=(sideLengths[0]+sideLengths[1]+sideLengths[2]+sideLengths[3])/4.0;
	const double minSide=*std::min_element(sideLengths,sideLengths+4);
	const double maxSide=*std::max_element(sideLengths,sideLengths+4);
	if(!std::isfinite(meanSide) || meanSide < 0.03 || meanSide > 0.20 || maxSide-minSide > 0.012)
	{
		reason = "depth-plane Tag quadrilateral has inconsistent side geometry";
		return false;
	}
	return true;
}

} // namespace

Rtabmap::Rtabmap() :
	_publishStats(Parameters::defaultRtabmapPublishStats()),
	_publishLastSignatureData(Parameters::defaultRtabmapPublishLastSignature()),
	_publishPdf(Parameters::defaultRtabmapPublishPdf()),
	_publishLikelihood(Parameters::defaultRtabmapPublishLikelihood()),
	_publishRAMUsage(Parameters::defaultRtabmapPublishRAMUsage()),
	_computeRMSE(Parameters::defaultRtabmapComputeRMSE()),
	_saveWMState(Parameters::defaultRtabmapSaveWMState()),
	_maxTimeAllowed(Parameters::defaultRtabmapTimeThr()), // 700 ms
	_maxMemoryAllowed(Parameters::defaultRtabmapMemoryThr()), // 0=inf
	_loopThr(Parameters::defaultRtabmapLoopThr()),
	_loopRatio(Parameters::defaultRtabmapLoopRatio()),
	_aggressiveLoopThr(Parameters::defaultRGBDAggressiveLoopThr()),
	_virtualPlaceLikelihoodRatio(Parameters::defaultRtabmapVirtualPlaceLikelihoodRatio()),
	_maxLoopClosureDistance(Parameters::defaultRGBDMaxLoopClosureDistance()),
	_verifyLoopClosureHypothesis(Parameters::defaultVhEpEnabled()),
	_maxRetrieved(Parameters::defaultRtabmapMaxRetrieved()),
	_maxLocalRetrieved(Parameters::defaultRGBDMaxLocalRetrieved()),
	_maxRepublished(Parameters::defaultRtabmapMaxRepublished()),
	_rawDataKept(Parameters::defaultMemImageKept()),
	_statisticLogsBufferedInRAM(Parameters::defaultRtabmapStatisticLogsBufferedInRAM()),
	_statisticLogged(Parameters::defaultRtabmapStatisticLogged()),
	_statisticLoggedHeaders(Parameters::defaultRtabmapStatisticLoggedHeaders()),
	_rgbdSlamMode(Parameters::defaultRGBDEnabled()),
	_rgbdLinearUpdate(Parameters::defaultRGBDLinearUpdate()),
	_rgbdAngularUpdate(Parameters::defaultRGBDAngularUpdate()),
	_rgbdLinearSpeedUpdate(Parameters::defaultRGBDLinearSpeedUpdate()),
	_rgbdAngularSpeedUpdate(Parameters::defaultRGBDAngularSpeedUpdate()),
	_newMapOdomChangeDistance(Parameters::defaultRGBDNewMapOdomChangeDistance()),
	_neighborLinkRefining(Parameters::defaultRGBDNeighborLinkRefining()),
	_proximityByTime(Parameters::defaultRGBDProximityByTime()),
	_proximityBySpace(Parameters::defaultRGBDProximityBySpace()),
	_scanMatchingIdsSavedInLinks(Parameters::defaultRGBDScanMatchingIdsSavedInLinks()),
	_loopClosureIdentityGuess(Parameters::defaultRGBDLoopClosureIdentityGuess()),
	_localRadius(Parameters::defaultRGBDLocalRadius()),
	_localImmunizationRatio(Parameters::defaultRGBDLocalImmunizationRatio()),
	_proximityMaxGraphDepth(Parameters::defaultRGBDProximityMaxGraphDepth()),
	_proximityMaxPaths(Parameters::defaultRGBDProximityMaxPaths()),
	_proximityMaxNeighbors(Parameters::defaultRGBDProximityPathMaxNeighbors()),
	_proximityFilteringRadius(Parameters::defaultRGBDProximityPathFilteringRadius()),
	_proximityRawPosesUsed(Parameters::defaultRGBDProximityPathRawPosesUsed()),
	_proximityAngle(Parameters::defaultRGBDProximityAngle()*M_PI/180.0f),
	_proximityOdomGuess(Parameters::defaultRGBDProximityOdomGuess()),
	_proximityMergedScanCovFactor(Parameters::defaultRGBDProximityMergedScanCovFactor()),
	_databasePath(""),
	_optimizeFromGraphEnd(Parameters::defaultRGBDOptimizeFromGraphEnd()),
	_optimizationMaxError(Parameters::defaultRGBDOptimizeMaxError()),
	_optimizationMaxErrorRepairRadius(Parameters::defaultRGBDOptimizeMaxErrorRepairRadius()),
	_startNewMapOnLoopClosure(Parameters::defaultRtabmapStartNewMapOnLoopClosure()),
	_crossMapLoopPending(Parameters::defaultRtabmapCrossMapLoopPending()),
	_crossMapLoopPendingCount(Parameters::defaultRtabmapCrossMapLoopPendingCount()),
	_crossMapLoopPendingMaxIdSpan(Parameters::defaultRtabmapCrossMapLoopPendingMaxIdSpan()),
	_crossMapLoopMinInliers(Parameters::defaultRtabmapCrossMapLoopMinInliers()),
	_crossMapLoopMinGridCells(Parameters::defaultRtabmapCrossMapLoopMinGridCells()),
	_crossMapLoopMin3DSpan(Parameters::defaultRtabmapCrossMapLoopMin3DSpan()),
	_crossMapLoopMinHessianRatio(Parameters::defaultRtabmapCrossMapLoopMinHessianRatio()),
	_crossMapLoopMaxTranslation(Parameters::defaultRtabmapCrossMapLoopMaxTranslation()),
	_crossMapLoopMaxRotation(Parameters::defaultRtabmapCrossMapLoopMaxRotation()),
	_crossMapLoopMaxNormalizedError(Parameters::defaultRtabmapCrossMapLoopMaxNormalizedError()),
	_crossMapLoopMaxFeatureJaccard(Parameters::defaultRtabmapCrossMapLoopMaxFeatureJaccard()),
	_crossMapLoopMinSourceTranslation(Parameters::defaultRtabmapCrossMapLoopMinSourceTranslation()),
	_crossMapLoopMinSourceRotation(Parameters::defaultRtabmapCrossMapLoopMinSourceRotation()),
	_startNewMapOnGoodSignature(Parameters::defaultRtabmapStartNewMapOnGoodSignature()),
	_goalReachedRadius(Parameters::defaultRGBDGoalReachedRadius()),
	_goalsSavedInUserData(Parameters::defaultRGBDGoalsSavedInUserData()),
	_pathStuckIterations(Parameters::defaultRGBDPlanStuckIterations()),
	_pathLinearVelocity(Parameters::defaultRGBDPlanLinearVelocity()),
	_pathAngularVelocity(Parameters::defaultRGBDPlanAngularVelocity()),
	_forceOdom3doF(Parameters::defaultRGBDForceOdom3DoF()),
	_restartAtOrigin(Parameters::defaultRGBDStartAtOrigin()),
	_loopCovLimited(Parameters::defaultRGBDLoopCovLimited()),
	_loopGPS(Parameters::defaultRtabmapLoopGPS()),
	_maxOdomCacheSize(Parameters::defaultRGBDMaxOdomCacheSize()),
	_localizationSmoothing(Parameters::defaultRGBDLocalizationSmoothing()),
	_localizationPriorInf(1.0/(Parameters::defaultRGBDLocalizationPriorError()*Parameters::defaultRGBDLocalizationPriorError())),
	_localizationSecondTryWithoutProximityLinks(Parameters::defaultRGBDLocalizationSecondTryWithoutProximityLinks()),
	_createGlobalScanMap(Parameters::defaultRGBDProximityGlobalScanMap()),
	_markerPriorsLinearVariance(Parameters::defaultMarkerPriorsVarianceLinear()),
	_markerPriorsAngularVariance(Parameters::defaultMarkerPriorsVarianceAngular()),
	_loopClosureHypothesis(0,0.0f),
	_resetEpoch(0),
	_skipNextOdomResetDetection(false),
	_safeRelocalizationTrustedSnapshotValid(false),
	_safeRelocalizationPromotionLimitsValid(false),
	_safeRelocalizationPromotionMaxOldPoseTranslation(0.0f),
	_safeRelocalizationPromotionMaxOldPoseRotation(0.0f),
	_safeRelocalizationGuardEnabled(false),
	_highestHypothesis(0,0.0f),
	_lastProcessTime(0.0),
	_someNodesHaveBeenTransferred(false),
	_distanceTravelled(0.0f),
	_distanceTravelledSinceLastLocalization(0.0f),
	_optimizeFromGraphEndChanged(false),
	_epipolarGeometry(0),
	_bayesFilter(0),
	_graphOptimizer(0),
	_memory(0),
	_foutFloat(0),
	_foutInt(0),
	_wDir(""),
	_mapCorrection(Transform::getIdentity()),
	_lastLocalizationNodeId(0),
	_currentSessionHasGPS(false),
	_lastRejectedLoopClosureIds(0,0),
	_pathStatus(0),
	_pathCurrentIndex(0),
	_pathGoalIndex(0),
	_pathTransformToGoal(Transform::getIdentity()),
	_pathStuckCount(0),
	_pathStuckDistance(0.0f)
#ifdef RTABMAP_PYTHON
	,_python(new PythonInterface())
#endif
{
}

Rtabmap::~Rtabmap() {
	UDEBUG("");
	this->close();
}

void Rtabmap::setupLogFiles(bool overwrite)
{
	flushStatisticLogs();
	// Log files
	if(_foutFloat)
	{
		fclose(_foutFloat);
		_foutFloat = 0;
	}
	if(_foutInt)
	{
		fclose(_foutInt);
		_foutInt = 0;
	}

	if(_statisticLogged && !_wDir.empty())
	{
		std::string attributes = "a+"; // append to log files
		if(overwrite)
		{
			// If a file with the same name already exists
			// its content is erased and the file is treated
			// as a new empty file.
			attributes = "w";
		}

		bool addLogFHeader = overwrite || !UFile::exists(_wDir+"/"+LOG_F);
		bool addLogIHeader = overwrite || !UFile::exists(_wDir+"/"+LOG_I);

	#ifdef _MSC_VER
		fopen_s(&_foutFloat, (_wDir+"/"+LOG_F).c_str(), attributes.c_str());
		fopen_s(&_foutInt, (_wDir+"/"+LOG_I).c_str(), attributes.c_str());
	#else
		_foutFloat = fopen((_wDir+"/"+LOG_F).c_str(), attributes.c_str());
		_foutInt = fopen((_wDir+"/"+LOG_I).c_str(), attributes.c_str());
	#endif
		// add header (column identification)
		if(_statisticLoggedHeaders && addLogFHeader && _foutFloat)
		{
			fprintf(_foutFloat, "Column headers:\n");
			fprintf(_foutFloat, " 1-Total iteration time (s)\n");
			fprintf(_foutFloat, " 2-Memory update time (s)\n");
			fprintf(_foutFloat, " 3-Retrieval time (s)\n");
			fprintf(_foutFloat, " 4-Likelihood time (s)\n");
			fprintf(_foutFloat, " 5-Posterior time (s)\n");
			fprintf(_foutFloat, " 6-Hypothesis selection time (s)\n");
			fprintf(_foutFloat, " 7-Hypothesis validation time (s)\n");
			fprintf(_foutFloat, " 8-Transfer time (s)\n");
			fprintf(_foutFloat, " 9-Statistics creation time (s)\n");
			fprintf(_foutFloat, " 10-Loop closure hypothesis value\n");
			fprintf(_foutFloat, " 11-NAN\n");
			fprintf(_foutFloat, " 12-NAN\n");
			fprintf(_foutFloat, " 13-NAN\n");
			fprintf(_foutFloat, " 14-NAN\n");
			fprintf(_foutFloat, " 15-NAN\n");
			fprintf(_foutFloat, " 16-Virtual place hypothesis\n");
			fprintf(_foutFloat, " 17-Join trash time (s)\n");
			fprintf(_foutFloat, " 18-Weight Update (rehearsal) similarity\n");
			fprintf(_foutFloat, " 19-Empty trash time (s)\n");
			fprintf(_foutFloat, " 20-Retrieval database access time (s)\n");
			fprintf(_foutFloat, " 21-Add loop closure link time (s)\n");
			fprintf(_foutFloat, " 22-Memory cleanup time (s)\n");
			fprintf(_foutFloat, " 23-Scan matching (odometry correction) time (s)\n");
			fprintf(_foutFloat, " 24-Local time loop closure detection time (s)\n");
			fprintf(_foutFloat, " 25-Local space loop closure detection time (s)\n");
			fprintf(_foutFloat, " 26-Map optimization (s)\n");
		}
		if(_statisticLoggedHeaders && addLogIHeader && _foutInt)
		{
			fprintf(_foutInt, "Column headers:\n");
			fprintf(_foutInt, " 1-Loop closure ID\n");
			fprintf(_foutInt, " 2-Highest loop closure hypothesis\n");
			fprintf(_foutInt, " 3-Locations transferred\n");
			fprintf(_foutInt, " 4-NAN\n");
			fprintf(_foutInt, " 5-Words extracted from the last image\n");
			fprintf(_foutInt, " 6-Vocabulary size\n");
			fprintf(_foutInt, " 7-Working memory size\n");
			fprintf(_foutInt, " 8-Is loop closure hypothesis rejected?\n");
			fprintf(_foutInt, " 9-NAN\n");
			fprintf(_foutInt, " 10-NAN\n");
			fprintf(_foutInt, " 11-Locations retrieved\n");
			fprintf(_foutInt, " 12-Retrieval location ID\n");
			fprintf(_foutInt, " 13-Unique words extraced from last image\n");
			fprintf(_foutInt, " 14-Retrieval ID\n");
			fprintf(_foutInt, " 15-Non-null likelihood values\n");
			fprintf(_foutInt, " 16-Weight Update ID\n");
			fprintf(_foutInt, " 17-Is last location merged through Weight Update?\n");
			fprintf(_foutInt, " 18-Local graph size\n");
			fprintf(_foutInt, " 19-Sensor data id\n");
			fprintf(_foutInt, " 20-Indexed words\n");
			fprintf(_foutInt, " 21-Index memory usage (KB)\n");
		}

		ULOGGER_DEBUG("Log file (int)=%s", (_wDir+"/"+LOG_I).c_str());
		ULOGGER_DEBUG("Log file (float)=%s", (_wDir+"/"+LOG_F).c_str());
	}
	else
	{
		if(_statisticLogged)
		{
			UWARN("Working directory is not set, log disabled!");
		}
		UDEBUG("Log disabled!");
	}
}

void Rtabmap::flushStatisticLogs()
{
	if(_foutFloat && _bufferedLogsF.size())
	{
		UDEBUG("_bufferedLogsF.size=%d", _bufferedLogsF.size());
		for(std::list<std::string>::iterator iter = _bufferedLogsF.begin(); iter!=_bufferedLogsF.end(); ++iter)
		{
			fprintf(_foutFloat, "%s", iter->c_str());
		}
		_bufferedLogsF.clear();
	}
	if(_foutInt && _bufferedLogsI.size())
	{
		UDEBUG("_bufferedLogsI.size=%d", _bufferedLogsI.size());
		for(std::list<std::string>::iterator iter = _bufferedLogsI.begin(); iter!=_bufferedLogsI.end(); ++iter)
		{
			fprintf(_foutInt, "%s", iter->c_str());
		}
		_bufferedLogsI.clear();
	}
}

void Rtabmap::init(const ParametersMap & parameters, const std::string & databasePath, bool loadDatabaseParameters)
{
	this->init(parameters, databasePath, loadDatabaseParameters, false);
}

void Rtabmap::init(const ParametersMap & parameters, const std::string & databasePath, bool loadDatabaseParameters, bool preboundEmptyIsNew)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("Rtabmap::init is disabled after the safe-relocalization process-lifetime latch is enabled");
		return;
	}
	UDEBUG("path=%s", databasePath.c_str());
	_databasePath = databasePath;
	if(!_databasePath.empty())
	{
		UASSERT(UFile::getExtension(_databasePath).compare("db") == 0);
		UINFO("Using database \"%s\".", _databasePath.c_str());
	}
	else
	{
		UWARN("Using empty database. Mapping session will not be saved unless it is closed with an output database path.");
	}

	bool newDatabase = _databasePath.empty() || !UFile::exists(_databasePath) || preboundEmptyIsNew;

	ParametersMap allParameters;
	if(!newDatabase && loadDatabaseParameters)
	{
		DBDriver * driver = DBDriver::create();
		if(driver->openConnection(_databasePath, false))
		{
			allParameters = driver->getLastParameters();
			// ignore working directory (we may be on a different computer)
			allParameters.erase(Parameters::kRtabmapWorkingDirectory());
		}
		delete driver;
	}

	uInsert(allParameters, parameters);
	ParametersMap::const_iterator iter;
	if((iter=allParameters.find(Parameters::kRtabmapWorkingDirectory())) != allParameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}

	// If doesn't exist, create a memory
	if(!_memory)
	{
		_memory = new Memory(allParameters);
		_memory->init(_databasePath, false, allParameters, true, preboundEmptyIsNew);
	}

	_optimizedPoses.clear();
	_constraints.clear();
	_globalScanMap.clear();
	_globalScanMapPoses.clear();
	_odomCachePoses.clear();
	_odomCacheConstraints.clear();
	_nodesToRepublish.clear();

	// Parse all parameters
	this->parseParameters(allParameters);

	Transform lastPose;
	_optimizedPoses = _memory->loadOptimizedPoses(&lastPose);
	if(!_memory->isIncremental())
	{
		if(_optimizedPoses.empty() &&
			_memory->getWorkingMem().size()>1 &&
			_memory->getWorkingMem().lower_bound(1)!=_memory->getWorkingMem().end())
		{
			cv::Mat cov;
			this->optimizeCurrentMap(
					_memory->getLastWorkingSignature(true)->id(),
					false, _optimizedPoses, cov, &_constraints);
		}
		if(_optimizedPoses.lower_bound(1) != _optimizedPoses.end())
		{
			if(_restartAtOrigin)
			{
				UWARN("last localization pose is ignored (%s=true), assuming we start at the first node of the map.", Parameters::kRGBDStartAtOrigin().c_str());
				lastPose = _optimizedPoses.lower_bound(1)->second;
			}
			_lastLocalizationPose = lastPose;

			UINFO("Loaded optimizedPoses=%d firstPose %d=%s lastLocalizationPose=%s",
					_optimizedPoses.size(),
					_optimizedPoses.lower_bound(1)->first,
					_optimizedPoses.lower_bound(1)->second.prettyPrint().c_str(),
					_lastLocalizationPose.prettyPrint().c_str());

			if(_constraints.empty())
			{
				std::map<int, Transform> tmp;
				// Get just the links
				_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false, true);
			}

			// Initialize Bayes' prediction matrix
			UTimer time;
			std::map<int, float> likelihood;
			likelihood.insert(std::make_pair(Memory::kIdVirtual, 1));
			for(std::map<int, Transform>::iterator iter=_optimizedPoses.lower_bound(1); iter!=_optimizedPoses.end(); ++iter)
			{
				if(_memory->getSignature(iter->first))
				{
					likelihood.insert(std::make_pair(iter->first, 0));
				}
			}
			_bayesFilter->computePosterior(_memory, likelihood);
			UINFO("Time initializing Bayes' prediction with %ld nodes: %fs", _optimizedPoses.size(), time.ticks());

			if(_createGlobalScanMap)
				createGlobalScanMap();
		}
		else
		{
			UINFO("Loaded optimizedPoses=0, last localization pose is ignored!");
		}
	}
	else
	{
		_lastLocalizationPose = lastPose;
		if(!_optimizedPoses.empty())
		{
			std::map<int, Transform> tmp;
			// Get just the links
			_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false, true);
		}
	}

	// Preserve the complete saved optimized graph for future continuation
	// transactions.  _constraints can be only the resident WM subset, so fetch
	// links for every saved optimized node from Memory's already-open database.
	// This read does not reactivate signatures and is finished before any live
	// proposal can be received.
	_safeRelocalizationTrustedPoses.clear();
	_safeRelocalizationTrustedConstraints.clear();
	_safeRelocalizationTrustedComponents.clear();
	_safeRelocalizationTrustedSnapshotValid = true;
	for(std::map<int, Transform>::const_iterator iter=_optimizedPoses.begin();
			iter!=_optimizedPoses.end(); ++iter)
	{
		if(iter->first > 0)
		{
			if(safeRigidTransform(iter->second))
			{
				_safeRelocalizationTrustedPoses.insert(*iter);
			}
			else
			{
				_safeRelocalizationTrustedSnapshotValid = false;
			}
		}
	}
	if(!_safeRelocalizationTrustedPoses.empty())
	{
		std::map<int, Transform> ignoredRawPoses;
		_memory->getMetricConstraints(
				uKeysSet(_safeRelocalizationTrustedPoses),
				ignoredRawPoses,
				_safeRelocalizationTrustedConstraints,
				true,
				false);
		rebuildSafeRelocalizationTrustedComponents();
	}

	if(_databasePath.empty())
	{
		_statisticLogged = false;
	}
	setupLogFiles(newDatabase);
}

void Rtabmap::init(const std::string & configFile, const std::string & databasePath, bool loadDatabaseParameters)
{
	this->init(configFile, databasePath, loadDatabaseParameters, false);
}

void Rtabmap::init(const std::string & configFile, const std::string & databasePath, bool loadDatabaseParameters, bool preboundEmptyIsNew)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("Rtabmap::init(config) is disabled after the safe-relocalization process-lifetime latch is enabled");
		return;
	}
	// fill ctrl struct with values from the configuration file
	ParametersMap param;// = Parameters::defaultParameters;

	if(!configFile.empty())
	{
		ULOGGER_DEBUG("Read parameters from = %s", configFile.c_str());
		Parameters::readINI(configFile, param);
	}

	this->init(param, databasePath, loadDatabaseParameters, preboundEmptyIsNew);
}

void Rtabmap::close(bool databaseSaved, const std::string & ouputDatabasePath)
{
	UINFO("databaseSaved=%d", databaseSaved?1:0);
	// A reset epoch which has not committed a verified bridge has its own
	// coordinate frame. Saving its live poses over Admin's old optimized graph
	// would make the trusted map disappear on reopening, even though Memory
	// still retains all raw signatures and links. Freeze the trusted graph
	// before clearing the epoch state below; never union unaligned frames.
	const bool saveTrustedSnapshot = databaseSaved &&
			_safeRelocalizationGuardEnabled && _resetEpoch > 0 &&
			_safeRelocalizationCommittedEpochs.find(_resetEpoch) ==
					_safeRelocalizationCommittedEpochs.end() &&
			_safeRelocalizationTrustedSnapshotValid &&
			!_safeRelocalizationTrustedPoses.empty();
	std::map<int, Transform> trustedPosesToSave;
	if(saveTrustedSnapshot)
	{
		trustedPosesToSave = _safeRelocalizationTrustedPoses;
	}
	_highestHypothesis = std::make_pair(0,0.0f);
	_loopClosureHypothesis = std::make_pair(0,0.0f);
	_crossMapLoopEvidence.clear();
	_resetEpoch = 0;
	_signatureResetEpochs.clear();
	_acceptedImageReceipts.clear();
	_lastAcceptedImageStampNs = 0;
	_skipNextOdomResetDetection = false;
	_safeRelocalizationTrustedPoses.clear();
	_safeRelocalizationTrustedConstraints.clear();
	_safeRelocalizationTrustedComponents.clear();
	_safeRelocalizationTrustedSnapshotValid = false;
	_safeRelocalizationCommittedEpochs.clear();
	_safeRelocalizationPromotionLimitsValid = false;
	_safeRelocalizationPromotionMaxOldPoseTranslation = 0.0f;
	_safeRelocalizationPromotionMaxOldPoseRotation = 0.0f;
	_lastProcessTime = 0.0;
	_someNodesHaveBeenTransferred = false;
	_constraints.clear();
	_mapCorrection.setIdentity();
	_mapCorrectionBackup.setNull();

	_localizationCovariance = cv::Mat();
	_lastLocalizationNodeId = 0;
	_odomCachePoses.clear();
	_odomCacheConstraints.clear();
	_distanceTravelled = 0.0f;
	_distanceTravelledSinceLastLocalization = 0.0f;
	_optimizeFromGraphEndChanged = false;
	if(_safeRelocalizationGuardEnabled)
	{
		// close() is the terminal lifecycle transition. Clear transient path
		// state directly so the public mutation guard remains latched and the
		// shutdown path does not report a false runtime violation.
		_pathStatus = 0;
		_path.clear();
		_pathCurrentIndex = 0;
		_pathGoalIndex = 0;
		_pathTransformToGoal.setIdentity();
		_pathUnreachableNodes.clear();
		_pathStuckCount = 0;
		_pathStuckDistance = 0.0f;
		if(_memory)
		{
			_memory->removeAllVirtualLinks();
		}
	}
	else
	{
		this->clearPath(0);
	}
	_gpsGeocentricCache.clear();
	_currentSessionHasGPS = false;

	_globalScanMap.clear();
	_globalScanMapPoses.clear();

	_nodesToRepublish.clear();

	flushStatisticLogs();
	if(_foutFloat)
	{
		fclose(_foutFloat);
		_foutFloat = 0;
	}
	if(_foutInt)
	{
		fclose(_foutInt);
		_foutInt = 0;
	}

	if(_epipolarGeometry)
	{
		delete _epipolarGeometry;
		_epipolarGeometry = 0;
	}
	if(_memory)
	{
		if(_memory->isReadOnly() && databaseSaved)
		{
			UWARN("Database is read-only, latest optimized poses, latest localization pose and latest state of the memory are not saved.");
			databaseSaved = false;
		}
		if(databaseSaved)
		{
			if(_memory->isGraphReduced() && _memory->isIncremental())
			{
				// Force reducing graph, then remove filtered nodes from the optimized poses
				std::map<int, int> reducedIds;
				_memory->incrementMapId(&reducedIds);
				for(std::map<int, int>::iterator iter=reducedIds.begin(); iter!=reducedIds.end(); ++iter)
				{
					_optimizedPoses.erase(iter->first);
					trustedPosesToSave.erase(iter->first);
				}
			}
			if(saveTrustedSnapshot)
			{
				// The current robot pose is unknown in the saved trusted frame.
				// An arbitrary old node pose would falsely claim localization.
				_memory->saveOptimizedPoses(trustedPosesToSave, Transform());
			}
			else
			{
				_memory->saveOptimizedPoses(_optimizedPoses, _lastLocalizationPose);
			}
		}
		_memory->close(databaseSaved, true, ouputDatabasePath);
		delete _memory;
		_memory = 0;
	}
	_optimizedPoses.clear();
	_lastLocalizationPose.setNull();

	if(_bayesFilter)
	{
		delete _bayesFilter;
		_bayesFilter = 0;
	}
	if(_graphOptimizer)
	{
		delete _graphOptimizer;
		_graphOptimizer = 0;
	}
	_databasePath.clear();
	// The guarded instance is closing and will be destroyed. Resetting defaults
	// is unnecessary and would correctly trip the process-lifetime mutation
	// latch, producing a misleading error after a successful database close.
	if(!_safeRelocalizationGuardEnabled)
	{
		parseParameters(Parameters::getDefaultParameters()); // reset to default parameters
	}
	_parameters.clear();
}

void Rtabmap::parseParameters(const ParametersMap & parameters)
{
	if(_safeRelocalizationGuardEnabled)
	{
		// The transaction proof binds the complete parameter snapshot.  Reject
		// the whole batch before uInsert() or any derived member changes; even a
		// seemingly harmless mixed batch must not partially mutate the proof
		// premises after the guard has latched.
		UERROR("Rtabmap::parseParameters is disabled after the safe-relocalization process-lifetime latch is enabled");
		return;
	}
	uInsert(_parameters, parameters);

	// place this before changing working directory
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLogsBufferedInRAM(), _statisticLogsBufferedInRAM);
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLogged(), _statisticLogged);
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLoggedHeaders(), _statisticLoggedHeaders);

	ULOGGER_DEBUG("");
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kRtabmapWorkingDirectory())) != parameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}

	Parameters::parse(parameters, Parameters::kRtabmapPublishStats(), _publishStats);
	Parameters::parse(parameters, Parameters::kRtabmapPublishLastSignature(), _publishLastSignatureData);
	Parameters::parse(parameters, Parameters::kRtabmapPublishPdf(), _publishPdf);
	Parameters::parse(parameters, Parameters::kRtabmapPublishLikelihood(), _publishLikelihood);
	Parameters::parse(parameters, Parameters::kRtabmapPublishRAMUsage(), _publishRAMUsage);
	Parameters::parse(parameters, Parameters::kRtabmapComputeRMSE(), _computeRMSE);
	Parameters::parse(parameters, Parameters::kRtabmapSaveWMState(), _saveWMState);
	Parameters::parse(parameters, Parameters::kRtabmapTimeThr(), _maxTimeAllowed);
	Parameters::parse(parameters, Parameters::kRtabmapMemoryThr(), _maxMemoryAllowed);
	Parameters::parse(parameters, Parameters::kRtabmapLoopThr(), _loopThr);
	Parameters::parse(parameters, Parameters::kRtabmapLoopRatio(), _loopRatio);
	Parameters::parse(parameters, Parameters::kRGBDAggressiveLoopThr(), _aggressiveLoopThr);
	Parameters::parse(parameters, Parameters::kRtabmapVirtualPlaceLikelihoodRatio(), _virtualPlaceLikelihoodRatio);

	Parameters::parse(parameters, Parameters::kRGBDMaxLoopClosureDistance(), _maxLoopClosureDistance);
	Parameters::parse(parameters, Parameters::kVhEpEnabled(), _verifyLoopClosureHypothesis);
	Parameters::parse(parameters, Parameters::kRtabmapMaxRetrieved(), _maxRetrieved);
	Parameters::parse(parameters, Parameters::kRGBDMaxLocalRetrieved(), _maxLocalRetrieved);
	Parameters::parse(parameters, Parameters::kRtabmapMaxRepublished(), _maxRepublished);
	if(_maxRepublished == 0 || !_publishLastSignatureData)
	{
		_nodesToRepublish.clear();
	}
	Parameters::parse(parameters, Parameters::kMemImageKept(), _rawDataKept);
	Parameters::parse(parameters, Parameters::kRGBDEnabled(), _rgbdSlamMode);
	Parameters::parse(parameters, Parameters::kRGBDLinearUpdate(), _rgbdLinearUpdate);
	Parameters::parse(parameters, Parameters::kRGBDAngularUpdate(), _rgbdAngularUpdate);
	Parameters::parse(parameters, Parameters::kRGBDLinearSpeedUpdate(), _rgbdLinearSpeedUpdate);
	Parameters::parse(parameters, Parameters::kRGBDAngularSpeedUpdate(), _rgbdAngularSpeedUpdate);
	Parameters::parse(parameters, Parameters::kRGBDNewMapOdomChangeDistance(), _newMapOdomChangeDistance);
	Parameters::parse(parameters, Parameters::kRGBDNeighborLinkRefining(), _neighborLinkRefining);
	Parameters::parse(parameters, Parameters::kRGBDProximityByTime(), _proximityByTime);
	Parameters::parse(parameters, Parameters::kRGBDProximityBySpace(), _proximityBySpace);
	Parameters::parse(parameters, Parameters::kRGBDScanMatchingIdsSavedInLinks(), _scanMatchingIdsSavedInLinks);
	Parameters::parse(parameters, Parameters::kRGBDLoopClosureIdentityGuess(), _loopClosureIdentityGuess);
	Parameters::parse(parameters, Parameters::kRGBDLocalRadius(), _localRadius);
	Parameters::parse(parameters, Parameters::kRGBDLocalImmunizationRatio(), _localImmunizationRatio);
	Parameters::parse(parameters, Parameters::kRGBDProximityMaxGraphDepth(), _proximityMaxGraphDepth);
	Parameters::parse(parameters, Parameters::kRGBDProximityMaxPaths(), _proximityMaxPaths);
	Parameters::parse(parameters, Parameters::kRGBDProximityPathMaxNeighbors(), _proximityMaxNeighbors);
	Parameters::parse(parameters, Parameters::kRGBDProximityPathFilteringRadius(), _proximityFilteringRadius);
	Parameters::parse(parameters, Parameters::kRGBDProximityPathRawPosesUsed(), _proximityRawPosesUsed);
	if(Parameters::parse(parameters, Parameters::kRGBDProximityAngle(), _proximityAngle))
	{
		_proximityAngle *= M_PI/180.0f;
	}
	Parameters::parse(parameters, Parameters::kRGBDProximityOdomGuess(), _proximityOdomGuess);
	Parameters::parse(parameters, Parameters::kRGBDProximityMergedScanCovFactor(), _proximityMergedScanCovFactor);
	UASSERT(_proximityMergedScanCovFactor>0.0);

	bool optimizeFromGraphEndPrevious = _optimizeFromGraphEnd;
	Parameters::parse(parameters, Parameters::kRGBDOptimizeFromGraphEnd(), _optimizeFromGraphEnd);
	if(optimizeFromGraphEndPrevious != _optimizeFromGraphEnd && !_optimizedPoses.empty())
	{
		_optimizeFromGraphEndChanged = true;
	}
	Parameters::parse(parameters, Parameters::kRGBDOptimizeMaxError(), _optimizationMaxError);
	Parameters::parse(parameters, Parameters::kRGBDOptimizeMaxErrorRepairRadius(), _optimizationMaxErrorRepairRadius);
	Parameters::parse(parameters, Parameters::kRtabmapStartNewMapOnLoopClosure(), _startNewMapOnLoopClosure);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopPending(), _crossMapLoopPending);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopPendingCount(), _crossMapLoopPendingCount);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopPendingMaxIdSpan(), _crossMapLoopPendingMaxIdSpan);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopMinInliers(), _crossMapLoopMinInliers);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopMinGridCells(), _crossMapLoopMinGridCells);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopMin3DSpan(), _crossMapLoopMin3DSpan);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopMinHessianRatio(), _crossMapLoopMinHessianRatio);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopMaxTranslation(), _crossMapLoopMaxTranslation);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopMaxRotation(), _crossMapLoopMaxRotation);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopMaxNormalizedError(), _crossMapLoopMaxNormalizedError);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopMaxFeatureJaccard(), _crossMapLoopMaxFeatureJaccard);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopMinSourceTranslation(), _crossMapLoopMinSourceTranslation);
	Parameters::parse(parameters, Parameters::kRtabmapCrossMapLoopMinSourceRotation(), _crossMapLoopMinSourceRotation);
	UASSERT(_crossMapLoopPendingCount >= 2);
	UASSERT(_crossMapLoopPendingMaxIdSpan >= _crossMapLoopPendingCount-1);
	UASSERT(_crossMapLoopMinInliers >= 3);
	UASSERT(_crossMapLoopMinGridCells >= 1 && _crossMapLoopMinGridCells <= 9);
	UASSERT(_crossMapLoopMin3DSpan >= 0.0f);
	UASSERT(_crossMapLoopMinHessianRatio >= 0.0f && _crossMapLoopMinHessianRatio <= 1.0f);
	UASSERT(_crossMapLoopMaxTranslation > 0.0f);
	UASSERT(_crossMapLoopMaxRotation > 0.0f);
	UASSERT(_crossMapLoopMaxNormalizedError > 0.0f);
	UASSERT(_crossMapLoopMaxFeatureJaccard >= 0.0f && _crossMapLoopMaxFeatureJaccard < 1.0f);
	UASSERT(_crossMapLoopMinSourceTranslation >= 0.0f);
	UASSERT(_crossMapLoopMinSourceRotation >= 0.0f && _crossMapLoopMinSourceRotation <= 180.0f);
	UASSERT(_crossMapLoopMinSourceTranslation > 0.0f || _crossMapLoopMinSourceRotation > 0.0f);
	if(!_crossMapLoopPending)
	{
		_crossMapLoopEvidence.clear();
	}
	Parameters::parse(parameters, Parameters::kRtabmapStartNewMapOnGoodSignature(), _startNewMapOnGoodSignature);
	Parameters::parse(parameters, Parameters::kRGBDGoalReachedRadius(), _goalReachedRadius);
	Parameters::parse(parameters, Parameters::kRGBDGoalsSavedInUserData(), _goalsSavedInUserData);
	Parameters::parse(parameters, Parameters::kRGBDPlanStuckIterations(), _pathStuckIterations);
	Parameters::parse(parameters, Parameters::kRGBDPlanLinearVelocity(), _pathLinearVelocity);
	Parameters::parse(parameters, Parameters::kRGBDPlanAngularVelocity(), _pathAngularVelocity);
	Parameters::parse(parameters, Parameters::kRGBDForceOdom3DoF(), _forceOdom3doF);
	Parameters::parse(parameters, Parameters::kRGBDStartAtOrigin(), _restartAtOrigin);
	Parameters::parse(parameters, Parameters::kRGBDLoopCovLimited(), _loopCovLimited);
	Parameters::parse(parameters, Parameters::kRtabmapLoopGPS(), _loopGPS);
	Parameters::parse(parameters, Parameters::kRGBDMaxOdomCacheSize(), _maxOdomCacheSize);
	Parameters::parse(parameters, Parameters::kRGBDLocalizationSmoothing(), _localizationSmoothing);
	double localizationPriorError = Parameters::defaultRGBDLocalizationPriorError();
	Parameters::parse(parameters, Parameters::kRGBDLocalizationPriorError(), localizationPriorError);
	UASSERT(localizationPriorError>0.0);
	_localizationPriorInf = 1.0/(localizationPriorError*localizationPriorError);
	Parameters::parse(parameters, Parameters::kRGBDLocalizationSecondTryWithoutProximityLinks(), _localizationSecondTryWithoutProximityLinks);
	Parameters::parse(parameters, Parameters::kRGBDProximityGlobalScanMap(), _createGlobalScanMap);

	Parameters::parse(parameters, Parameters::kMarkerPriorsVarianceLinear(), _markerPriorsLinearVariance);
	UASSERT(_markerPriorsLinearVariance>0.0f);
	Parameters::parse(parameters, Parameters::kMarkerPriorsVarianceAngular(), _markerPriorsAngularVariance);
	UASSERT(_markerPriorsAngularVariance>0.0f);
	std::string markerPriorsStr;
	if(Parameters::parse(parameters, Parameters::kMarkerPriors(), markerPriorsStr))
	{
		_markerPriors.clear();
		std::list<std::string> strList = uSplit(markerPriorsStr, '|');
		for(std::list<std::string>::iterator iter=strList.begin(); iter!=strList.end(); ++iter)
		{
			std::string markerStr = *iter;
			while(!markerStr.empty() && !uIsDigit(markerStr[0]))
			{
				markerStr.erase(markerStr.begin());
			}
			if(!markerStr.empty())
			{
				std::string idStr = uSplitNumChar(markerStr).front();
				int id = uStr2Int(idStr);
				Transform prior = Transform::fromString(markerStr.substr(idStr.size()));
				if(!prior.isNull() && id>0)
				{
					_markerPriors.insert(std::make_pair(-id, prior));
					UDEBUG("Added landmark prior %d: %s", id, prior.prettyPrint().c_str());
				}
				else
				{
					UERROR("Failed to parse element \"%s\" in parameter %s", markerStr.c_str(), Parameters::kMarkerPriors().c_str());
				}
			}
			else if(!iter->empty())
			{
				UERROR("Failed to parse parameter %s, value=\"%s\"", Parameters::kMarkerPriors().c_str(), iter->c_str());
			}
		}
	}

	UASSERT(_rgbdLinearUpdate >= 0.0f);
	UASSERT(_rgbdAngularUpdate >= 0.0f);
	UASSERT(_rgbdLinearSpeedUpdate >= 0.0f);
	UASSERT(_rgbdAngularSpeedUpdate >= 0.0f);
	UASSERT(_maxOdomCacheSize >= 0);

	// By default, we create our strategies if they are not already created.
	// If they already exists, we check the parameters if a change is requested

	// Graph optimizer
	Optimizer::Type optimizerType = Optimizer::kTypeUndef;
	if((iter=parameters.find(Parameters::kOptimizerStrategy())) != parameters.end())
	{
		optimizerType = (Optimizer::Type)std::atoi((*iter).second.c_str());
	}
	if(optimizerType!=Optimizer::kTypeUndef)
	{
		UDEBUG("new detector strategy %d", int(optimizerType));
		if(_graphOptimizer)
		{
			delete _graphOptimizer;
			_graphOptimizer = 0;
		}

		_graphOptimizer = Optimizer::create(optimizerType, _parameters);
	}
	else if(_graphOptimizer)
	{
		_graphOptimizer->parseParameters(parameters);
	}
	else
	{
		optimizerType = (Optimizer::Type)Parameters::defaultOptimizerStrategy();
		_graphOptimizer = Optimizer::create(optimizerType, parameters);
	}

	if(!_createGlobalScanMap)
	{
		_globalScanMap.clear();
		_globalScanMapPoses.clear();
	}

	if(_memory)
	{
		bool isMemIncremental = _memory->isIncremental();
		if(Parameters::parse(parameters, Parameters::kMemIncrementalMemory(), isMemIncremental) &&
			isMemIncremental != _memory->isIncremental())
		{
			// Mode has changed from Mapping to Localization, cleanup the local graph
			if(_memory->isIncremental())
			{
				if(_memory->isGraphReduced())
				{
					// Force reducing graph, then remove filtered nodes from the optimized poses
					std::map<int, int> reducedIds;
					_memory->incrementMapId(&reducedIds);
					for(std::map<int, int>::iterator iter=reducedIds.begin(); iter!=reducedIds.end(); ++iter)
					{
						_optimizedPoses.erase(iter->first);
					}
				}
				_odomCachePoses.clear();
				_odomCacheConstraints.clear();
			}

			// In both cases, we save the latest optimized graph and latest localization pose
			_memory->saveOptimizedPoses(_optimizedPoses, _lastLocalizationPose);
		}

		_memory->parseParameters(parameters);
		if(_memory->isIncremental() && !_globalScanMap.empty())
		{
			UWARN("Map is now incremental, clearing global scan map...");
			_globalScanMap.clear();
			_globalScanMapPoses.clear();
		}

		if(_createGlobalScanMap && !_memory->isIncremental() && _globalScanMap.empty() && !_optimizedPoses.empty())
		{
			this->createGlobalScanMap();
		}
	}

	if(!_epipolarGeometry)
	{
		_epipolarGeometry = new EpipolarGeometry(_parameters);
	}
	else
	{
		_epipolarGeometry->parseParameters(parameters);
	}

	// Bayes filter, create one if not exists
	if(!_bayesFilter)
	{
		_bayesFilter = new BayesFilter(_parameters);
	}
	else
	{
		_bayesFilter->parseParameters(parameters);
	}
}

bool Rtabmap::acceptCrossMapLoopEvidence(
		const Signature & current,
		const Signature & old,
		unsigned long long currentResetEpoch,
		unsigned long long oldResetEpoch,
		const Transform & currentToOld,
		const RegistrationInfo & info,
		std::string & reason)
{
	reason.clear();
	if(current.mapId() == old.mapId() && currentResetEpoch == oldResetEpoch)
	{
		return true;
	}
	if(info.inliers < _crossMapLoopMinInliers ||
			(int)info.inliersIDs.size() < _crossMapLoopMinInliers)
	{
		reason = uFormat("inliers=%d ids=%d minimum=%d",
				info.inliers, (int)info.inliersIDs.size(), _crossMapLoopMinInliers);
		return false;
	}
	if(!finiteTransform(current.getPose()) || !finiteTransform(old.getPose()) ||
			!finiteTransform(currentToOld))
	{
		reason = "null or non-finite pose/transform";
		return false;
	}

	std::set<int> inlierWords(info.inliersIDs.begin(), info.inliersIDs.end());
	if((int)inlierWords.size() < _crossMapLoopMinInliers)
	{
		reason = uFormat("unique inlier words=%d minimum=%d",
				(int)inlierWords.size(), _crossMapLoopMinInliers);
		return false;
	}

	int currentWidth=0;
	int currentHeight=0;
	int oldWidth=0;
	int oldHeight=0;
	if(!signatureImageSize(current, currentWidth, currentHeight) ||
			!signatureImageSize(old, oldWidth, oldHeight))
	{
		reason = "missing image dimensions";
		return false;
	}
	int currentCells = occupiedGridCells(current, inlierWords, currentWidth, currentHeight);
	int oldCells = occupiedGridCells(old, inlierWords, oldWidth, oldHeight);
	if(currentCells < _crossMapLoopMinGridCells || oldCells < _crossMapLoopMinGridCells)
	{
		reason = uFormat("3x3 grid coverage current=%d old=%d minimum=%d",
				currentCells, oldCells, _crossMapLoopMinGridCells);
		return false;
	}

	std::vector<cv::Point3f> currentPoints = validInlierPoints(current, inlierWords);
	std::vector<cv::Point3f> oldPoints = validInlierPoints(old, inlierWords);
	int minimum3DPoints = std::max(6, _crossMapLoopMinInliers/2);
	if((int)currentPoints.size() < minimum3DPoints || (int)oldPoints.size() < minimum3DPoints)
	{
		reason = uFormat("valid 3D inliers current=%d old=%d minimum=%d",
				(int)currentPoints.size(), (int)oldPoints.size(), minimum3DPoints);
		return false;
	}
	double currentSpan=0.0;
	double currentHessianRatio=0.0;
	double oldSpan=0.0;
	double oldHessianRatio=0.0;
	if(!pointSetGeometry(currentPoints, currentSpan, currentHessianRatio) ||
			!pointSetGeometry(oldPoints, oldSpan, oldHessianRatio))
	{
		reason = "cannot compute finite 3D geometry";
		return false;
	}
	if(currentSpan < _crossMapLoopMin3DSpan || oldSpan < _crossMapLoopMin3DSpan)
	{
		reason = uFormat("3D second-axis span current=%f old=%f minimum=%f",
				currentSpan, oldSpan, _crossMapLoopMin3DSpan);
		return false;
	}
	if(currentHessianRatio < _crossMapLoopMinHessianRatio ||
			oldHessianRatio < _crossMapLoopMinHessianRatio)
	{
		reason = uFormat("6DoF Hessian ratio current=%g old=%g minimum=%g",
				currentHessianRatio, oldHessianRatio, _crossMapLoopMinHessianRatio);
		return false;
	}

	double linearVariance=0.0;
	double angularVariance=0.0;
	if(!covarianceVariances(info.covariance, linearVariance, angularVariance))
	{
		reason = "invalid or unconstrained covariance";
		return false;
	}

	CrossMapLoopEvidence evidence;
	evidence.newMapId = current.mapId();
	evidence.oldMapId = old.mapId();
	evidence.newResetEpoch = currentResetEpoch;
	evidence.oldResetEpoch = oldResetEpoch;
	evidence.newId = current.id();
	evidence.oldId = old.id();
	evidence.sourcePose = current.getPose();
	evidence.anchor = current.getPose() * currentToOld * old.getPose().inverse();
	evidence.linearVariance = linearVariance;
	evidence.angularVariance = angularVariance;
	evidence.inlierWords = inlierWords;
	evidence.featureJaccardMaximum = 0.0;
	if(!finiteTransform(evidence.anchor))
	{
		reason = "non-finite fixed-anchor transform";
		return false;
	}

	if(!_crossMapLoopEvidence.empty() &&
			(_crossMapLoopEvidence.front().newMapId != evidence.newMapId ||
			 _crossMapLoopEvidence.front().oldMapId != evidence.oldMapId ||
			 _crossMapLoopEvidence.front().newResetEpoch != evidence.newResetEpoch ||
			 _crossMapLoopEvidence.front().oldResetEpoch != evidence.oldResetEpoch))
	{
		_crossMapLoopEvidence.clear();
	}
	while(!_crossMapLoopEvidence.empty() &&
			evidence.newId-_crossMapLoopEvidence.front().newId > _crossMapLoopPendingMaxIdSpan)
	{
		_crossMapLoopEvidence.erase(_crossMapLoopEvidence.begin());
	}

	for(std::vector<CrossMapLoopEvidence>::const_iterator iter=_crossMapLoopEvidence.begin();
			iter!=_crossMapLoopEvidence.end(); ++iter)
	{
		if(iter->newId == evidence.newId)
		{
			reason = uFormat("duplicate current keyframe id=%d", evidence.newId);
			return false;
		}
		double sourceTranslation = iter->sourcePose.getDistance(evidence.sourcePose);
		double sourceRotation = iter->sourcePose.getAngle(evidence.sourcePose);
		if(sourceTranslation < _crossMapLoopMinSourceTranslation &&
				sourceRotation < _crossMapLoopMinSourceRotation*M_PI/180.0)
		{
			reason = uFormat("source view is not independent: d=%fm a=%fdeg minimum=(%fm or %fdeg)",
					sourceTranslation, sourceRotation*180.0/M_PI,
					_crossMapLoopMinSourceTranslation, _crossMapLoopMinSourceRotation);
			return false;
		}
		evidence.featureJaccardMaximum = std::max(
				evidence.featureJaccardMaximum,
				featureJaccard(iter->inlierWords, inlierWords));

		double translationError = iter->anchor.getDistance(evidence.anchor);
		double rotationError = iter->anchor.getAngle(evidence.anchor);
		double normalizedTranslation = translationError /
				std::sqrt(iter->linearVariance+evidence.linearVariance);
		double normalizedRotation = rotationError /
				std::sqrt(iter->angularVariance+evidence.angularVariance);
		if(!std::isfinite(normalizedTranslation) || !std::isfinite(normalizedRotation) ||
				translationError > _crossMapLoopMaxTranslation ||
				rotationError > _crossMapLoopMaxRotation*M_PI/180.0 ||
				normalizedTranslation > _crossMapLoopMaxNormalizedError ||
				normalizedRotation > _crossMapLoopMaxNormalizedError)
		{
			_crossMapLoopEvidence.clear();
			_crossMapLoopEvidence.push_back(evidence);
			reason = uFormat("inconsistent fixed-anchor transform: d=%fm a=%fdeg normalized=(%f,%f); restarted pending window 1/%d",
					translationError, rotationError*180.0/M_PI,
					normalizedTranslation, normalizedRotation, _crossMapLoopPendingCount);
			return false;
		}
	}

	_crossMapLoopEvidence.push_back(evidence);
	if((int)_crossMapLoopEvidence.size() < _crossMapLoopPendingCount)
	{
		reason = uFormat("consistent view-diverse evidence %d/%d (current=%d old=%d maps=%d->%d epochs=%llu->%llu max_word_jaccard=%f diagnostic_only)",
				(int)_crossMapLoopEvidence.size(), _crossMapLoopPendingCount,
				current.id(), old.id(), current.mapId(), old.mapId(),
				currentResetEpoch, oldResetEpoch, evidence.featureJaccardMaximum);
		return false;
	}

	reason = uFormat("accepted %d consistent independent observations over id span %d",
			(int)_crossMapLoopEvidence.size(),
			_crossMapLoopEvidence.back().newId-_crossMapLoopEvidence.front().newId);
	_crossMapLoopEvidence.clear();
	return true;
}

int Rtabmap::getLastLocationId() const
{
	int id = 0;
	if(_memory)
	{
		id = _memory->getLastSignatureId();
	}
	return id;
}

std::list<int> Rtabmap::getWM() const
{
	std::list<int> mem;
	if(_memory)
	{
		mem = uKeysList(_memory->getWorkingMem());
		mem.remove(-1);// Ignore the virtual signature (if here)
	}
	return mem;
}

int Rtabmap::getWMSize() const
{
	if(_memory)
	{
		return (int)_memory->getWorkingMem().size()-1; // remove virtual place
	}
	return 0;
}

std::map<int, int> Rtabmap::getWeights() const
{
	std::map<int, int> weights;
	if(_memory)
	{
		weights = _memory->getWeights();
		weights.erase(-1);// Ignore the virtual signature (if here)
	}
	return weights;
}

std::set<int> Rtabmap::getSTM() const
{
	if(_memory)
	{
		return _memory->getStMem();
	}
	return std::set<int>();
}

int Rtabmap::getSTMSize() const
{
	if(_memory)
	{
		return (int)_memory->getStMem().size();
	}
	return 0;
}

int Rtabmap::getTotalMemSize() const
{
	if(_memory)
	{
		const Signature * s  =_memory->getLastWorkingSignature(false);
		if(s)
		{
			return s->id();
		}
	}
	return 0;
}

bool Rtabmap::isInSTM(int locationId) const
{
	if(_memory)
	{
		return _memory->isInSTM(locationId);
	}
	return false;
}

bool Rtabmap::isIDsGenerated() const
{
	if(_memory)
	{
		return _memory->isIDsGenerated();
	}
	return Parameters::defaultMemGenerateIds();
}

const Statistics & Rtabmap::getStatistics() const
{
	return statistics_;
}

Transform Rtabmap::getPose(int locationId) const
{
	return uValue(_optimizedPoses, locationId, Transform());
}

void Rtabmap::setInitialPose(const Transform & initialPose)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("setInitialPose is disabled while safe relocalization owns graph alignment");
		return;
	}
	if(_memory)
	{
		if(!_memory->isIncremental())
		{
			_lastLocalizationPose = initialPose;
			_localizationCovariance = cv::Mat();
			_lastLocalizationNodeId = 0;
			_odomCachePoses.clear();
			_odomCacheConstraints.clear();
			_mapCorrection.setIdentity();
			_mapCorrectionBackup.setNull();

			if(_memory->getLastWorkingSignature(true)->id() &&
				_optimizedPoses.empty())
			{
				cv::Mat covariance;
				this->optimizeCurrentMap(_memory->getLastWorkingSignature(true)->id(), false, _optimizedPoses, covariance, &_constraints);
			}
		}
		else
		{
			UWARN("Initial pose can only be set in localization mode (%s=false), ignoring it...", Parameters::kMemIncrementalMemory().c_str());
		}
	}
}

int Rtabmap::triggerNewMap()
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("triggerNewMap is disabled while safe relocalization owns graph mutations");
		return -1;
	}
	return triggerNewMapImpl();
}

int Rtabmap::triggerNewMapImpl()
{
	int mapId = -1;
	if(_memory)
	{
		_localizationCovariance = cv::Mat();
		_lastLocalizationNodeId = 0;
		_odomCachePoses.clear();
		_odomCacheConstraints.clear();
		_distanceTravelled = 0.0f;
		_distanceTravelledSinceLastLocalization = 0.0f;

		if(!_memory->isIncremental())
		{
			_mapCorrection.setIdentity();
			if(_restartAtOrigin)
			{
				_lastLocalizationPose.setIdentity();
			}
			return mapId;
		}
		// Epoch zero is the initial/base map and is frozen at its first boundary.
		// A later epoch is first promoted by its guarded three-bridge commit.  It
		// may then grow through ordinary same-epoch mapping before another odometry
		// loss.  Promote that validated continuation at the next boundary, while
		// retaining disconnected trusted components and rejecting missing or
		// rewritten bridge history.  Without this step, nodes added after the first
		// commit disappear from the trusted snapshot at the second reset.
		if(_safeRelocalizationGuardEnabled && _resetEpoch == 0)
		{
			rememberCurrentGraphForSafeRelocalization();
		}
		else if(_safeRelocalizationGuardEnabled &&
				_safeRelocalizationCommittedEpochs.find(_resetEpoch) !=
						_safeRelocalizationCommittedEpochs.end())
		{
			std::string promotionReason;
			if(!promoteCommittedEpochForSafeRelocalization(promotionReason))
			{
				// Promotion is part of the boundary transaction.  Do not mutate the
				// map id, reset epoch or caches when the committed graph cannot be
				// proven identical to the trusted history.
				UERROR("Safe relocalization could not promote committed reset epoch %llu at the next boundary: %s. Refusing the new-map transaction.",
						_resetEpoch, promotionReason.c_str());
				return -1;
			}
		}
		std::map<int, int> reducedIds;
		++_resetEpoch;
		_crossMapLoopEvidence.clear();
		mapId = _memory->incrementMapId(&reducedIds);
		UINFO("New map triggered, new map = %d, reset epoch = %llu", mapId, _resetEpoch);
		_optimizedPoses.clear();
		_constraints.clear();
		_lastRejectedLoopClosureIds = std::make_pair(0,0);

		if(_bayesFilter)
		{
			_bayesFilter->reset();
		}

		//Verify if there are nodes that were merged through graph reduction
		if(reducedIds.size() && _path.size())
		{
			for(unsigned int i=0; i<_path.size(); ++i)
			{
				std::map<int, int>::const_iterator iter = reducedIds.find(_path[i].first);
				if(iter!= reducedIds.end())
				{
					// change path ID to loop closure ID
					_path[i].first = iter->second;
				}
			}
		}
	}
	return mapId;
}

int Rtabmap::triggerNewMapForContinuation()
{
	const int mapId = triggerNewMapImpl();
	if(mapId >= 0 && _memory && _memory->isIncremental())
	{
		// Consumed exactly once by process().  The wrapper calls this method only
		// immediately before submitting the first valid frame of a deliberately
		// separated odometry epoch.
		_skipNextOdomResetDetection = true;
	}
	return mapId;
}

bool Rtabmap::labelLocation(int id, const std::string & label)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("labelLocation is disabled while safe relocalization owns persistent map mutations");
		return false;
	}
	if(_memory)
	{
		if(id > 0)
		{
			return _memory->labelSignature(id, label);
		}
		else if(_memory->isIncremental() && _memory->getLastWorkingSignature(true))
		{
			return _memory->labelSignature(_memory->getLastWorkingSignature(true)->id(), label);
		}
		else if(!_memory->isIncremental() && !_lastLocalizationPose.isNull() && !_lastLocalizationPose.isIdentity())
		{
			std::map<int, Transform> nearestNodes = getNodesInRadius(_lastLocalizationPose, _localRadius, 1);
			if(!nearestNodes.empty())
			{
				return _memory->labelSignature(nearestNodes.begin()->first, label);
			}
			else
			{
				UERROR("No nodes found inside %s=%fm of the current pose (%s). Cannot set label \"%s\"",
						Parameters::kRGBDLocalRadius().c_str(),
						_localRadius,
						_lastLocalizationPose.prettyPrint().c_str(),
						label.c_str());
			}
		}
		else
		{
			UERROR("Last signature is null! Cannot set label \"%s\"", label.c_str());
		}
	}
	return false;
}

bool Rtabmap::setUserData(int id, const cv::Mat & data)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("setUserData is disabled while safe relocalization owns persistent map mutations");
		return false;
	}
	if(_memory)
	{
		if(id > 0)
		{
			return _memory->setUserData(id, data);
		}
		else if(_memory->getLastWorkingSignature(true))
		{
			return _memory->setUserData(_memory->getLastWorkingSignature(true)->id(), data);
		}
		else
		{
			UERROR("Last signature is null! Cannot set user data!");
		}
	}
	return false;
}

void Rtabmap::generateDOTGraph(const std::string & path, int id, int margin)
{
	if(_memory)
	{
		_memory->joinTrashThread(); // make sure the trash is flushed

		if(id > 0)
		{
			std::map<int, int> ids = _memory->getNeighborsId(id, margin, -1, false);

			if(ids.size() > 0)
			{
				ids.insert(std::pair<int,int>(id, 0));
				std::set<int> idsSet;
				for(std::map<int, int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
				{
					idsSet.insert(idsSet.end(), iter->first);
				}
				_memory->generateGraph(path, idsSet);
			}
			else
			{
				UERROR("No neighbors found for signature %d.", id);
			}
		}
		else
		{
			_memory->generateGraph(path);
		}
	}
}

void Rtabmap::exportPoses(const std::string & path, bool optimized, bool global, int format)
{
	if(_memory && _memory->getLastWorkingSignature(!global))
	{
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;

		if(optimized)
		{
			cv::Mat covariance;
			this->optimizeCurrentMap(_memory->getLastWorkingSignature(!global)->id(), global, poses, covariance, &constraints);
		}
		else
		{
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature(!global)->id(), 0, global?-1:0, true);
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, global);
		}

		std::map<int, double> stamps;
		if(format == 1 || format == 10 || format == 11)
		{
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				Transform o,g;
				int m, w;
				std::string l;
				double stamp = 0.0;
				std::vector<float> v;
				GPS gps;
				EnvSensors sensors;
				_memory->getNodeInfo(iter->first, o, m, w, l, stamp, g, v, gps, sensors, true);
				stamps.insert(std::make_pair(iter->first, stamp));
			}
		}

		graph::exportPoses(path, format, poses, constraints, stamps, _parameters);
	}
}

void Rtabmap::resetMemory()
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("resetMemory is disabled while safe relocalization owns graph mutations");
		return;
	}
	UDEBUG("");
	_highestHypothesis = std::make_pair(0,0.0f);
	_loopClosureHypothesis = std::make_pair(0,0.0f);
	_crossMapLoopEvidence.clear();
	_resetEpoch = 0;
	_signatureResetEpochs.clear();
	_acceptedImageReceipts.clear();
	_lastAcceptedImageStampNs = 0;
	_skipNextOdomResetDetection = false;
	_safeRelocalizationTrustedPoses.clear();
	_safeRelocalizationTrustedConstraints.clear();
	_safeRelocalizationTrustedComponents.clear();
	_safeRelocalizationTrustedSnapshotValid = false;
	_safeRelocalizationCommittedEpochs.clear();
	_safeRelocalizationPromotionLimitsValid = false;
	_safeRelocalizationPromotionMaxOldPoseTranslation = 0.0f;
	_safeRelocalizationPromotionMaxOldPoseRotation = 0.0f;
	_lastProcessTime = 0.0;
	_someNodesHaveBeenTransferred = false;
	_optimizedPoses.clear();
	_constraints.clear();
	_mapCorrection.setIdentity();
	_mapCorrectionBackup.setNull();
	_lastLocalizationPose.setNull();
	_localizationCovariance = cv::Mat();
	_lastLocalizationNodeId = 0;
	_odomCachePoses.clear();
	_odomCacheConstraints.clear();
	_distanceTravelled = 0.0f;
	_distanceTravelledSinceLastLocalization = 0.0f;
	_optimizeFromGraphEndChanged = false;
	_globalScanMap.clear();
	_globalScanMapPoses.clear();
	_nodesToRepublish.clear();
	_lastRejectedLoopClosureIds = std::make_pair(0,0);
	this->clearPath(0);

	if(_memory)
	{
		if(_memory->isReadOnly())
		{
			UWARN("Memory is reset but the database won't be cleared because read-only mode is enabled.");
		}
		_memory->init(_databasePath, true, _parameters, true);
		if(_memory->getLastWorkingSignature(true))
		{
			cv::Mat covariance;
			optimizeCurrentMap(_memory->getLastWorkingSignature(true)->id(), false, _optimizedPoses, covariance, &_constraints);
		}
		if(_bayesFilter)
		{
			_bayesFilter->reset();
		}
	}
	else
	{
		UERROR("RTAB-Map is not initialized. No memory to reset...");
	}

	if(_graphOptimizer)
	{
		delete _graphOptimizer;
		_graphOptimizer = Optimizer::create(_parameters);
	}

	this->setupLogFiles(true);
}

class NearestPathKey
{
public:
	NearestPathKey(float l, int i, float d) :
		likelihood(l),
		id(i),
		distance(d){}
	bool operator<(const NearestPathKey & k) const
	{
		if(likelihood < k.likelihood)
		{
			return true;
		}
		else if(likelihood == k.likelihood)
		{
			if(distance > k.distance)
			{
				return true;
			}
			else if(distance == k.distance && id < k.id)
			{
				return true;
			}
		}
		return false;
	}
	float likelihood;
	int id;
	float distance;
};

//============================================================
// MAIN LOOP
//============================================================
bool Rtabmap::process(
		const cv::Mat & image,
		int id,
		const std::map<std::string, float> & externalStats)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("Safe relocalization rejected image-only process() because it has no authenticated RGB-D/odometry envelope");
		return false;
	}
	return this->process(SensorData(image, id), Transform());
}
bool Rtabmap::process(
			const SensorData & data,
			Transform odomPose,
			float odomLinearVariance,
			float odomAngularVariance,
			const std::vector<float> & odomVelocity,
			const std::map<std::string, float> & externalStats)
{
	if(!odomPose.isNull())
	{
		UASSERT(odomLinearVariance>0.0f);
		UASSERT(odomAngularVariance>0.0f);
	}
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	covariance.at<double>(0,0) = odomLinearVariance;
	covariance.at<double>(1,1) = odomLinearVariance;
	covariance.at<double>(2,2) = odomLinearVariance;
	covariance.at<double>(3,3) = odomAngularVariance;
	covariance.at<double>(4,4) = odomAngularVariance;
	covariance.at<double>(5,5) = odomAngularVariance;
	return process(data, odomPose, covariance, odomVelocity, externalStats);
}
bool Rtabmap::process(
		const SensorData & data,
		Transform odomPose,
		const cv::Mat & odomCovariance,
		const std::vector<float> & odomVelocity,
		const std::map<std::string, float> & externalStats)
{
	UDEBUG("");
	if(_safeRelocalizationGuardEnabled &&
			(!data.globalPose().isNull() ||
			 data.gps().stamp() > 0.0 ||
			 !data.landmarks().empty()))
	{
		UERROR("Safe relocalization rejected SensorData carrying a global-pose, GPS prior or native landmark before Memory update");
		return false;
	}

	//============================================================
	// Initialization
	//============================================================
	UTimer timer;
	UTimer timerTotal;
	double timeMemoryUpdate = 0;
	double timeNeighborLinkRefining = 0;
	double timeProximityByTimeDetection = 0;
	double timeProximityBySpaceSearch = 0;
	double timeProximityBySpaceVisualDetection = 0;
	double timeProximityBySpaceDetection = 0;
	double timeCleaningNeighbors = 0;
	double timeReactivations = 0;
	double timeAddLoopClosureLink = 0;
	double timeMapOptimization = 0;
	double timeRetrievalDbAccess = 0;
	double timeLikelihoodCalculation = 0;
	double timePosteriorCalculation = 0;
	double timeHypothesesCreation = 0;
	double timeHypothesesValidation = 0;
	double timeRealTimeLimitReachedProcess = 0;
	double timeMemoryCleanup = 0;
	double timeEmptyingTrash = 0;
	double timeFinalizingStatistics = 0;
	double timeJoiningTrash = 0;
	double timeStatsCreation = 0;

	float hypothesisRatio = 0.0f; // Only used for statistics
	bool rejectedLoopClosure = false;

	std::map<int, float> rawLikelihood;
	std::map<int, float> adjustedLikelihood;
	std::map<int, float> likelihood;
	std::map<int, int> weights;
	std::map<int, float> posterior;
	std::list<std::pair<int, float> > reactivateHypotheses;

	std::map<int, int> childCount;
	std::set<int> signaturesRetrieved;
	int proximityDetectionsInTimeFound = 0;

	const Signature * signature = 0;
	const Signature * sLoop = 0;
	// A continuation suppresses reset detection for the first *accepted* data
	// update.  Do not consume this one-shot flag before Memory::update(): a
	// rejected/empty frame must not expose the following valid identity odometry
	// frame to a second reset and create an extra epoch.
	bool consumeSkipResetDetectionAfterUpdate = false;

	_loopClosureHypothesis = std::make_pair(0,0.0f);
	std::pair<int, float> lastHighestHypothesis = _highestHypothesis;
	_highestHypothesis = std::make_pair(0,0.0f);

	std::set<int> immunizedLocations;

	statistics_ = Statistics(); // reset
	for(std::map<std::string, float>::const_iterator iter=externalStats.begin(); iter!=externalStats.end(); ++iter)
	{
		statistics_.addStatistic(iter->first, iter->second);
	}

	//============================================================
	// Wait for an image...
	//============================================================
	ULOGGER_INFO("getting data...");

	timer.start();
	timerTotal.start();

	UASSERT_MSG(_memory, "RTAB-Map is not initialized!");
	UASSERT_MSG(_bayesFilter, "RTAB-Map is not initialized!");
	UASSERT_MSG(_graphOptimizer, "RTAB-Map is not initialized!");

	//============================================================
	// If RGBD SLAM is enabled, a pose must be set.
	//============================================================
	bool fakeOdom = false;
	if(_rgbdSlamMode)
	{
		if(!odomPose.isNull())
		{
			// If we are doing 2D mapping, make sure the pose is 3DoF so that landmark logic works.
			if(_forceOdom3doF && _graphOptimizer->isSlam2d() && !odomPose.is3DoF())
			{
				odomPose = odomPose.to3DoF();
			}

			// this will make sure that all inverse operations will work!
			if(!odomPose.isInvertible())
			{
				UWARN("Input odometry is not invertible! pose = %s\n"
						"[%f %f %f %f;\n"
						" %f %f %f %f;\n"
						" %f %f %f %f;\n"
						" 0 0 0 1]\n"
						"Trying to normalize rotation to see if it makes it invertible...",
						odomPose.prettyPrint().c_str(),
						odomPose.r11(), odomPose.r12(), odomPose.r13(), odomPose.o14(),
						odomPose.r21(), odomPose.r22(), odomPose.r23(), odomPose.o24(),
						odomPose.r31(), odomPose.r32(), odomPose.r33(), odomPose.o34());
				odomPose.normalizeRotation();
				UASSERT_MSG(odomPose.isInvertible(), uFormat("Odometry pose is not invertible!\n"
						"[%f %f %f %f;\n"
						" %f %f %f %f;\n"
						" %f %f %f %f;\n"
						" 0 0 0 1]", odomPose.prettyPrint().c_str(),
						odomPose.r11(), odomPose.r12(), odomPose.r13(), odomPose.o14(),
						odomPose.r21(), odomPose.r22(), odomPose.r23(), odomPose.o24(),
						odomPose.r31(), odomPose.r32(), odomPose.r33(), odomPose.o34()).c_str());
				UWARN("Normalizing rotation succeeded! fixed pose = %s\n"
						"[%f %f %f %f;\n"
						" %f %f %f %f;\n"
						" %f %f %f %f;\n"
						" 0 0 0 1]\n"
						"If the resulting rotation is very different from original one, try to fix the odometry or TF.",
						odomPose.prettyPrint().c_str(),
						odomPose.r11(), odomPose.r12(), odomPose.r13(), odomPose.o14(),
						odomPose.r21(), odomPose.r22(), odomPose.r23(), odomPose.o24(),
						odomPose.r31(), odomPose.r32(), odomPose.r33(), odomPose.o34());
			}
		}

		UDEBUG("incremental=%d odomPose=%s optimizedPoses=%d mapCorrection=%s lastLocalizationPose=%s lastLocalizationNodeId=%d",
				_memory->isIncremental()?1:0,
				odomPose.prettyPrint().c_str(),
				(int)_optimizedPoses.size(),
				_mapCorrection.prettyPrint().c_str(),
				_lastLocalizationPose.prettyPrint().c_str(),
				_lastLocalizationNodeId);

		if(!_memory->isIncremental() &&
			!odomPose.isNull() &&
			_optimizedPoses.size() &&
			_mapCorrection.isIdentity() &&
			!_lastLocalizationPose.isNull() &&
			_lastLocalizationNodeId == 0)
		{
			// Localization mode
			if(!_optimizeFromGraphEnd)
			{
				//set map->odom so that odom is moved back to last saved localization
				if(_graphOptimizer->isSlam2d())
				{
					_mapCorrection = _lastLocalizationPose.to3DoF() * odomPose.to3DoF().inverse();
				}
				else if((!data.imu().empty() || _memory->isOdomGravityUsed()) && _graphOptimizer->gravitySigma()>0.0f)
				{
					_mapCorrection = _lastLocalizationPose.to4DoF() * odomPose.to4DoF().inverse();
				}
				else
				{
					_mapCorrection = _lastLocalizationPose * odomPose.inverse();
				}
				std::map<int, Transform> nodesOnly(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				_lastLocalizationNodeId = graph::findNearestNode(nodesOnly, _lastLocalizationPose);
				UWARN("Update map correction based on last localization saved in database! correction = %s, nearest id = %d of last pose = %s, odom = %s",
						_mapCorrection.prettyPrint().c_str(),
						_lastLocalizationNodeId,
						_lastLocalizationPose.prettyPrint().c_str(),
						odomPose.prettyPrint().c_str());
			}
			else
			{
				//move optimized poses accordingly to last saved localization
				Transform mapCorrectionInv;
				if(_graphOptimizer->isSlam2d())
				{
					mapCorrectionInv = odomPose.to3DoF() * _lastLocalizationPose.to3DoF().inverse();
				}
				else if((!data.imu().empty() || _memory->isOdomGravityUsed()) && _graphOptimizer->gravitySigma()>0.0f)
				{
					mapCorrectionInv = odomPose.to4DoF() * _lastLocalizationPose.to4DoF().inverse();
				}
				else
				{
					mapCorrectionInv = odomPose * _lastLocalizationPose.inverse();
				}
				for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
				{
					iter->second = mapCorrectionInv * iter->second;
				}

				std::map<int, Transform> nodesOnly(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				_lastLocalizationNodeId = graph::findNearestNode(nodesOnly, _lastLocalizationPose);
				UWARN("Transformed map accordingly to last localization pose saved in database (%s=true)! nearest id = %d of last pose = %s",
						Parameters::kRGBDOptimizeFromGraphEnd().c_str(),
						_lastLocalizationNodeId,
						_lastLocalizationPose.prettyPrint().c_str());
			}
		}

		if(odomPose.isNull())
		{
			if(_memory->isIncremental())
			{
				UERROR("RGB-D SLAM mode is enabled, memory is incremental but no odometry is provided. "
					   "Image %d is ignored!", data.id());
				return false;
			}
			else // fake localization
			{
				if(!_mapCorrectionBackup.isNull())
				{
					_mapCorrection = _mapCorrectionBackup;
					_mapCorrectionBackup.setNull();
				}
				if(_lastLocalizationPose.isNull())
				{
					_lastLocalizationPose = Transform::getIdentity();
				}
				fakeOdom = true;
				odomPose = _mapCorrection.inverse() * _lastLocalizationPose;
				UDEBUG("Map correction = %s", _mapCorrection.prettyPrint().c_str());
				UDEBUG("Last localization pose: %s", _lastLocalizationPose.prettyPrint().c_str());
				UDEBUG("Fake odom: %s", odomPose.prettyPrint().c_str());
			}
		}
		else if(_memory->isIncremental()) // only in mapping mode
		{
			// Detect if the odometry is reset. If yes, trigger a new map.
			const bool skipResetDetection = _skipNextOdomResetDetection;
			consumeSkipResetDetectionAfterUpdate = skipResetDetection;
			// In safe-relocalization mode the ROS wrapper's unique-stamp
			// lost/recovered quarantine is the sole owner of reset epochs.  The
			// historical identity/large-jump detector is intentionally disabled:
			// either branch can otherwise split the graph on a single sample and
			// bypass the quarantine transaction.
			if(!_safeRelocalizationGuardEnabled && !skipResetDetection &&
					_memory->getLastWorkingSignature(false))
			{
				const Transform & lastPose = _memory->getLastWorkingSignature(false)->getPose(); // use raw odometry

				// look for identity
				if(!lastPose.isIdentity() && odomPose.isIdentity())
				{
					int mapId = triggerNewMap();
					UWARN("Odometry is reset (identity pose detected). Increment map id to %d!", mapId);
				}
				else if(_newMapOdomChangeDistance > 0.0)
				{
					// look for large change
					Transform lastPoseToNewPose = lastPose.inverse() * odomPose;
					float x,y,z, roll,pitch,yaw;
					lastPoseToNewPose.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
					if((x*x + y*y + z*z) > _newMapOdomChangeDistance*_newMapOdomChangeDistance)
					{
						int mapId = triggerNewMap();
						UWARN("Odometry is reset (large odometry change detected > %f). A new map (%d) is created! Last pose = %s, new pose = %s",
								_newMapOdomChangeDistance,
								mapId,
								lastPose.prettyPrint().c_str(),
								odomPose.prettyPrint().c_str());
					}
				}
			}
		}
	}

	//============================================================
	// Memory Update : Location creation + Add to STM + Weight Update (Rehearsal)
	//============================================================
	ULOGGER_INFO("Updating memory...");
	if(_rgbdSlamMode)
	{
		if(!_memory->update(data, odomPose, odomCovariance, odomVelocity, &statistics_))
		{
			return false;
		}
	}
	else
	{
		if(!_memory->update(data, Transform(), cv::Mat(), std::vector<float>(), &statistics_))
		{
			return false;
		}
	}
	if(consumeSkipResetDetectionAfterUpdate)
	{
		_skipNextOdomResetDetection = false;
	}

	signature = _memory->getLastWorkingSignature(false);
	if(_rgbdSlamMode && _memory->isIncremental() && signature != 0 && signature->id() > 0)
	{
		_signatureResetEpochs[signature->id()] = _resetEpoch;
	}
	_currentSessionHasGPS = _currentSessionHasGPS || signature->sensorData().gps().stamp() > 0.0;
	if(!signature)
	{
		UFATAL("Not supposed to be here...last signature is null?!?");
	}

	ULOGGER_INFO("Processing signature %d (%f) w=%d map=%d", signature->id(), signature->getStamp(), signature->getWeight(), signature->mapId());
	timeMemoryUpdate = timer.ticks();
	ULOGGER_INFO("timeMemoryUpdate=%fs", timeMemoryUpdate);

	//============================================================
	// Metric
	//============================================================
	bool smallDisplacement = false;
	bool tooFastMovement = false;
	std::list<int> signaturesRemoved;
	bool neighborLinkRefined = false;
	bool addedNewLandmark = false;
	float distanceToClosestNodeInTheGraph = 0;
	float angleToClosestNodeInTheGraph = 0;
	if(_rgbdSlamMode)
	{
		double linVar = odomCovariance.empty()?0.0f:uMax3(odomCovariance.at<double>(0,0), odomCovariance.at<double>(1,1)>=9999?0:odomCovariance.at<double>(1,1), odomCovariance.at<double>(2,2)>=9999?0:odomCovariance.at<double>(2,2));
		double angVar = odomCovariance.empty()?0.0f:uMax3(odomCovariance.at<double>(3,3)>=9999?0:odomCovariance.at<double>(3,3), odomCovariance.at<double>(4,4)>=9999?0:odomCovariance.at<double>(4,4), odomCovariance.at<double>(5,5));
		statistics_.addStatistic(Statistics::kMemoryOdometry_variance_lin(), (float)linVar);
		statistics_.addStatistic(Statistics::kMemoryOdometry_variance_ang(), (float)angVar);

		//Verify if there was a rehearsal
		int rehearsedId = (int)uValue(statistics_.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
		if(rehearsedId > 0 && !_memory->getSignature(rehearsedId))
			_acceptedImageReceipts.erase(rehearsedId);
		if(rehearsedId > 0)
		{
			_optimizedPoses.erase(rehearsedId);
		}
		else
		{
			bool linkedToIntermediateNode = false;
			Transform t;
			if(_memory->isIncremental())
			{
				// Check small motion if current node is not an intermediate node already
				if(signature->getWeight() >= 0)
				{
					// It should contain only the query and its first (non-intermediate) neighbor (smaller id)
					std::map<int, int> neighbors = _memory->getNeighborsId(signature->id(), 2, 0, true, true, true, true);
					if(neighbors.size() == 2)
					{
						int nid = neighbors.begin()->first;
						const std::multimap<int, Link> & links = signature->getLinks();
						if(links.find(nid) != links.end())
						{
							// direct neighbor
							t = links.find(nid)->second.transform();
						}
						else
						{
							// Use optimized poses to check how far it is from the latest non-intermediate node
							std::map<int, Transform>::iterator niter = _optimizedPoses.find(nid);
							if(niter != _optimizedPoses.end())
							{
								t = niter->second.inverse() * _mapCorrection * signature->getPose();
							}
							// not direct link, it means there are intermediate nodes
							linkedToIntermediateNode = true; 
						}
					}
				}
			}
			else if(!_odomCachePoses.empty())
			{
				t = _odomCachePoses.rbegin()->second.inverse() * signature->getPose();
			}
			
			if(_rgbdLinearUpdate > 0.0f || _rgbdAngularUpdate > 0.0f)
			{
				//============================================================
				// Minimum displacement required to add to Memory
				//============================================================
				if(!t.isNull())
				{
					float x,y,z, roll,pitch,yaw;
					t.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
					bool isMoving = (_rgbdLinearUpdate > 0.0f && (
										fabs(x) > _rgbdLinearUpdate ||
										fabs(y) > _rgbdLinearUpdate ||
										fabs(z) > _rgbdLinearUpdate)) ||
									(_rgbdAngularUpdate>0.0f && (
										fabs(roll) > _rgbdAngularUpdate ||
										fabs(pitch) > _rgbdAngularUpdate ||
										fabs(yaw) > _rgbdAngularUpdate));
					if(!isMoving)
					{
						// This will disable global loop closure detection, only retrieval will be done.
						// The location will also be deleted at the end.
						smallDisplacement = true;
						UDEBUG("smallDisplacement: %f %f %f %f %f %f", x,y,z, roll,pitch,yaw);
					}
				}
			}
			if(odomVelocity.size() == 6 && signature->getWeight() != -1)
			{
				// This will disable global loop closure detection, only retrieval will be done.
				// The location will also be deleted at the end.
				tooFastMovement =
						(_rgbdLinearSpeedUpdate>0.0f && uMax3(fabs(odomVelocity[0]), fabs(odomVelocity[1]), fabs(odomVelocity[2])) > _rgbdLinearSpeedUpdate) ||
						(_rgbdAngularSpeedUpdate>0.0f && uMax3(fabs(odomVelocity[3]), fabs(odomVelocity[4]), fabs(odomVelocity[5])) > _rgbdAngularSpeedUpdate);
			}
			if(linkedToIntermediateNode && (smallDisplacement || tooFastMovement))
			{
				_memory->convertToIntermediate(signature->id());
			}
		}

		// Update optimizedPoses with the newly added node
		Transform newPose;
		bool intermediateNodeRefining = false;
		if(_neighborLinkRefining &&
			signature->getLinks().size() &&
			signature->getLinks().begin()->second.type() == Link::kNeighbor &&
		   _memory->isIncremental() && // ignore pose matching in localization mode
		   rehearsedId == 0 && // don't do it if rehearsal happened
		   !tooFastMovement) // ignore if too fast movement has been detected
		{
			int oldId = signature->getLinks().begin()->first;
			const Signature * oldS = _memory->getSignature(oldId);
			UASSERT(oldS != 0);

			if(signature->getWeight() >= 0 && oldS->getWeight()>=0) // ignore intermediate nodes
			{
				Transform guess = signature->getLinks().begin()->second.transform().inverse();

				if(smallDisplacement)
				{
					if(signature->getLinks().begin()->second.transVariance() == 1)
					{
						// set small variance
						UDEBUG("Set small variance. The robot is not moving.");
						_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), guess, cv::Mat::eye(6,6,CV_64FC1)*1000));
					}
				}
				else
				{
					//============================================================
					// Refine neighbor links
					//============================================================
					UINFO("Odometry refining: guess = %s", guess.prettyPrint().c_str());
					RegistrationInfo info;
					Transform t = _memory->computeTransform(oldId, signature->id(), guess, &info);
					if(!t.isNull())
					{
						UASSERT(!info.covariance.empty() && info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
						UINFO("Odometry refining: update neighbor link (%d->%d, variance:lin=%f, ang=%f) from %s to %s",
								oldId,
								signature->id(),
								info.covariance.at<double>(0,0),
								info.covariance.at<double>(5,5),
								guess.prettyPrint().c_str(),
								t.prettyPrint().c_str());
						_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), t, info.covariance.inv()));

						if(_optimizeFromGraphEnd)
						{
							// update all previous nodes
							// Normally _mapCorrection should be identity, but if _optimizeFromGraphEnd
							// parameters just changed state, we should put back all poses without map correction.
							Transform u = guess * t.inverse();
							std::map<int, Transform>::iterator jter = _optimizedPoses.find(oldId);
							UASSERT(jter!=_optimizedPoses.end());
							Transform up = jter->second * u * jter->second.inverse();
							Transform mapCorrectionInv = _mapCorrection.inverse();
							for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
							{
								iter->second = mapCorrectionInv * up * iter->second;
							}
						}
					}
					else
					{
						UINFO("Odometry refining rejected: %s", info.rejectedMsg.c_str());
						if(!info.covariance.empty() && info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(0,0) != 1.0 && info.covariance.at<double>(5,5) > 0.0 && info.covariance.at<double>(5,5) != 1.0)
						{
							if(ULogger::level() <= ULogger::kInfo)
							{
								std::cout << info.covariance << std::endl;
							}
							_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), guess, (info.covariance*100.0).inv()));
						}
					}
					neighborLinkRefined = !t.isNull();
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningAccepted(),neighborLinkRefined?1.0f:0);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningInliers(), info.inliers);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_inliers_ratio(), info.icpInliersRatio);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_rotation(), info.icpRotation);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_translation(), info.icpTranslation);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_complexity(), info.icpStructuralComplexity);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningPts(), signature->sensorData().laserScanRaw().size());
				}
				timeNeighborLinkRefining = timer.ticks();
				ULOGGER_INFO("timeOdometryRefining=%fs", timeNeighborLinkRefining);

				UASSERT(oldS->hasLink(signature->id()));
				UASSERT(uContains(_optimizedPoses, oldId));

				statistics_.addStatistic(Statistics::kNeighborLinkRefiningVariance(), oldS->getLinks().find(signature->id())->second.transVariance());

				newPose = _optimizedPoses.at(oldId) * oldS->getLinks().find(signature->id())->second.transform();
				_mapCorrection = newPose * signature->getPose().inverse();
				if(_mapCorrection.getNormSquared() > 0.001f && _optimizeFromGraphEnd)
				{
					UERROR("Map correction should be identity when optimizing from the last node. T=%s NewPose=%s OldPose=%s",
							_mapCorrection.prettyPrint().c_str(),
							newPose.prettyPrint().c_str(),
							signature->getPose().prettyPrint().c_str());
				}
			}
			else
			{
				newPose = _mapCorrection * signature->getPose();
				intermediateNodeRefining = true;
			}
		}
		else
		{
			newPose = _mapCorrection * signature->getPose();
		}

		// Get statistics about the closest node in the graph
		if(!_memory->isIncremental())
		{
			int closestNode = 0;
			float sqrdDistance = 0.0f;
			if(_optimizedPoses.begin()->first < 0)
			{
				std::map<int, Transform> poses(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				closestNode = graph::findNearestNode(poses, newPose, &sqrdDistance);
			}
			else
			{
				closestNode = graph::findNearestNode(_optimizedPoses, newPose, &sqrdDistance);
			}
			if(closestNode>0 && sqrdDistance>0.0f)
			{
				distanceToClosestNodeInTheGraph = sqrt(sqrdDistance);
				UDEBUG("Last localization pose = %s, closest node=%d (%f m)", newPose.prettyPrint().c_str(), closestNode, distanceToClosestNodeInTheGraph);
				angleToClosestNodeInTheGraph = newPose.getAngle(_optimizedPoses.at(closestNode));
			}
		}

		UDEBUG("Added pose %s (odom=%s)", newPose.prettyPrint().c_str(), signature->getPose().prettyPrint().c_str());
		// Update Poses and Constraints
		_optimizedPoses.insert(std::make_pair(signature->id(), newPose));
		if(_safeRelocalizationGuardEnabled)
		{
			UASSERT_MSG(signature->getLandmarks().empty(),
					"Safe relocalization forbids native SensorData landmarks; use the authenticated three-view transaction");
		}
		else if(_memory->isIncremental() && signature->getWeight() >= 0)
		{
			for(std::map<int, Link>::const_iterator iter = signature->getLandmarks().begin(); iter!=signature->getLandmarks().end(); ++iter)
			{
				if(_optimizedPoses.find(iter->first) == _optimizedPoses.end())
				{
					_optimizedPoses.insert(std::make_pair(iter->first, newPose*iter->second.transform()));
					UDEBUG("Added landmark %d : %s", iter->first, (newPose*iter->second.transform()).prettyPrint().c_str());
					addedNewLandmark = true;
				}
				_constraints.insert(std::make_pair(iter->first, iter->second.inverse()));
			}
		}

		float distanceTravelledOld = _distanceTravelled;

		// only in mapping mode we add a neighbor link
		if(signature->getLinks().size() &&
		   signature->getLinks().begin()->second.type() == Link::kNeighbor)
		{
			// link should be old to new
			UASSERT_MSG(signature->id() > signature->getLinks().begin()->second.to(),
					"Only forward links should be added.");

			Link tmp = signature->getLinks().begin()->second.inverse();

			if(!smallDisplacement)
			{
				_distanceTravelled += tmp.transform().getNorm();
			}

			// if the previous node is an intermediate node, remove it from the local graph
			if(_constraints.size() &&
			   _constraints.rbegin()->second.to() == signature->getLinks().begin()->second.to())
			{
				const Signature * s = _memory->getSignature(signature->getLinks().begin()->second.to());
				UASSERT(s!=0);
				if(s->getWeight() == -1)
				{
					tmp = _constraints.rbegin()->second.merge(tmp, tmp.type());
					_optimizedPoses.erase(s->id());
					_constraints.erase(--_constraints.end());
				}
			}

			_constraints.insert(std::make_pair(tmp.from(), tmp));
		}
		// Localization mode stuff
		if( signature->getWeight() >= 0 &&
			!smallDisplacement &&
		    odomCovariance.cols == 6 &&
			odomCovariance.rows == 6 &&
			odomCovariance.type() == CV_64FC1 &&
			odomCovariance.at<double>(0,0) < 1)
		{
			if( _memory->isIncremental() && _localizationCovariance.empty())
			{
				_localizationCovariance = cv::Mat::zeros(6,6,CV_64FC1);
			}
			if(_localizationCovariance.total() == 36)
			{
#ifdef RTABMAP_MRPT
				// Transform odometry covariance (which in base frame) into global frame
				// "odometry error propagation law"
				Eigen::Quaterniond rotation = _lastLocalizationPose.getQuaterniond();
				mrpt::poses::CPose3D pose = mrpt::poses::CPose3D::FromQuaternion(mrpt::math::CQuaternionDouble(rotation.w(), rotation.x(), rotation.y(), rotation.z()));
				mrpt::math::CMatrixDouble66 gaussian;
				gaussian.loadFromRawPointer((const double*)odomCovariance.data);
				mrpt::poses::CPose3DPDFGaussian gaussianTransformed(mrpt::poses::CPose3D(), gaussian);
				gaussianTransformed.changeCoordinatesReference(pose);
				_localizationCovariance += cv::Mat(6,6,CV_64FC1, gaussianTransformed.cov.data());
#else
				// Assuming diagonal uniform covariance matrix!
				// If variance is different for each axis,
				// build rtabmap with MRPT to use approach above.
				_localizationCovariance += odomCovariance;
#endif
			}
		}
		_lastLocalizationPose = newPose; // keep in cache the latest corrected pose
		if(signature->getWeight() >= 0)
		{
			UDEBUG("Update odometry localization cache (size=%d/%d)", (int)_odomCachePoses.size(), _maxOdomCacheSize);
			if(!_odomCachePoses.empty())
			{
				float odomDistance = (_odomCachePoses.rbegin()->second.inverse() * signature->getPose()).getNorm();
				if(!smallDisplacement)
				{
					_distanceTravelled += odomDistance;
				}

				while(!_odomCachePoses.empty() && (int)_odomCachePoses.size() > _maxOdomCacheSize)
				{
					_odomCacheConstraints.erase(_odomCachePoses.begin()->first);
					_odomCachePoses.erase(_odomCachePoses.begin());
				}
				if(!_odomCachePoses.empty())
				{
					Link odomLink(_odomCachePoses.rbegin()->first,
							signature->id(),
							Link::kNeighbor,
							_odomCachePoses.rbegin()->second.inverse() * signature->getPose(),
							odomCovariance.inv());
					_odomCacheConstraints.insert(std::make_pair(_odomCachePoses.rbegin()->first, odomLink));
					UDEBUG("Added odom cov = %f %f", odomLink.transVariance(), odomLink.rotVariance());
				}
			}

			_odomCachePoses.insert(std::make_pair(signature->id(), signature->getPose()));
		}
		_distanceTravelledSinceLastLocalization += _distanceTravelled - distanceTravelledOld;

		//============================================================
		// Reduced graph
		//============================================================
		//Verify if there are nodes that were merged through graph reduction
		if(statistics_.reducedIds().size())
		{
			for(unsigned int i=0; i<_path.size(); ++i)
			{
				std::map<int, int>::const_iterator iter = statistics_.reducedIds().find(_path[i].first);
				if(iter!= statistics_.reducedIds().end())
				{
					// change path ID to loop closure ID
					_path[i].first = iter->second;
				}
			}

			for(std::map<int, int>::const_iterator iter=statistics_.reducedIds().begin();
				iter!=statistics_.reducedIds().end();
				++iter)
			{
				int erased = (int)_optimizedPoses.erase(iter->first);
				if(erased)
				{
					for(std::multimap<int, Link>::iterator jter = _constraints.begin(); jter!=_constraints.end();)
					{
						if(jter->second.from() == iter->first || jter->second.to() == iter->first)
						{
							_constraints.erase(jter++);
						}
						else
						{
							++jter;
						}
					}
				}
			}
		}

		//============================================================
		// Local loop closure in TIME
		//============================================================
		if((_proximityByTime || intermediateNodeRefining) &&
		   rehearsedId == 0 && // don't do it if rehearsal happened
		   _memory->isIncremental() && // don't do it in localization mode
		   signature->getWeight()>=0)
		{
			const std::set<int> & stm = _memory->getStMem();
			for(std::set<int>::const_reverse_iterator iter = stm.rbegin(); iter!=stm.rend(); ++iter)
			{
				if(*iter != signature->id() &&
				   signature->getLinks().find(*iter) == signature->getLinks().end() &&
				   _memory->getSignature(*iter)->mapId() == signature->mapId() &&
				   _memory->getSignature(*iter)->getWeight()>=0)
				{
					std::string rejectedMsg;
					UDEBUG("Check local transform between %d and %d", signature->id(), *iter);
					RegistrationInfo info;
					Transform guess;
					if(_optimizedPoses.find(*iter) != _optimizedPoses.end())
					{
						guess = _optimizedPoses.at(*iter).inverse() * newPose;
					}

					// For proximity by time, correspondences should be already enough precise, so don't recompute them
					Transform transform = _memory->computeTransform(*iter, signature->id(), guess, &info, true);

					if(!transform.isNull())
					{
						transform = transform.inverse();
						UDEBUG("Add local loop closure in TIME (%d->%d) %s",
								signature->id(),
								*iter,
								transform.prettyPrint().c_str());
						// Add a loop constraint
						UASSERT(!info.covariance.empty() && info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
						if(_safeRelocalizationGuardEnabled &&
								getSignatureResetEpoch(signature->id()) != getSignatureResetEpoch(*iter))
						{
							UINFO("Observed but did not add cross-reset local-time closure %d->%d; guarded transaction is the only cross-reset graph mutation path",
									signature->id(), *iter);
						}
						else if(_memory->addLink(Link(signature->id(), *iter, Link::kLocalTimeClosure, transform, getInformation(info.covariance))))
						{
							++proximityDetectionsInTimeFound;
							UINFO("Local loop closure found between %d and %d with t=%s",
									*iter, signature->id(), transform.prettyPrint().c_str());
						}
						else
						{
							UWARN("Cannot add local loop closure between %d and %d ?!?",
									*iter, signature->id());
						}
					}
					else
					{
						UINFO("Local loop closure (time) between %d and %d rejected: %s",
								*iter, signature->id(), rejectedMsg.c_str());
					}

					if(!_proximityByTime && intermediateNodeRefining)
					{
						// Do it only with the latest non-intermediate node
						break;
					}
				}
			}
		}
	}

	timeProximityByTimeDetection = timer.ticks();
	UINFO("timeProximityByTimeDetection=%fs", timeProximityByTimeDetection);

	//============================================================
	// Bayes filter update
	//============================================================
	bool localizationOnPreviousUpdate = false;
	if(_memory->isIncremental())
	{
		localizationOnPreviousUpdate =
			signature->getLinks().size() &&
			signature->getLinks().begin()->first!=signature->id() &&
			_memory->getLoopClosureLinks(signature->getLinks().begin()->first, false).size() != 0;
	}
	else
	{
		// localization mode

		// Count how many localization links are in the constraints
		int localizationLinks = 0;
		int previousIdWithLocalizationLink = 0;
		for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin();
				iter!=_odomCacheConstraints.end(); ++iter)
		{
			if(previousIdWithLocalizationLink == iter->first)
			{
				// ignore links with node already counted
				continue;
			}
			if(iter->second.type() == Link::kGlobalClosure ||
			   iter->second.type() == Link::kLocalSpaceClosure ||
			   iter->second.type() == Link::kLocalTimeClosure ||
			   iter->second.type() == Link::kUserClosure ||
			   iter->second.type() == Link::kNeighborMerged ||
			   iter->second.type() == Link::kLandmark)
			{
				++localizationLinks;
				previousIdWithLocalizationLink = iter->first;
			}
		}

		localizationOnPreviousUpdate = localizationLinks > 1; // need two links in case we have delayed localization
	}

	// Not a bad signature, not an intermediate node, not a small displacement unless the previous signature didn't have a loop closure, not too fast movement
	if(!signature->isBadSignature() && signature->getWeight()>=0 && (!smallDisplacement || !localizationOnPreviousUpdate) && !tooFastMovement)
	{
		// If the working memory is empty, don't do the detection. It happens when it
		// is the first time the detector is started (there needs some images to
		// fill the short-time memory before a signature is added to the working memory).
		if(_memory->getWorkingMem().size())
		{
			//============================================================
			// Likelihood computation
			// Get the likelihood of the new signature
			// with all images contained in the working memory + reactivated.
			//============================================================
			ULOGGER_INFO("computing likelihood...");

			std::list<int> signaturesToCompare;
			GPS originGPS;
			Transform originOffsetENU = Transform::getIdentity();
			if(_loopGPS)
			{
				originGPS = signature->sensorData().gps();
				if(originGPS.stamp() == 0.0 && _currentSessionHasGPS)
				{
					UTimer tmpT;
					if(_optimizedPoses.size() && _memory->isIncremental())
					{
						//Search for latest node having GPS linked to current signature not too far.
						std::map<int, float> nearestIds = graph::findNearestNodes(signature->id(), _optimizedPoses, _localRadius);
						for(std::map<int, float>::reverse_iterator iter=nearestIds.rbegin(); iter!=nearestIds.rend() && iter->first>0; ++iter)
						{
							const Signature * s = _memory->getSignature(iter->first);
							UASSERT(s!=0);
							if(s->sensorData().gps().stamp() > 0.0)
							{
								originGPS = s->sensorData().gps();
								const Transform & sPose = _optimizedPoses.at(s->id());
								Transform localToENU(0,0,(float)((-(originGPS.bearing()-90))*M_PI/180.0) - sPose.theta());
								originOffsetENU = localToENU * (sPose.rotation()*(sPose.inverse()*_optimizedPoses.at(signature->id())));
								break;
							}
						}
					}
					//else if(!_memory->isIncremental()) // TODO, how can we estimate current GPS position in localization?
					//{
					//}
				}
				if(originGPS.stamp() > 0.0)
				{
					// no need to save it if it is in localization mode
					_gpsGeocentricCache.insert(std::make_pair(signature->id(), std::make_pair(originGPS.toGeodeticCoords().toGeocentric_WGS84(), originOffsetENU)));
				}
			}

			for(std::map<int, double>::const_iterator iter=_memory->getWorkingMem().begin();
				iter!=_memory->getWorkingMem().end();
				++iter)
			{
				if(iter->first > 0)
				{
					const Signature * s = _memory->getSignature(iter->first);
					UASSERT(s!=0);
					if(s->getWeight() != -1) // ignore intermediate nodes
					{
						bool accept = true;
						if(originGPS.stamp()>0.0)
						{
							std::map<int, std::pair<cv::Point3d, Transform> >::iterator cacheIter = _gpsGeocentricCache.find(s->id());
							if(cacheIter == _gpsGeocentricCache.end())
							{
								GPS gps = s->sensorData().gps();
								Transform offsetENU = Transform::getIdentity();
								if(gps.stamp()==0.0)
								{
									_memory->getGPS(s->id(), gps, offsetENU, false);
								}
								if(gps.stamp() > 0.0)
								{
									cacheIter = _gpsGeocentricCache.insert(
											std::make_pair(s->id(),
													std::make_pair(gps.toGeodeticCoords().toGeocentric_WGS84(), offsetENU))).first;
								}
							}


							if(cacheIter != _gpsGeocentricCache.end())
							{
								std::map<int, std::pair<cv::Point3d, Transform> >::iterator originIter = _gpsGeocentricCache.find(signature->id());
								UASSERT(originIter != _gpsGeocentricCache.end());
								cv::Point3d relativePose = GeodeticCoords::Geocentric_WGS84ToENU_WGS84(cacheIter->second.first, originIter->second.first, originGPS.toGeodeticCoords());
								const double & error = originGPS.error();
								const Transform & offsetENU = cacheIter->second.second;
								relativePose.x += offsetENU.x() - originOffsetENU.x();
								relativePose.y += offsetENU.y() - originOffsetENU.y();
								relativePose.z += offsetENU.z() - originOffsetENU.z();
								 // ignore altitude if difference is under GPS error
								if(relativePose.z>error)
								{
									relativePose.z -= error;
								}
								else if(relativePose.z < -error)
								{
									relativePose.z += error;
								}
								else
								{
									relativePose.z = 0;
								}
								accept = uNormSquared(relativePose.x, relativePose.y, relativePose.z) < _localRadius*_localRadius;
							}
						}

						if(accept)
						{
							signaturesToCompare.push_back(iter->first);
						}
					}
				}
				else
				{
					// virtual signature should be added
					signaturesToCompare.push_back(iter->first);
				}
			}

			rawLikelihood = _memory->computeLikelihood(signature, signaturesToCompare);

			// Adjust the likelihood (with mean and std dev)
			likelihood = rawLikelihood;
			this->adjustLikelihood(likelihood);

			timeLikelihoodCalculation = timer.ticks();
			ULOGGER_INFO("timeLikelihoodCalculation=%fs",timeLikelihoodCalculation);

			//============================================================
			// Apply the Bayes filter
			//  Posterior = Likelihood x Prior
			//============================================================
			ULOGGER_INFO("getting posterior...");

			// Compute the posterior
			posterior = _bayesFilter->computePosterior(_memory, likelihood);
			timePosteriorCalculation = timer.ticks();
			ULOGGER_INFO("timePosteriorCalculation=%fs",timePosteriorCalculation);

			// For statistics, copy weights
			if(_publishStats && (_publishLikelihood || _publishPdf))
			{
				weights = _memory->getWeights();
			}

			//============================================================
			// Select the highest hypothesis
			//============================================================
			ULOGGER_INFO("creating hypotheses...");
			if(posterior.size())
			{
				for(std::map<int, float>::const_reverse_iterator iter = posterior.rbegin(); iter != posterior.rend(); ++iter)
				{
					if(iter->first > 0 && iter->second > _highestHypothesis.second)
					{
						_highestHypothesis = *iter;
					}
				}
				// With the virtual place, use sum of LC probabilities (1 - virtual place hypothesis).
				_highestHypothesis.second = 1-posterior.begin()->second;
			}
			timeHypothesesCreation = timer.ticks();
			ULOGGER_INFO("Highest hypothesis=%d, value=%f, timeHypothesesCreation=%fs", _highestHypothesis.first, _highestHypothesis.second, timeHypothesesCreation);

			if(_highestHypothesis.first > 0)
			{
				float loopThr = _loopThr;
				bool hasLoopClosureConstraints = false;
				for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin(); iter!=_odomCacheConstraints.end() && !hasLoopClosureConstraints; ++iter)
				{
					hasLoopClosureConstraints =
							iter->second.type() == Link::kGlobalClosure ||
							iter->second.type() == Link::kLocalSpaceClosure ||
							iter->second.type() == Link::kLandmark;
				}
				if(	(( _memory->isIncremental() && !uContains(_optimizedPoses, _highestHypothesis.first)) || // not linked to previous map of that hypothesis
					 (!_memory->isIncremental() && !hasLoopClosureConstraints)) && // not yet localized to any previous sessions
					_memory->getWorkingMem().size()>1 && // should have an old map (beside virtual signature)
					_rgbdSlamMode &&
					loopThr > _aggressiveLoopThr)
				{
					// use the best hypothesis directly.
					UDEBUG("Using %s=%f", Parameters::kRGBDAggressiveLoopThr().c_str(), _aggressiveLoopThr);
					loopThr = _aggressiveLoopThr;
				}

				// Loop closure Threshold
				if(_highestHypothesis.second >= loopThr)
				{
					rejectedLoopClosure = true;
					if(posterior.size() <= 2 && loopThr>0.0f)
					{
						// Ignore loop closure if there is only one loop closure hypothesis
						UDEBUG("rejected hypothesis: single hypothesis");
					}
					else if(_verifyLoopClosureHypothesis && !_epipolarGeometry->check(signature, _memory->getSignature(_highestHypothesis.first)))
					{
						UWARN("rejected hypothesis: by epipolar geometry");
					}
					else if(_loopRatio > 0.0f && lastHighestHypothesis.second && _highestHypothesis.second < _loopRatio*lastHighestHypothesis.second)
					{
						UWARN("rejected hypothesis: not satisfying hypothesis ratio (%f < %f * %f)",
								_highestHypothesis.second, _loopRatio, lastHighestHypothesis.second);
					}
					else if(_loopRatio > 0.0f && lastHighestHypothesis.second == 0)
					{
						UWARN("rejected hypothesis: last closure hypothesis is null (loop ratio is on)");
					}
					else
					{
						_loopClosureHypothesis = _highestHypothesis;
						rejectedLoopClosure = false;
					}

					timeHypothesesValidation = timer.ticks();
					ULOGGER_INFO("timeHypothesesValidation=%fs",timeHypothesesValidation);
				}
				else if(_highestHypothesis.second < _loopRatio*lastHighestHypothesis.second)
				{
					// Used for Precision-Recall computation.
					// When analyzing logs, it's convenient to know
					// if the hypothesis would be rejected if T_loop would be lower.
					rejectedLoopClosure = true;
					UDEBUG("rejected hypothesis: under loop ratio %f < %f", _highestHypothesis.second, _loopRatio*lastHighestHypothesis.second);
				}

				//for statistic...
				hypothesisRatio = _loopClosureHypothesis.second>0?_highestHypothesis.second/_loopClosureHypothesis.second:0;
			}
		} // if(_memory->getWorkingMemSize())
	}// !isBadSignature
	else if(!signature->isBadSignature() && signature->getWeight()>=0 && (smallDisplacement || tooFastMovement))
	{
		_highestHypothesis = lastHighestHypothesis;
		UDEBUG("smallDisplacement=%d tooFastMovement=%d", smallDisplacement?1:0, tooFastMovement?1:0);
	}
	else
	{
		UDEBUG("Ignoring likelihood and loop closure hypotheses as current signature doesn't have enough visual features.");
	}

	//============================================================
	// Before retrieval, make sure the trash has finished
	//============================================================
	_memory->joinTrashThread();
	timeEmptyingTrash = _memory->getDbSavingTime();
	timeJoiningTrash = timer.ticks();
	ULOGGER_INFO("Time emptying memory trash = %fs,  joining (actual overhead) = %fs", timeEmptyingTrash, timeJoiningTrash);

	//============================================================
	// RETRIEVAL 1/3 : Loop closure neighbors reactivation
	//============================================================
	int retrievalId = _highestHypothesis.first;
	std::list<int> reactivatedIds;
	double timeGetNeighborsTimeDb = 0.0;
	double timeGetNeighborsSpaceDb = 0.0;
	int immunizedGlobally = 0;
	int immunizedLocally = 0;
	int maxLocalLocationsImmunized = 0;
	if(_maxTimeAllowed != 0 || _maxMemoryAllowed != 0)
	{
		// with memory management, we have to immunize some nodes
		maxLocalLocationsImmunized = _localImmunizationRatio * float(_memory->getWorkingMem().size());
	}
	// no need to do retrieval or immunization of locations if memory management
	// is disabled and all nodes are in WM.
	// Also skip memory mangement on intermediate nodes
	if(!(_memory->allNodesInWM() && maxLocalLocationsImmunized == 0) && signature->getWeight()>=0)
	{
		if(retrievalId > 0)
		{
			//Load neighbors
			ULOGGER_INFO("Retrieving locations... around id=%d", retrievalId);
			int neighborhoodSize = (int)_bayesFilter->getPredictionLC().size()-1;
			UASSERT(neighborhoodSize >= 0);
			ULOGGER_DEBUG("margin=%d maxRetieved=%d", neighborhoodSize, _maxRetrieved);

			UTimer timeGetN;
			unsigned int nbLoadedFromDb = 0;
			std::set<int> reactivatedIdsSet;
			std::map<int, int> neighbors;
			int nbDirectNeighborsInDb = 0;

			// priority in time
			// Direct neighbors TIME
			ULOGGER_DEBUG("In TIME");
			neighbors = _memory->getNeighborsId(retrievalId,
					neighborhoodSize,
					_maxRetrieved,
					true,
					true,
					false,
					true,
					std::set<int>(),
					&timeGetNeighborsTimeDb);
			ULOGGER_DEBUG("neighbors of %d in time = %d", retrievalId, (int)neighbors.size());
			//Priority to locations near in time (direct neighbor) then by space (loop closure)
			bool firstPassDone = false; // just to avoid checking to STM after the first pass
			int m = 0;
			while(m < neighborhoodSize)
			{
				std::set<int> idsSorted;
				for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
				{
					if(!firstPassDone && _memory->isInSTM(iter->first))
					{
						neighbors.erase(iter++);
					}
					else if(iter->second == m)
					{
						if(reactivatedIdsSet.find(iter->first) == reactivatedIdsSet.end())
						{
							idsSorted.insert(iter->first);
							reactivatedIdsSet.insert(iter->first);

							if(m == 1 && _memory->getSignature(iter->first) == 0)
							{
								++nbDirectNeighborsInDb;
							}

							//immunized locations in the neighborhood from being transferred
							if(immunizedLocations.insert(iter->first).second)
							{
								++immunizedGlobally;
							}

							//UDEBUG("nt=%d m=%d immunized=1", iter->first, iter->second);
						}
						neighbors.erase(iter++);
					}
					else
					{
						++iter;
					}
				}
				firstPassDone = true;
				reactivatedIds.insert(reactivatedIds.end(), idsSorted.rbegin(), idsSorted.rend());
				++m;
			}

			// neighbors SPACE, already added direct neighbors will be ignored
			ULOGGER_DEBUG("In SPACE");
			neighbors = _memory->getNeighborsId(retrievalId,
					neighborhoodSize,
					_maxRetrieved,
					true,
					false,
					false,
					false,
					std::set<int>(),
					&timeGetNeighborsSpaceDb);
			ULOGGER_DEBUG("neighbors of %d in space = %d", retrievalId, (int)neighbors.size());
			firstPassDone = false;
			m = 0;
			while(m < neighborhoodSize)
			{
				std::set<int> idsSorted;
				for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
				{
					if(!firstPassDone && _memory->isInSTM(iter->first))
					{
						neighbors.erase(iter++);
					}
					else if(iter->second == m)
					{
						if(reactivatedIdsSet.find(iter->first) == reactivatedIdsSet.end())
						{
							idsSorted.insert(iter->first);
							reactivatedIdsSet.insert(iter->first);

							if(m == 1 && _memory->getSignature(iter->first) == 0)
							{
								++nbDirectNeighborsInDb;
							}
							//UDEBUG("nt=%d m=%d", iter->first, iter->second);
						}
						neighbors.erase(iter++);
					}
					else
					{
						++iter;
					}
				}
				firstPassDone = true;
				reactivatedIds.insert(reactivatedIds.end(), idsSorted.rbegin(), idsSorted.rend());
				++m;
			}
			ULOGGER_INFO("neighborhoodSize=%d, "
					"reactivatedIds.size=%d, "
					"nbLoadedFromDb=%d, "
					"nbDirectNeighborsInDb=%d, "
					"time=%fs (%fs %fs)",
					neighborhoodSize,
					reactivatedIds.size(),
					(int)nbLoadedFromDb,
					nbDirectNeighborsInDb,
					timeGetN.ticks(),
					timeGetNeighborsTimeDb,
					timeGetNeighborsSpaceDb);

		}
	}

	//============================================================
	// RETRIEVAL 2/3 : Update planned path and get next nodes to retrieve
	//============================================================
	std::list<int> retrievalLocalIds;
	if(_rgbdSlamMode && signature->getWeight()>=0)
	{
		// Priority on locations on the planned path
		if(_path.size())
		{
			// Note: retrieval on path with intermediate nodes is not supported. Note that the planned path would 
			//       eventually fail anyway because intermediate nodes are not in _optimizedPoses.
			updateGoalIndex();

			float distanceSoFar = 0.0f;
			// immunize all nodes after current node and
			// retrieve nodes after current node in the maximum radius from the current node
			for(unsigned int i=_pathCurrentIndex; i<_path.size(); ++i)
			{
				if(_localRadius > 0.0f && i != _pathCurrentIndex)
				{
					distanceSoFar += _path[i-1].second.getDistance(_path[i].second);
				}

				if(_memory->getSignature(_path[i].first) != 0)
				{
					if(immunizedLocations.insert(_path[i].first).second)
					{
						++immunizedLocally;
					}
					UDEBUG("Path immunization: node %d (dist=%fm)", _path[i].first, distanceSoFar);
				}
				else if(retrievalLocalIds.size() < _maxLocalRetrieved)
				{
					UINFO("retrieval of node %d on path (dist=%fm)", _path[i].first, distanceSoFar);
					retrievalLocalIds.push_back(_path[i].first);
					// retrieved locations are automatically immunized
				}
				
				if(distanceSoFar > _localRadius)
				{
					UDEBUG("Stop on node %d (dist=%fm > %fm)",
							_path[i].first, distanceSoFar, _localRadius);
					break;
				}
			}
		}

		if(!(_memory->allNodesInWM() && maxLocalLocationsImmunized == 0))
		{
			// immunize the path from the nearest local location to the current location
			if(immunizedLocally < maxLocalLocationsImmunized &&
				_memory->isIncremental()) // Can only work in mapping mode
			{
				std::map<int ,Transform> poses;
				// remove poses from STM
				for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
				{
					if(iter->first > 0 && !_memory->isInSTM(iter->first))
					{
						poses.insert(*iter);
					}
				}
				int nearestId = graph::findNearestNode(poses, _optimizedPoses.at(signature->id()));

				if(nearestId > 0 &&
					(_localRadius==0 ||
					 _optimizedPoses.at(signature->id()).getDistance(_optimizedPoses.at(nearestId)) < _localRadius))
				{
					std::multimap<int, int> links;
					for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end(); ++iter)
					{
						if(uContains(_optimizedPoses, iter->second.from()) && uContains(_optimizedPoses, iter->second.to()))
						{
							links.insert(std::make_pair(iter->second.from(), iter->second.to()));
							links.insert(std::make_pair(iter->second.to(), iter->second.from())); // <->
						}
					}

					std::list<std::pair<int, Transform> > path = graph::computePath(_optimizedPoses, links, nearestId, signature->id());
					if(path.size() == 0)
					{
						UWARN("Could not compute a path between %d and %d", nearestId, signature->id());
					}
					else
					{
						for(std::list<std::pair<int, Transform> >::iterator iter=path.begin();
							iter!=path.end();
							++iter)
						{
							if(iter->first>0)
							{
								if(immunizedLocally >= maxLocalLocationsImmunized)
								{
									// set 20 to avoid this warning when starting mapping
									if(maxLocalLocationsImmunized > 20 && _someNodesHaveBeenTransferred)
									{
										UWARN("Could not immunize the whole local path (%d) between "
											  "%d and %d (max location immunized=%d). You may want "
											  "to increase RGBD/LocalImmunizationRatio (current=%f (%d of WM=%d)) "
											  "to be able to immunize longer paths.",
												(int)path.size(),
												nearestId,
												signature->id(),
												maxLocalLocationsImmunized,
												_localImmunizationRatio,
												maxLocalLocationsImmunized,
												(int)_memory->getWorkingMem().size());
									}
									break;
								}
								else if(!_memory->isInSTM(iter->first))
								{
									if(immunizedLocations.insert(iter->first).second)
									{
										++immunizedLocally;
									}
									//UDEBUG("local node %d on path immunized=1", iter->first);
								}
							}
						}
					}
				}
			}

			// retrieval based on the nodes close the the nearest pose in WM
			// immunize closest nodes
			std::map<int, float> nearNodes = graph::findNearestNodes(signature->id(), _optimizedPoses, _localRadius);
			// sort by distance
			std::multimap<float, int> nearNodesByDist;
			for(std::map<int, float>::iterator iter=nearNodes.lower_bound(1); iter!=nearNodes.end(); ++iter)
			{
				nearNodesByDist.insert(std::make_pair(iter->second, iter->first));
			}
			UINFO("near nodes=%d, max local immunized=%d (immunized by path so far=%d), ratio=%f WM=%d",
					(int)nearNodesByDist.size(),
					maxLocalLocationsImmunized,
					immunizedLocally,
					_localImmunizationRatio,
					(int)_memory->getWorkingMem().size());
			std::list<int> retrievalLocalIdsIntermediate;
			for(std::multimap<float, int>::iterator iter=nearNodesByDist.begin();
				iter!=nearNodesByDist.end() && (retrievalLocalIds.size() < _maxLocalRetrieved || immunizedLocally < maxLocalLocationsImmunized);
				++iter)
			{
				const Signature * s = _memory->getSignature(iter->second);
				if(s!=0)
				{
					if(s->getWeight() != -1 && retrievalLocalIds.size() < _maxLocalRetrieved)
					{
						// If there is a change of direction, better to be retrieving
						// all nearest signatures than only newest neighbors.
						// Use getNeighborsId instead of direct links to support intermediate nodes.
						std::map<int, int> ids = _memory->getNeighborsId(s->id(), 2, _maxLocalRetrieved-retrievalLocalIds.size(), true, false, false);
						for(std::map<int, int>::const_reverse_iterator jter=ids.rbegin();
							jter!=ids.rend() && (retrievalLocalIds.size() < _maxLocalRetrieved || jter->second == 0);
							++jter)
						{
							if(_memory->getSignature(jter->first) == 0)
							{
								if(jter->second == 0)
								{
									UINFO("retrieval of intermediate node %d (margin=%d) on local map (from=%d)", jter->first, jter->second, s->id());
									retrievalLocalIdsIntermediate.push_back(jter->first);
								}
								else
								{
									UINFO("retrieval of node %d (margin=%d) on local map (from=%d)", jter->first, jter->second, s->id());
									retrievalLocalIds.push_back(jter->first);
								}
							}
						}
					}
					if(!_memory->isInSTM(s->id()) && immunizedLocally < maxLocalLocationsImmunized)
					{
						if(immunizedLocations.insert(s->id()).second)
						{
							++immunizedLocally;
						}
						//UDEBUG("local node %d (%f m) immunized=1", iter->second, iter->first);
					}
				}
			}
			// well, if the maximum retrieved is not reached, look for neighbors in database
			if(retrievalLocalIds.size() < _maxLocalRetrieved)
			{
				std::set<int> retrievalLocalIdsSet(retrievalLocalIds.begin(), retrievalLocalIds.end());
				retrievalLocalIdsSet.insert(retrievalLocalIdsIntermediate.begin(), retrievalLocalIdsIntermediate.end());
				for(std::list<int>::iterator iter=retrievalLocalIds.begin();
					iter!=retrievalLocalIds.end() && retrievalLocalIds.size() < _maxLocalRetrieved;
					++iter)
				{
					std::map<int, int> ids = _memory->getNeighborsId(*iter, 2, _maxLocalRetrieved - (unsigned int)retrievalLocalIds.size() + 1, true, false, false);
					for(std::map<int, int>::reverse_iterator jter=ids.rbegin();
						jter!=ids.rend() && (retrievalLocalIds.size() < _maxLocalRetrieved || jter->second == 0);
						++jter)
					{
						if(_memory->getSignature(jter->first) == 0 &&
						   retrievalLocalIdsSet.find(jter->first) == retrievalLocalIdsSet.end())
						{
							if(jter->second == 0)
							{
								UINFO("retrieval of intermediate node %d (margin=%d) on local map (from=%d)", jter->first, jter->second, *iter);
								retrievalLocalIdsIntermediate.push_back(jter->first);
							}
							else
							{
								UINFO("retrieval of node %d (margin=%d) on local map (from=%d)", jter->first, jter->second, *iter);
								retrievalLocalIds.push_back(jter->first);
							}
							retrievalLocalIdsSet.insert(jter->first);
						}
					}
				}
			}

			// update Age of the close signatures (oldest the farthest)
			for(std::multimap<float, int>::reverse_iterator iter=nearNodesByDist.rbegin(); iter!=nearNodesByDist.rend(); ++iter)
			{
				_memory->updateAge(iter->second);
			}

			// insert them first to make sure they are loaded.
			reactivatedIds.insert(reactivatedIds.begin(), retrievalLocalIdsIntermediate.begin(), retrievalLocalIdsIntermediate.end());
			reactivatedIds.insert(reactivatedIds.begin(), retrievalLocalIds.begin(), retrievalLocalIds.end());
		}
	}

	//============================================================
	// RETRIEVAL 3/3 : Load signatures from the database
	//============================================================
	if(reactivatedIds.size())
	{
		// Not important if the loop closure hypothesis don't have all its neighbors loaded,
		// only a loop closure link is added...
		signaturesRetrieved = _memory->reactivateSignatures(
				reactivatedIds,
				_maxRetrieved+(unsigned int)retrievalLocalIds.size(), // add path retrieved
				timeRetrievalDbAccess);

		ULOGGER_INFO("retrieval of %d (db time = %fs)", (int)signaturesRetrieved.size(), timeRetrievalDbAccess);

		timeRetrievalDbAccess += timeGetNeighborsTimeDb + timeGetNeighborsSpaceDb;
		UINFO("total timeRetrievalDbAccess=%fs", timeRetrievalDbAccess);

		// Immunize just retrieved signatures
		immunizedLocations.insert(signaturesRetrieved.begin(), signaturesRetrieved.end());

		if(!signaturesRetrieved.empty() && !_globalScanMap.empty())
		{
			UWARN("Some signatures have been retrieved from memory management, clearing global scan map...");
			_globalScanMap.clear();
			_globalScanMapPoses.clear();
		}
	}
	timeReactivations = timer.ticks();
	ULOGGER_INFO("timeReactivations=%fs", timeReactivations);

	//============================================================
	// Proximity detections
	//============================================================
	std::list<std::pair<int, int> > loopClosureLinksAdded;
	int loopClosureVisualInliers = 0; // for statistics
	float loopClosureVisualInliersRatio = 0.0f;
	int loopClosureVisualMatches = 0;
	float loopClosureVisualVariance = 0.0f;
	float loopClosureLinearVariance = 0.0f;
	float loopClosureAngularVariance = 0.0f;
	float loopClosureVisualInliersMeanDist = 0;
	float loopClosureVisualInliersDistribution = 0;

	int proximityDetectionsAddedVisually = 0;
	int proximityDetectionsAddedByICPMulti = 0;
	int proximityDetectionsAddedByICPGlobal = 0;
	int lastProximitySpaceClosureId = 0;
	int proximitySpacePaths = 0;
	int localVisualPathsChecked = 0;
	int localScanPathsChecked = 0;
	int loopIdSuppressedByProximity = 0;

	if(_proximityBySpace &&
	   _localRadius > 0 &&
	   _rgbdSlamMode &&
	   signature->getWeight() >= 0) // not an intermediate node
	{
		if(_startNewMapOnLoopClosure &&
			_memory->getWorkingMem().size()>=2 && // must have an old map (+1 virtual place)
			_localizationCovariance.empty() && // if we didn't localize yet
			graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size() == 0) // alone in new session
		{
			UINFO("Proximity detection by space disabled as if we force to have a global loop "
					"closure with previous map before doing proximity detections (%s=true).",
					Parameters::kRtabmapStartNewMapOnLoopClosure().c_str());
		}
		else if(_graphOptimizer->iterations() == 0)
		{
			UWARN("Cannot do local loop closure detection in space if graph optimization is disabled!");
		}
		else
		{
			// In localization mode, no need to check local loop
			// closures if we are already localized by a landmark.

			// don't do it if it is a small displacement unless the previous signature didn't have a loop closure
			// don't do it if there is a too fast movement
			if((!smallDisplacement || !localizationOnPreviousUpdate) && !tooFastMovement)
			{

				//============================================================
				// LOCAL LOOP CLOSURE SPACE
				//============================================================

				//
				// 1) compare visually with nearest locations
				//
				UDEBUG("Proximity detection (local loop closure in SPACE using matching images, local radius=%fm)", _localRadius);
				std::map<int, float> nearestIds = graph::findNearestNodes(signature->id(), _optimizedPoses, _localRadius);
				UDEBUG("nearestIds=%d/%d", (int)nearestIds.size(), (int)_optimizedPoses.size());
				std::map<int, Transform> nearestPoses;
				std::map<int, Transform> optimizedPosesWithOdomCache;
				std::multimap<int, int> links;
				std::map<int, Transform> * refPoses = &_optimizedPoses;
				if(_memory->isIncremental() && _proximityMaxGraphDepth>0)
				{
					// get bidirectional links
					for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end(); ++iter)
					{
						if(uContains(_optimizedPoses, iter->second.from()) && uContains(_optimizedPoses, iter->second.to()))
						{
							links.insert(std::make_pair(iter->second.from(), iter->second.to()));
							links.insert(std::make_pair(iter->second.to(), iter->second.from())); // <->
						}
					}
					if(_odomCachePoses.size() > 1)
					{
						// Add odometry cache if it contains a loop closure
						// That could happen when we just switched from localization mode to 
						// mapping mode while being localized on the previous session.
						optimizedPosesWithOdomCache = _optimizedPoses;
						optimizedPosesWithOdomCache.insert(_odomCachePoses.begin(), _odomCachePoses.end());
						refPoses = &optimizedPosesWithOdomCache;
						for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin(); iter!=_odomCacheConstraints.end(); ++iter)
						{
							if(uContains(optimizedPosesWithOdomCache, iter->second.from()) && 
							   uContains(optimizedPosesWithOdomCache, iter->second.to()) && 
							   iter->second.from() != iter->second.to())
							{
								links.insert(std::make_pair(iter->second.from(), iter->second.to()));
								links.insert(std::make_pair(iter->second.to(), iter->second.from())); // <->
							}
						}
					}
				}
				for(std::map<int, float>::iterator iter=nearestIds.lower_bound(1); iter!=nearestIds.end(); ++iter)
				{
					if(_memory->getStMem().find(iter->first) == _memory->getStMem().end())
					{
						if(_memory->isIncremental() && _proximityMaxGraphDepth > 0)
						{
							std::list<std::pair<int, Transform> > path = graph::computePath(*refPoses, links, signature->id(), iter->first);
							UDEBUG("Graph depth to %d = %ld", iter->first, path.size());
							if(!path.empty() && (int)path.size() <= _proximityMaxGraphDepth)
							{
								nearestPoses.insert(std::make_pair(iter->first, _optimizedPoses.at(iter->first)));
							}
						}
						else
						{
							nearestPoses.insert(std::make_pair(iter->first, _optimizedPoses.at(iter->first)));
						}
					}
				}
				UDEBUG("nearestPoses=%d", (int)nearestPoses.size());

				// segment poses by paths, only one detection per path, landmarks are ignored
				std::map<int, std::map<int, Transform> > nearestPathsNotSorted = getPaths(nearestPoses, _optimizedPoses.at(signature->id()), _proximityMaxGraphDepth);
				UDEBUG("got %d paths", (int)nearestPathsNotSorted.size());
				// sort nearest paths by highest likelihood (if two have same likelihood, sort by id)
				std::map<NearestPathKey, std::map<int, Transform> > nearestPaths;
				Transform currentPoseInv = _optimizedPoses.at(signature->id()).inverse();
				for(std::map<int, std::map<int, Transform> >::const_iterator iter=nearestPathsNotSorted.begin();iter!=nearestPathsNotSorted.end(); ++iter)
				{
					const std::map<int, Transform> & path = iter->second;
					float highestLikelihood = 0.0f;
					int highestLikelihoodId = iter->first;
					float smallestDistanceSqr = -1;
					for(std::map<int, Transform>::const_iterator jter=path.begin(); jter!=path.end(); ++jter)
					{
						float v = uValue(likelihood, jter->first, 0.0f);
						float distance = (currentPoseInv * jter->second).getNormSquared();
						if(v > highestLikelihood || (v == highestLikelihood && (smallestDistanceSqr < 0 || distance < smallestDistanceSqr)))
						{
							highestLikelihood = v;
							highestLikelihoodId = jter->first;
							smallestDistanceSqr = distance;
						}
					}
					nearestPaths.insert(std::make_pair(NearestPathKey(highestLikelihood, highestLikelihoodId, smallestDistanceSqr), path));
				}
				UDEBUG("nearestPaths=%d proximityMaxPaths=%d", (int)nearestPaths.size(), _proximityMaxPaths);

				timeProximityBySpaceSearch = timer.ticks();
				ULOGGER_INFO("timeProximityBySpaceSearch=%fs", timeProximityBySpaceSearch);

				float proximityFilteringRadius = _proximityFilteringRadius;
				if(_maxLoopClosureDistance>0.0f && (proximityFilteringRadius <= 0.0f || _maxLoopClosureDistance<proximityFilteringRadius))
				{
					proximityFilteringRadius = _maxLoopClosureDistance;
				}
				for(std::map<NearestPathKey, std::map<int, Transform> >::const_reverse_iterator iter=nearestPaths.rbegin();
					iter!=nearestPaths.rend() &&
					(_proximityMaxPaths <= 0 || localVisualPathsChecked < _proximityMaxPaths);
					++iter)
				{
					std::map<int, Transform> path = iter->second;
					UASSERT(path.size());

					//find the nearest pose on the path looking in the same direction
					path.insert(std::make_pair(signature->id(), _optimizedPoses.at(signature->id())));
					path = graph::findNearestPoses(signature->id(), path, _localRadius, _proximityAngle);
					//take the one with highest likelihood if not null
					int nearestId = 0;
					if(iter->first.likelihood > 0.0f &&
					   path.find(iter->first.id)!=path.end())
					{
						nearestId = iter->first.id;
					}
					else
					{
						nearestId = rtabmap::graph::findNearestNode(path, _optimizedPoses.at(signature->id()));
					}

					if(nearestId > 0)
					{
						// nearest pose must not be linked to current location and enough close
						if(!signature->hasLink(nearestId) &&
							(proximityFilteringRadius <= 0.0f ||
							 _optimizedPoses.at(signature->id()).getDistanceSquared(_optimizedPoses.at(nearestId)) < proximityFilteringRadius*proximityFilteringRadius))
						{
							++localVisualPathsChecked;
							RegistrationInfo info;
							Transform guess;
							if(_proximityOdomGuess)
							{
								// Use odometry as guess so that correspondences can be computed by projection
								guess = _optimizedPoses.at(nearestId).inverse()*_optimizedPoses.at(signature->id());
							} //else: guess is null to make sure visual correspondences are globally computed
							Transform transform = _memory->computeTransform(nearestId, signature->id(), guess, &info);
							if(!transform.isNull())
							{
								transform = transform.inverse();
								if(proximityFilteringRadius <= 0 || transform.getNormSquared() <= proximityFilteringRadius*proximityFilteringRadius)
								{
									UINFO("[Visual] Add local loop closure in SPACE (%d->%d) %s",
											signature->id(),
											nearestId,
											transform.prettyPrint().c_str());
									UASSERT(!info.covariance.empty() && info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);

									//for statistics
									loopClosureVisualInliersMeanDist = info.inliersMeanDistance;
									loopClosureVisualInliersDistribution = info.inliersDistribution;

									loopClosureVisualInliers = info.inliers;
									loopClosureVisualInliersRatio = info.inliersRatio;
									loopClosureVisualMatches = info.matches;
									loopClosureVisualVariance = info.variance;

									cv::Mat information = getInformation(info.covariance);
									loopClosureLinearVariance = 1.0/information.at<double>(0,0);
									loopClosureAngularVariance = 1.0/information.at<double>(5,5);

									Link::Type type = Link::kLocalSpaceClosure;
									bool suppressGlobalHypothesis = false;
									if(_loopClosureHypothesis.first>0 &&
										nearestIds.find(_loopClosureHypothesis.first)!=nearestIds.end())
									{
										// Avoid transform computation on the global loop closure if a visual proximity
										// one has been detected close (inside proximity radius) to that hypothesis.
										UDEBUG("Proximity detection on %d is close to loop closure %d, ignoring loop closure transform estimation...",
												nearestId, _loopClosureHypothesis.first);

										if(nearestId == _loopClosureHypothesis.first)
										{
											type = Link::kGlobalClosure;
											suppressGlobalHypothesis = true;
										}
										else if(loopIdSuppressedByProximity == 0)
										{
											suppressGlobalHypothesis = true;
										}
									}

									if(_safeRelocalizationGuardEnabled &&
											getSignatureResetEpoch(signature->id()) != getSignatureResetEpoch(nearestId))
									{
										UINFO("Observed but did not add cross-reset visual proximity closure %d->%d; guarded transaction is required",
												signature->id(), nearestId);
									}
									else if(_memory->addLink(Link(signature->id(), nearestId, type, transform, information)))
									{
										++proximityDetectionsAddedVisually;
										lastProximitySpaceClosureId = nearestId;
										if(suppressGlobalHypothesis)
										{
											loopIdSuppressedByProximity = nearestId;
										}
										loopClosureLinksAdded.push_back(std::make_pair(signature->id(), nearestId));
									}
									else
									{
										UWARN("Cannot add visual proximity closure %d->%d", signature->id(), nearestId);
									}
								}
								else
								{
									UWARN("Ignoring local loop closure with %d because resulting "
										  "transform is too large!? (%fm > %fm)",
											nearestId, transform.getNorm(), proximityFilteringRadius);
								}
							}
						}
						else if(!signature->hasLink(nearestId) && proximityFilteringRadius>0.0f)
						{
							UDEBUG("Skipping path %d as most likely ID %d is too far %f > %f (%s)",
								iter->first.id,
								nearestId,
								_optimizedPoses.at(signature->id()).getDistance(_optimizedPoses.at(nearestId)),
								proximityFilteringRadius,
								Parameters::kRGBDProximityPathFilteringRadius().c_str());
						}
					}
				}

				timeProximityBySpaceVisualDetection = timer.ticks();
				ULOGGER_INFO("timeProximityBySpaceVisualDetection=%fs", timeProximityBySpaceVisualDetection);

				//
				// 2) compare locally with nearest locations by scan matching
				//
				UDEBUG("Proximity detection (local loop closure in SPACE with scan matching)");
				if( _proximityMaxNeighbors <= 0)
				{
					UDEBUG("Proximity by scan matching is disabled (%s=%d).", Parameters::kRGBDProximityPathMaxNeighbors().c_str(), _proximityMaxNeighbors);
				}
				else if(!signature->sensorData().laserScanCompressed().isEmpty())
				{
					proximitySpacePaths = (int)nearestPaths.size();
					for(std::map<NearestPathKey, std::map<int, Transform> >::const_reverse_iterator iter=nearestPaths.rbegin();
							iter!=nearestPaths.rend() &&
							(_proximityMaxPaths <= 0 || localScanPathsChecked < _proximityMaxPaths);
							++iter)
					{
						std::map<int, Transform> path = iter->second; // should contain only nodes (no landmarks)
						UASSERT(path.size());
						UASSERT(path.begin()->first > 0);

						//find the nearest pose on the path
						int nearestId = rtabmap::graph::findNearestNode(path, _optimizedPoses.at(signature->id()));
						UASSERT(nearestId > 0);
						//UDEBUG("Path %d (size=%d) distance=%fm", nearestId, (int)path.size(), _optimizedPoses.at(signature->id()).getDistance(_optimizedPoses.at(nearestId)));

						// nearest pose must be close and not linked to current location
						if(!signature->hasLink(nearestId))
						{
							if(_proximityMaxNeighbors < _proximityMaxGraphDepth || _proximityMaxGraphDepth == 0)
							{
								std::map<int, Transform> filteredPath;
								int i=0;
								std::map<int, Transform>::iterator nearestIdIter = path.find(nearestId);
								// "_proximityMaxNeighbors-1" means that if _proximityMaxNeighbors=1,
								// only nearest node on the path is taken (no scan merging). Useful to find
								// proximity detection between only 2 nodes with 360x360 lidar scans.
								for(std::map<int, Transform>::iterator iter=nearestIdIter; iter!=path.end() && i<=_proximityMaxNeighbors-1; ++iter, ++i)
								{
									filteredPath.insert(*iter);
								}
								i=1;
								for(std::map<int, Transform>::reverse_iterator iter(nearestIdIter); iter!=path.rend() && i<=_proximityMaxNeighbors-1; ++iter, ++i)
								{
									filteredPath.insert(*iter);
								}
								path = filteredPath;
							}

							// Assemble scans in the path and do ICP only
							std::map<int, Transform> optimizedLocalPath;
							if(_globalScanMap.empty() && _proximityRawPosesUsed)
							{
								//optimize the path's poses locally
								cv::Mat covariance;
								path = optimizeGraph(nearestId, uKeysSet(path), std::map<int, Transform>(), false, covariance);
								// transform local poses in optimized graph referential
								if(!uContains(path, nearestId))
								{
									UERROR("Proximity path not containing nearest ID ?! Skipping this path.");
									continue;
								}
								Transform t = _optimizedPoses.at(nearestId) * path.at(nearestId).inverse();

								for(std::map<int, Transform>::iterator jter=path.lower_bound(1); jter!=path.end(); ++jter)
								{
									optimizedLocalPath.insert(std::make_pair(jter->first, t * jter->second));
								}
							}
							else
							{
								optimizedLocalPath = path;
							}

							std::map<int, Transform> filteredPath;
							if(_globalScanMap.empty() && optimizedLocalPath.size() > 2 && proximityFilteringRadius > 0.0f)
							{
								// path filtering
								filteredPath = graph::radiusPosesFiltering(optimizedLocalPath, proximityFilteringRadius, 0, true);
								// make sure the current pose is still here
								filteredPath.insert(*optimizedLocalPath.find(nearestId));
							}
							else
							{
								filteredPath = optimizedLocalPath;
							}

							if(filteredPath.size() > 0)
							{
								// add current node to poses
								filteredPath.insert(std::make_pair(signature->id(), _optimizedPoses.at(signature->id())));
								//The nearest will be the reference for a loop closure transform
								if(signature->getLinks().find(nearestId) == signature->getLinks().end())
								{
									++localScanPathsChecked;
									RegistrationInfo info;
									Transform transform;
									bool icpMulti = true;
									if(_globalScanMap.empty())
									{
										transform = _memory->computeIcpTransformMulti(signature->id(), nearestId, filteredPath, &info);
									}
									else
									{
										UASSERT_MSG(_globalScanMapPoses.find(nearestId) != _globalScanMapPoses.end(), uFormat("Pose of %d not found in global scan poses", nearestId).c_str());
										icpMulti = false;
										// use pre-assembled scan map
										SensorData assembledData;
										assembledData.setId(nearestId);
										assembledData.setLaserScan(
												LaserScan(_globalScanMap,
													signature->sensorData().laserScanCompressed().maxPoints(),
													signature->sensorData().laserScanCompressed().rangeMax(),
													_globalScanMapPoses.at(nearestId).inverse() * (signature->sensorData().laserScanCompressed().is2d()?Transform(0,0,signature->sensorData().laserScanCompressed().localTransform().z(),0,0,0):Transform::getIdentity())));
										Signature nearestNode(assembledData);
										Transform guess = filteredPath.at(nearestId).inverse() * filteredPath.at(signature->id());
										transform = _memory->computeIcpTransform(nearestNode, *signature, guess, &info);
										if(!transform.isNull())
										{
											transform = transform.inverse();
										}
									}

									if(!transform.isNull())
									{
										UINFO("[Scan matching] Add local loop closure in SPACE (%d->%d) %s",
												signature->id(),
												nearestId,
												transform.prettyPrint().c_str());

										cv::Mat scanMatchingIds;
										if(_scanMatchingIdsSavedInLinks)
										{
											std::stringstream stream;
											stream << "SCANS:";
											for(std::map<int, Transform>::iterator iter=optimizedLocalPath.begin(); iter!=optimizedLocalPath.end(); ++iter)
											{
												if(iter != optimizedLocalPath.begin())
												{
													stream << ";";
												}
												stream << uNumber2Str(iter->first);
											}
											std::string scansStr = stream.str();
											scanMatchingIds = cv::Mat(1, int(scansStr.size()+1), CV_8SC1, (void *)scansStr.c_str());
											scanMatchingIds = compressData2(scanMatchingIds); // compressed
										}

										// set Identify covariance for laser scan matching only
										UASSERT(!info.covariance.empty() && info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
										if(_safeRelocalizationGuardEnabled &&
												getSignatureResetEpoch(signature->id()) != getSignatureResetEpoch(nearestId))
										{
											UINFO("Observed but did not add cross-reset scan proximity closure %d->%d; guarded transaction is required",
													signature->id(), nearestId);
										}
										else if(_memory->addLink(Link(signature->id(), nearestId, Link::kLocalSpaceClosure, transform, getInformation(info.covariance)/_proximityMergedScanCovFactor, scanMatchingIds)))
										{
											loopClosureLinksAdded.push_back(std::make_pair(signature->id(), nearestId));
											if(icpMulti)
											{
												++proximityDetectionsAddedByICPMulti;
											}
											else
											{
												++proximityDetectionsAddedByICPGlobal;
											}

											// no local loop closure added visually
											if(proximityDetectionsAddedVisually == 0)
											{
												lastProximitySpaceClosureId = nearestId;
											}
										}
										else
										{
											UWARN("Cannot add scan proximity closure %d->%d", signature->id(), nearestId);
										}
									}
									else
									{
										UINFO("Local scan matching rejected: %s", info.rejectedMsg.c_str());
									}
									if(!_globalScanMap.empty())
									{
										break;
									}
								}
							}
						}
						else
						{
							//UDEBUG("Path %d ignored", nearestId);
						}
					}
				}
			}
		}
	}
	timeProximityBySpaceDetection = timer.ticks();
	ULOGGER_INFO("timeProximityBySpaceDetection=%fs", timeProximityBySpaceDetection);

	//=============================================================
	// Global loop closure detection
	// (updated: place this after retrieval to be sure that neighbors of the loop closure are in RAM)
	//=============================================================
	if(_loopClosureHypothesis.first>0)
	{
		if(loopIdSuppressedByProximity==0)
		{
			//Compute transform if metric data are present
			Transform transform;
			RegistrationInfo info;
			info.covariance = cv::Mat::eye(6,6,CV_64FC1);
			if(_rgbdSlamMode)
			{
				transform = _memory->computeTransform(
						_loopClosureHypothesis.first,
						signature->id(),
						_loopClosureIdentityGuess?Transform::getIdentity():Transform(),
						&info);

				loopClosureVisualInliersMeanDist = info.inliersMeanDistance;
				loopClosureVisualInliersDistribution = info.inliersDistribution;

				loopClosureVisualInliers = info.inliers;
				loopClosureVisualInliersRatio = info.inliersRatio;
				loopClosureVisualMatches = info.matches;
				loopClosureVisualVariance = info.variance;
				rejectedLoopClosure = transform.isNull();
				if(rejectedLoopClosure)
				{
					UWARN("Rejected loop closure %d -> %d: %s",
							_loopClosureHypothesis.first, signature->id(), info.rejectedMsg.c_str());
				}
				else if(_maxLoopClosureDistance>0.0f && transform.getNorm() > _maxLoopClosureDistance)
				{
					rejectedLoopClosure = true;
					UWARN("Rejected localization %d -> %d because distance to map (%fm) is over %s=%fm.",
							_loopClosureHypothesis.first, signature->id(), transform.getNorm(), Parameters::kRGBDMaxLoopClosureDistance().c_str(), _maxLoopClosureDistance);
				}
				else
				{
					transform = transform.inverse();
				}
			}
			if(!rejectedLoopClosure)
			{
				if(_safeRelocalizationGuardEnabled && _rgbdSlamMode)
				{
					const unsigned long long currentResetEpoch = getSignatureResetEpoch(signature->id());
					const unsigned long long oldResetEpoch = getSignatureResetEpoch(_loopClosureHypothesis.first);
					if(currentResetEpoch != oldResetEpoch)
					{
						rejectedLoopClosure = true;
						UINFO("Observed but did not add cross-reset global loop %d -> %d (epochs %llu -> %llu): safe relocalization requires the guarded JSON transaction",
								signature->id(), _loopClosureHypothesis.first,
								currentResetEpoch, oldResetEpoch);
					}
				}
				if(!rejectedLoopClosure && _crossMapLoopPending && _rgbdSlamMode)
				{
					const Signature * oldSignature = _memory->getSignature(_loopClosureHypothesis.first);
					if(oldSignature == 0)
					{
						rejectedLoopClosure = true;
						UWARN("Rejected global loop %d -> %d by cross-map pending gate: old signature is not in memory.",
								_loopClosureHypothesis.first, signature->id());
					}
					else
					{
						std::map<int, unsigned long long>::const_iterator currentEpochIter =
								_signatureResetEpochs.find(signature->id());
						std::map<int, unsigned long long>::const_iterator oldEpochIter =
								_signatureResetEpochs.find(oldSignature->id());
						const unsigned long long currentResetEpoch =
								currentEpochIter == _signatureResetEpochs.end()? 0 : currentEpochIter->second;
						const unsigned long long oldResetEpoch =
								oldEpochIter == _signatureResetEpochs.end()? 0 : oldEpochIter->second;
						const bool crossesRiskBoundary =
								signature->mapId() != oldSignature->mapId() ||
								currentResetEpoch != oldResetEpoch;
						if(crossesRiskBoundary)
						{
							std::string pendingReason;
							bool accepted = acceptCrossMapLoopEvidence(
									*signature, *oldSignature,
									currentResetEpoch, oldResetEpoch,
									transform, info, pendingReason);
							if(!accepted)
							{
								rejectedLoopClosure = true;
								UINFO("Pending/rejected cross-epoch global loop %d -> %d: %s",
										_loopClosureHypothesis.first, signature->id(), pendingReason.c_str());
							}
							else
							{
								UINFO("Accepted cross-epoch global loop %d -> %d: %s",
										_loopClosureHypothesis.first, signature->id(), pendingReason.c_str());
							}
						}
					}
				}
			}
			if(!rejectedLoopClosure)
			{
				// Make the new one the parent of the old one
				UASSERT(!info.covariance.empty() && info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
				
				loopClosureLinearVariance = uMax3(info.covariance.at<double>(0,0), info.covariance.at<double>(1,1)>=9999?0:info.covariance.at<double>(1,1), info.covariance.at<double>(2,2)>=9999?0:info.covariance.at<double>(2,2));
				loopClosureAngularVariance = uMax3(info.covariance.at<double>(3,3)>=9999?0:info.covariance.at<double>(3,3), info.covariance.at<double>(4,4)>=9999?0:info.covariance.at<double>(4,4), info.covariance.at<double>(5,5));
				cv::Mat information = getInformation(info.covariance);
				rejectedLoopClosure = !_memory->addLink(Link(signature->id(), _loopClosureHypothesis.first, Link::kGlobalClosure, transform, information));
				if(!rejectedLoopClosure)
				{
					loopClosureLinksAdded.push_back(std::make_pair(signature->id(), _loopClosureHypothesis.first));
				}
			}

			if(rejectedLoopClosure)
			{
				_loopClosureHypothesis.first = 0;
			}
		}
		else if(loopIdSuppressedByProximity != _loopClosureHypothesis.first)
		{
			_loopClosureHypothesis.first = 0;
		}
	}

	timeAddLoopClosureLink = timer.ticks();
	ULOGGER_INFO("timeAddLoopClosureLink=%fs", timeAddLoopClosureLink);

	//============================================================
	// Landmark
	//============================================================
	std::map<int, std::set<int> > landmarksDetected; // <Landmark ID, list of nodes that saw this landmark>
	if(!signature->getLandmarks().empty() && !_graphOptimizer->landmarksIgnored() &&
			!_safeRelocalizationGuardEnabled && signature->getWeight()!=-1)
	{
		bool hasGlobalLoopClosuresInOdomCache = !graph::filterLinks(_odomCacheConstraints, Link::kGlobalClosure, true).empty() || _loopClosureHypothesis.first != 0;
		UDEBUG("hasGlobalLoopClosuresInOdomCache=%d", hasGlobalLoopClosuresInOdomCache?1:0);
		for(std::map<int, Link>::const_iterator iter=signature->getLandmarks().begin(); iter!=signature->getLandmarks().end(); ++iter)
		{
			if(uContains(_memory->getLandmarksIndex(), iter->first) &&
					_memory->getLandmarksIndex().find(iter->first)->second.size()>1)
			{
				if(!_memory->isIncremental() &&          // In localization mode
					!hasGlobalLoopClosuresInOdomCache && // If there are global loop closures in odom cache, we can keep far landmarks
					_localRadius>0.0 &&
					iter->second.transform().getNormSquared() > _localRadius*_localRadius)
				{
					// Ignore landmark detections over local radius
					UWARN("Ignoring landmark %d for localization as it is too far (%fm > %s=%f) "
							"and odom cache doesn't contain global loop closure(s).",
							iter->first,
							iter->second.transform().getNorm(),
							Parameters::kRGBDLocalRadius().c_str(),
							_localRadius);
				}
				else
				{
					UINFO("Landmark %d observed again! Seen the first time by node %d.", -iter->first, *_memory->getLandmarksIndex().find(iter->first)->second.begin());
					landmarksDetected.insert(std::make_pair(iter->first, _memory->getLandmarksIndex().find(iter->first)->second));
					rejectedLoopClosure = false; // If it was true, it will be set back to false if landmarks are rejected on graph optimization
					loopClosureLinksAdded.push_back(std::make_pair(signature->id(), iter->first));
				}
			}
		}
	}

	//============================================================
	// Add virtual links if a path is activated
	//============================================================
	if(_path.size())
	{
		// Add a virtual loop closure link to keep the path linked to local map
		if( signature->id() != _path[_pathCurrentIndex].first &&
			!signature->hasLink(_path[_pathCurrentIndex].first))
		{
			UASSERT(uContains(_optimizedPoses, signature->id()));
			UASSERT_MSG(uContains(_optimizedPoses, _path[_pathCurrentIndex].first), uFormat("id=%d", _path[_pathCurrentIndex].first).c_str());
			Transform virtualLoop = _optimizedPoses.at(signature->id()).inverse() * _optimizedPoses.at(_path[_pathCurrentIndex].first);
			if(_safeRelocalizationGuardEnabled &&
					getSignatureResetEpoch(signature->id()) !=
					getSignatureResetEpoch(_path[_pathCurrentIndex].first))
			{
				UINFO("Did not add cross-reset virtual path link %d->%d; guarded transaction is the only cross-reset graph mutation path",
						signature->id(), _path[_pathCurrentIndex].first);
			}
			else
			{
				_memory->addLink(Link(signature->id(), _path[_pathCurrentIndex].first, Link::kVirtualClosure, virtualLoop, cv::Mat::eye(6,6,CV_64FC1)*0.01)); // set high variance
			}
		}
	}

	//============================================================
	// Optimize map graph
	//============================================================
	graph::MaxGraphErrors maxGraphErrors;
	std::pair<int, int> maxGraphErrorsLinearIds(0,0);
	std::pair<int, int> maxGraphErrorsAngularIds(0,0);
	std::pair<int, int> maxGraphErrorsRemovedIds(0,0);
	int maxGraphErrorsRemovedCount = 0;
	double optimizationError = 0.0;
	int optimizationIterations = 0;
	Transform previousMapCorrection;
	bool delayedLocalization = false;
	int odomCacheProximityLinksCleared = 0;
	UDEBUG("RGB-D SLAM mode: %d", _rgbdSlamMode?1:0);
	UDEBUG("Incremental: %d", _memory->isIncremental());
	UDEBUG("Loop hyp: %d", _loopClosureHypothesis.first);
	UDEBUG("Last prox: %d", lastProximitySpaceClosureId);
	UDEBUG("Reduced ids: %d", (int)statistics_.reducedIds().size());
	UDEBUG("Has prior: %d (prior ignored=%d)", signature->hasLink(signature->id(), Link::kPosePrior)?1:0, _graphOptimizer->priorsIgnored()?1:0);
	UDEBUG("Has gravity: %d (sigma=%f, odomGravity=%d, refined=%d)", signature->hasLink(signature->id(), Link::kGravity)?1:0, _graphOptimizer->gravitySigma(), _memory->isOdomGravityUsed()?1:0, neighborLinkRefined?1:0);
	UDEBUG("Has virtual link: %d", (int)graph::filterLinks(signature->getLinks(), Link::kVirtualClosure, true).size());
	UDEBUG("Prox Time: %d", proximityDetectionsInTimeFound);
	UDEBUG("Landmarks: %d", (int)landmarksDetected.size());
	UDEBUG("Retrieved: %d", (int)signaturesRetrieved.size());
	UDEBUG("Not self ref links: %d", (int)graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size());

	if(_rgbdSlamMode
		&&
		signature->getWeight() != -1 // Ignore graph optimization on intermediate nodes
		&&
		(_loopClosureHypothesis.first>0 ||
	     lastProximitySpaceClosureId>0 || // can be different map of the current one
	     statistics_.reducedIds().size() ||
		 (signature->hasLink(signature->id(), Link::kPosePrior) && !_graphOptimizer->priorsIgnored()) || // prior edge
		 (signature->hasLink(signature->id(), Link::kGravity) && _graphOptimizer->gravitySigma()>0.0f && (!_memory->isOdomGravityUsed() || neighborLinkRefined)) || // gravity edge
	     proximityDetectionsInTimeFound>0 ||
		 !landmarksDetected.empty() ||
		 signaturesRetrieved.size()) // can be different map of the current one
		 &&
		 (_memory->isIncremental() ||
		  // In localization mode, the new node should be linked to another node or a landmark already in the working memory
		  graph::filterLinks(graph::filterLinks(signature->getLinks(), Link::kVirtualClosure), Link::kSelfRefLink).size() ||
		  !landmarksDetected.empty()))
	{
		UASSERT(uContains(_optimizedPoses, signature->id()));

		//used in localization mode: filter virtual links
		std::multimap<int, Link> localizationLinks = graph::filterLinks(signature->getLinks(), Link::kVirtualClosure);
		localizationLinks = graph::filterLinks(localizationLinks, Link::kSelfRefLink);
		if(!landmarksDetected.empty() && !_memory->isIncremental())
		{
			for(std::map<int, std::set<int> >::iterator iter=landmarksDetected.begin(); iter!=landmarksDetected.end(); ++iter)
			{
				if(_optimizedPoses.find(iter->first)!=_optimizedPoses.end())
				{
					UASSERT(uContains(signature->getLandmarks(), iter->first));
					localizationLinks.insert(std::make_pair(iter->first, signature->getLandmarks().at(iter->first)));
				}
			}
		}

		bool allLocalizationLinksInGraph = !localizationLinks.empty();
		for(std::multimap<int, Link>::iterator iter=localizationLinks.begin(); iter!=localizationLinks.end(); ++iter)
		{
			if(!uContains(_optimizedPoses, iter->first))
			{
				allLocalizationLinksInGraph = false;
				break;
			}
		}

		// Note that in localization mode, we don't re-optimize the graph
		// if:
		//  1- there are no signatures retrieved,
		//  2- we are relocalizing on a node already in the optimized graph
		if(!_memory->isIncremental() &&
		   signaturesRetrieved.empty() &&
		   !localizationLinks.empty() &&
		   allLocalizationLinksInGraph)
		{
			bool rejectLocalization = _odomCachePoses.empty();
			if(!_odomCachePoses.empty())
			{
				// Verify if the new localization is valid by checking if there is
				// not too much deformation using current odometry poses
				// This will also refine localization links

				std::map<int, Transform> poses = _odomCachePoses;
				std::multimap<int, Link> constraints = _odomCacheConstraints;
				// add self referring links (e.g., gravity)
				std::multimap<int, Link> selfLinks = graph::filterLinks(signature->getLinks(), Link::kSelfRefLink, true);
				if(_graphOptimizer->priorsIgnored())
				{
					selfLinks = graph::filterLinks(selfLinks, Link::kPosePrior);
				}
				constraints.insert(selfLinks.begin(), selfLinks.end());
				for(std::multimap<int, Link>::iterator iter=localizationLinks.begin(); iter!=localizationLinks.end(); ++iter)
				{
					constraints.insert(std::make_pair(iter->second.from(), iter->second));
				}

				cv::Mat priorInfMat = cv::Mat::eye(6,6, CV_64FC1)*_localizationPriorInf;
				for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end(); ++iter)
				{
					std::map<int, Transform>::iterator iterPose = _optimizedPoses.find(iter->second.to());
					if(iterPose != _optimizedPoses.end() && poses.find(iterPose->first) == poses.end())
					{
						poses.insert(*iterPose);

						// make the poses in the map fixed
						constraints.insert(std::make_pair(iterPose->first, Link(iterPose->first, iterPose->first, Link::kPosePrior, iterPose->second, priorInfMat)));
						UDEBUG("Constraint %d->%d: %s (type=%s, var=%f)", iterPose->first, iterPose->first, iterPose->second.prettyPrint().c_str(), Link::typeName(Link::kPosePrior).c_str(), 1./_localizationPriorInf);
					}
					UDEBUG("Constraint %d->%d: %s (type=%s, var = %f %f)", iter->second.from(), iter->second.to(), iter->second.transform().prettyPrint().c_str(), iter->second.typeName().c_str(), iter->second.transVariance(), iter->second.rotVariance());
				}

				std::map<int, Transform> posesOut;
				std::multimap<int, Link> edgeConstraintsOut;

				bool priorsIgnored = _graphOptimizer->priorsIgnored();
				UDEBUG("priorsIgnored was %s", priorsIgnored?"true":"false");
				_graphOptimizer->setPriorsIgnored(false); //temporary set false to use priors above to fix nodes of the map

				// If slam2d: get connected graph while keeping original roll,pitch,z values.
				_graphOptimizer->getConnectedGraph(signature->id(), poses, constraints, posesOut, edgeConstraintsOut);
				
				if(ULogger::level() == ULogger::kDebug)
				{
					for(std::map<int, Transform>::iterator iter=posesOut.begin(); iter!=posesOut.end(); ++iter)
					{
						UDEBUG("Pose %d %s", iter->first, iter->second.prettyPrint().c_str());
					}
				}
				cv::Mat locOptCovariance;
				std::map<int, Transform> optPoses;
				if(!posesOut.empty() &&
				   posesOut.begin()->first < _odomCachePoses.begin()->first)
				{
					optPoses = _graphOptimizer->optimize(posesOut.begin()->first, posesOut, edgeConstraintsOut, locOptCovariance, 0, &optimizationError, &optimizationIterations);
				}
				else
				{
					UERROR("Invalid localization constraints");
				}
				_graphOptimizer->setPriorsIgnored(priorsIgnored); // set back
				for(std::map<int, Transform>::iterator iter=optPoses.begin(); iter!=optPoses.end(); ++iter)
				{
					UDEBUG("Opt  %d %s", iter->first, iter->second.prettyPrint().c_str());
				}

				std::list<std::pair<int, int> > removedLinks;
				if(optPoses.empty())
				{
					UWARN("Optimization failed, rejecting localization!");
					rejectLocalization = true;
				}
				else
				{
					UINFO("Compute max graph errors...");
					maxGraphErrors = graph::computeMaxGraphErrors(
							optPoses,
							edgeConstraintsOut,
							_graphOptimizer->isSlam2d());
					if(!maxGraphErrors.linearLink.isValid() && !maxGraphErrors.angularLink.isValid())
					{
						UWARN("Could not compute graph errors! Rejecting localization!");
						rejectLocalization = true;
					}

					if(maxGraphErrors.linearLink.isValid())
					{
						maxGraphErrorsLinearIds = std::make_pair(maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to());
						UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
								maxGraphErrors.linear,
								maxGraphErrors.linearLink.from(),
								maxGraphErrors.linearLink.to(),
								maxGraphErrors.linearLink.transVariance(),
								maxGraphErrors.linear/sqrt(maxGraphErrors.linearLink.transVariance()),
								_optimizationMaxError);
						if(_optimizationMaxError > 0.0f && maxGraphErrors.linearRatio > _optimizationMaxError)
						{
							if( _optimizationMaxErrorRepairRadius > 0.0 &&
								maxGraphErrorsLinearIds == _lastRejectedLoopClosureIds &&
								graph::findLink(edgeConstraintsOut, maxGraphErrorsLinearIds.first, maxGraphErrorsLinearIds.second) != edgeConstraintsOut.end())
							{
								UWARN("We detected 2 consecutive loop closure rejections because of the same loop closure link (%d->%d), trying optimization again without that link...",
									maxGraphErrorsLinearIds.first, maxGraphErrorsLinearIds.second);
	
								UDEBUG("priorsIgnored was %s", priorsIgnored?"true":"false");
								_graphOptimizer->setPriorsIgnored(false); //temporary set false to use priors above to fix nodes of the map
								removedLinks = repairGraph(
									maxGraphErrors,
									optPoses,
									edgeConstraintsOut,
									optimizationError,
									optimizationIterations,
									locOptCovariance);
								_graphOptimizer->setPriorsIgnored(priorsIgnored); // set back
	
								if(removedLinks.empty())
								{
									UWARN("Optimization failed when trying to repair the graph.");
									rejectLocalization = true;
								}
							}
							else {
								rejectLocalization = true;
							}
	
							if(rejectLocalization)
							{
								UWARN("Rejecting localization (%d <-> %d) in this "
										"iteration because a wrong loop closure has been "
										"detected after graph optimization, resulting in "
										"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
										"maximum error ratio parameter \"%s\" is %f of std deviation.",
										localizationLinks.rbegin()->second.from(),
										localizationLinks.rbegin()->second.to(),
										maxGraphErrors.linearRatio,
										maxGraphErrors.linearLink.from(),
										maxGraphErrors.linearLink.to(),
										maxGraphErrors.linearLink.type(),
										maxGraphErrors.linear,
										sqrt(maxGraphErrors.linearLink.transVariance()),
										Parameters::kRGBDOptimizeMaxError().c_str(),
										_optimizationMaxError);
								
								if(maxGraphErrors.linearLink.type() != Link::kNeighbor)
								{
									_lastRejectedLoopClosureIds = maxGraphErrorsLinearIds;
								}
							}
						}
						else if(_optimizationMaxError == 0.0f && maxGraphErrors.linearRatio>100 && !_graphOptimizer->isRobust())
						{
							UERROR("Huge optimization error detected!"
									"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
									"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
									maxGraphErrors.linearRatio,
									maxGraphErrors.linearLink.from(),
									maxGraphErrors.linearLink.to(),
									maxGraphErrors.linearLink.type(),
									maxGraphErrors.linear,
									sqrt(maxGraphErrors.linearLink.transVariance()),
									Parameters::kRGBDOptimizeMaxError().c_str());
						}
					}
					if(maxGraphErrors.angularLink.isValid())
					{
						maxGraphErrorsAngularIds = std::make_pair(maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to());
						UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
								maxGraphErrors.angular*180.0f/CV_PI,
								maxGraphErrors.angularLink.from(),
								maxGraphErrors.angularLink.to(),
								maxGraphErrors.angularLink.rotVariance(),
								maxGraphErrors.angular/sqrt(maxGraphErrors.angularLink.rotVariance()),
								_optimizationMaxError);
						if(_optimizationMaxError > 0.0f && maxGraphErrors.angularRatio > _optimizationMaxError)
						{
							UWARN("Rejecting localization (%d <-> %d) in this "
									"iteration because a wrong loop closure has been "
									"detected after graph optimization, resulting in "
									"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
									"maximum error ratio parameter \"%s\" is %f of std deviation.",
									localizationLinks.rbegin()->second.from(),
									localizationLinks.rbegin()->second.to(),
									maxGraphErrors.angularRatio,
									maxGraphErrors.angularLink.from(),
									maxGraphErrors.angularLink.to(),
									maxGraphErrors.angularLink.type(),
									maxGraphErrors.angular*180.0f/CV_PI,
									sqrt(maxGraphErrors.angularLink.rotVariance()),
									Parameters::kRGBDOptimizeMaxError().c_str(),
									_optimizationMaxError);
							rejectLocalization = true;
						}
						else if(_optimizationMaxError == 0.0f && maxGraphErrors.angularRatio>100 && !_graphOptimizer->isRobust())
						{
							UERROR("Huge optimization error detected!"
									"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
									"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
									maxGraphErrors.angularRatio,
									maxGraphErrors.angularLink.from(),
									maxGraphErrors.angularLink.to(),
									maxGraphErrors.angularLink.type(),
									maxGraphErrors.angular*180.0f/CV_PI,
									sqrt(maxGraphErrors.angularLink.rotVariance()),
									Parameters::kRGBDOptimizeMaxError().c_str());
						}
					}
				}

				bool hasGlobalLoopClosuresOrLandmarks = false;
				if(rejectLocalization && 
					(_localizationSecondTryWithoutProximityLinks && !graph::filterLinks(constraints, Link::kLocalSpaceClosure, true).empty()))
				{
					// Let's try again without local loop closures
					localizationLinks = graph::filterLinks(localizationLinks, Link::kLocalSpaceClosure);
					constraints = graph::filterLinks(constraints, Link::kLocalSpaceClosure);
					for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end() && !hasGlobalLoopClosuresOrLandmarks; ++iter)
					{
						hasGlobalLoopClosuresOrLandmarks =
								iter->second.type() == Link::kGlobalClosure ||
								iter->second.type() == Link::kLandmark;
					}
					if(hasGlobalLoopClosuresOrLandmarks && !localizationLinks.empty())
					{
						rejectLocalization = false;
						UWARN("Global and loop closures seem not tallying together, try again to optimize without local loop closures...");
						UDEBUG("priorsIgnored was %s", priorsIgnored?"true":"false");
						_graphOptimizer->setPriorsIgnored(false); //temporary set false to use priors above to fix nodes of the map
						// If slam2d: get connected graph while keeping original roll,pitch,z values.
						_graphOptimizer->getConnectedGraph(signature->id(), poses, constraints, posesOut, edgeConstraintsOut);
						optPoses.clear();
						if(!posesOut.empty() &&
						   posesOut.begin()->first < _odomCachePoses.begin()->first)
						{
							optPoses = _graphOptimizer->optimize(posesOut.begin()->first, posesOut, edgeConstraintsOut, locOptCovariance, 0, &optimizationError, &optimizationIterations);
						}
						else
						{
							UERROR("Invalid localization constraints");
						}
						_graphOptimizer->setPriorsIgnored(priorsIgnored); // set back
						for(std::map<int, Transform>::iterator iter=optPoses.begin(); iter!=optPoses.end(); ++iter)
						{
							UDEBUG("Opt2  %d %s", iter->first, iter->second.prettyPrint().c_str());
						}

						if(optPoses.empty())
						{
							UWARN("Optimization failed, rejecting localization!");
							rejectLocalization = true;
						}
						else
						{
							UINFO("Compute max graph errors...");
							maxGraphErrors = graph::computeMaxGraphErrors(
									optPoses,
									edgeConstraintsOut,
									_graphOptimizer->isSlam2d());
							if(!maxGraphErrors.linearLink.isValid() && !maxGraphErrors.angularLink.isValid())
							{
								UWARN("Could not compute graph errors! Rejecting localization!");
								rejectLocalization = true;
							}

							if(maxGraphErrors.linearLink.isValid())
							{
								maxGraphErrorsLinearIds = std::make_pair(maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to());
								UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
										maxGraphErrors.linear,
										maxGraphErrors.linearLink.from(),
										maxGraphErrors.linearLink.to(),
										maxGraphErrors.linearLink.transVariance(),
										maxGraphErrors.linear/sqrt(maxGraphErrors.linearLink.transVariance()),
										_optimizationMaxError);
								if(_optimizationMaxError > 0.0f && maxGraphErrors.linearRatio > _optimizationMaxError)
								{
									UWARN("Rejecting localization (%d <-> %d) in this "
											"iteration because a wrong loop closure has been "
											"detected after graph optimization, resulting in "
											"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
											"maximum error ratio parameter \"%s\" is %f of std deviation.",
											localizationLinks.rbegin()->second.from(),
											localizationLinks.rbegin()->second.to(),
											maxGraphErrors.linearRatio,
											maxGraphErrors.linearLink.from(),
											maxGraphErrors.linearLink.to(),
											maxGraphErrors.linearLink.type(),
											maxGraphErrors.linear,
											sqrt(maxGraphErrors.linearLink.transVariance()),
											Parameters::kRGBDOptimizeMaxError().c_str(),
											_optimizationMaxError);
									rejectLocalization = true;
								}
								else if(_optimizationMaxError == 0.0f && maxGraphErrors.linearRatio>100 && !_graphOptimizer->isRobust())
								{
									UERROR("Huge optimization error detected!"
											"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
											"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
											maxGraphErrors.linearRatio,
											maxGraphErrors.linearLink.from(),
											maxGraphErrors.linearLink.to(),
											maxGraphErrors.linearLink.type(),
											maxGraphErrors.linear,
											sqrt(maxGraphErrors.linearLink.transVariance()),
											Parameters::kRGBDOptimizeMaxError().c_str());
								}
							}
							if(maxGraphErrors.angularLink.isValid())
							{
								maxGraphErrorsAngularIds = std::make_pair(maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to());
								UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
										maxGraphErrors.angular*180.0f/CV_PI,
										maxGraphErrors.angularLink.from(),
										maxGraphErrors.angularLink.to(),
										maxGraphErrors.angularLink.rotVariance(),
										maxGraphErrors.angular/sqrt(maxGraphErrors.angularLink.rotVariance()),
										_optimizationMaxError);
								if(_optimizationMaxError > 0.0f && maxGraphErrors.angularRatio > _optimizationMaxError)
								{
									UWARN("Rejecting localization (%d <-> %d) in this "
											"iteration because a wrong loop closure has been "
											"detected after graph optimization, resulting in "
											"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
											"maximum error ratio parameter \"%s\" is %f of std deviation.",
											localizationLinks.rbegin()->second.from(),
											localizationLinks.rbegin()->second.to(),
											maxGraphErrors.angularRatio,
											maxGraphErrors.angularLink.from(),
											maxGraphErrors.angularLink.to(),
											maxGraphErrors.angularLink.type(),
											maxGraphErrors.angular*180.0f/CV_PI,
											sqrt(maxGraphErrors.angularLink.rotVariance()),
											Parameters::kRGBDOptimizeMaxError().c_str(),
											_optimizationMaxError);
									rejectLocalization = true;
								}
								else if(_optimizationMaxError == 0.0f && maxGraphErrors.angularRatio>100 && !_graphOptimizer->isRobust())
								{
									UERROR("Huge optimization error detected!"
											"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
											"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
											maxGraphErrors.angularRatio,
											maxGraphErrors.angularLink.from(),
											maxGraphErrors.angularLink.to(),
											maxGraphErrors.angularLink.type(),
											maxGraphErrors.angular*180.0f/CV_PI,
											sqrt(maxGraphErrors.angularLink.rotVariance()),
											Parameters::kRGBDOptimizeMaxError().c_str());
								}
							}
						}
					}
				}

				if(!rejectLocalization)
				{
					if(hasGlobalLoopClosuresOrLandmarks)
					{
						// We successfully optimize the graph without local loop closures,
						// clear them as some of them may be wrong.
						size_t before = _odomCacheConstraints.size();
						_odomCacheConstraints = graph::filterLinks(_odomCacheConstraints, Link::kLocalSpaceClosure);
						if(before != _odomCacheConstraints.size())
						{
							UWARN("Successfully optimized without local loop closures! Clearing them from local odometry cache. %ld/%ld have been removed.",
									before - _odomCacheConstraints.size(), before);
						}
						else
						{
							UWARN("Successfully optimized without local loop closures!");
						}
						odomCacheProximityLinksCleared = before - _odomCacheConstraints.size();
					}
					else if(!removedLinks.empty())
					{
						// If we removed links (repaired the graph)
						for(auto link: removedLinks)
						{
							UWARN("Removing link %d->%d from odometry cache", link.first, link.second);
							auto iter = graph::findLink(_odomCacheConstraints, link.first, link.second);
							if(iter!=_odomCacheConstraints.end()) {
								_odomCacheConstraints.erase(iter);
							}
						}
						UWARN("Successfully repaired the graph.");
						maxGraphErrorsRemovedIds = removedLinks.front();
						maxGraphErrorsRemovedCount = removedLinks.size();
					}

					// Count how many localization links are in the constraints
					bool hadAlreadyLocalizationLinks = false;
					for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin();
							iter!=_odomCacheConstraints.end(); ++iter)
					{
						if(iter->second.type() == Link::kGlobalClosure ||
						   iter->second.type() == Link::kLocalSpaceClosure ||
						   iter->second.type() == Link::kLocalTimeClosure ||
						   iter->second.type() == Link::kUserClosure ||
						   iter->second.type() == Link::kNeighborMerged ||
						   iter->second.type() == Link::kLandmark)
						{
							hadAlreadyLocalizationLinks = true;
							break;
						}
					}

					// update localization links
					UASSERT(uContains(optPoses, signature->id()));
					Transform newOptPoseInv = optPoses.at(signature->id()).inverse();
					for(std::multimap<int, Link>::iterator iter=localizationLinks.begin(); iter!=localizationLinks.end(); ++iter)
					{
						if(!_localizationSmoothing)
						{
							// Add original link without optimization
							UDEBUG("Adding new odom cache constraint %d->%d (%s)", 
								iter->second.from(), iter->second.to(), iter->second.transform().prettyPrint().c_str());
						}
						else
						{
							// Adjust with optimized poses, this will smooth the localization
							UASSERT(uContains(optPoses, iter->first));
							Transform newT = newOptPoseInv * optPoses.at(iter->first);
							UDEBUG("Adjusted localization link %d->%d after optimization", iter->second.from(), iter->second.to());
							UDEBUG("from %s", iter->second.transform().prettyPrint().c_str());
							UDEBUG("  to %s", newT.prettyPrint().c_str());
							iter->second.setTransform(newT);

							// Update link in the referred signatures
							if(iter->first > 0)
								_memory->updateLink(iter->second, false);
						}
						
						_odomCacheConstraints.insert(std::make_pair(signature->id(), iter->second));
					}

					_odomCacheConstraints.insert(selfLinks.begin(), selfLinks.end());

					// At least 2 localizations at 2 different time required
					if(hadAlreadyLocalizationLinks || _maxOdomCacheSize == 0)
					{
						UINFO("Update localization");

						// update odomCachePoses with optimized poses (but make sure to put them back in odom frame)
						Transform mapToOdomCache = signature->getPose() * newOptPoseInv;
						for(std::map<int, Transform>::iterator iter = _odomCachePoses.begin(); iter!=_odomCachePoses.end(); ++iter)
						{
							iter->second = mapToOdomCache * optPoses.at(iter->first);
						}

						if(_optimizeFromGraphEnd)
						{
							// update all previous nodes
							// Normally _mapCorrection should be identity, but if _optimizeFromGraphEnd
							// parameters just changed state, we should put back all poses without map correction.
							Transform oldPose = _optimizedPoses.at(localizationLinks.rbegin()->first);
							Transform mapCorrectionInv = _mapCorrection.inverse();
							Transform u = signature->getPose() * localizationLinks.rbegin()->second.transform();
							if(_graphOptimizer->gravitySigma() > 0)
							{
                                // Adjust transform with gravity
                                Transform transform = localizationLinks.rbegin()->second.transform();
                                int loopId = localizationLinks.rbegin()->first;
                                int landmarkId = 0;
                                if(loopId<0)
                                {
                                    //For landmarks, use transform against other node looking the landmark
                                    // (because we don't assume that landmarks are aligned with gravity)
                                    landmarkId = loopId;
                                    UASSERT(landmarksDetected.find(landmarkId) != landmarksDetected.end() &&
                                            !landmarksDetected.at(landmarkId).empty());
                                    loopId = *landmarksDetected.at(landmarkId).begin();
                                }

                                const Signature * loopS = _memory->getSignature(loopId);
                                UASSERT(loopS !=0);
                                std::multimap<int, Link>::const_iterator iterGravityLoop = graph::findLink(loopS->getLinks(), loopS->id(), loopS->id(), false, Link::kGravity);
                                std::multimap<int, Link>::const_iterator iterGravitySign = graph::findLink(signature->getLinks(), signature->id(), signature->id(), false, Link::kGravity);
                                if(iterGravityLoop!=loopS->getLinks().end() &&
                                   iterGravitySign!=signature->getLinks().end())
                                {
                                    float roll,pitch,yaw;
                                    if(landmarkId < 0)
                                    {
                                        iterGravityLoop->second.transform().getEulerAngles(roll, pitch, yaw);
                                        Transform gravityCorr = Transform(_optimizedPoses.at(loopS->id()).x(),
                                                                              _optimizedPoses.at(loopS->id()).y(),
                                                                              _optimizedPoses.at(loopS->id()).z(),
                                                                              roll, pitch, _optimizedPoses.at(loopS->id()).theta()) * _optimizedPoses.at(loopS->id()).inverse();
                                        (gravityCorr * _optimizedPoses.at(landmarkId)).getEulerAngles(roll,pitch,yaw);
                                    }
                                    else
                                    {
                                        iterGravityLoop->second.transform().getEulerAngles(roll, pitch, yaw);
                                    }
                                    Transform targetRotation = iterGravitySign->second.transform().rotation()*transform.rotation();
                                    targetRotation = Transform(0,0,0,roll,pitch,targetRotation.theta());
                                    Transform error = transform.rotation().inverse() * iterGravitySign->second.transform().rotation().inverse() * targetRotation;
                                    transform *= error;
                                    u  = signature->getPose() * transform;
                                }
                                else if(iterGravityLoop!=loopS->getLinks().end() ||
                                        iterGravitySign!=signature->getLinks().end())
                                {
                                    UWARN("Gravity link not found for %d or %d, localization won't be corrected with gravity.", loopId, signature->id());
                                }
							}
							Transform up = u * oldPose.inverse();
							if(_graphOptimizer->isSlam2d())
							{
								// in case of 3d landmarks, transform constraint to 2D
								up.to3DoF();
							}
							for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
							{
								iter->second = mapCorrectionInv * up * iter->second;
							}
							_optimizedPoses.at(signature->id()) = signature->getPose();
						}
						else
						{
							Transform newPose = _optimizedPoses.at(localizationLinks.rbegin()->first) * localizationLinks.rbegin()->second.transform().inverse();
							UDEBUG("newPose=%s", newPose.prettyPrint().c_str());
							if(_graphOptimizer->isSlam2d() && signature->getPose().is3DoF())
							{
								// in case of 3d landmarks, transform constraint to 2D
								newPose = newPose.to3DoF();
								UDEBUG("newPose 2D=%s", newPose.prettyPrint().c_str());
							}
							else if(_graphOptimizer->gravitySigma() > 0)
							{
								// Adjust transform with gravity
								std::multimap<int, Link>::const_iterator iterGravitySign = graph::findLink(signature->getLinks(), signature->id(), signature->id(), false, Link::kGravity);
								if(iterGravitySign!=signature->getLinks().end())
								{
                                    Transform transform = localizationLinks.rbegin()->second.transform();
									float roll,pitch,yaw;
									float tmp1,tmp2;
									UDEBUG("Gravity link = %s", iterGravitySign->second.transform().prettyPrint().c_str());
                                    _optimizedPoses.at(localizationLinks.rbegin()->first).getEulerAngles(roll, pitch, yaw);
                                    Transform targetRotation = iterGravitySign->second.transform().rotation()*transform.rotation();
                                    targetRotation = Transform(0,0,0,roll,pitch,targetRotation.theta());
                                    Transform error = transform.rotation().inverse() * iterGravitySign->second.transform().rotation().inverse() * targetRotation;
                                    transform *= error;
                                    newPose = _optimizedPoses.at(localizationLinks.rbegin()->first) * transform.inverse();
                                    iterGravitySign->second.transform().getEulerAngles(roll, pitch, tmp1);
                                    newPose.getEulerAngles(tmp1, tmp2, yaw);
                                    newPose = Transform(newPose.x(), newPose.y(), newPose.z(), roll, pitch, yaw);
									UDEBUG("newPose gravity=%s", newPose.prettyPrint().c_str());
								}
								else if(iterGravitySign!=signature->getLinks().end())
								{
									UWARN("Gravity link not found for %d, localization won't be corrected with gravity.", signature->id());
								}
							}
							_optimizedPoses.at(signature->id()) = newPose;
						}
						_localizationCovariance = locOptCovariance.empty()?localizationLinks.rbegin()->second.infMatrix().inv():locOptCovariance;
					}
					else //delayed localization (wait for more than 1 link)
					{
						UWARN("Localization was good, but waiting for another one to be more accurate (%s>0)", Parameters::kRGBDMaxOdomCacheSize().c_str());
						delayedLocalization = true;
						rejectLocalization = true;
					}
				}
			}

			if(rejectLocalization)
			{
				_loopClosureHypothesis.first = 0;
				lastProximitySpaceClosureId = 0;
				rejectedLoopClosure = true;
			}
		}
		else
		{
			UINFO("Update map correction");
			std::map<int, Transform> poses = _optimizedPoses;

			// if _optimizeFromGraphEnd parameter just changed state, don't use optimized poses as guess
			if(_optimizeFromGraphEndChanged)
			{
				UWARN("Optimization: clearing guess poses as %s has changed state, now %s",
						Parameters::kRGBDOptimizeFromGraphEnd().c_str(), _optimizeFromGraphEnd?"true":"false");
				poses.clear();
				_optimizeFromGraphEndChanged = false;
			}

			std::multimap<int, Link> constraints;
			cv::Mat covariance;
			optimizeCurrentMap(signature->id(), false, poses, covariance, &constraints, &optimizationError, &optimizationIterations);

			// Check added loop closures have broken the graph
			// (in case of wrong loop closures).
			bool updateConstraints = true;
			if(poses.empty())
			{
				UWARN("Graph optimization failed! Rejecting last loop closures added.");
				for(std::list<std::pair<int, int> >::iterator iter=loopClosureLinksAdded.begin(); iter!=loopClosureLinksAdded.end(); ++iter)
				{
					_memory->removeLink(iter->first, iter->second);
					UWARN("Loop closure %d->%d rejected!", iter->first, iter->second);
				}
				updateConstraints = false;
				_loopClosureHypothesis.first = 0;
				lastProximitySpaceClosureId = 0;
				rejectedLoopClosure = true;
			}
			else if(_memory->isIncremental() &&
			  loopClosureLinksAdded.size() &&
			  optimizationIterations > 0 &&
			  constraints.size())
			{
				UINFO("Compute max graph errors...");
				maxGraphErrors = graph::computeMaxGraphErrors(
						poses,
						constraints);
				if(!maxGraphErrors.linearLink.isValid() && !maxGraphErrors.angularLink.isValid())
				{
					UWARN("Could not compute graph errors! Wrong loop closures could be accepted!");
				}

				bool reject = false;
				if(maxGraphErrors.linearLink.isValid())
				{
					maxGraphErrorsLinearIds = std::make_pair(maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to());
					UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f)", maxGraphErrors.linear, maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to(), maxGraphErrors.linearLink.transVariance(), maxGraphErrors.linear/sqrt(maxGraphErrors.linearLink.transVariance()));
					if(_optimizationMaxError > 0.0f && maxGraphErrors.linearRatio > _optimizationMaxError)
					{
						if( _optimizationMaxErrorRepairRadius > 0.0 &&
							maxGraphErrorsLinearIds == _lastRejectedLoopClosureIds &&
							graph::findLink(constraints, maxGraphErrorsLinearIds.first, maxGraphErrorsLinearIds.second) != constraints.end())
						{
							UWARN("We detected 2 consecutive loop closure rejections because of the same loop closure link (%d->%d), trying optimization again without that link...",
								maxGraphErrorsLinearIds.first, maxGraphErrorsLinearIds.second);

							std::list<std::pair<int, int> > removedLinks = repairGraph(
								maxGraphErrors,
								poses,
								constraints,
								optimizationError,
								optimizationIterations,
								covariance);

							if(removedLinks.empty())
							{
								UWARN("Optimization failed when trying to repair the graph.");
								reject = true;
							}
							else
							{
								for(auto link: removedLinks)
								{
									UWARN("Removing link %d->%d from memory", link.first, link.second);
									_memory->removeLink(link.first, link.second);
								}
								UWARN("Successfully repaired the graph.");
								maxGraphErrorsRemovedIds = removedLinks.front();
								maxGraphErrorsRemovedCount = removedLinks.size();
							}
						}
						else {
							reject = true;
						}

						if(reject)
						{
							UWARN("Rejecting all added loop closures (%d, first is %d <-> %d) in this "
								"iteration because a wrong loop closure has been "
								"detected after graph optimization, resulting in "
								"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
								"maximum error ratio parameter \"%s\" is %f of std deviation.",
								(int)loopClosureLinksAdded.size(),
								loopClosureLinksAdded.front().first,
								loopClosureLinksAdded.front().second,
								maxGraphErrors.linearRatio,
								maxGraphErrors.linearLink.from(),
								maxGraphErrors.linearLink.to(),
								maxGraphErrors.linearLink.type(),
								maxGraphErrors.linear,
								sqrt(maxGraphErrors.linearLink.transVariance()),
								Parameters::kRGBDOptimizeMaxError().c_str(),
								_optimizationMaxError);

							if(maxGraphErrors.linearLink.type() != Link::kNeighbor)
							{
								_lastRejectedLoopClosureIds = maxGraphErrorsLinearIds;
							}
						}
					}
					else if(_optimizationMaxError == 0.0f && maxGraphErrors.linearRatio>100 && !_graphOptimizer->isRobust())
					{
						UERROR("Huge optimization error detected!"
								"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
								"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
								maxGraphErrors.linearRatio,
								maxGraphErrors.linearLink.from(),
								maxGraphErrors.linearLink.to(),
								maxGraphErrors.linearLink.type(),
								maxGraphErrors.linear,
								sqrt(maxGraphErrors.linearLink.transVariance()),
								Parameters::kRGBDOptimizeMaxError().c_str());
					}
				}

				if(maxGraphErrors.angularLink.isValid())
				{
					maxGraphErrorsAngularIds = std::make_pair(maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to());
					UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f)", maxGraphErrors.angular*180.0f/CV_PI, maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to(), maxGraphErrors.angularLink.rotVariance(), maxGraphErrors.angular/sqrt(maxGraphErrors.angularLink.rotVariance()));
					if(_optimizationMaxError > 0.0f && maxGraphErrors.angularRatio > _optimizationMaxError)
					{
						UWARN("Rejecting all added loop closures (%d, first is %d <-> %d) in this "
							  "iteration because a wrong loop closure has been "
							  "detected after graph optimization, resulting in "
							  "a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
							  "maximum error ratio parameter \"%s\" is %f of std deviation.",
							  (int)loopClosureLinksAdded.size(),
							  loopClosureLinksAdded.front().first,
							  loopClosureLinksAdded.front().second,
							  maxGraphErrors.angularRatio,
							  maxGraphErrors.angularLink.from(),
							  maxGraphErrors.angularLink.to(),
							  maxGraphErrors.angularLink.type(),
							  maxGraphErrors.angular*180.0f/CV_PI,
							  sqrt(maxGraphErrors.angularLink.rotVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
						reject = true;
					}
					else if(_optimizationMaxError == 0.0f && maxGraphErrors.angularRatio>100 && !_graphOptimizer->isRobust())
					{
						UERROR("Huge optimization error detected!"
								"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
								"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
								maxGraphErrors.angularRatio,
								maxGraphErrors.angularLink.from(),
								maxGraphErrors.angularLink.to(),
								maxGraphErrors.angularLink.type(),
								maxGraphErrors.angular*180.0f/CV_PI,
								sqrt(maxGraphErrors.angularLink.rotVariance()),
								Parameters::kRGBDOptimizeMaxError().c_str());
					}
				}

				if(reject)
				{
					for(std::list<std::pair<int, int> >::iterator iter=loopClosureLinksAdded.begin(); iter!=loopClosureLinksAdded.end(); ++iter)
					{
						_memory->removeLink(iter->first, iter->second);
						UWARN("Loop closure %d->%d rejected!", iter->first, iter->second);
					}
					updateConstraints = false;
					_loopClosureHypothesis.first = 0;
					lastProximitySpaceClosureId = 0;
					rejectedLoopClosure = true;
				}
			}

			if(updateConstraints)
			{
				UINFO("Updated local map (old size=%d, new size=%d)", (int)_optimizedPoses.size(), (int)poses.size());
				_optimizedPoses = poses;
				_constraints = constraints;
				_localizationCovariance = covariance;
			}
		}

		// Update map correction, it should be identify when optimizing from the last node
		UASSERT(_optimizedPoses.find(signature->id()) != _optimizedPoses.end());
		if(fakeOdom && _mapCorrectionBackup.isNull())
		{
			_mapCorrectionBackup = _mapCorrection;
		}
		previousMapCorrection = _mapCorrection;
		_mapCorrection = _optimizedPoses.at(signature->id()) * signature->getPose().inverse();
		// Update statistics about the closest node in the graph using the actual loop closure
		if(!_memory->isIncremental() && !_lastLocalizationPose.isNull())
		{
			int closestNode = _loopClosureHypothesis.first>0?_loopClosureHypothesis.first:lastProximitySpaceClosureId;
			if(closestNode>0)
			{
				distanceToClosestNodeInTheGraph = _lastLocalizationPose.getDistance(_optimizedPoses.at(closestNode));
				UDEBUG("Last localization pose = %s, updated closest node=%d (%f m)", _lastLocalizationPose.prettyPrint().c_str(), closestNode, distanceToClosestNodeInTheGraph);
				angleToClosestNodeInTheGraph = _lastLocalizationPose.getAngle(_optimizedPoses.at(closestNode));
			}
		}
		_lastLocalizationPose = _optimizedPoses.at(signature->id()); // update
		if(_mapCorrection.getNormSquared() > 0.1f && _optimizeFromGraphEnd)
		{
			bool hasPrior = signature->hasLink(signature->id());
			if(!_graphOptimizer->priorsIgnored())
			{
				for(std::multimap<int, Link>::reverse_iterator iter=_constraints.rbegin(); !hasPrior && iter!=_constraints.rend(); ++iter)
				{
					if(iter->second.type() == Link::kPosePrior)
					{
						hasPrior = true;
					}
				}
			}
			if((!hasPrior || _graphOptimizer->priorsIgnored()) && _graphOptimizer->gravitySigma()==0.0f)
			{
				UERROR("Map correction should be identity when optimizing from the last node. T=%s", _mapCorrection.prettyPrint().c_str());
			}
		}
	}
	int newLocId = _loopClosureHypothesis.first>0?_loopClosureHypothesis.first:lastProximitySpaceClosureId>0?lastProximitySpaceClosureId:0;
	_lastLocalizationNodeId = newLocId!=0?newLocId:_lastLocalizationNodeId;
	if(newLocId==0 && !landmarksDetected.empty())
	{
		std::map<int, std::set<int> >::const_iterator iter = _memory->getLandmarksIndex().find(landmarksDetected.begin()->first);
		if(iter!=_memory->getLandmarksIndex().end())
		{
			if(iter->second.size() && *iter->second.begin()!=signature->id())
			{
				_lastLocalizationNodeId = *iter->second.begin();
			}
		}
	}

	timeMapOptimization = timer.ticks();
	ULOGGER_INFO("timeMapOptimization=%fs", timeMapOptimization);

	//============================================================
	// Prepare statistics
	//============================================================
	// Data used for the statistics event and for the log files
	int dictionarySize = 0;
	int refWordsCount = 0;
	int refUniqueWordsCount = 0;
	int lcHypothesisReactivated = 0;
	float rehearsalValue = uValue(statistics_.data(), Statistics::kMemoryRehearsal_sim(), 0.0f);
	int rehearsalMaxId = (int)uValue(statistics_.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
	sLoop = _memory->getSignature(_loopClosureHypothesis.first?_loopClosureHypothesis.first:lastProximitySpaceClosureId?lastProximitySpaceClosureId:_highestHypothesis.first);
	if(sLoop)
	{
		lcHypothesisReactivated = sLoop->isSaved()?1.0f:0.0f;
	}
	dictionarySize = (int)_memory->getVWDictionary()->getVisualWords().size();
	refWordsCount = (int)signature->getWords().size();
	refUniqueWordsCount = (int)uUniqueKeys(signature->getWords()).size();

	if(_graphOptimizer->isSlam2d() && _localizationCovariance.total() == 36)
	{
		// set very small
		_localizationCovariance.at<double>(2,2) = Registration::COVARIANCE_LINEAR_EPSILON;
		_localizationCovariance.at<double>(3,3) = Registration::COVARIANCE_ANGULAR_EPSILON;
		_localizationCovariance.at<double>(4,4) = Registration::COVARIANCE_ANGULAR_EPSILON;
	}

	// Posterior is empty if a bad signature is detected
	float vpHypothesis = posterior.size()?posterior.at(Memory::kIdVirtual):0.0f;
	int loopId = _loopClosureHypothesis.first>0?_loopClosureHypothesis.first:lastProximitySpaceClosureId;

	// prepare statistics
	if(_loopClosureHypothesis.first || _publishStats)
	{
		ULOGGER_INFO("sending stats...");
		statistics_.setRefImageId(_memory->getLastSignatureId()); // Use last id from Memory (in case of rehearsal)
		statistics_.setRefImageMapId(signature->mapId());
		statistics_.setStamp(data.stamp());
		if(_loopClosureHypothesis.first != Memory::kIdInvalid)
		{
			statistics_.setLoopClosureId(_loopClosureHypothesis.first);
			statistics_.setLoopClosureMapId(_memory->getMapId(_loopClosureHypothesis.first));
			ULOGGER_INFO("Loop closure detected! With id=%d", _loopClosureHypothesis.first);
		}

		if(_publishStats)
		{
			ULOGGER_INFO("send all stats...");
			statistics_.setExtended(1);

			statistics_.addStatistic(Statistics::kLoopAccepted_hypothesis_id(), _loopClosureHypothesis.first);
			statistics_.addStatistic(Statistics::kLoopSuppressed_hypothesis_id(), loopIdSuppressedByProximity);
			statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_id(), _highestHypothesis.first);
			statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_value(), _highestHypothesis.second);
			statistics_.addStatistic(Statistics::kLoopHypothesis_reactivated(), lcHypothesisReactivated);
			statistics_.addStatistic(Statistics::kLoopVp_hypothesis(), vpHypothesis);
			statistics_.addStatistic(Statistics::kLoopReactivate_id(), retrievalId);
			statistics_.addStatistic(Statistics::kLoopHypothesis_ratio(), hypothesisRatio);
			statistics_.addStatistic(Statistics::kLoopVisual_inliers(), loopClosureVisualInliers);
			statistics_.addStatistic(Statistics::kLoopVisual_inliers_ratio(), loopClosureVisualInliersRatio);
			statistics_.addStatistic(Statistics::kLoopVisual_matches(), loopClosureVisualMatches);
			statistics_.addStatistic(Statistics::kLoopVisual_variance(), loopClosureVisualVariance);
			statistics_.addStatistic(Statistics::kLoopLinear_variance(), loopClosureLinearVariance);
			statistics_.addStatistic(Statistics::kLoopAngular_variance(), loopClosureAngularVariance);
			statistics_.addStatistic(Statistics::kLoopLast_id(), _memory->getLastGlobalLoopClosureId());
			if(maxGraphErrors.linear>=0 || maxGraphErrors.angular>=0)
			{
				statistics_.addStatistic(Statistics::kLoopOptimization_max_error(), maxGraphErrors.linear);
				statistics_.addStatistic(Statistics::kLoopOptimization_max_error_ratio(), maxGraphErrors.linearRatio);
				statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error(), maxGraphErrors.angular*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error_ratio(), maxGraphErrors.angularRatio);
				statistics_.addStatistic(Statistics::kLoopOptimization_max_error_from_id(), maxGraphErrorsLinearIds.first);
				statistics_.addStatistic(Statistics::kLoopOptimization_max_error_to_id(), maxGraphErrorsLinearIds.second);
				statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error_from_id(), maxGraphErrorsAngularIds.first);
				statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error_to_id(), maxGraphErrorsAngularIds.second);

				if(_optimizationMaxErrorRepairRadius > 0)
				{
					statistics_.addStatistic(Statistics::kLoopOptimization_max_error_removed_from_id(), maxGraphErrorsRemovedIds.first);
					statistics_.addStatistic(Statistics::kLoopOptimization_max_error_removed_to_id(), maxGraphErrorsRemovedIds.second);
					statistics_.addStatistic(Statistics::kLoopOptimization_max_error_removed_count(), maxGraphErrorsRemovedCount);
				}
			}
			statistics_.addStatistic(Statistics::kLoopOptimization_error(), optimizationError);
			statistics_.addStatistic(Statistics::kLoopOptimization_iterations(), optimizationIterations);
			statistics_.addStatistic(Statistics::kLoopLandmark_detected(), landmarksDetected.empty()?0:-landmarksDetected.begin()->first);
			statistics_.addStatistic(Statistics::kLoopLandmark_detected_node_ref(), landmarksDetected.empty() || landmarksDetected.begin()->second.empty()?0:*landmarksDetected.begin()->second.begin());
			statistics_.addStatistic(Statistics::kLoopVisual_inliers_mean_dist(), loopClosureVisualInliersMeanDist);
			statistics_.addStatistic(Statistics::kLoopVisual_inliers_distribution(), loopClosureVisualInliersDistribution);

			statistics_.addStatistic(Statistics::kProximityTime_detections(), proximityDetectionsInTimeFound);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_visually(), proximityDetectionsAddedVisually);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_icp_multi(), proximityDetectionsAddedByICPMulti);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_icp_global(), proximityDetectionsAddedByICPGlobal);
			statistics_.addStatistic(Statistics::kProximitySpace_paths(), proximitySpacePaths);
			statistics_.addStatistic(Statistics::kProximitySpace_visual_paths_checked(), localVisualPathsChecked);
			statistics_.addStatistic(Statistics::kProximitySpace_scan_paths_checked(), localScanPathsChecked);
			statistics_.addStatistic(Statistics::kProximitySpace_last_detection_id(), lastProximitySpaceClosureId);
			statistics_.setProximityDetectionId(lastProximitySpaceClosureId);
			statistics_.setProximityDetectionMapId(_memory->getMapId(lastProximitySpaceClosureId));

			statistics_.addStatistic(Statistics::kLoopId(), loopId);
			statistics_.addStatistic(Statistics::kLoopMap_id(), (loopId>0 && sLoop)?sLoop->mapId():-1);

			statistics_.addStatistic(Statistics::kLoopDistance_since_last_loc(), _distanceTravelledSinceLastLocalization);

			float x,y,z,roll,pitch,yaw;
			if(_loopClosureHypothesis.first || lastProximitySpaceClosureId || (!rejectedLoopClosure && !landmarksDetected.empty()))
			{
				if(_loopClosureHypothesis.first || lastProximitySpaceClosureId)
				{
					// Loop closure transform
					UASSERT(sLoop);
					std::multimap<int, Link>::const_iterator loopIter =  sLoop->getLinks().find(signature->id());
					UASSERT(loopIter!=sLoop->getLinks().end());
					UINFO("Set loop closure transform = %s", loopIter->second.transform().prettyPrint().c_str());
					statistics_.setLoopClosureTransform(loopIter->second.transform());

					statistics_.addStatistic(Statistics::kLoopVisual_words(), sLoop->getWords().size());

					// if ground truth exists, compute localization error
					if(!sLoop->getGroundTruthPose().isNull() && !signature->getGroundTruthPose().isNull())
					{
						Transform transformGT = sLoop->getGroundTruthPose().inverse() * signature->getGroundTruthPose();
						statistics_.addStatistic(Statistics::kGtLocalization_linear_error(), loopIter->second.transform().getDistance(transformGT));
						statistics_.addStatistic(Statistics::kGtLocalization_angular_error(), loopIter->second.transform().getAngle(transformGT)*180/M_PI);
					}
				}

				_distanceTravelledSinceLastLocalization = 0.0f;

				statistics_.addStatistic(Statistics::kLoopMapToOdom_norm(), _mapCorrection.getNorm());
				statistics_.addStatistic(Statistics::kLoopMapToOdom_angle(), _mapCorrection.getAngle(Transform::getIdentity())*180.0f/M_PI);
				_mapCorrection.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_x(), x);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_y(), y);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_z(), z);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_roll(),  roll*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_pitch(),  pitch*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_yaw(), yaw*180.0f/M_PI);

				// Odom correction (actual odometry pose change), ignore correction from first localization
				if(!odomPose.isNull() && !previousMapCorrection.isNull() && !previousMapCorrection.isIdentity())
				{
					Transform odomCorrection = (previousMapCorrection*odomPose).inverse()*_mapCorrection*odomPose;
					statistics_.addStatistic(Statistics::kLoopOdom_correction_norm(), odomCorrection.getNorm());
					statistics_.addStatistic(Statistics::kLoopOdom_correction_angle(), odomCorrection.getAngle(Transform::getIdentity())*180.0f/M_PI);
					odomCorrection.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_x(), x);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_y(), y);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_z(), z);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_roll(),  roll*180.0f/M_PI);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_pitch(),  pitch*180.0f/M_PI);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_yaw(), yaw*180.0f/M_PI);
				}
			}
			if(!_lastLocalizationPose.isNull() && !_lastLocalizationPose.isIdentity())
			{
				_lastLocalizationPose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
				statistics_.addStatistic(Statistics::kLoopMapToBase_x(), x);
				statistics_.addStatistic(Statistics::kLoopMapToBase_y(), y);
				statistics_.addStatistic(Statistics::kLoopMapToBase_z(), z);
				statistics_.addStatistic(Statistics::kLoopMapToBase_roll(),  roll*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToBase_pitch(),  pitch*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToBase_yaw(), yaw*180.0f/M_PI);
				UINFO("Localization pose = %s", _lastLocalizationPose.prettyPrint().c_str());

				if(_localizationSecondTryWithoutProximityLinks) {
					statistics_.addStatistic(Statistics::kLoopProximity_links_cleared(), (float)odomCacheProximityLinksCleared);
				}

				if(_localizationCovariance.total()==36)
				{
					double varLin = _graphOptimizer->isSlam2d()?
							std::max(_localizationCovariance.at<double>(0,0), _localizationCovariance.at<double>(1,1)):
							uMax3(_localizationCovariance.at<double>(0,0), _localizationCovariance.at<double>(1,1), _localizationCovariance.at<double>(2,2));

					statistics_.addStatistic(Statistics::kLoopMapToBase_lin_std(), sqrt(varLin));
					statistics_.addStatistic(Statistics::kLoopMapToBase_lin_var(), varLin);
				}
			}

			statistics_.setMapCorrection(_mapCorrection);
			UINFO("Set map correction = %s", _mapCorrection.prettyPrint().c_str());
			statistics_.setLocalizationCovariance(_localizationCovariance);

			// timings...
			statistics_.addStatistic(Statistics::kTimingMemory_update(), timeMemoryUpdate*1000);
			statistics_.addStatistic(Statistics::kTimingNeighbor_link_refining(), timeNeighborLinkRefining*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_time(), timeProximityByTimeDetection*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_space_search(), timeProximityBySpaceSearch*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_space_visual(), timeProximityBySpaceVisualDetection*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_space(), timeProximityBySpaceDetection*1000);
			statistics_.addStatistic(Statistics::kTimingReactivation(), timeReactivations*1000);
			statistics_.addStatistic(Statistics::kTimingAdd_loop_closure_link(), timeAddLoopClosureLink*1000);
			statistics_.addStatistic(Statistics::kTimingMap_optimization(), timeMapOptimization*1000);
			statistics_.addStatistic(Statistics::kTimingLikelihood_computation(), timeLikelihoodCalculation*1000);
			statistics_.addStatistic(Statistics::kTimingPosterior_computation(), timePosteriorCalculation*1000);
			statistics_.addStatistic(Statistics::kTimingHypotheses_creation(), timeHypothesesCreation*1000);
			statistics_.addStatistic(Statistics::kTimingHypotheses_validation(), timeHypothesesValidation*1000);
			statistics_.addStatistic(Statistics::kTimingCleaning_neighbors(), timeCleaningNeighbors*1000);

			// retrieval
			statistics_.addStatistic(Statistics::kMemorySignatures_retrieved(), (float)signaturesRetrieved.size());

			// Feature specific parameters
			statistics_.addStatistic(Statistics::kKeypointDictionary_size(), dictionarySize);
			statistics_.addStatistic(Statistics::kKeypointCurrent_frame(), refWordsCount);
			statistics_.addStatistic(Statistics::kKeypointIndexed_words(), _memory->getVWDictionary()->getIndexedWordsCount());
			statistics_.addStatistic(Statistics::kKeypointIndex_memory_usage(), _memory->getVWDictionary()->getIndexMemoryUsed());

			//Epipolar geometry constraint
			statistics_.addStatistic(Statistics::kLoopRejectedHypothesis(), rejectedLoopClosure?1.0f:0);

			statistics_.addStatistic(Statistics::kMemorySmall_movement(), smallDisplacement?1.0f:0);
			statistics_.addStatistic(Statistics::kMemoryDistance_travelled(), _distanceTravelled);
			statistics_.addStatistic(Statistics::kMemoryFast_movement(), tooFastMovement?1.0f:0);
			statistics_.addStatistic(Statistics::kMemoryNew_landmark(), addedNewLandmark?1.0f:0);

			if(distanceToClosestNodeInTheGraph>0.0)
			{
				statistics_.addStatistic(Statistics::kMemoryClosest_node_distance(), distanceToClosestNodeInTheGraph);
				statistics_.addStatistic(Statistics::kMemoryClosest_node_angle(), angleToClosestNodeInTheGraph);
			}

			if(_publishRAMUsage)
			{
				UTimer ramTimer;
				statistics_.addStatistic(Statistics::kMemoryRAM_usage(), UProcessInfo::getMemoryUsage()/(1024*1024));
				long estimatedMemoryUsage = sizeof(Rtabmap);
				estimatedMemoryUsage += _optimizedPoses.size() * (sizeof(int) + sizeof(Transform) + 12 * sizeof(float) + sizeof(std::map<int, Transform>::iterator)) + sizeof(std::map<int, Transform>);
				estimatedMemoryUsage += _constraints.size() * (sizeof(int) + sizeof(Transform) + 12 * sizeof(float) + sizeof(cv::Mat) + 36 * sizeof(double) + sizeof(std::map<int, Link>::iterator)) + sizeof(std::map<int, Link>);
				estimatedMemoryUsage += _memory->getMemoryUsed();
				estimatedMemoryUsage += _bayesFilter->getMemoryUsed();
				estimatedMemoryUsage += _parameters.size()*(sizeof(std::string)*2+sizeof(ParametersMap::iterator)) + sizeof(ParametersMap);
				statistics_.addStatistic(Statistics::kMemoryRAM_estimated(), (float)(estimatedMemoryUsage/(1024*1024)));//MB
				statistics_.addStatistic(Statistics::kTimingRAM_estimation(), ramTimer.ticks()*1000);
			}

			if(_publishLikelihood || _publishPdf)
			{
				// Child count by parent signature on the root of the memory ... for statistics
				statistics_.setWeights(weights);
				if(_publishPdf)
				{
					statistics_.setPosterior(posterior);
				}
				if(_publishLikelihood)
				{
					statistics_.setLikelihood(likelihood);
					statistics_.setRawLikelihood(rawLikelihood);
				}
			}

			statistics_.setLabels(_memory->getAllLabels());

			// Path
			if(_path.size())
			{
				statistics_.setLocalPath(this->getPathNextNodes());
				statistics_.setCurrentGoalId(this->getPathCurrentGoalId());
			}
		}

		timeStatsCreation = timer.ticks();
		ULOGGER_INFO("Time creating stats = %f...", timeStatsCreation);
	}

	Signature lastSignatureData = *signature;
	Transform lastSignatureLocalizedPose;
	if(_optimizedPoses.find(signature->id()) != _optimizedPoses.end())
	{
		lastSignatureLocalizedPose = _optimizedPoses.at(signature->id());
	}
	if(!_publishLastSignatureData)
	{
		lastSignatureData.sensorData().clearCompressedData(true, true, true, false);
		lastSignatureData.sensorData().clearRawData();
	}
	if(!_rawDataKept)
	{
		_memory->removeRawData(signature->id(), true, !_neighborLinkRefining && !_proximityBySpace, true, false);
	}

	// Localization mode and saving localization data: save odometry covariance in a prior link
	// so that DBReader can republish the covariance of localization data
	if(!_memory->isIncremental() && _memory->isLocalizationDataSaved() && !odomCovariance.empty())
	{
		_memory->addLink(Link(signature->id(), signature->id(), Link::kPosePrior, odomPose, odomCovariance.inv()));
	}
	bool lastSignatureWasIntermediateNode = signature->getWeight() == -1;

	// remove last signature if the memory is not incremental or is a bad signature (if bad signatures are ignored)
	int signatureRemoved = _memory->cleanup();
	if(signatureRemoved)
	{
		signaturesRemoved.push_back(signatureRemoved);
		// cleanup() has already removed this transient/rehearsed signature from
		// Memory.  Keep the reset-epoch index in lockstep so a later guarded
		// transaction does not mistake a legitimately discarded node for a
		// missing persistent graph node.
		_signatureResetEpochs.erase(signatureRemoved);
		_acceptedImageReceipts.erase(signatureRemoved);
	}

	// If this option activated, add new nodes only if there are linked with a previous map.
	// Used when rtabmap is first started, it will wait a
	// global loop closure detection before starting the new map,
	// otherwise it deletes the current node.
	if(signatureRemoved != lastSignatureData.id())
	{
		if(_startNewMapOnLoopClosure &&
			_memory->isIncremental() &&              // only in mapping mode
			graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size() == 0 &&      // alone in the current map
			(landmarksDetected.empty() || rejectedLoopClosure) &&      // if we re not seeing a landmark from a previous map
			_memory->getWorkingMem().size()>=2)       // The working memory should not be empty (beside virtual signature)
		{
			UWARN("Ignoring location %d because a global loop closure is required before starting a new map!",
					signature->id());
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
			if(_memory->getSignature(signature->id()) == 0)
			{
				_signatureResetEpochs.erase(signature->id());
		_acceptedImageReceipts.erase(signature->id());
			}
		}
		else if(_startNewMapOnGoodSignature &&
				(signature->getLandmarks().empty() && signature->isBadSignature()) &&
				graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size() == 0)     // alone in the current map
		{
			UWARN("Ignoring location %d because a good signature (with enough features or with a landmark detected) is required before starting a new map!",
					signature->id());
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
			if(_memory->getSignature(signature->id()) == 0)
			{
				_signatureResetEpochs.erase(signature->id());
		_acceptedImageReceipts.erase(signature->id());
			}
		}
		else if((!_memory->isIncremental() || signature->getWeight()>=0) &&
				(smallDisplacement || tooFastMovement) &&
				_loopClosureHypothesis.first == 0 &&
				lastProximitySpaceClosureId == 0 &&
				(rejectedLoopClosure || landmarksDetected.empty()) &&
				!addedNewLandmark)
		{
			// Don't delete the location if a loop closure is detected
			UINFO("Ignoring location %d because the displacement is too small! (d=%f a=%f)",
				  signature->id(), _rgbdLinearUpdate, _rgbdAngularUpdate);
			// If there is a too small displacement, remove the node
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
			if(_memory->getSignature(signature->id()) == 0)
			{
				_signatureResetEpochs.erase(signature->id());
		_acceptedImageReceipts.erase(signature->id());
			}

			// Update odom cache (if we just switched from mapping mode to localization mode)
			_odomCachePoses.erase(signature->id());
			for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin(); iter!=_odomCacheConstraints.end();)
			{
				if(iter->second.from() == signature->id() || iter->second.to() == signature->id())
				{
					_odomCacheConstraints.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
		}
		else
		{
			_memory->saveLocationData(signature->id());
		}
	}
	else if(!_memory->isIncremental() &&
			(smallDisplacement || tooFastMovement) &&
			_loopClosureHypothesis.first == 0 &&
			lastProximitySpaceClosureId == 0 &&
			!delayedLocalization)
	{
		_odomCachePoses.erase(signatureRemoved);
		for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin(); iter!=_odomCacheConstraints.end();)
		{
			if(iter->second.from() == signatureRemoved || iter->second.to() == signatureRemoved)
			{
				_odomCacheConstraints.erase(iter++);
			}
			else
			{
				++iter;
			}
		}
	}

	// Pass this point signature should not be used, since it could have been transferred...
	signature = 0;

	timeMemoryCleanup = timer.ticks();
	ULOGGER_INFO("timeMemoryCleanup = %fs... %d signatures removed", timeMemoryCleanup, (int)signaturesRemoved.size());



	//============================================================
	// TRANSFER
	//============================================================
	// If time allowed for the detection exceeds the limit of
	// real-time, move the oldest signature with less frequency
	// entry (from X oldest) from the short term memory to the
	// long term memory.
	//============================================================
	double totalTime = timerTotal.ticks();
	ULOGGER_INFO("Total time processing = %fs...", totalTime);
	if(!lastSignatureWasIntermediateNode && // skip memory management on intermediate nodes
		((_maxTimeAllowed != 0 && totalTime*1000>_maxTimeAllowed) ||
		 (_maxMemoryAllowed != 0 && _memory->getWorkingMem().size() > _maxMemoryAllowed)))
	{
		if(_maxTimeAllowed!=0 && totalTime*1000>_maxTimeAllowed)
		{
			ULOGGER_INFO("Removing old signatures because time limit is reached %f ms > %f ms...",
				totalTime*1000, _maxTimeAllowed);
		}
		if(_maxMemoryAllowed != 0 && _memory->getWorkingMem().size() > _maxMemoryAllowed)
		{
			ULOGGER_INFO("Removing old signatures because memory limit is reached %d > %d...",
				_memory->getWorkingMem().size(), _maxMemoryAllowed);
		}
		immunizedLocations.insert(_lastLocalizationNodeId); // keep the latest localization in working memory
		std::list<int> transferred = _memory->forget(immunizedLocations);
		signaturesRemoved.insert(signaturesRemoved.end(), transferred.begin(), transferred.end());
		if(!_someNodesHaveBeenTransferred && transferred.size())
		{
			_someNodesHaveBeenTransferred = true; // only used to hide a warning on close nodes immunization
		}
	}
	_lastProcessTime = totalTime;

	// cleanup cached gps values
	for(std::list<int>::iterator iter=signaturesRemoved.begin(); iter!=signaturesRemoved.end() && _gpsGeocentricCache.size(); ++iter)
	{
		_gpsGeocentricCache.erase(*iter);
	}

	//Remove optimized poses from signatures transferred
	if(signaturesRemoved.size() && (_optimizedPoses.size() || _constraints.size()))
	{
		//refresh the local map because some transferred nodes may have broken the tree
		int id = 0;
		if(!_memory->isIncremental() && (_lastLocalizationNodeId > 0 || _path.size()))
		{
			if(_path.size())
			{
				// priority on node on the path
				UASSERT(_pathCurrentIndex < _path.size());
				UASSERT_MSG(uContains(_optimizedPoses, _path.at(_pathCurrentIndex).first), uFormat("id=%d", _path.at(_pathCurrentIndex).first).c_str());
				id = _path.at(_pathCurrentIndex).first;
				UDEBUG("Refresh local map from %d", id);
			}
			else
			{
				if(uContains(_optimizedPoses, _lastLocalizationNodeId))
				{
					id = _lastLocalizationNodeId;
					UDEBUG("Refresh local map from %d", id);
				}
				else
				{
					UDEBUG("Clearing _lastLocalizationNodeId(%d)", _lastLocalizationNodeId);
					_lastLocalizationNodeId = 0;
				}
			}
		}
		else if(_memory->isIncremental() &&
				_optimizedPoses.size() &&
				_memory->getLastWorkingSignature(true))
		{
			id = _memory->getLastWorkingSignature(true)->id();
			UDEBUG("Refresh local map from %d", id);
		}
		UDEBUG("id=%d _optimizedPoses=%d", id, (int)_optimizedPoses.size());
		if(id > 0)
		{
			if(_lastLocalizationNodeId != 0)
			{
				_lastLocalizationNodeId = id;
			}
			UASSERT_MSG(_memory->getSignature(id) != 0, uFormat("id=%d", id).c_str());

			if(signaturesRemoved.size() == 1 && signaturesRemoved.front() == lastSignatureData.id())
			{
				int lastId = signaturesRemoved.front();
				UDEBUG("Detected that only last signature has been removed (lastId=%d)", lastId);
				_optimizedPoses.erase(lastId);
				// Constraints may store only the canonical direction (previous -> last).
				// Looking up lastId as a key misses those incoming edges and leaves a
				// dangling constraint after the pose and Memory signature are removed.
				for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end();)
				{
					if(iter->second.from() == lastId || iter->second.to() == lastId)
					{
						_constraints.erase(iter++);
					}
					else
					{
						++iter;
					}
				}
			}
			else
			{

				std::map<int, int> ids = _memory->getNeighborsId(id, 0, 0, true);
				for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end();)
				{
					if(iter->first > 0 && !uContains(ids, iter->first))
					{
						UDEBUG("Removed %d from local map", iter->first);
						UASSERT(iter->first != _lastLocalizationNodeId);
						_optimizedPoses.erase(iter++);

						if(!_globalScanMap.empty())
						{
							UWARN("optimized poses have been modified, clearing global scan map...");
							_globalScanMap.clear();
							_globalScanMapPoses.clear();
						}
					}
					else
					{
						++iter;
					}
				}
				for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end();)
				{
					if(iter->first > 0 && (!uContains(ids, iter->second.from()) || !uContains(ids, iter->second.to())))
					{
						_constraints.erase(iter++);
					}
					else
					{
						++iter;
					}
				}
			}
		}
		else
		{
			if(!_optimizedPoses.empty())
				UDEBUG("Optimized poses cleared!");
			_optimizedPoses.clear();
			_constraints.clear();
		}
	}
	// just some verifications to make sure that planning path is still in the local map!
	if(_path.size())
	{
		UASSERT(_pathCurrentIndex < _path.size());
		UASSERT(_pathGoalIndex < _path.size());
		UASSERT_MSG(uContains(_optimizedPoses, _path.at(_pathCurrentIndex).first), uFormat("local map size=%d, id=%d", (int)_optimizedPoses.size(), _path.at(_pathCurrentIndex).first).c_str());
		UASSERT_MSG(uContains(_optimizedPoses, _path.at(_pathGoalIndex).first), uFormat("local map size=%d, id=%d", (int)_optimizedPoses.size(), _path.at(_pathGoalIndex).first).c_str());
	}


	timeRealTimeLimitReachedProcess = timer.ticks();
	ULOGGER_INFO("Time limit reached processing = %f...", timeRealTimeLimitReachedProcess);

	//==============================================================
	// Finalize statistics and log files
	//==============================================================
	int localGraphSize = 0;
	if(_publishStats)
	{
		statistics_.addStatistic(Statistics::kTimingStatistics_creation(), timeStatsCreation*1000);
		statistics_.addStatistic(Statistics::kTimingTotal(), totalTime*1000);
		statistics_.addStatistic(Statistics::kTimingForgetting(), timeRealTimeLimitReachedProcess*1000);
		statistics_.addStatistic(Statistics::kTimingJoining_trash(), timeJoiningTrash*1000);
		statistics_.addStatistic(Statistics::kTimingEmptying_trash(), timeEmptyingTrash*1000);
		statistics_.addStatistic(Statistics::kTimingMemory_cleanup(), timeMemoryCleanup*1000);

		// Transfer
		statistics_.addStatistic(Statistics::kMemorySignatures_removed(), signaturesRemoved.size());
		statistics_.addStatistic(Statistics::kMemoryImmunized_globally(), immunizedGlobally);
		statistics_.addStatistic(Statistics::kMemoryImmunized_locally(), immunizedLocally);
		statistics_.addStatistic(Statistics::kMemoryImmunized_locally_max(), maxLocalLocationsImmunized);

		// place after transfer because the memory/local graph may have changed
		statistics_.addStatistic(Statistics::kMemoryWorking_memory_size(), _memory->getWorkingMem().size());
		statistics_.addStatistic(Statistics::kMemoryShort_time_memory_size(), _memory->getStMem().size());
		statistics_.addStatistic(Statistics::kMemoryDatabase_memory_used(), _memory->getDatabaseMemoryUsed());

		// Set local graph
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		if(!_rgbdSlamMode)
		{
			UDEBUG("");
			// no optimization on appearance-only mode, create a local graph
			std::map<int, int> ids = _memory->getNeighborsId(lastSignatureData.id(), 0, 0, true);
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, false);
		}
		else // RGBD-SLAM mode
		{
			poses = _optimizedPoses;
			constraints = _constraints;
		}
		UINFO("Adding data %d [%d] (rgb/left=%d depth/right=%d)", lastSignatureData.id(), lastSignatureData.mapId(), lastSignatureData.sensorData().imageRaw().empty()?0:1, lastSignatureData.sensorData().depthOrRightRaw().empty()?0:1);
			statistics_.addSignatureData(lastSignatureData);

		if(_nodesToRepublish.size())
		{
			std::multimap<int, int> missingIds;

			// priority to loopId
			int tmpId = loopId>0?loopId:_highestHypothesis.first;
			if(tmpId>0 && _nodesToRepublish.find(tmpId) != _nodesToRepublish.end())
			{
				missingIds.insert(std::make_pair(-1, tmpId));
			}

			if(!_lastLocalizationPose.isNull())
			{
				// Republish data from closest nodes of the current localization
				std::map<int, Transform> nodesOnly(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				int id = rtabmap::graph::findNearestNode(nodesOnly, _lastLocalizationPose);
				if(id>0)
				{
					std::map<int, int> ids = _memory->getNeighborsId(id, 0, 0, true, false, true);
					for(std::map<int, int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
					{
						if(iter->first != loopId &&
								_nodesToRepublish.find(iter->first) != _nodesToRepublish.end())
						{
							missingIds.insert(std::make_pair(iter->second, iter->first));
						}
					}

					if(_nodesToRepublish.size() != missingIds.size())
					{
						// remove requested nodes not anymore in the graph
						for(std::set<int>::iterator iter=_nodesToRepublish.begin(); iter!=_nodesToRepublish.end();)
						{
							if(ids.find(*iter) == ids.end())
							{
								iter = _nodesToRepublish.erase(iter);
							}
							else
							{
								++iter;
							}
						}
					}
				}
			}

			int loaded = 0;
			std::stringstream stream;
			for(std::multimap<int, int>::iterator iter=missingIds.begin(); iter!=missingIds.end() && loaded<(int)_maxRepublished; ++iter)
			{
				statistics_.addSignatureData(getSignatureCopy(iter->second, true, true, true, true, true, true));
				_nodesToRepublish.erase(iter->second);
				++loaded;
				stream << iter->second << " ";
			}
			if(loaded)
			{
				UWARN("Republishing data of requested node(s) %s(%s=%d)",
						stream.str().c_str(),
						Parameters::kRtabmapMaxRepublished().c_str(),
						_maxRepublished);
			}
		}

		UDEBUG("");
		localGraphSize = (int)poses.size();
		if(!lastSignatureLocalizedPose.isNull())
		{
			poses.insert(std::make_pair(lastSignatureData.id(), lastSignatureLocalizedPose)); // in case we are in localization
		}
		statistics_.setPoses(poses);
		statistics_.setConstraints(constraints);

		statistics_.addStatistic(Statistics::kMemoryLocal_graph_size(), poses.size());

		statistics_.setOdomCachePoses(_odomCachePoses);
		statistics_.setOdomCacheConstraints(_odomCacheConstraints);
		statistics_.addStatistic(Statistics::kMemoryOdom_cache_poses(), _odomCachePoses.size());
		statistics_.addStatistic(Statistics::kMemoryOdom_cache_links(), _odomCacheConstraints.size());

		if(_computeRMSE && _memory->getGroundTruths().size())
		{
			UDEBUG("Computing RMSE...");
			float translational_rmse = 0.0f;
			float translational_mean = 0.0f;
			float translational_median = 0.0f;
			float translational_std = 0.0f;
			float translational_min = 0.0f;
			float translational_max = 0.0f;
			float rotational_rmse = 0.0f;
			float rotational_mean = 0.0f;
			float rotational_median = 0.0f;
			float rotational_std = 0.0f;
			float rotational_min = 0.0f;
			float rotational_max = 0.0f;

			graph::calcRMSE(
					_memory->getGroundTruths(),
					poses,
					translational_rmse,
					translational_mean,
					translational_median,
					translational_std,
					translational_min,
					translational_max,
					rotational_rmse,
					rotational_mean,
					rotational_median,
					rotational_std,
					rotational_min,
					rotational_max);

			statistics_.addStatistic(Statistics::kGtTranslational_rmse(), translational_rmse);
			statistics_.addStatistic(Statistics::kGtTranslational_mean(), translational_mean);
			statistics_.addStatistic(Statistics::kGtTranslational_median(), translational_median);
			statistics_.addStatistic(Statistics::kGtTranslational_std(), translational_std);
			statistics_.addStatistic(Statistics::kGtTranslational_min(), translational_min);
			statistics_.addStatistic(Statistics::kGtTranslational_max(), translational_max);
			statistics_.addStatistic(Statistics::kGtRotational_rmse(), rotational_rmse);
			statistics_.addStatistic(Statistics::kGtRotational_mean(), rotational_mean);
			statistics_.addStatistic(Statistics::kGtRotational_median(), rotational_median);
			statistics_.addStatistic(Statistics::kGtRotational_std(), rotational_std);
			statistics_.addStatistic(Statistics::kGtRotational_min(), rotational_min);
			statistics_.addStatistic(Statistics::kGtRotational_max(), rotational_max);
			UDEBUG("Computing RMSE...done!");
		}

		std::vector<int> ids;
		ids.reserve(_memory->getWorkingMem().size() + _memory->getStMem().size());
		for(std::set<int>::const_iterator iter=_memory->getStMem().begin(); iter!=_memory->getStMem().end(); ++iter)
		{
			ids.push_back(*iter);
		}
		for(std::map<int, double>::const_iterator iter=_memory->getWorkingMem().lower_bound(0); iter!=_memory->getWorkingMem().end(); ++iter)
		{
			ids.push_back(iter->first);
		}
		statistics_.setWmState(ids);
		UDEBUG("wmState=%d", (int)ids.size());
	}

	//Save statistics to database
	if(_memory->isIncremental() || _memory->isLocalizationDataSaved())
	{
		_memory->saveStatistics(statistics_, _saveWMState);
	}

	//Start trashing
	UDEBUG("Empty trash...");
	_memory->emptyTrash();

	// Log info...
	// TODO : use a specific class which will handle the RtabmapEvent
	if(_foutFloat && _foutInt)
	{
		UDEBUG("Logging...");
		std::string logF = uFormat("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
									totalTime,
									timeMemoryUpdate,
									timeReactivations,
									timeLikelihoodCalculation,
									timePosteriorCalculation,
									timeHypothesesCreation,
									timeHypothesesValidation,
									timeRealTimeLimitReachedProcess,
									timeStatsCreation,
									_highestHypothesis.second,
									0.0f,
									0.0f,
									0.0f,
									0.0f,
									0.0f,
									vpHypothesis,
									timeJoiningTrash,
									rehearsalValue,
									timeEmptyingTrash,
									timeRetrievalDbAccess,
									timeAddLoopClosureLink,
									timeMemoryCleanup,
									timeNeighborLinkRefining,
									timeProximityByTimeDetection,
									timeProximityBySpaceDetection,
									timeMapOptimization);
		std::string logI = uFormat("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
									_loopClosureHypothesis.first,
									_highestHypothesis.first,
									(int)signaturesRemoved.size(),
									0,
									refWordsCount,
									dictionarySize,
									int(_memory->getWorkingMem().size()),
									rejectedLoopClosure?1:0,
									0,
									0,
									int(signaturesRetrieved.size()),
									lcHypothesisReactivated,
									refUniqueWordsCount,
									retrievalId,
									0,
									rehearsalMaxId,
									rehearsalMaxId>0?1:0,
									localGraphSize,
									data.id(),
									_memory->getVWDictionary()->getIndexedWordsCount(),
									_memory->getVWDictionary()->getIndexMemoryUsed());
		if(_statisticLogsBufferedInRAM)
		{
			_bufferedLogsF.push_back(logF);
			_bufferedLogsI.push_back(logI);
		}
		else
		{
			if(_foutFloat)
			{
				fprintf(_foutFloat, "%s", logF.c_str());
			}
			if(_foutInt)
			{
				fprintf(_foutInt, "%s", logI.c_str());
			}
		}
		UINFO("Time logging = %f...", timer.ticks());
		//ULogger::flush();
	}
	timeFinalizingStatistics = timer.ticks();
	UDEBUG("End process, timeFinalizingStatistics=%fs", timeFinalizingStatistics);
	if(_publishStats)
	{
		statistics_.addStatistic(Statistics::kTimingFinalizing_statistics(), timeFinalizingStatistics*1000);
	}

	return true;
}

bool Rtabmap::getAcceptedImageReceipt(int id, AcceptedImageReceipt & output) const
{
    output = AcceptedImageReceipt();
    const auto receipt = _acceptedImageReceipts.find(id);
    const auto epoch = _signatureResetEpochs.find(id);
    if(receipt == _acceptedImageReceipts.end() || epoch == _signatureResetEpochs.end() ||
       epoch->second != receipt->second.resetEpoch || !_memory) return false;
    const Signature * signature = _memory->getSignature(id);
    if(!signature || signature->getWeight() < 0 ||
       signature->getStamp() != double(receipt->second.input.headerStampNs)/1e9)
        return false;
    output = receipt->second;
    return true;
}

bool Rtabmap::processWithAuthenticatedImage(
        const SensorData & data, Transform odomPose, const cv::Mat & covariance,
        const std::vector<float> & velocity, const std::map<std::string, float> & externalStats,
        const ImageInputIdentity & input, std::string & receiptReason)
{
    // Receipt failure does not change ordinary mapping semantics. It prevents
    // this input from becoming evidence for an external Tag correction.
    bool eligible = _memory && _rgbdSlamMode && _memory->isIncremental() && data.id() >= 0;
    const int previousMaximumId = _memory ? _memory->getLastSignatureId() : 0;
    if(!eligible) receiptReason = "not_an_incremental_image_input";
    if(eligible) eligible = receiptIdentityMatches(data, input, receiptReason);
    if(eligible && input.headerStampNs <= _lastAcceptedImageStampNs)
        { eligible = false; receiptReason = "nonmonotonic_or_repeated_input_header"; }
    const bool succeeded = process(data, odomPose, covariance, velocity, externalStats);
    if(!succeeded) { receiptReason = "core_process_failed"; return false; }
    if(!eligible) return true;
    // Rehearsal can copy OLD SensorData into a NEW Signature with a NEW
    // Signature stamp. Stored SensorData stamp is normally zero in this
    // Memory version, so it cannot authenticate input. Reject on rehearsal even if two
    // different integer headers round to the same SensorData double stamp.
    // Weight-only rehearsals are conservatively excluded too. Existing
    // receipts for retained old images remain valid.
    const auto merged = statistics_.data().find(Statistics::kMemoryRehearsal_merged());
    if(merged != statistics_.data().end() && merged->second > 0)
        { receiptReason = "rehearsal_input_not_independently_retained"; return true; }
    const Signature * signature = _memory->getLastWorkingSignature(false);
    if(!signature || signature->id() <= previousMaximumId || signature->getWeight() < 0 ||
       signature->getStamp() != data.stamp() ||
       signature->sensorData().id() != signature->id())
        { receiptReason = "input_not_retained_as_a_new_signature"; return true; }
    const auto epoch = _signatureResetEpochs.find(signature->id());
    if(epoch == _signatureResetEpochs.end() || epoch->second != _resetEpoch ||
       _acceptedImageReceipts.count(signature->id()))
        { receiptReason = "signature_epoch_missing_or_identity_reused"; return true; }
    const auto & models = signature->sensorData().cameraModels();
    if(models.size() != 1 || !receiptCameraMatches(models.front(), input) ||
       canonicalImageSha256(data.imageRaw()) != input.pixelSha256)
        { receiptReason = "memory_changed_input_pixels_or_camera_model"; return true; }
    AcceptedImageReceipt receipt;
    receipt.signatureId = signature->id();
    receipt.mapId = signature->mapId();
    receipt.resetEpoch = epoch->second;
    receipt.input = input;
    for(int i = 0; i < 12; ++i)
    {
        receipt.bodyFromCamera[i] = models.front().localTransform()[i];
        if(!std::isfinite(receipt.bodyFromCamera[i]))
            { receiptReason = "invalid_accepted_camera_transform"; return true; }
    }
    _acceptedImageReceipts.emplace(receipt.signatureId, receipt);
    _lastAcceptedImageStampNs = input.headerStampNs;
    receiptReason = "accepted_pre_memory_image_input";
    return true;
}


// SETTERS
void Rtabmap::setSafeRelocalizationGuardEnabled(bool enabled)
{
	if(enabled)
	{
		_safeRelocalizationGuardEnabled = true;
	}
	else if(_safeRelocalizationGuardEnabled)
	{
		UERROR("Safe-relocalization guard is a process-lifetime latch and cannot be disabled; destroy the Rtabmap instance instead");
	}
}

void Rtabmap::setTimeThreshold(float maxTimeAllowed)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("setTimeThreshold is disabled after the safe-relocalization process-lifetime latch is enabled");
		return;
	}
	//must be positive, 0 mean inf time allowed (no time limit)
	_maxTimeAllowed = maxTimeAllowed;
	if(_maxTimeAllowed < 0)
	{
		ULOGGER_WARN("maxTimeAllowed < 0, then setting it to 0 (inf).");
		_maxTimeAllowed = 0;
	}
	else if(_maxTimeAllowed > 0.0f && _maxTimeAllowed < 1.0f)
	{
		ULOGGER_WARN("Time threshold set to %fms, it is not in seconds!", _maxTimeAllowed);
	}
}
void Rtabmap::setMemoryThreshold(int maxMemoryAllowed)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("setMemoryThreshold is disabled after the safe-relocalization process-lifetime latch is enabled");
		return;
	}
	//must be positive, 0 mean inf memory allowed (no memory limit)
	_maxMemoryAllowed = maxMemoryAllowed;
	if(maxMemoryAllowed < 0)
	{
		ULOGGER_WARN("maxMemoryAllowed < 0, then setting it to 0 (inf).");
		_maxMemoryAllowed = 0;
	}
}

void Rtabmap::setWorkingDirectory(std::string path)
{
	path = uReplaceChar(path, '~', UDirectory::homeDir());
	if(!path.empty() && UDirectory::exists(path))
	{
		ULOGGER_DEBUG("Comparing new working directory path \"%s\" with \"%s\"", path.c_str(), _wDir.c_str());
		if(path.compare(_wDir) != 0)
		{
			if (_foutFloat || _foutInt)
			{
				UWARN("Working directory has been changed from \"%s\" with \"%s\", new log files will be created.",
					path.c_str(), _wDir.c_str());
			}
			_wDir = path;
			setupLogFiles();
		}
	}
	else if(path.empty())
	{
		_wDir.clear();
		setupLogFiles();
	}
	else
	{
		ULOGGER_ERROR("Directory \"%s\" doesn't exist!", path.c_str());
	}
}

void Rtabmap::rejectLastLoopClosure()
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("rejectLastLoopClosure is disabled while safe relocalization owns graph mutations");
		return;
	}
	if(!_memory)
	{
		return;
	}
	const Signature * lastS = _memory->getLastWorkingSignature(true); // last non-intermediate
	if(lastS && _memory->getStMem().find(lastS->id())!=_memory->getStMem().end())
	{
		std::multimap<int, Link> links = _memory->getLinks(lastS->id(), false);
		bool linksRemoved = false;
		for(std::multimap<int, Link>::iterator iter = links.begin(); iter!=links.end(); ++iter)
		{
			if(iter->second.type() == Link::kGlobalClosure ||
				iter->second.type() == Link::kLocalSpaceClosure ||
				iter->second.type() == Link::kLocalTimeClosure ||
				iter->second.type() == Link::kUserClosure)
			{
				_memory->removeLink(iter->second.from(), iter->second.to());
				std::multimap<int, Link>::iterator jter = graph::findLink(_constraints, iter->second.from(), iter->second.to(), true);
				if(jter!=_constraints.end())
				{
					_constraints.erase(jter);
					// second time if link is also inverted
					jter = graph::findLink(_constraints, iter->second.from(), iter->second.to(), true);
					if(jter!=_constraints.end())
					{
						_constraints.erase(jter);
					}
				}
				linksRemoved = true;
			}
		}

		if(linksRemoved)
		{
			_loopClosureHypothesis.first = 0;

			// we have to re-optimize the graph without the rejected links
			if(_memory->isIncremental() && _optimizedPoses.size())
			{
				UINFO("Update graph");
				std::map<int, Transform> poses = _optimizedPoses;
				std::multimap<int, Link> constraints;
				cv::Mat covariance;
				optimizeCurrentMap(lastS->id(), false, poses, covariance, &constraints);

				if(poses.empty())
				{
					UWARN("Graph optimization failed after removing loop closure links from last location!");
				}
				else
				{
					UINFO("Updated local map (old size=%d, new size=%d)", (int)_optimizedPoses.size(), (int)poses.size());
					_optimizedPoses = poses;
					_constraints = constraints;
					_mapCorrection = _optimizedPoses.at(lastS->id()) * lastS->getPose().inverse();
				}
			}
		}
	}
}

void Rtabmap::deleteLastLocation()
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("deleteLastLocation is disabled while safe relocalization owns graph mutations");
		return;
	}
	if(_memory && _memory->getStMem().size())
	{
		int lastId = *_memory->getStMem().rbegin();
		const Signature * s = _memory->getSignature(lastId);
		UASSERT(s);
		if(s->getWeight() == -1)
		{
			UERROR("Deleting last location with inermediate nodes is not supported. Aborting.");
			return;
		}
		_memory->deleteLocation(lastId);
		if(_memory->getSignature(lastId) == 0)
		{
			_signatureResetEpochs.erase(lastId);
		_acceptedImageReceipts.erase(lastId);
		}
		// we have to re-optimize the graph without the deleted location
		if(_memory->isIncremental() && _optimizedPoses.size())
		{
			UINFO("Update graph");
			_optimizedPoses.erase(lastId);
			std::map<int, Transform> poses = _optimizedPoses;
			//remove all constraints with last localization id
			for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end();)
			{
				if(iter->second.from() == lastId || iter->second.to() == lastId)
				{
					_constraints.erase(iter++);
				}
				else
				{
					++iter;
				}
			}

			if(poses.empty())
			{
				_mapCorrection.setIdentity();
			}
			else
			{
				std::multimap<int, Link> constraints;
				cv::Mat covariance;
				optimizeCurrentMap(_memory->getLastWorkingSignature(true)->id(), false, poses, covariance, &constraints);

				if(poses.empty())
				{
					UWARN("Graph optimization failed after deleting the last location!");
				}
				else
				{
					_optimizedPoses = poses;
					_constraints = constraints;
					_mapCorrection = _optimizedPoses.at(_memory->getLastWorkingSignature(true)->id()) * _memory->getLastWorkingSignature(true)->getPose().inverse();
				}
			}
		}
	}
}

void Rtabmap::setOptimizedPoses(const std::map<int, Transform> & poses, const std::multimap<int, Link> & constraints)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("setOptimizedPoses is disabled while safe relocalization owns graph mutations");
		return;
	}
	_optimizedPoses = poses;
    _constraints = constraints;
}

void Rtabmap::dumpData() const
{
	UDEBUG("");
	if(_memory)
	{
		if(this->getWorkingDir().empty())
		{
			UERROR("Working directory not set.");
		}
		else
		{
			_memory->dumpMemory(this->getWorkingDir());
		}
	}
}

// fromId must be in _memory and in _optimizedPoses
// Get poses in front of the robot, return optimized poses
std::map<int, Transform> Rtabmap::getForwardWMPoses(
		int fromId,
		int maxNearestNeighbors,
		float radius,
		int maxGraphDepth // 0 means ignore
		) const
{
	std::map<int, Transform> poses;
	if(_memory && fromId > 0)
	{
		UDEBUG("");
		const Signature * fromS = _memory->getSignature(fromId);
		UASSERT(fromS != 0);
		UASSERT(_optimizedPoses.find(fromId) != _optimizedPoses.end());

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(_optimizedPoses.size());
		std::vector<int> ids(_optimizedPoses.size());
		int oi = 0;
		const std::set<int> & stm = _memory->getStMem();
		//get distances
		std::map<int, float> foundIds;
		if(_memory->isIncremental())
		{
			foundIds = _memory->getNeighborsIdRadius(fromId, radius, _optimizedPoses, maxGraphDepth);
		}
		else
		{
			foundIds = graph::findNearestNodes(fromId, _optimizedPoses, radius);
		}

		float radiusSqrd = radius * radius;
		for(std::map<int, Transform>::const_iterator iter = _optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
		{
			if(iter->first != fromId)
			{
				if(stm.find(iter->first) == stm.end() &&
				   uContains(foundIds, iter->first) &&
				   (radiusSqrd==0 || foundIds.at(iter->first) <= radiusSqrd))
				{
					(*cloud)[oi] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
					ids[oi++] = iter->first;
				}
			}
		}

		cloud->resize(oi);
		ids.resize(oi);

		Transform fromT = _optimizedPoses.at(fromId);

		if(cloud->size())
		{
			//if(cloud->size())
			//{
			//	pcl::io::savePCDFile("radiusPoses.pcd", *cloud);
			//	UWARN("Saved radiusPoses.pcd");
			//}

			//filter poses in front of the fromId
			float x,y,z, roll,pitch,yaw;
			fromT.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);

			pcl::CropBox<pcl::PointXYZ> cropbox;
			cropbox.setInputCloud(cloud);
			cropbox.setMin(Eigen::Vector4f(-1, -radius, -999999, 0));
			cropbox.setMax(Eigen::Vector4f(radius, radius, 999999, 0));
			cropbox.setRotation(Eigen::Vector3f(roll, pitch, yaw));
			cropbox.setTranslation(Eigen::Vector3f(x, y, z));
			cropbox.setRotation(Eigen::Vector3f(roll,pitch,yaw));
			pcl::IndicesPtr indices(new std::vector<int>());
			cropbox.filter(*indices);

			//if(indices->size())
			//{
			//	pcl::io::savePCDFile("radiusCrop.pcd", *cloud, *indices);
			//	UWARN("Saved radiusCrop.pcd");
			//}

			if(indices->size())
			{
				pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
				kdTree->setInputCloud(cloud, indices);
				std::vector<int> ind;
				std::vector<float> dist;
				pcl::PointXYZ pt(fromT.x(), fromT.y(), fromT.z());
				kdTree->radiusSearch(pt, radius, ind, dist, maxNearestNeighbors);
				//pcl::PointCloud<pcl::PointXYZ> inliers;
				for(unsigned int i=0; i<ind.size(); ++i)
				{
					if(ind[i] >=0)
					{
						Transform tmp = _optimizedPoses.find(ids[ind[i]])->second;
						//inliers.push_back(pcl::PointXYZ(tmp.x(), tmp.y(), tmp.z()));
						UDEBUG("Inlier %d: %s", ids[ind[i]], tmp.prettyPrint().c_str());
						poses.insert(std::make_pair(ids[ind[i]], tmp));
					}
				}

				//if(inliers.size())
				//{
				//	pcl::io::savePCDFile("radiusInliers.pcd", inliers);
				//}
				//if(nearestId >0)
				//{
				//	pcl::PointCloud<pcl::PointXYZ> c;
				//	Transform ct = _optimizedPoses.find(nearestId)->second;
				//	c.push_back(pcl::PointXYZ(ct.x(), ct.y(), ct.z()));
				//	pcl::io::savePCDFile("radiusNearestPt.pcd", c);
				//}
			}
		}
	}
	return poses;
}

std::map<int, std::map<int, Transform> > Rtabmap::getPaths(const std::map<int, Transform> & posesIn, const Transform & target, int maxGraphDepth) const
{
	std::map<int, std::map<int, Transform> > paths;
	std::set<int> nodesSet;
	std::map<int, Transform> poses;
	for(std::map<int, Transform>::const_iterator iter=posesIn.lower_bound(1); iter!=posesIn.end(); ++iter)
	{
		nodesSet.insert(iter->first);
		poses.insert(*iter);
	}
	if(_memory && nodesSet.size() && !target.isNull())
	{
		double e0=0,e1=0,e2=0,e3=0,e4=0;
		UTimer t;
		e0 = t.ticks();
		// Segment poses connected only by neighbor links
		while(poses.size())
		{
			std::map<int, Transform> path;
			// select nearest pose and iterate neighbors from there
			int nearestId = rtabmap::graph::findNearestNode(poses, target);

			e1+=t.ticks();

			if(nearestId == 0)
			{
				UWARN("Nearest id of %s in %d poses is 0 !? Returning empty path.", target.prettyPrint().c_str(), (int)poses.size());
				break;
			}
			std::map<int, int> ids = _memory->getNeighborsId(nearestId, maxGraphDepth, 0, true, true, true, true, nodesSet);

			e2+=t.ticks();

			for(std::map<int, int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
			{
				std::map<int, Transform>::iterator jter = poses.find(iter->first);
				if(jter != poses.end())
				{
					bool valid = path.empty();
					if(!valid)
					{
						// make sure it has a neighbor added to path
						std::multimap<int, Link> links = _memory->getNeighborLinks(iter->first);
						for(std::multimap<int, Link>::iterator kter=links.begin(); kter!=links.end() && !valid; ++kter)
						{
							valid = path.find(kter->first) != path.end();
						}
					}

					if(valid)
					{
						//UDEBUG("%d <- %d", nearestId, jter->first);
						path.insert(*jter);
						poses.erase(jter);
					}
				}
			}

			e3+=t.ticks();

			if (path.size())
			{
				if (maxGraphDepth > 0 && !_memory->isGraphReduced() && (int)path.size() > maxGraphDepth * 2 + 1)
				{
					UWARN("%s=Off but path(%d) > maxGraphDepth(%d)*2+1, nearestId=%d ids=%d. Is reduce graph activated before?",
						Parameters::kMemReduceGraph().c_str(), (int)path.size(), maxGraphDepth, nearestId, (int)ids.size());
				}
				paths.insert(std::make_pair(nearestId, path));
			}
			else
			{
				UWARN("path.size()=0!? nearestId=%d ids=%d, aborting...", nearestId, (int)ids.size());
				break;
			}

			e4+=t.ticks();
		}
		UDEBUG("e0=%fs e1=%fs e2=%fs e3=%fs e4=%fs", e0, e1, e2, e3, e4);
	}
	return paths;
}

void Rtabmap::optimizeCurrentMap(
		int id,
		bool lookInDatabase,
		std::map<int, Transform> & optimizedPoses,
		cv::Mat & covariance,
		std::multimap<int, Link> * constraints,
		double * error,
		int * iterationsDone) const
{
	//Optimize the map
	UINFO("Optimize map: around location %d (lookInDatabase=%s)", id, lookInDatabase?"true":"false");
	if(_memory && id > 0)
	{
		if(!lookInDatabase && (!_memory->getSignature(id) || _memory->getSignature(id)->getWeight() == -1))
		{
			UERROR("When doing a local optimization, the root id (%d) must exist and not be an intermediate node! Aborting...", id);
			optimizedPoses.clear();
			if(constraints)
			{
				constraints->clear();
			}
			return;
		}

		UTimer timer;
		std::map<int, int> ids = _memory->getNeighborsId(id, 0, lookInDatabase?-1:0, true, false);
		if(!_optimizeFromGraphEnd && ids.size() > 1)
		{
			if(lookInDatabase)
			{
				id = ids.begin()->first;
			}
			else
			{
				// Find first node that is not intermediate
				for(auto pair: ids)
				{
					// Make sure fromId is not an intermediate node
					const Signature * s = _memory->getSignature(pair.first);
					if(s && s->getWeight() != -1)
					{
						id = pair.first;
						break;
					}
					else if(!s)
					{
						UWARN("Not found node %d in memory?!", pair.first);
					}
				}
			}
		}
		UINFO("get %d ids time %f s", (int)ids.size(), timer.ticks());

		std::map<int, Transform> poses = Rtabmap::optimizeGraph(id, uKeysSet(ids), optimizedPoses, lookInDatabase, covariance, constraints, error, iterationsDone);
		UINFO("optimize time %f s", timer.ticks());

		if(poses.size())
		{
			optimizedPoses = poses;

			if(_memory->getSignature(id) && uContains(optimizedPoses, id))
			{
				Transform t = optimizedPoses.at(id) * _memory->getSignature(id)->getPose().inverse();
				UINFO("Correction (from node %d) %s", id, t.prettyPrint().c_str());
			}
		}
		else
		{
			UWARN("Failed to optimize the graph! returning empty optimized poses...");
			optimizedPoses.clear();
			if(constraints)
			{
				constraints->clear();
			}
		}
	}
}

std::map<int, Transform> Rtabmap::optimizeGraph(
		int fromId,
		const std::set<int> & ids,
		const std::map<int, Transform> & guessPoses,
		bool lookInDatabase,
		cv::Mat & covariance,
		std::multimap<int, Link> * constraints,
		double * error,
		int * iterationsDone) const
{
	UTimer timer;
	std::map<int, Transform> optimizedPoses;
	std::map<int, Transform> poses;
	std::multimap<int, Link> edgeConstraints;
	UDEBUG("ids=%d", (int)ids.size());
	_memory->getMetricConstraints(ids, poses, edgeConstraints, lookInDatabase, !_graphOptimizer->landmarksIgnored());
	UINFO("get constraints (ids=%d, %d poses, %d edges) time %f s", (int)ids.size(), (int)poses.size(), (int)edgeConstraints.size(), timer.ticks());

	// add landmark priors if there are some
	for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end() && iter->first < 0; ++iter)
	{
		if(_markerPriors.find(iter->first) != _markerPriors.end())
		{
			cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
			infMatrix(cv::Range(0,3), cv::Range(0,3)) /= _markerPriorsLinearVariance;
			infMatrix(cv::Range(3,6), cv::Range(3,6)) /= _markerPriorsAngularVariance;
			edgeConstraints.insert(std::make_pair(iter->first, Link(iter->first, iter->first, Link::kPosePrior, _markerPriors.at(iter->first), infMatrix)));
			UDEBUG("Added prior %d : %s (variance: lin=%f ang=%f)", iter->first, _markerPriors.at(iter->first).prettyPrint().c_str(),
					_markerPriorsLinearVariance, _markerPriorsAngularVariance);
		}
	}

	// Once a recovery transaction is committed, the accepted component is a
	// frozen reference. Ordinary mapping may estimate only nodes appended after
	// it; otherwise the next routine optimization can undo a valid recovery.
	if(_safeRelocalizationGuardEnabled &&
			_safeRelocalizationCommittedEpochs.count(_resetEpoch))
	{
		OptimizerG2O * fixedOptimizer=dynamic_cast<OptimizerG2O *>(_graphOptimizer);
		if(!fixedOptimizer || _graphOptimizer->iterations()<=0 || _graphOptimizer->isSlam2d() ||
				!_graphOptimizer->landmarksIgnored() || !_safeRelocalizationTrustedSnapshotValid)
		{
			UERROR("Committed mapping has no compatible fixed-pose optimizer");return optimizedPoses;
		}
		std::set<int> fixedIds;
		for(std::map<int,Transform>::iterator p=poses.begin();p!=poses.end();++p)
		{
			std::map<int,Transform>::const_iterator trusted=_safeRelocalizationTrustedPoses.find(p->first);
			if(trusted!=_safeRelocalizationTrustedPoses.end())
			{
				if(p->first<=0 || !safeRigidTransform(trusted->second))return optimizedPoses;
				p->second=trusted->second;fixedIds.insert(p->first);
			}
		}
		if(fixedIds.empty())
		{
			UERROR("Committed mapping lost its trusted component");return optimizedPoses;
		}
		std::map<int,Transform> connected;
		std::multimap<int,Link> connectedLinks;
		_graphOptimizer->getConnectedGraph(*fixedIds.rbegin(),poses,edgeConstraints,connected,connectedLinks);
		if(connected.size()!=poses.size())
		{
			UERROR("Committed mapping extension is disconnected");return optimizedPoses;
		}
		for(int id:fixedIds)connected.at(id)=_safeRelocalizationTrustedPoses.at(id);
		if(connected.size()==fixedIds.size())
		{
			covariance=cv::Mat::eye(6,6,CV_64F)*9999.;optimizedPoses=connected;
			if(error)*error=0.;if(iterationsDone)*iterationsDone=0;
		}
		else optimizedPoses=fixedOptimizer->optimizeWithFixedPoses(*fixedIds.begin(),connected,
				connectedLinks,fixedIds,covariance,0,error,iterationsDone);
		if(constraints)*constraints=connectedLinks;
		return optimizedPoses;
	}

	if(_graphOptimizer->iterations() > 0)
	{
		for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			// Apply guess poses (if some), ignore for rootid to avoid origin drifting
			std::map<int, Transform>::const_iterator foundGuess = guessPoses.find(iter->first);
			if(foundGuess!=guessPoses.end() && iter->first != fromId)
			{
				iter->second = foundGuess->second;
			}
		}
	}


	UASSERT(_graphOptimizer!=0);
	if(_graphOptimizer->iterations() == 0)
	{
		// Optimization disabled! Return not optimized poses.
		optimizedPoses = poses;
		if(constraints)
		{
			*constraints = edgeConstraints;
		}
	}
	else
	{
		bool hasLandmarks = !edgeConstraints.empty() && edgeConstraints.begin()->first < 0;
		if(poses.size() != guessPoses.size() || hasLandmarks)
		{
			UDEBUG("recompute poses using only links (robust to multi-session)");
			std::map<int, Transform> posesOut;
			std::multimap<int, Link> edgeConstraintsOut;
			_graphOptimizer->getConnectedGraph(fromId, poses, edgeConstraints, posesOut, edgeConstraintsOut);
			const auto traceCall = traceNextCall();
            traceOptimizer(traceCall, "before", fromId, posesOut, edgeConstraintsOut, _graphOptimizer);
            optimizedPoses = _graphOptimizer->optimize(fromId, posesOut, edgeConstraintsOut, covariance, 0, error, iterationsDone);
            traceOptimizer(traceCall, "after", fromId, optimizedPoses, edgeConstraintsOut, _graphOptimizer);
			if(constraints)
			{
				*constraints = edgeConstraintsOut;
			}
		}
		else
		{
			UDEBUG("use input guess poses");
			const auto traceCall = traceNextCall();
            traceOptimizer(traceCall, "before", fromId, poses, edgeConstraints, _graphOptimizer);
            optimizedPoses = _graphOptimizer->optimize(fromId, poses, edgeConstraints, covariance, 0, error, iterationsDone);
            traceOptimizer(traceCall, "after", fromId, optimizedPoses, edgeConstraints, _graphOptimizer);
			if(constraints)
			{
				*constraints = edgeConstraints;
			}
		}

		if(!poses.empty() && optimizedPoses.empty())
		{
			UWARN("Optimization has failed (poses=%d, guess=%d, links=%d)...",
				  (int)poses.size(), (int)guessPoses.size(), (int)edgeConstraints.size());
		}
	}

	UINFO("Optimization time %f s", timer.ticks());

	return optimizedPoses;
}

// If repairing works, all input arguments are updated accordingly to new graph
// Returns IDs of the links removed from constraints
std::list<std::pair<int, int> > Rtabmap::repairGraph(
	graph::MaxGraphErrors & maxGraphErrors,
	std::map<int, Transform> & poses,
	std::multimap<int, Link> & constraints,
	double & optimizationError,
	int & optimizationIterations,
	cv::Mat & optimizationCovariance)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UWARN("Graph repair is disabled while safe relocalization protects committed bridge transactions.");
		return std::list<std::pair<int, int> >();
	}
	UASSERT(maxGraphErrors.linearLink.isValid());
	UASSERT(graph::findLink(constraints, maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to()) != constraints.end());

	int originalMaxErrorLinkFrom = maxGraphErrors.linearLink.from();
	int originalMaxErrorLinkTo = maxGraphErrors.linearLink.to();

	graph::MaxGraphErrors subMaxGraphErrors = maxGraphErrors;
	std::list<std::pair<int, int> > removedLinks;
	std::map<int, Transform> subPoses = poses;
	std::multimap<int, Link> subConstraints = constraints;
	while(subMaxGraphErrors.linearLink.isValid() && subMaxGraphErrors.linearRatio > _optimizationMaxError)
	{
		removedLinks.push_back(std::make_pair(subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to()));
		subConstraints.erase(graph::findLink(subConstraints, subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to()));
		subMaxGraphErrors.linearLink = Link();

		// Get connected graph just to check if a node got disconnected (localization mode)
		std::map<int, Transform> posesOut;
		std::multimap<int, Link> subConstraintsOut;
		_graphOptimizer->getConnectedGraph(subPoses.rbegin()->first, subPoses, subConstraints, posesOut, subConstraintsOut);
		if(subPoses.size() - posesOut.size() > 1)
		{
			UWARN("More than one pose filtered in one iteration when trying to repair graph. Aborting repairing.");
			break;
		}
		else if(subPoses.size() != posesOut.size())
		{
			for(std::map<int, Transform>::iterator pter=subPoses.begin(); pter!=subPoses.end(); ++pter)
			{
				if(posesOut.find(pter->first) == posesOut.end())
				{
					subPoses.erase(pter);
					break; // should be only one if different, break now
				}
			}
		}
		cv::Mat subOptimizationCovariance;
		double subOptimizationError = 0.0;
		int subOptimizationIterations = 0;

		int fromId = subPoses.rbegin()->first;
		if(!_optimizeFromGraphEnd)
		{
			// Find first node that is not intermediate
			for(std::map<int, Transform>::iterator iter = subPoses.lower_bound(1); iter!=subPoses.end(); ++iter)
			{
				// Make sure fromId is not an intermediate node
				const Signature * s = _memory->getSignature(iter->first);
				if(s && s->getWeight() != -1)
				{
					fromId = iter->first;
					break;
				}
				else if(!s)
				{
					UWARN("Not found node %d in memory?!", iter->first);
				}
			}
		}

		subPoses = _graphOptimizer->optimize(
			fromId,
			subPoses,
			subConstraintsOut,
			subOptimizationCovariance,
			0,
			&subOptimizationError,
			&subOptimizationIterations);

		if(subPoses.empty())
		{
			UWARN("Optimization failed when trying to repair graph.");
		}
		else
		{
			subMaxGraphErrors = graph::computeMaxGraphErrors(
					subPoses,
					subConstraintsOut);
			if(!subMaxGraphErrors.linearLink.isValid())
			{
				UWARN("Could not compute graph errors! Wrong loop closures could be accepted!");
			}
			else if(subMaxGraphErrors.linearRatio > _optimizationMaxError)
			{
				float distance = poses.at(originalMaxErrorLinkFrom).getDistance(poses.at(subMaxGraphErrors.linearLink.from()));
				if(subMaxGraphErrors.linearLink.type() != Link::kNeighbor && distance < _optimizationMaxErrorRepairRadius)
				{
					UWARN("Optimization error is still high (%f, on link %d->%d type=%d) after removing the loop closure with the highest error. "
						"As it is close (%f m < %s=%f m) to original loop closure with high error (%d->%d), we will reject again this one to see if it helps.",
						subMaxGraphErrors.linearRatio, subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to(), subMaxGraphErrors.linearLink.type(),
						distance, Parameters::kRGBDOptimizeMaxErrorRepairRadius().c_str(), _optimizationMaxErrorRepairRadius,
						originalMaxErrorLinkFrom, originalMaxErrorLinkTo);
				}
				else
				{
					UWARN("Optimization error is still high (%f, on link %d->%d) after removing loop closure with highest error.",
						subMaxGraphErrors.linearRatio, subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to());
					subMaxGraphErrors.linearLink = Link();
				}
			}
			else
			{
				UWARN("Optimization error is lower (%f, on link %d->%d) after removing loop "
					"closure with highest error. We will remove the old link (%d->%d collaterals=%ld) and accept the new one.",
					subMaxGraphErrors.linearRatio, subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to(),
					removedLinks.front().first, removedLinks.front().second,
					removedLinks.size()-1);

				maxGraphErrors = subMaxGraphErrors;

				// Make sure the link pointers in maxGraphErrors point on same constraints
				std::swap(poses, subPoses);
				std::swap(constraints, subConstraintsOut);

				optimizationCovariance = subOptimizationCovariance;
				optimizationError = subOptimizationError;
				optimizationIterations = subOptimizationIterations;

				return removedLinks;
			}
		}
	}
	return std::list<std::pair<int, int> >();
}

void Rtabmap::adjustLikelihood(std::map<int, float> & likelihood) const
{
	ULOGGER_DEBUG("likelihood.size()=%d", likelihood.size());
	UTimer timer;
	timer.start();
	if(likelihood.size()==0)
	{
		return;
	}

	// Use only non-null values (ignore virtual place)
	std::list<float> values;
	bool likelihoodNullValuesIgnored = true;
	for(std::map<int, float>::iterator iter = ++likelihood.begin(); iter!=likelihood.end(); ++iter)
	{
		if((iter->second >= 0 && !likelihoodNullValuesIgnored) ||
		   (iter->second > 0 && likelihoodNullValuesIgnored))
		{
			values.push_back(iter->second);
		}
	}
	UDEBUG("values.size=%d", values.size());

	float mean = uMean(values);
	float stdDev = std::sqrt(uVariance(values, mean));


	//Adjust likelihood with mean and standard deviation (see Angeli phd)
	float epsilon = 0.0001;
	float max = 0.0f;
	int maxId = 0;
	for(std::map<int, float>::iterator iter=++likelihood.begin(); iter!= likelihood.end(); ++iter)
	{
		float value = iter->second;
		iter->second = 1.0f;
		if(value > mean+stdDev)
		{
			if(_virtualPlaceLikelihoodRatio==0 && mean)
			{
				iter->second = (value-(stdDev-epsilon))/mean;
			}
			else if(_virtualPlaceLikelihoodRatio!=0 && stdDev)
			{
				iter->second = (value-mean)/stdDev;
			}
		}

		if(value > max)
		{
			max = value;
			maxId = iter->first;
		}
	}

	if(_virtualPlaceLikelihoodRatio==0 && stdDev > epsilon && max)
	{
		likelihood.begin()->second = mean/stdDev + 1.0f;
	}
	else if(_virtualPlaceLikelihoodRatio!=0 && max > mean)
	{
		likelihood.begin()->second = stdDev/(max-mean) + 1.0f;
	}
	else
	{
		likelihood.begin()->second = 2.0f; //2 * std dev
	}

	double time = timer.ticks();
	UDEBUG("mean=%f, stdDev=%f, max=%f, maxId=%d, time=%fs", mean, stdDev, max, maxId, time);
}

void Rtabmap::dumpPrediction() const
{
	if(_memory && _bayesFilter)
	{
		if(this->getWorkingDir().empty())
		{
			UERROR("Working directory not set.");
			return;
		}
		std::list<int> signaturesToCompare;
		for(std::map<int, double>::const_iterator iter=_memory->getWorkingMem().begin();
			iter!=_memory->getWorkingMem().end();
			++iter)
		{
			if(iter->first > 0)
			{
				const Signature * s = _memory->getSignature(iter->first);
				UASSERT(s!=0);
				if(s->getWeight() != -1) // ignore intermediate nodes
				{
					signaturesToCompare.push_back(iter->first);
				}
			}
			else
			{
				// virtual signature should be added
				signaturesToCompare.push_back(iter->first);
			}
		}
		cv::Mat prediction = _bayesFilter->generatePrediction(_memory, uListToVector(signaturesToCompare));

		FILE* fout = 0;
		std::string fileName = this->getWorkingDir() + "/DumpPrediction.txt";
		#ifdef _MSC_VER
			fopen_s(&fout, fileName.c_str(), "w");
		#else
			fout = fopen(fileName.c_str(), "w");
		#endif

		if(fout)
		{
			for(int i=0; i<prediction.rows; ++i)
			{
				for(int j=0; j<prediction.cols; ++j)
				{
					fprintf(fout, "%f ",((float*)prediction.data)[j + i*prediction.cols]);
				}
				fprintf(fout, "\n");
			}
			fclose(fout);
		}
	}
	else
	{
		UWARN("Memory and/or the Bayes filter are not created");
	}
}

Signature Rtabmap::getSignatureCopy(int id, bool images, bool scan, bool userData, bool occupancyGrid, bool withWords, bool withGlobalDescriptors) const
{
	Signature s;
	if(_memory)
	{
		Transform odomPoseLocal;
		int weight = -1;
		int mapId = -1;
		std::string label;
		double stamp = 0;
		Transform groundTruth;
		std::vector<float> velocity;
		GPS gps;
		EnvSensors sensors;
		_memory->getNodeInfo(id, odomPoseLocal, mapId, weight, label, stamp, groundTruth, velocity, gps, sensors, true);
		SensorData data;
		data.setId(id);
		if(images || scan || userData || occupancyGrid)
		{
			data = _memory->getNodeData(id, images, scan, userData, occupancyGrid);
		}
		if(!images && withWords)
		{
			std::vector<CameraModel> models;
			std::vector<StereoCameraModel> stereoModels;
			_memory->getNodeCalibration(id, models, stereoModels);
			data.setCameraModels(models);
			data.setStereoCameraModels(stereoModels);
		}

		s=Signature(id,
				mapId,
				weight,
				stamp,
				label,
				odomPoseLocal,
				groundTruth,
				data);

		std::multimap<int, Link> links = _memory->getLinks(id, true, true);
		for(std::multimap<int, Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
		{
			if(iter->second.type() == Link::kLandmark)
			{
				s.addLandmark(iter->second);
			}
			else
			{
				s.addLink(iter->second);
			}
		}

		if(withWords || withGlobalDescriptors)
		{
			std::multimap<int, int> words;
			std::vector<cv::KeyPoint> wordsKpts;
			std::vector<cv::Point3f> words3;
			cv::Mat wordsDescriptors;
			std::vector<rtabmap::GlobalDescriptor> globalDescriptors;
			_memory->getNodeWordsAndGlobalDescriptors(id, words, wordsKpts, words3, wordsDescriptors, globalDescriptors);
			if(withWords)
			{
				s.setWords(words, wordsKpts, words3, wordsDescriptors);
			}
			if(withGlobalDescriptors)
			{
				s.sensorData().setGlobalDescriptors(globalDescriptors);
			}
		}
		if(velocity.size()==6)
		{
			s.setVelocity(velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]);
		}
		s.sensorData().setGPS(gps);
		s.sensorData().setEnvSensors(sensors);
	}
	return s;
}

void Rtabmap::get3DMap(
		std::map<int, Signature> & signatures,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		bool optimized,
		bool global) const
{
	UDEBUG("");
	return getGraph(poses, constraints, optimized, global, &signatures, true, true, true, true);
}

void Rtabmap::getGraph(
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		bool optimized,
		bool global,
		std::map<int, Signature> * signatures,
		bool withImages,
		bool withScan,
		bool withUserData,
		bool withGrid,
		bool withWords,
		bool withGlobalDescriptors) const
{
	if(_memory && _memory->getLastWorkingSignature(!global))
	{
		if(_rgbdSlamMode)
		{
			if(optimized)
			{
				poses = _optimizedPoses; // guess
				cv::Mat covariance;
				this->optimizeCurrentMap(_memory->getLastWorkingSignature(!global)->id(), global, poses, covariance, &constraints);
				if(!global && !_optimizedPoses.empty())
				{
					// We send directly the already optimized poses if they are set
					UDEBUG("_optimizedPoses=%ld poses=%ld", _optimizedPoses.size(), poses.size());
					poses = _optimizedPoses;
				}
			}
			else
			{
				std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature(!global)->id(), 0, global?-1:0, true);
				_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, global);
			}
		}
		else
		{
			// no optimization on appearance-only mode
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature(!global)->id(), 0, global?-1:0, true);
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, global);
		}

		if(signatures)
		{
			// Get data
			std::set<int> ids = uKeysSet(_memory->getWorkingMem()); // WM

			//remove virtual signature
			ids.erase(Memory::kIdVirtual);

			ids.insert(_memory->getStMem().begin(), _memory->getStMem().end()); // STM + WM
			if(global)
			{
				ids = _memory->getAllSignatureIds(); // STM + WM + LTM, ignoreChildren=true
			}

			for(std::set<int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
			{
				signatures->insert(std::make_pair(*iter, getSignatureCopy(*iter, withImages, withScan, withUserData, withGrid, withWords, withGlobalDescriptors)));
			}
		}
	}
	else if(_memory && (_memory->getStMem().size() || _memory->getWorkingMem().size() > 1))
	{
		UERROR("Last working signature is null!?");
	}
	else if(_memory == 0)
	{
		UWARN("Memory not initialized...");
	}
}

std::map<int, Transform> Rtabmap::getNodesInRadius(const Transform & pose, float radius, int k, std::map<int, float> * distsSqr)
{
	std::map<int, float> nearestNodesTmp;
	std::map<int, float> * nearestNodesPtr = distsSqr == 0? &nearestNodesTmp : distsSqr;
	*nearestNodesPtr = graph::findNearestNodes(pose, _optimizedPoses, radius<=0?_localRadius:radius, 0, k);
	std::map<int, Transform> nearestPoses;
	for(std::map<int, float>::iterator iter=nearestNodesPtr->begin(); iter!=nearestNodesPtr->end(); ++iter)
	{
		nearestPoses.insert(*_optimizedPoses.find(iter->first));
	}
	return nearestPoses;
}

std::map<int, Transform> Rtabmap::getNodesInRadius(int nodeId, float radius, int k, std::map<int, float> * distsSqr)
{
	UDEBUG("nodeId=%d, radius=%f", nodeId, radius);
	std::map<int, float> nearestNodesTmp;
	std::map<int, float> * nearestNodesPtr = distsSqr == 0? &nearestNodesTmp : distsSqr;
	if(nodeId==0 && !_lastLocalizationPose.isNull() && !_lastLocalizationPose.isIdentity())
	{
		*nearestNodesPtr = graph::findNearestNodes(_lastLocalizationPose, _optimizedPoses, radius<=0?_localRadius:radius, 0, k);
	}
	else
	{
		if(nodeId==0 && !_optimizedPoses.empty())
		{
			nodeId = _optimizedPoses.rbegin()->first;
		}

		if(_optimizedPoses.find(nodeId) != _optimizedPoses.end())
		{
			*nearestNodesPtr = graph::findNearestNodes(nodeId, _optimizedPoses, radius<=0?_localRadius:radius, 0, k);
		}
	}

	std::map<int, Transform> nearestPoses;
	for(std::map<int, float>::iterator iter=nearestNodesPtr->begin(); iter!=nearestNodesPtr->end(); ++iter)
	{
		nearestPoses.insert(*_optimizedPoses.find(iter->first));
	}

	return nearestPoses;
}

int Rtabmap::detectMoreLoopClosures(
		float clusterRadiusMax,
		float clusterAngle,
		int iterations,
		bool intraSession,
		bool interSession,
		const ProgressState * processState,
		float clusterRadiusMin,
		int toFromMapId)
{
	UDEBUG("");
	UASSERT(iterations>0);
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("detectMoreLoopClosures is disabled while safe relocalization owns graph mutations");
		return -1;
	}

	if(_graphOptimizer->iterations() <= 0)
	{
		UERROR("Cannot detect more loop closures if graph optimization iterations = 0");
		return -1;
	}
	if(!_rgbdSlamMode)
	{
		UERROR("Detecting more loop closures can be done only in RGBD-SLAM mode.");
		return -1;
	}
	if(!intraSession && !interSession)
	{
		UERROR("Intra and/or inter session argument should be true.");
		return -1;
	}

	std::list<Link> loopClosuresAdded;
	std::multimap<int, int> checkedLoopClosures;

	std::map<int, Transform> posesToCheckLoopClosures;
	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	this->getGraph(poses, links, true, true);

	std::map<int, int> mapIds;
	UDEBUG("remove all invalid or intermediate nodes, fill mapIds");
	for(std::map<int, Transform>::iterator iter=poses.upper_bound(0); iter!=poses.end();++iter)
	{
		Transform odom, gt;
		int mapId, weight;
		std::string l;
		double s;
		std::vector<float> v;
		GPS gps;
		EnvSensors srs;
		if(_memory->getNodeInfo(iter->first, odom, mapId, weight, l, s, gt, v, gps, srs, true) && weight >= 0)
		{
			posesToCheckLoopClosures.insert(*iter);
			mapIds.insert(std::make_pair(iter->first, mapId));
		}
	}

	for(int n=0; n<iterations; ++n)
	{
		UINFO("Looking for more loop closures, clustering poses... (iteration=%d/%d, radius=%f m angle=%f rad)",
				n+1, iterations, clusterRadiusMax, clusterAngle);

		std::multimap<int, int> clusters = graph::radiusPosesClustering(
				posesToCheckLoopClosures,
				clusterRadiusMax,
				clusterAngle);

		UINFO("Looking for more loop closures: clustering poses... found %ld clusters.", clusters.size());

		if(toFromMapId >=0)
		{
			size_t clustersBefore = clusters.size();
			for(std::multimap<int, int>::iterator iter=clusters.begin(); iter!=clusters.end();)
			{
				int mapId = uValue(mapIds, iter->first, 0);
				if(mapId != toFromMapId)
				{
					iter = clusters.erase(iter);
				}
				else {
					++iter;
				}
			}
			UINFO("Looking for more loop closures: filtered %ld/%ld clusters for map session %d.", clustersBefore-clusters.size(), clustersBefore, toFromMapId);
			if(clusters.empty())
			{
				UERROR("No clusters belong to mapId %d, aborting.", toFromMapId);
				break;
			}
		}

		if(_memory->getMaxStMemSize() > 1)
		{
			size_t clustersBefore = clusters.size();
			for(std::multimap<int, int>::iterator iter=clusters.begin(); iter!=clusters.end();)
			{
				if(abs(iter->first - iter->second) < _memory->getMaxStMemSize())
				{
					iter = clusters.erase(iter);
				}
				else
				{
					// compute path to know how far we are in terms of graph length
					std::map<int, int> ids = _memory->getNeighborsId(iter->first, _memory->getMaxStMemSize(), -1, true, true, true);
					if(ids.find(iter->second) != ids.end())
					{
						iter = clusters.erase(iter);
					}
					else
					{
						++iter;
					}
				}
			}
			UINFO("Looking for more loop closures: filtered %ld/%ld clusters for too close nodes (below %s=%d).",
				clustersBefore-clusters.size(), clustersBefore, Parameters::kMemSTMSize().c_str(), _memory->getMaxStMemSize());
		}

		int i=0;
		std::set<int> addedLinks;
		for(std::multimap<int, int>::iterator iter=clusters.begin(); iter!= clusters.end(); ++iter, ++i)
		{
			if(processState && processState->isCanceled())
			{
				return -1;
				break;
			}

			int from = iter->first;
			int to = iter->second;
			if(from > to)
			{
				from = iter->second;
				to = iter->first;
			}

			int mapIdFrom = uValue(mapIds, from, 0);
			int mapIdTo = uValue(mapIds, to, 0);

			if((interSession && mapIdFrom != mapIdTo) ||
			   (intraSession && mapIdFrom == mapIdTo))
			{

				bool alreadyChecked = false;
				for(std::multimap<int, int>::iterator jter = checkedLoopClosures.lower_bound(from);
					!alreadyChecked && jter!=checkedLoopClosures.end() && jter->first == from;
					++jter)
				{
					if(to == jter->second)
					{
						alreadyChecked = true;
					}
				}

				if(!alreadyChecked)
				{
					// only add new links and one per cluster per iteration
					if(addedLinks.find(from) == addedLinks.end() &&
					   addedLinks.find(to) == addedLinks.end() &&
					   rtabmap::graph::findLink(links, from, to) == links.end())
					{
						// Reverify if in the bounds with the current optimized graph
						Transform delta = poses.at(from).inverse() * poses.at(to);
						if(delta.getNorm() < clusterRadiusMax &&
						   delta.getNorm() >= clusterRadiusMin)
						{
							checkedLoopClosures.insert(std::make_pair(from, to));

							Signature fromS = getSignatureCopy(from, false, true, false, false, true, false);
							Signature toS = getSignatureCopy(to, false, true, false, false, true, false);
							UASSERT(fromS.getWeight()>=0);
							UASSERT(toS.getWeight()>=0);

							Transform guess;
							if(_proximityBySpace && uContains(poses, from) && uContains(poses, to))
							{
								guess = poses.at(from).inverse() * poses.at(to);
							}

							RegistrationInfo info;
							// use signatures instead of IDs because some signatures may not be in WM
							Transform t = _memory->computeTransform(fromS, toS, guess, &info);

							if(!t.isNull())
							{
								bool updateConstraints = true;
								if(_safeRelocalizationGuardEnabled &&
										getSignatureResetEpoch(from) != getSignatureResetEpoch(to))
								{
									// This post-processing routine reuses its locally optimized
									// pose map across candidates. Reject before linksIn/optimize so
									// a forbidden bridge cannot contaminate a later accepted result.
									UINFO("Observed but did not optimize/add post-processing cross-reset closure %d->%d; guarded transaction is required", from, to);
									continue;
								}

								//optimize the graph to see if the new constraint is globally valid

								int fromId = from;
								int mapId = fromS.mapId();
								// use first node of the map containing from
								for(std::map<int, Transform>::iterator ster=posesToCheckLoopClosures.begin(); ster!=posesToCheckLoopClosures.end(); ++ster)
								{
									if(uValue(mapIds, ster->first, 0) == mapId)
									{
										fromId = ster->first;
										break;
									}
								}
								std::multimap<int, Link> linksIn = links;
								linksIn.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, t, getInformation(info.covariance))));
								graph::MaxGraphErrors maxGraphErrors;
								std::map<int, Transform> optimizedPoses;
								std::multimap<int, Link> linksOut;
								UASSERT(poses.find(fromId) != poses.end());
								UASSERT_MSG(poses.find(from) != poses.end(), uFormat("id=%d poses=%d links=%d", from, (int)poses.size(), (int)links.size()).c_str());
								UASSERT_MSG(poses.find(to) != poses.end(), uFormat("id=%d poses=%d links=%d", to, (int)poses.size(), (int)links.size()).c_str());
								_graphOptimizer->getConnectedGraph(fromId, poses, linksIn, optimizedPoses, linksOut);
								UASSERT(optimizedPoses.find(fromId) != optimizedPoses.end());
								UASSERT_MSG(optimizedPoses.find(from) != optimizedPoses.end(), uFormat("id=%d poses=%d links=%d", from, (int)optimizedPoses.size(), (int)linksOut.size()).c_str());
								UASSERT_MSG(optimizedPoses.find(to) != optimizedPoses.end(), uFormat("id=%d poses=%d links=%d", to, (int)optimizedPoses.size(), (int)linksOut.size()).c_str());
								UASSERT(graph::findLink(linksOut, from, to) != linksOut.end());
								optimizedPoses = _graphOptimizer->optimize(fromId, optimizedPoses, linksOut);
								std::string msg;
								if(optimizedPoses.size())
								{
									maxGraphErrors = graph::computeMaxGraphErrors(
											optimizedPoses,
											linksOut);
									if(maxGraphErrors.linearLink.isValid())
									{
										UINFO("Max optimization linear error = %f m (link %d->%d)", maxGraphErrors.linear, maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to());
										if(_optimizationMaxError > 0.0f && maxGraphErrors.linearRatio > _optimizationMaxError)
										{
											msg = uFormat("Rejecting edge %d->%d because "
														"graph error is too large after optimization (%f m for edge %d->%d with ratio %f > std=%f m). "
														"\"%s\" is %f.",
														from,
														to,
														maxGraphErrors.linear,
														maxGraphErrors.linearLink.from(),
														maxGraphErrors.linearLink.to(),
														maxGraphErrors.linearRatio,
														sqrt(maxGraphErrors.linearLink.transVariance()),
														Parameters::kRGBDOptimizeMaxError().c_str(),
														_optimizationMaxError);
										}
										else if(_optimizationMaxError == 0.0f && maxGraphErrors.linearRatio>100 && !_graphOptimizer->isRobust())
										{
											UERROR("Huge optimization error detected!"
													"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
													"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
													maxGraphErrors.linearRatio,
													maxGraphErrors.linearLink.from(),
													maxGraphErrors.linearLink.to(),
													maxGraphErrors.linearLink.type(),
													maxGraphErrors.linear,
													sqrt(maxGraphErrors.linearLink.transVariance()),
													Parameters::kRGBDOptimizeMaxError().c_str());
										}
									}
									else if(maxGraphErrors.angularLink.isValid())
									{
										UINFO("Max optimization angular error = %f deg (link %d->%d)", maxGraphErrors.angular*180.0f/M_PI, maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to());
										if(_optimizationMaxError > 0.0f && maxGraphErrors.angularRatio > _optimizationMaxError)
										{
											msg = uFormat("Rejecting edge %d->%d because "
														"graph error is too large after optimization (%f deg for edge %d->%d with ratio %f > std=%f deg). "
														"\"%s\" is %f m.",
														from,
														to,
														maxGraphErrors.angular*180.0f/M_PI,
														maxGraphErrors.angularLink.from(),
														maxGraphErrors.angularLink.to(),
														maxGraphErrors.angularRatio,
														sqrt(maxGraphErrors.angularLink.rotVariance()),
														Parameters::kRGBDOptimizeMaxError().c_str(),
														_optimizationMaxError);
										}
										else if(_optimizationMaxError == 0.0f && maxGraphErrors.angularRatio>100 && !_graphOptimizer->isRobust())
										{
											UERROR("Huge optimization error detected!"
													"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
													"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
													maxGraphErrors.angularRatio,
													maxGraphErrors.angularLink.from(),
													maxGraphErrors.angularLink.to(),
													maxGraphErrors.angularLink.type(),
													maxGraphErrors.angular*180.0f/CV_PI,
													sqrt(maxGraphErrors.angularLink.rotVariance()),
													Parameters::kRGBDOptimizeMaxError().c_str());
										}
									}
								}
								else
								{
									msg = uFormat("Rejecting edge %d->%d because graph optimization has failed!",
												from,
												to);
								}
								if(!msg.empty())
								{
									UWARN("%s", msg.c_str());
									updateConstraints = false;
								}
								else
								{
									poses = optimizedPoses;
								}

								if(updateConstraints)
								{
									addedLinks.insert(from);
									addedLinks.insert(to);
									cv::Mat inf = getInformation(info.covariance);
									links.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, t, inf)));
									loopClosuresAdded.push_back(Link(from, to, Link::kUserClosure, t, inf));
									std::string msg = uFormat("Iteration %d/%d: Added loop closure %d->%d! (%d/%d)", n+1, iterations, from, to, i+1, (int)clusters.size());
									UINFO(msg.c_str());

									if(processState)
									{
										UINFO(msg.c_str());
										if(!processState->callback(msg))
										{
											return -1;
										}
									}
								}
							}
						}
					}
				}
			}
		}

		if(processState)
		{
			std::string msg = uFormat("Iteration %d/%d: Detected %d total loop closures!", n+1, iterations, (int)addedLinks.size()/2);
			UINFO(msg.c_str());
			if(!processState->callback(msg))
			{
				return -1;
			}
		}
		else
		{
			UINFO("Iteration %d/%d: Detected %d total loop closures!", n+1, iterations, (int)addedLinks.size()/2);
		}

		if(addedLinks.size() == 0)
		{
			break;
		}

		UINFO("Optimizing graph with new links (%d nodes, %d constraints)...",
				(int)poses.size(), (int)links.size());
		int fromId = _optimizeFromGraphEnd?poses.rbegin()->first:poses.begin()->first;
		poses = _graphOptimizer->optimize(fromId, poses, links, 0);
		if(poses.size() == 0)
		{
			UERROR("Optimization failed! Rejecting all loop closures...");
			loopClosuresAdded.clear();
			return -1;
		}
		UINFO("Optimizing graph with new links... done!");
	}
	UINFO("Total added %d loop closures.", (int)loopClosuresAdded.size());

	if(loopClosuresAdded.size())
	{
		for(std::list<Link>::iterator iter=loopClosuresAdded.begin(); iter!=loopClosuresAdded.end(); ++iter)
		{
			_memory->addLink(*iter, true);
		}
		// Update optimized poses
		for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
		{
			std::map<int, Transform>::iterator jter = poses.find(iter->first);
			if(jter != poses.end())
			{
				iter->second = jter->second;
			}
		}
		std::map<int, Transform> tmp;
		// Update also the links if some have been added in WM
		_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false);
		// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
		_memory->save2DMap(cv::Mat(), 0, 0, 0);
	}
	return (int)loopClosuresAdded.size();
}

bool Rtabmap::globalBundleAdjustment(
		int optimizerType,
		bool rematchFeatures,
		int iterations,
		float pixelVariance)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("globalBundleAdjustment is disabled while safe relocalization owns graph mutations");
		return false;
	}
	if(!_optimizedPoses.empty() && !_constraints.empty())
	{
		int iterations = Parameters::defaultOptimizerIterations();
		float pixelVariance = Parameters::defaultg2oPixelVariance();
		ParametersMap params = _parameters;
		Parameters::parse(params, Parameters::kOptimizerIterations(), iterations);
		Parameters::parse(params, Parameters::kg2oPixelVariance(), pixelVariance);
		if(iterations > 0)
		{
			uInsert(params, ParametersPair(Parameters::kOptimizerIterations(), uNumber2Str(iterations)));
		}
		if(pixelVariance > 0.0f)
		{
			uInsert(params, ParametersPair(Parameters::kg2oPixelVariance(), uNumber2Str(pixelVariance)));
		}

		std::map<int, Signature> signatures;
		for(std::map<int, Transform>::iterator iter=_optimizedPoses.lower_bound(1); iter!=_optimizedPoses.end(); ++iter)
		{
			if(_memory->getSignature(iter->first))
			{
				signatures.insert(std::make_pair(iter->first, *_memory->getSignature(iter->first)));
			}
		}

		Optimizer * optimizer = Optimizer::create((Optimizer::Type)optimizerType, params);
		std::map<int, Transform> poses = optimizer->optimizeBA(
				_optimizeFromGraphEnd?_optimizedPoses.lower_bound(1)->first:_optimizedPoses.rbegin()->first,
				_optimizedPoses,
				_constraints,
				signatures,
				rematchFeatures);
		delete optimizer;

		if(poses.empty())
		{
			UERROR("Optimization failed!");
		}
		else
		{
			_optimizedPoses = poses;
			// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
			_memory->save2DMap(cv::Mat(), 0, 0, 0);
			return true;
		}
	}
	else
	{
		UERROR("Optimized poses (%ld) or constraints (%ld) are empty!", _optimizedPoses.size(), _constraints.size());
	}
	return false;
}

int Rtabmap::cleanupLocalGrids(
		const std::map<int, Transform> & poses,
		const cv::Mat & map,
		float xMin,
		float yMin,
		float cellSize,
		int cropRadius,
		bool filterScans)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("cleanupLocalGrids is disabled while safe relocalization owns graph mutations");
		return -1;
	}
	if(_memory)
	{
		return _memory->cleanupLocalGrids(
				poses,
				map,
				xMin,
				yMin,
				cellSize,
				cropRadius,
				filterScans);
	}
	return -1;
}

int Rtabmap::refineLinks()
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("refineLinks is disabled while safe relocalization owns graph mutations");
		return -1;
	}
	if(!_rgbdSlamMode)
	{
		UERROR("Refining links can be done only in RGBD-SLAM mode.");
		return -1;
	}

	std::list<Link> linksRefined;

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	std::map<int, Signature> signatures;
	this->getGraph(poses, links, false, true, &signatures);

	int i=0;
	for(std::multimap<int, Link>::iterator iter=links.lower_bound(1); iter!= links.end(); ++iter)
	{
		int from = iter->second.from();
		int to = iter->second.to();
		if(_safeRelocalizationGuardEnabled && from > 0 && to > 0 &&
				getSignatureResetEpoch(from) != getSignatureResetEpoch(to))
		{
			UINFO("Did not refine cross-reset link %d->%d; a guarded three-bridge transaction cannot be rewritten one edge at a time", from, to);
			continue;
		}

		UASSERT(signatures.find(from) != signatures.end());
		UASSERT(signatures.find(to) != signatures.end());

		RegistrationInfo info;
		// use signatures instead of IDs because some signatures may not be in WM
		Transform t = _memory->computeTransform(signatures.at(from), signatures.at(to), iter->second.transform(), &info);

		if(!t.isNull())
		{
			linksRefined.push_back(Link(from, to, iter->second.type(), t, info.covariance.inv()));
			UINFO("Refined link %d->%d! (%d/%d)", from, to, ++i, (int)links.size());
		}
	}
	UINFO("Total refined %d links.", (int)linksRefined.size());

	if(linksRefined.size())
	{
		for(std::list<Link>::iterator iter=linksRefined.begin(); iter!=linksRefined.end(); ++iter)
		{
			_memory->updateLink(*iter, true);
		}
	}
	return (int)linksRefined.size();
}

bool Rtabmap::addLink(const Link & link)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("External addLink is disabled while safe relocalization owns graph mutations");
		return false;
	}
	const Transform & t = link.transform();
	if(!_rgbdSlamMode)
	{
		UERROR("Adding new link can be done only in RGBD-SLAM mode.");
		return false;
	}
	if(!_memory)
	{
		UERROR("Memory is not initialized.");
		return false;
	}
	if(t.isNull())
	{
		UERROR("Link's transform is null! (%d->%d type=%s)", link.from(), link.to(), link.typeName().c_str());
		return false;
	}
	if(_memory->isIncremental())
	{
		if(_safeRelocalizationGuardEnabled && link.from() > 0 && link.to() > 0 &&
				getSignatureResetEpoch(link.from()) != getSignatureResetEpoch(link.to()))
		{
			UERROR("External addLink rejected for cross-reset edge %d->%d (epochs %llu->%llu): use the guarded safe-relocalization transaction",
					link.from(), link.to(),
					getSignatureResetEpoch(link.from()), getSignatureResetEpoch(link.to()));
			return false;
		}
		if(_memory->getSignature(link.from()) == 0)
		{
			UERROR("Link's \"from id\" %d is not in working memory", link.from());
			return false;
		}
		if(_memory->getSignature(link.to()) == 0)
		{
			UERROR("Link's \"to id\" %d is not in working memory", link.to());
			return false;
		}

		if(_optimizedPoses.find(link.from()) == _optimizedPoses.end() &&
		   _optimizedPoses.find(link.to()) == _optimizedPoses.end())
		{
			UERROR("Neither nodes %d or %d are in the local graph (size=%d). One of the 2 nodes should be in the local graph.", (int)_optimizedPoses.size(), link.from(), link.to());
			return false;
		}

		// add temporary the link
		if(!_memory->addLink(link))
		{
			UERROR("Cannot add new link %d->%d to memory", link.from(), link.to());
			return false;
		}

		// optimize with new link
		std::map<int, Transform> poses = _optimizedPoses;
		std::multimap<int, Link> links;
		cv::Mat covariance;
		optimizeCurrentMap(_memory->getLastWorkingSignature(true)->id(), false, poses, covariance, &links);

		if(poses.find(link.from()) == poses.end())
		{
			UERROR("Link's \"from id\" %d is not in the graph (size=%d)", link.from(), (int)poses.size());
			_memory->removeLink(link.from(), link.to());
			return false;
		}
		if(poses.find(link.to()) == poses.end())
		{
			UERROR("Link's \"to id\" %d is not in the graph (size=%d)", link.to(), (int)poses.size());
			_memory->removeLink(link.from(), link.to());
			return false;
		}

		std::string msg;
		if(poses.empty())
		{
			msg = uFormat("Rejecting edge %d->%d because graph optimization has failed!", link.from(), link.to());
		}
		else
		{
			graph::MaxGraphErrors maxGraphErrors;

			maxGraphErrors = graph::computeMaxGraphErrors(
					poses,
					links);
			if(maxGraphErrors.linearLink.isValid())
			{
				UINFO("Max optimization linear error = %f m (link %d->%d)", maxGraphErrors.linear, maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to());
				if(_optimizationMaxError > 0.0f && maxGraphErrors.linearRatio > _optimizationMaxError)
				{
					msg = uFormat("Rejecting edge %d->%d because "
							  "graph error is too large after optimization (%f m for edge %d->%d with ratio %f > std=%f m). "
							  "\"%s\" is %f.",
							  link.from(),
							  link.to(),
							  maxGraphErrors.linear,
							  maxGraphErrors.linearLink.from(),
							  maxGraphErrors.linearLink.to(),
							  maxGraphErrors.linearRatio,
							  sqrt(maxGraphErrors.linearLink.transVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
				}
				else if(_optimizationMaxError == 0.0f && maxGraphErrors.linearRatio>100 && !_graphOptimizer->isRobust())
				{
					UERROR("Huge optimization error detected!"
							"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
							"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
							maxGraphErrors.linearRatio,
							maxGraphErrors.linearLink.from(),
							maxGraphErrors.linearLink.to(),
							maxGraphErrors.linearLink.type(),
							maxGraphErrors.linear,
							sqrt(maxGraphErrors.linearLink.transVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str());
				}
			}
			else if(maxGraphErrors.angularLink.isValid())
			{
				UINFO("Max optimization angular error = %f deg (link %d->%d)", maxGraphErrors.angular*180.0f/M_PI, maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to());
				if(_optimizationMaxError > 0.0f && maxGraphErrors.angularRatio > _optimizationMaxError)
				{
					msg = uFormat("Rejecting edge %d->%d because "
							  "graph error is too large after optimization (%f deg for edge %d->%d with ratio %f > std=%f deg). "
							  "\"%s\" is %f m.",
							  link.from(),
							  link.to(),
							  maxGraphErrors.angular*180.0f/M_PI,
							  maxGraphErrors.angularLink.from(),
							  maxGraphErrors.angularLink.to(),
							  maxGraphErrors.angularRatio,
							  sqrt(maxGraphErrors.angularLink.rotVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
				}
				else if(_optimizationMaxError == 0.0f && maxGraphErrors.angularRatio>100 && !_graphOptimizer->isRobust())
				{
					UERROR("Huge optimization error detected!"
							"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
							"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
							maxGraphErrors.angularRatio,
							maxGraphErrors.angularLink.from(),
							maxGraphErrors.angularLink.to(),
							maxGraphErrors.angularLink.type(),
							maxGraphErrors.angular*180.0f/CV_PI,
							sqrt(maxGraphErrors.angularLink.rotVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str());
				}
			}
		}
		if(!msg.empty())
		{
			UERROR("%s", msg.c_str());
			_memory->removeLink(link.from(), link.to());
			return false;
		}

		// Update optimized poses
		for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
		{
			std::map<int, Transform>::iterator jter = poses.find(iter->first);
			if(jter != poses.end())
			{
				iter->second = jter->second;
			}
		}
		if(!_optimizeFromGraphEnd)
		{
			_mapCorrection = _optimizedPoses.rbegin()->second * _memory->getSignature(_optimizedPoses.rbegin()->first)->getPose().inverse();
		}

		std::map<int, Transform> tmp;
		// Update also the links if some have been added in WM
		_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false);
		// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
		_memory->save2DMap(cv::Mat(), 0, 0, 0);

		return true;
	}
	else // localization mode
	{
		int oldestId = link.from()>link.to()?link.to():link.from();
		int newestId = link.from()<link.to()?link.to():link.from();

		if(_memory->getSignature(oldestId) == 0)
		{
			UERROR("Link's id %d is not in working memory", oldestId);
			return false;
		}
		if(_optimizedPoses.find(oldestId) == _optimizedPoses.end())
		{
			UERROR("Link's id %d is not in the optimized graph (_optimizedPoses=%d)", oldestId, (int)_optimizedPoses.size());
			return false;
		}
		if(_optimizeFromGraphEnd)
		{
			UERROR("Adding link with %s=true in localization mode is not supported.", Parameters::kRGBDOptimizeFromGraphEnd().c_str());
			return false;
		}
		if(_odomCachePoses.find(newestId) == _odomCachePoses.end())
		{
			if(!_odomCachePoses.empty())
			{
				UERROR("Link's id %d is not in the odometry cache (oldest=%d, newest=%d, %s=%d)",
						newestId,
						_odomCachePoses.begin()->first,
						_odomCachePoses.rbegin()->first,
						Parameters::kRGBDMaxOdomCacheSize().c_str(),
						_maxOdomCacheSize);
			}
			else
			{
				UERROR("Link's id %d is not in the odometry cache (%s=%d).",
						newestId,
						Parameters::kRGBDMaxOdomCacheSize().c_str(),
						_maxOdomCacheSize);
			}
			return false;
		}

		// Verify if the new localization is valid by checking if there is
		// not too much deformation using current odometry poses
		// This will also refine localization links

		std::map<int, Transform> poses = _odomCachePoses;
		std::multimap<int, Link> constraints = _odomCacheConstraints;
		constraints.insert(std::make_pair(link.from(), link));
		cv::Mat priorInfMat = cv::Mat::eye(6,6, CV_64FC1)*_localizationPriorInf;
		for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end(); ++iter)
		{
			std::map<int, Transform>::iterator iterPose = _optimizedPoses.find(iter->second.to());
			if(iterPose != _optimizedPoses.end() && poses.find(iterPose->first) == poses.end())
			{
				poses.insert(*iterPose);
				// make the poses in the map fixed
				constraints.insert(std::make_pair(iterPose->first, Link(iterPose->first, iterPose->first, Link::kPosePrior, iterPose->second, priorInfMat)));
			}
		}

		std::map<int, Transform> posesOut;
		std::multimap<int, Link> edgeConstraintsOut;
		bool priorsIgnored = _graphOptimizer->priorsIgnored();
		_graphOptimizer->setPriorsIgnored(false); //temporary set false to use priors above to fix nodes of the map
		_graphOptimizer->getConnectedGraph(newestId, poses, constraints, posesOut, edgeConstraintsOut);
		std::map<int, Transform> optPoses = _graphOptimizer->optimize(poses.begin()->first, posesOut, edgeConstraintsOut);
		_graphOptimizer->setPriorsIgnored(priorsIgnored); // set back

		bool rejectLocalization = false;
		if(optPoses.empty())
		{
			UWARN("Optimization failed, rejecting localization!");
			rejectLocalization = true;
		}
		else
		{
			UINFO("Compute max graph errors...");
			graph::MaxGraphErrors maxGraphErrors = graph::computeMaxGraphErrors(
					optPoses,
					edgeConstraintsOut,
					_graphOptimizer->isSlam2d());
			if(!maxGraphErrors.linearLink.isValid() && !maxGraphErrors.angularLink.isValid())
			{
				UWARN("Could not compute graph errors! Wrong loop closures could be accepted!");
			}

			if(maxGraphErrors.linearLink.isValid())
			{
				UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f)", maxGraphErrors.linear, maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to(), maxGraphErrors.linearLink.transVariance(), maxGraphErrors.linear/sqrt(maxGraphErrors.linearLink.transVariance()));
				if(_optimizationMaxError > 0.0f && maxGraphErrors.linearRatio > _optimizationMaxError)
				{
					UWARN("Rejecting localization (%d <-> %d) in this "
							"iteration because a wrong loop closure has been "
							"detected after graph optimization, resulting in "
							"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
							"maximum error ratio parameter \"%s\" is %f of std deviation.",
							link.from(),
							link.to(),
							maxGraphErrors.linearRatio,
							maxGraphErrors.linearLink.from(),
							maxGraphErrors.linearLink.to(),
							maxGraphErrors.linearLink.type(),
							maxGraphErrors.linear,
							sqrt(maxGraphErrors.linearLink.transVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str(),
							_optimizationMaxError);
					rejectLocalization = true;
				}
				else if(_optimizationMaxError == 0.0f && maxGraphErrors.linearRatio>100 && !_graphOptimizer->isRobust())
				{
					UERROR("Huge optimization error detected!"
							"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
							"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
							maxGraphErrors.linearRatio,
							maxGraphErrors.linearLink.from(),
							maxGraphErrors.linearLink.to(),
							maxGraphErrors.linearLink.type(),
							maxGraphErrors.linear,
							sqrt(maxGraphErrors.linearLink.transVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str());
				}
			}
			if(maxGraphErrors.angularLink.isValid())
			{
				UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f)", maxGraphErrors.angular*180.0f/CV_PI, maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to(), maxGraphErrors.angularLink.rotVariance(), maxGraphErrors.angular/sqrt(maxGraphErrors.angularLink.rotVariance()));
				if(_optimizationMaxError > 0.0f && maxGraphErrors.angularRatio > _optimizationMaxError)
				{
					UWARN("Rejecting localization (%d <-> %d) in this "
							"iteration because a wrong loop closure has been "
							"detected after graph optimization, resulting in "
							"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
							"maximum error ratio parameter \"%s\" is %f of std deviation.",
							link.from(),
							link.to(),
							maxGraphErrors.angularRatio,
							maxGraphErrors.angularLink.from(),
							maxGraphErrors.angularLink.to(),
							maxGraphErrors.angularLink.type(),
							maxGraphErrors.angular*180.0f/CV_PI,
							sqrt(maxGraphErrors.angularLink.rotVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str(),
							_optimizationMaxError);
					rejectLocalization = true;
				}
				else if(_optimizationMaxError == 0.0f && maxGraphErrors.angularRatio>100 && !_graphOptimizer->isRobust())
				{
					UERROR("Huge optimization error detected!"
							"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
							"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
							maxGraphErrors.angularRatio,
							maxGraphErrors.angularLink.from(),
							maxGraphErrors.angularLink.to(),
							maxGraphErrors.angularLink.type(),
							maxGraphErrors.angular*180.0f/CV_PI,
							sqrt(maxGraphErrors.angularLink.rotVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str());
				}
			}
		}

		if(!rejectLocalization)
		{
			Transform newOptPoseInv = optPoses.at(link.from()).inverse();
			Transform newT = newOptPoseInv * optPoses.at(link.to());
			Link linkTmp = link;
			linkTmp.setTransform(newT);
			if(oldestId == link.from())
			{
				_lastLocalizationPose = _optimizedPoses.at(link.from()) * linkTmp.transform();
				_odomCacheConstraints.insert(std::make_pair(linkTmp.to(), linkTmp.inverse()));
			}
			else
			{
				_lastLocalizationPose = _optimizedPoses.at(link.to()) * linkTmp.transform().inverse();
				_odomCacheConstraints.insert(std::make_pair(linkTmp.from(), linkTmp));
			}
			UINFO("Set _lastLocalizationPose=%s", _lastLocalizationPose.prettyPrint().c_str());
			if(_graphOptimizer->isSlam2d())
			{
				// transform constraint to 2D
				_lastLocalizationPose = _lastLocalizationPose.to3DoF();
			}
			Transform odomPose = _odomCachePoses.find(newestId)->second;
			_mapCorrection = _lastLocalizationPose * odomPose.inverse();
			_lastLocalizationNodeId = oldestId;
			return true;
		}
	}
	return false;
}
void Rtabmap::rememberCurrentGraphForSafeRelocalization()
{
	if(_optimizedPoses.empty())
	{
		return;
	}
	bool trustedSnapshotValid = true;
	std::map<int, Transform> trustedPoses;
	for(std::map<int, Transform>::const_iterator iter=_optimizedPoses.begin();
			iter!=_optimizedPoses.end(); ++iter)
	{
		if(iter->first > 0)
		{
			if(safeRigidTransform(iter->second))
			{
				trustedPoses.insert(*iter);
			}
			else
			{
				trustedSnapshotValid = false;
			}
		}
	}
	std::multimap<int, Link> trustedConstraints;
	for(std::multimap<int, Link>::const_iterator iter=_constraints.begin();
			iter!=_constraints.end(); ++iter)
	{
		const Link & link = iter->second;
		if(link.from() <= 0 || link.to() <= 0 ||
				trustedPoses.find(link.from()) == trustedPoses.end() ||
				trustedPoses.find(link.to()) == trustedPoses.end())
		{
			if(link.from() > 0 && link.to() > 0)
			{
				trustedSnapshotValid = false;
			}
			continue;
		}
		trustedConstraints.insert(std::make_pair(link.from(), link));
	}
	// Replace the trusted snapshot exactly. A union/update would retain links
	// removed from the live graph and could resurrect a rejected edge during a
	// later reset epoch.
	_safeRelocalizationTrustedPoses.swap(trustedPoses);
	_safeRelocalizationTrustedConstraints.swap(trustedConstraints);
	_safeRelocalizationTrustedSnapshotValid = trustedSnapshotValid;
	rebuildSafeRelocalizationTrustedComponents();
}

bool Rtabmap::promoteCommittedEpochForSafeRelocalization(std::string & reason)
{
	reason.clear();
	if(_safeRelocalizationCommittedEpochs.find(_resetEpoch) ==
			_safeRelocalizationCommittedEpochs.end() ||
			!_safeRelocalizationPromotionLimitsValid ||
			!std::isfinite(_safeRelocalizationPromotionMaxOldPoseTranslation) ||
			!std::isfinite(_safeRelocalizationPromotionMaxOldPoseRotation) ||
			_safeRelocalizationPromotionMaxOldPoseTranslation <= 0.0f ||
			_safeRelocalizationPromotionMaxOldPoseRotation <= 0.0f ||
			!_safeRelocalizationTrustedSnapshotValid ||
			_safeRelocalizationTrustedPoses.empty() || _optimizedPoses.empty())
	{
		reason = "committed epoch or complete trusted/live graph is unavailable";
		return false;
	}

	std::set<int> representedComponents;
	std::set<int> liveIds;
	for(std::map<int, Transform>::const_iterator iter=_optimizedPoses.begin();
			iter!=_optimizedPoses.end(); ++iter)
	{
		if(iter->first <= 0)
		{
			continue;
		}
		if(!safeRigidTransform(iter->second))
		{
			reason = "live committed graph contains an invalid pose";
			return false;
		}
		liveIds.insert(iter->first);
		std::map<int, int>::const_iterator trusted =
				_safeRelocalizationTrustedComponents.find(iter->first);
		if(trusted != _safeRelocalizationTrustedComponents.end())
		{
			representedComponents.insert(trusted->second);
		}
		else if(getSignatureResetEpoch(iter->first) != _resetEpoch)
		{
			reason = "live graph contains an untrusted node from another reset epoch";
			return false;
		}
	}
	if(liveIds.empty() || representedComponents.size() != 1)
	{
		reason = "live committed graph must contain exactly one trusted component";
		return false;
	}
	const int representedComponent = *representedComponents.begin();
	std::set<int> replacedTrustedIds;
	std::set<int> untouchedTrustedIds;
	for(std::map<int, int>::const_iterator iter=_safeRelocalizationTrustedComponents.begin();
			iter!=_safeRelocalizationTrustedComponents.end(); ++iter)
	{
		if(iter->second == representedComponent)
		{
			replacedTrustedIds.insert(iter->first);
			if(liveIds.find(iter->first) == liveIds.end())
			{
				reason = "live graph dropped a node from its committed trusted component";
				return false;
			}
		}
		else
		{
			untouchedTrustedIds.insert(iter->first);
		}
	}
	const int gaugeReference = *replacedTrustedIds.begin();
	const Transform liveToFrozen =
			_safeRelocalizationTrustedPoses.at(gaugeReference) *
			_optimizedPoses.at(gaugeReference).inverse();
	if(!safeRigidTransform(liveToFrozen))
	{
		reason = "cannot establish a finite rigid gauge from live to frozen trusted poses";
		return false;
	}
	for(std::set<int>::const_iterator iter=replacedTrustedIds.begin();
			iter!=replacedTrustedIds.end(); ++iter)
	{
		const Transform aligned = liveToFrozen * _optimizedPoses.at(*iter);
		const Transform & frozen = _safeRelocalizationTrustedPoses.at(*iter);
		if(!safeRigidTransform(aligned) ||
				aligned.getDistance(frozen) >
						_safeRelocalizationPromotionMaxOldPoseTranslation ||
				aligned.getAngle(frozen) >
						_safeRelocalizationPromotionMaxOldPoseRotation*M_PI/180.0f)
		{
			reason = "post-commit mapping non-rigidly deformed a frozen trusted pose";
			return false;
		}
	}

	std::multimap<int, Link> liveConstraints;
	std::map<int, std::set<int> > adjacency;
	for(std::set<int>::const_iterator iter=liveIds.begin(); iter!=liveIds.end(); ++iter)
	{
		adjacency[*iter];
	}
	for(std::multimap<int, Link>::const_iterator iter=_constraints.begin();
			iter!=_constraints.end(); ++iter)
	{
		const Link & link = iter->second;
		if(link.from() <= 0 || link.to() <= 0)
		{
			continue;
		}
		if(liveIds.find(link.from()) == liveIds.end() ||
				liveIds.find(link.to()) == liveIds.end() ||
				!safeRigidTransform(link.transform()))
		{
			reason = "live committed graph contains a dangling or invalid constraint";
			return false;
		}
		std::string informationReason;
		if(!safeInformationMatrix(link.infMatrix(), 100000000.0, informationReason, false))
		{
			reason = "live committed graph contains unsafe constraint information: " + informationReason;
			return false;
		}
		if(getSignatureResetEpoch(link.from()) != getSignatureResetEpoch(link.to()))
		{
			const bool exactCommittedBridge = containsConstraintMeasurementUndirected(
					_safeRelocalizationTrustedConstraints, link);
			if(!exactCommittedBridge)
			{
				reason = "live graph contains a new or rewritten cross-reset constraint";
				return false;
			}
		}
		liveConstraints.insert(*iter);
		if(link.from() != link.to())
		{
			adjacency[link.from()].insert(link.to());
			adjacency[link.to()].insert(link.from());
		}
	}

	// Every frozen edge in the represented component, including the exact
	// three committed bridges, must still be present with the same measurement.
	for(std::multimap<int, Link>::const_iterator iter=_safeRelocalizationTrustedConstraints.begin();
			iter!=_safeRelocalizationTrustedConstraints.end(); ++iter)
	{
		const Link & expected = iter->second;
		if(replacedTrustedIds.find(expected.from()) == replacedTrustedIds.end() ||
				replacedTrustedIds.find(expected.to()) == replacedTrustedIds.end())
		{
			continue;
		}
		const bool found = containsConstraintMeasurementUndirected(
				liveConstraints, expected);
		if(!found)
		{
			reason = "a frozen trusted constraint was removed or rewritten after commit";
			return false;
		}
	}

	std::set<int> visited;
	std::queue<int> pending;
	pending.push(*replacedTrustedIds.begin());
	visited.insert(*replacedTrustedIds.begin());
	while(!pending.empty())
	{
		const int id = pending.front();
		pending.pop();
		for(std::set<int>::const_iterator neighbor=adjacency[id].begin();
				neighbor!=adjacency[id].end(); ++neighbor)
		{
			if(visited.insert(*neighbor).second)
			{
				pending.push(*neighbor);
			}
		}
	}
	if(visited != liveIds)
	{
		reason = "post-commit live graph is not fully connected to its trusted component";
		return false;
	}

	// Preserve every already-trusted pose exactly.  A graph optimizer can move
	// the whole live graph and also deform the frozen part slightly.  Using a
	// gauge estimated from an arbitrary old node for a new extension can then
	// leave a discontinuity at the first odometry edge after the frozen
	// boundary.  Anchor the complete extension with that exact boundary edge.
	// This keeps the frozen graph byte-for-byte stable and preserves every
	// relative edge inside the extension under one rigid transform.
	std::set<int> extensionIds;
	for(std::set<int>::const_iterator iter=liveIds.begin(); iter!=liveIds.end(); ++iter)
	{
		if(_safeRelocalizationTrustedPoses.find(*iter) ==
				_safeRelocalizationTrustedPoses.end())
		{
			extensionIds.insert(*iter);
		}
	}
	Transform extensionToFrozen = liveToFrozen;
	if(!extensionIds.empty())
	{
		int boundaryNeighborCount = 0;
		for(std::multimap<int, Link>::const_iterator iter=liveConstraints.begin();
				iter!=liveConstraints.end(); ++iter)
		{
			const Link & link = iter->second;
			const bool fromFrozen = replacedTrustedIds.find(link.from()) != replacedTrustedIds.end();
			const bool toFrozen = replacedTrustedIds.find(link.to()) != replacedTrustedIds.end();
			const bool fromExtension = extensionIds.find(link.from()) != extensionIds.end();
			const bool toExtension = extensionIds.find(link.to()) != extensionIds.end();
			if(link.type() == Link::kNeighbor &&
					((fromFrozen && toExtension) || (toFrozen && fromExtension)))
			{
				++boundaryNeighborCount;
				const int frozenId = fromFrozen?link.from():link.to();
				const int extensionId = fromExtension?link.from():link.to();
				const Transform expectedExtension = fromFrozen?
						_safeRelocalizationTrustedPoses.at(frozenId)*link.transform():
						_safeRelocalizationTrustedPoses.at(frozenId)*link.transform().inverse();
				const Transform correction = expectedExtension*
						_optimizedPoses.at(extensionId).inverse();
				if(!safeRigidTransform(expectedExtension) || !safeRigidTransform(correction))
				{
					reason = "cannot establish a finite frozen-boundary correction for the post-commit extension";
					return false;
				}
				extensionToFrozen = correction;
			}
		}
		if(boundaryNeighborCount != 1)
		{
			reason = "post-commit extension must have exactly one neighbor edge crossing its frozen boundary";
			return false;
		}
	}
	std::map<int, Transform> promotedPoses = _safeRelocalizationTrustedPoses;
	for(std::map<int, Transform>::const_iterator iter=_optimizedPoses.begin();
			iter!=_optimizedPoses.end(); ++iter)
	{
		if(iter->first > 0 &&
				_safeRelocalizationTrustedPoses.find(iter->first) ==
						_safeRelocalizationTrustedPoses.end())
		{
			const Transform aligned = extensionToFrozen * iter->second;
			if(!safeRigidTransform(aligned))
			{
				reason = "post-commit extension cannot be transformed through its frozen boundary";
				return false;
			}
			promotedPoses.insert(std::make_pair(iter->first, aligned));
		}
	}
	// Every live edge crossing from frozen history to the new extension must
	// agree with the promoted poses within the same fail-closed bounds used for
	// frozen-pose deformation.  The selected neighbor edge is exact up to
	// floating-point roundoff; this additional check catches conflicting loop
	// closures at the boundary.
	for(std::multimap<int, Link>::const_iterator iter=liveConstraints.begin();
			iter!=liveConstraints.end(); ++iter)
	{
		const Link & link = iter->second;
		const bool crossesBoundary =
				(replacedTrustedIds.find(link.from()) != replacedTrustedIds.end() &&
				 extensionIds.find(link.to()) != extensionIds.end()) ||
				(extensionIds.find(link.from()) != extensionIds.end() &&
				 replacedTrustedIds.find(link.to()) != replacedTrustedIds.end());
		if(crossesBoundary)
		{
			const Transform predicted = promotedPoses.at(link.from()).inverse() *
					promotedPoses.at(link.to());
			if(!safeRigidTransform(predicted) ||
					predicted.getDistance(link.transform()) >
							_safeRelocalizationPromotionMaxOldPoseTranslation ||
					predicted.getAngle(link.transform()) >
							_safeRelocalizationPromotionMaxOldPoseRotation*M_PI/180.0f)
			{
				reason = "post-commit extension conflicts with a frozen-boundary constraint";
				return false;
			}
		}
	}
	std::multimap<int, Link> promotedConstraints;
	for(std::multimap<int, Link>::const_iterator iter=_safeRelocalizationTrustedConstraints.begin();
			iter!=_safeRelocalizationTrustedConstraints.end(); ++iter)
	{
		if(untouchedTrustedIds.find(iter->second.from()) != untouchedTrustedIds.end() &&
				untouchedTrustedIds.find(iter->second.to()) != untouchedTrustedIds.end())
		{
			promotedConstraints.insert(*iter);
		}
	}
	promotedConstraints.insert(liveConstraints.begin(), liveConstraints.end());
	_safeRelocalizationTrustedPoses.swap(promotedPoses);
	_safeRelocalizationTrustedConstraints.swap(promotedConstraints);
	_safeRelocalizationTrustedSnapshotValid = true;
	rebuildSafeRelocalizationTrustedComponents();
	return true;
}

bool Rtabmap::validateAndFreezeCommittedEpochForSafeRelocalization(std::string & reason)
{
	reason.clear();
	if(!_safeRelocalizationGuardEnabled ||
			_safeRelocalizationCommittedEpochs.find(_resetEpoch) ==
					_safeRelocalizationCommittedEpochs.end())
	{
		return true;
	}
	std::set<int> livePositiveIds;
	for(std::map<int, Transform>::const_iterator iter=_optimizedPoses.begin();
			iter!=_optimizedPoses.end(); ++iter)
	{
		if(iter->first > 0)
		{
			livePositiveIds.insert(iter->first);
		}
	}
	if(!promoteCommittedEpochForSafeRelocalization(reason))
	{
		_safeRelocalizationTrustedSnapshotValid = false;
		return false;
	}
	// publish/save only the frozen gauge validated above. Disconnected trusted
	// components remain in the private snapshot but are not injected into the
	// current local optimized graph.
	for(std::set<int>::const_iterator iter=livePositiveIds.begin();
			iter!=livePositiveIds.end(); ++iter)
	{
		std::map<int, Transform>::const_iterator frozen =
				_safeRelocalizationTrustedPoses.find(*iter);
		if(frozen == _safeRelocalizationTrustedPoses.end() ||
				!safeRigidTransform(frozen->second))
		{
			reason = "validated committed graph lacks a frozen live pose";
			_safeRelocalizationTrustedSnapshotValid = false;
			return false;
		}
		_optimizedPoses[*iter] = frozen->second;
	}
	const Signature * last = _memory?_memory->getLastWorkingSignature(true):0;
	if(!last || _optimizedPoses.find(last->id()) == _optimizedPoses.end() ||
			!safeRigidTransform(last->getPose()))
	{
		reason = "validated committed graph cannot bind the frozen gauge to its last working signature";
		_safeRelocalizationTrustedSnapshotValid = false;
		return false;
	}
	_mapCorrection = _optimizedPoses.at(last->id()) * last->getPose().inverse();
	_lastLocalizationPose = _optimizedPoses.at(last->id());
	return true;
}

void Rtabmap::rebuildSafeRelocalizationTrustedComponents()
{
	_safeRelocalizationTrustedComponents.clear();
	std::map<int, std::set<int> > adjacency;
	for(std::map<int, Transform>::const_iterator iter=_safeRelocalizationTrustedPoses.begin();
			iter!=_safeRelocalizationTrustedPoses.end(); ++iter)
	{
		adjacency[iter->first];
	}
	for(std::multimap<int, Link>::const_iterator iter=_safeRelocalizationTrustedConstraints.begin();
			iter!=_safeRelocalizationTrustedConstraints.end(); ++iter)
	{
		const Link & link = iter->second;
		if(link.from() != link.to() && link.from() > 0 && link.to() > 0 &&
				adjacency.find(link.from()) != adjacency.end() &&
				adjacency.find(link.to()) != adjacency.end())
		{
			adjacency[link.from()].insert(link.to());
			adjacency[link.to()].insert(link.from());
		}
	}
	std::set<int> visited;
	for(std::map<int, std::set<int> >::const_iterator begin=adjacency.begin();
			begin!=adjacency.end(); ++begin)
	{
		if(visited.find(begin->first) != visited.end())
		{
			continue;
		}
		const int component = begin->first;
		std::queue<int> pending;
		pending.push(begin->first);
		visited.insert(begin->first);
		while(!pending.empty())
		{
			const int id = pending.front();
			pending.pop();
			_safeRelocalizationTrustedComponents[id] = component;
			const std::set<int> & neighbors = adjacency.at(id);
			for(std::set<int>::const_iterator neighbor=neighbors.begin();
					neighbor!=neighbors.end(); ++neighbor)
			{
				if(visited.insert(*neighbor).second)
				{
					pending.push(*neighbor);
				}
			}
		}
	}
}

unsigned long long Rtabmap::getSignatureResetEpoch(int id) const
{
	std::map<int, unsigned long long>::const_iterator iter = _signatureResetEpochs.find(id);
	if(iter != _signatureResetEpochs.end())
	{
		return iter->second;
	}
	// Nodes loaded from an existing database predate this process.  Epoch zero
	// is deliberately reserved for that trusted, stopped-map history.
	return 0;
}

bool Rtabmap::validateAcceptedImagesForTransaction(
        const std::vector<Link> & evidence,
        const std::vector<AcceptedImageReceipt> & receipts,
        unsigned long long sourceResetEpoch,
        const AcceptedImageSourceContext & context,
        std::string & reason) const
{
    const size_t gidSize = context.rgbPublisherGidHex.size();
    if(!_memory || sourceResetEpoch == 0 || sourceResetEpoch != _resetEpoch ||
       evidence.size() != 3 || receipts.size() != 6 ||
       !receiptHex(context.sessionTokenSha256, 64) || context.sessionTokenSha256 == std::string(64, '0') ||
       !receiptHex(context.databaseSha256, 64) || gidSize < 32 || gidSize > 64 || gidSize % 2 ||
       !receiptHex(context.rgbPublisherGidHex, gidSize) || !receiptHex(context.infoPublisherGidHex, gidSize) ||
       context.rgbPublisherGidHex == std::string(gidSize, '0') || context.infoPublisherGidHex == std::string(gidSize, '0') ||
       (context.provenanceScope != "managed_ros_rgbd" && context.provenanceScope != "offline_native_fixture"))
        { reason = "image receipt transaction context is invalid or stale"; return false; }
    std::set<int> currentIds, oldIds, endpoints, seen;
    for(const Link & link : evidence)
    {
        if(link.from() <= 0 || link.to() <= 0 || !currentIds.insert(link.from()).second ||
           !oldIds.insert(link.to()).second || !endpoints.insert(link.from()).second || !endpoints.insert(link.to()).second)
            { reason = "image receipt endpoints are not six independent nodes"; return false; }
    }
    for(const AcceptedImageReceipt & expected : receipts)
    {
        if(!endpoints.count(expected.signatureId) || !seen.insert(expected.signatureId).second)
            { reason = "image receipt set does not exactly cover bridge endpoints"; return false; }
        const ImageInputIdentity & input = expected.input;
        if(input.sessionTokenSha256 != context.sessionTokenSha256 || input.databaseSha256 != context.databaseSha256 ||
           input.rgbPublisherGidHex != context.rgbPublisherGidHex || input.infoPublisherGidHex != context.infoPublisherGidHex ||
           input.provenanceScope != context.provenanceScope)
            { reason = "image receipt source or session no longer matches the managed context"; return false; }
        AcceptedImageReceipt owned;
        if(!getAcceptedImageReceipt(expected.signatureId, owned) || !sameAcceptedImageReceipt(expected, owned))
            { reason = "image receipt is missing stale or differs from Core owned input"; return false; }
        const Signature * signature = _memory->getSignature(expected.signatureId);
        if(!signature || signature->mapId() != expected.mapId || signature->sensorData().cameraModels().size() != 1 ||
           !receiptCameraMatches(signature->sensorData().cameraModels().front(), input))
            { reason = "image receipt no longer matches the current signature camera or map"; return false; }
        const Transform & extrinsic = signature->sensorData().cameraModels().front().localTransform();
        if(extrinsic.isNull()) { reason = "image receipt signature has no camera transform"; return false; }
        for(int i=0; i<12; ++i)
            if(double(extrinsic[i]) != expected.bodyFromCamera[i])
                { reason = "image receipt camera transform changed"; return false; }
        if(currentIds.count(expected.signatureId))
        {
            if(expected.resetEpoch != sourceResetEpoch || !_optimizedPoses.count(expected.signatureId) ||
               !safeRigidTransform(_optimizedPoses.at(expected.signatureId)))
                { reason = "image receipt current node is outside the active graph or epoch"; return false; }
        }
        else if(expected.resetEpoch >= sourceResetEpoch || !_safeRelocalizationTrustedSnapshotValid ||
                !_safeRelocalizationTrustedPoses.count(expected.signatureId) ||
                !_safeRelocalizationTrustedComponents.count(expected.signatureId) ||
                !safeRigidTransform(_safeRelocalizationTrustedPoses.at(expected.signatureId)))
            { reason = "image receipt old node is outside the trusted graph or epoch"; return false; }
    }
    if(seen != endpoints) { reason = "image receipt endpoint coverage is incomplete"; return false; }
    return true;
}

bool Rtabmap::addLinksWithAcceptedImagesAtomically(
        const std::vector<Link> & evidence,
        unsigned long long sourceResetEpoch,
        const SafeRelocalizationLimits & limits,
        const std::vector<AcceptedImageReceipt> & receipts,
        const AcceptedImageSourceContext & context,
        std::string & reason)
{
    // CoreWrapper owns the authenticated context and serializes graph mutations.
    // All receipt checks are repeated inside the same transaction immediately
    // before commit preparation; this is not an external snapshot-then-add API.
    return addLinksAtomicallyImpl(evidence, sourceResetEpoch, limits, &receipts, &context, reason);
}

bool Rtabmap::buildDepthCornerEvidence(
		const std::vector<Link> & endpointEvidence,
		const std::vector<AcceptedImageReceipt> & receipts,
		const std::vector<AcceptedTagCornerObservation> & corners,
		std::vector<Link> & measuredEvidence,
		std::string & reason) const
{
	measuredEvidence.clear();
	if(!_memory || endpointEvidence.size()!=3 || receipts.size()!=6 || corners.size()!=6)
	{
		reason = "depth-corner transaction requires three pairs and six receipts/observations";
		return false;
	}
	std::map<int, AcceptedImageReceipt> receiptsById;
	for(std::vector<AcceptedImageReceipt>::const_iterator receipt=receipts.begin(); receipt!=receipts.end(); ++receipt)
	{
		if(receipt->signatureId <= 0 || !receiptsById.insert(std::make_pair(receipt->signatureId,*receipt)).second)
		{
			reason = "depth-corner receipts contain an invalid or repeated signature";
			return false;
		}
	}
	std::map<int, std::array<cv::Point3d,4> > localCorners;
	int commonTagId=-1;
	for(std::vector<AcceptedTagCornerObservation>::const_iterator observation=corners.begin();
			observation!=corners.end(); ++observation)
	{
		std::map<int, AcceptedImageReceipt>::const_iterator receipt=receiptsById.find(observation->signatureId);
		if(receipt==receiptsById.end() || observation->tagId < 0 ||
				(commonTagId>=0 && observation->tagId!=commonTagId) ||
				observation->cornerOrder!="opencv_aruco_canonical_0_1_2_3" ||
				!receiptHex(observation->pixelRecordSha256,64) ||
				observation->imageSha256!=receipt->second.input.pixelSha256 ||
				!localCorners.insert(std::make_pair(observation->signatureId,
						std::array<cv::Point3d,4>())).second)
		{
			reason = "depth-corner observation is not uniquely bound to one accepted image and Tag";
			return false;
		}
		commonTagId=observation->tagId;
		const Signature * signature=_memory->getSignature(observation->signatureId);
		// A saved WM signature may retain camera/RGB identity while its depth blob
		// has already moved to the Core-owned database. Ask Memory for images so
		// saved anchors are reloaded through the same owner instead of trusting an
		// external depth copy.
		const SensorData data=_memory->getNodeData(observation->signatureId,true,false,false,false);
		if(!signature || data.cameraModels().size()!=1 ||
				!receiptCameraMatches(data.cameraModels().front(),receipt->second.input) ||
				!depthTagCorners(data,*observation,
				localCorners.at(observation->signatureId),reason))
		{
			if(reason.empty()) reason="depth-corner signature is unavailable";
			return false;
		}
	}
	std::array<std::array<cv::Point3d,4>,3> currentWorldByPair;
	std::array<std::array<cv::Point3d,4>,3> oldWorldByPair;
	for(size_t pair=0; pair<endpointEvidence.size(); ++pair)
	{
		const Link & endpoint=endpointEvidence[pair];
		if(endpoint.from()<=0 || endpoint.to()<=0 || endpoint.from()==endpoint.to() ||
				endpoint.type()!=Link::kUserClosure ||
				localCorners.find(endpoint.from())==localCorners.end() ||
				localCorners.find(endpoint.to())==localCorners.end() ||
				_optimizedPoses.find(endpoint.from())==_optimizedPoses.end() ||
				_safeRelocalizationTrustedPoses.find(endpoint.to())==_safeRelocalizationTrustedPoses.end())
		{
			reason = "depth-corner pair endpoints are outside the current/trusted graphs";
			return false;
		}
		const Transform & currentPose=_optimizedPoses.at(endpoint.from());
		const Transform & oldPose=_safeRelocalizationTrustedPoses.at(endpoint.to());
		for(int corner=0; corner<4; ++corner)
		{
			currentWorldByPair[pair][corner]=transformPoint(currentPose,localCorners.at(endpoint.from())[corner]);
			oldWorldByPair[pair][corner]=transformPoint(oldPose,localCorners.at(endpoint.to())[corner]);
		}
	}
	auto fitAlignment=[&](int excludedPair, Transform & alignment)->bool {
		cv::Point3d currentCenter(0.0,0.0,0.0);
		cv::Point3d oldCenter(0.0,0.0,0.0);
		int count=0;
		for(int pair=0; pair<3; ++pair)
		{
			if(pair==excludedPair) continue;
			for(int corner=0; corner<4; ++corner)
			{
				currentCenter+=currentWorldByPair[pair][corner];
				oldCenter+=oldWorldByPair[pair][corner];
				++count;
			}
		}
		if(count<8) return false;
		currentCenter*=1.0/double(count);
		oldCenter*=1.0/double(count);
		double dotXY=0.0;
		double crossXY=0.0;
		for(int pair=0; pair<3; ++pair)
		{
			if(pair==excludedPair) continue;
			for(int corner=0; corner<4; ++corner)
			{
				const cv::Point3d a=currentWorldByPair[pair][corner]-currentCenter;
				const cv::Point3d b=oldWorldByPair[pair][corner]-oldCenter;
				dotXY += a.x*b.x+a.y*b.y;
				crossXY += a.x*b.y-a.y*b.x;
			}
		}
		if(!std::isfinite(dotXY) || !std::isfinite(crossXY) ||
				std::hypot(dotXY,crossXY)<1e-12) return false;
		const double yaw=std::atan2(crossXY,dotXY);
		const double c=std::cos(yaw);
		const double s=std::sin(yaw);
		cv::Mat rotation=(cv::Mat_<double>(3,3)<<c,-s,0.0,s,c,0.0,0.0,0.0,1.0);
		cv::Mat current=(cv::Mat_<double>(3,1)<<currentCenter.x,currentCenter.y,currentCenter.z);
		cv::Mat old=(cv::Mat_<double>(3,1)<<oldCenter.x,oldCenter.y,oldCenter.z);
		cv::Mat translation=old-rotation*current;
		alignment=Transform(
			(float)rotation.at<double>(0,0),(float)rotation.at<double>(0,1),(float)rotation.at<double>(0,2),(float)translation.at<double>(0),
			(float)rotation.at<double>(1,0),(float)rotation.at<double>(1,1),(float)rotation.at<double>(1,2),(float)translation.at<double>(1),
			(float)rotation.at<double>(2,0),(float)rotation.at<double>(2,1),(float)rotation.at<double>(2,2),(float)translation.at<double>(2));
		return safeRigidTransform(alignment);
	};
	auto residualPass=[&](const Transform & alignment, int evaluatedPair)->bool {
		double squared=0.0;
		double maximum=0.0;
		int count=0;
		for(int pair=0; pair<3; ++pair)
		{
			if(evaluatedPair>=0 && pair!=evaluatedPair) continue;
			for(int corner=0; corner<4; ++corner)
			{
				const double residual=cv::norm(transformPoint(alignment,currentWorldByPair[pair][corner])-oldWorldByPair[pair][corner]);
				squared+=residual*residual;
				maximum=std::max(maximum,residual);
				++count;
			}
		}
		return count>0 && std::sqrt(squared/double(count))<=0.018 && maximum<=0.025;
	};
	Transform alignment;
	if(!fitAlignment(-1,alignment) || !residualPass(alignment,-1))
	{
		reason="shared gravity-constrained depth-corner alignment exceeds metric residual limits";
		return false;
	}
	for(int holdout=0; holdout<3; ++holdout)
	{
		Transform predicted;
		if(!fitAlignment(holdout,predicted) || !residualPass(predicted,holdout))
		{
			reason="leave-one-pair-out gravity-constrained depth-corner alignment failed";
			return false;
		}
	}
	for(size_t pair=0; pair<endpointEvidence.size(); ++pair)
	{
		const Link & endpoint=endpointEvidence[pair];
		const Transform & currentPose=_optimizedPoses.at(endpoint.from());
		const Transform & oldPose=_safeRelocalizationTrustedPoses.at(endpoint.to());
		const Transform measured=(alignment*currentPose).inverse()*oldPose;
		if(!safeRigidTransform(measured))
		{
			reason="depth-corner pair produced an invalid bridge";
			return false;
		}
		// The metric estimate comes from depth, not nominal Tag size or PnP
		// sensitivity. Use the replay-validated conservative 2 cm / 3 degree
		// diagonal instead of forwarding a PnP-conditioned information matrix.
		cv::Mat information=cv::Mat::eye(6,6,CV_64FC1);
		for(int axis=0; axis<3; ++axis) information.at<double>(axis,axis)=2500.0;
		const double angularInformation=1.0/std::pow(3.0*M_PI/180.0,2.0);
		for(int axis=3; axis<6; ++axis) information.at<double>(axis,axis)=angularInformation;
		measuredEvidence.push_back(Link(endpoint.from(),endpoint.to(),Link::kUserClosure,
				measured,information));
	}
	return measuredEvidence.size()==3;
}

bool Rtabmap::addDepthCornerLinksWithAcceptedImagesAtomically(
		const std::vector<Link> & endpointEvidence,
		unsigned long long sourceResetEpoch,
		const SafeRelocalizationLimits & limits,
		const std::vector<AcceptedImageReceipt> & receipts,
		const std::vector<AcceptedTagCornerObservation> & corners,
		const AcceptedImageSourceContext & context,
		std::string & reason)
{
	if(!validateAcceptedImagesForTransaction(endpointEvidence,receipts,sourceResetEpoch,context,reason))
	{
		return false;
	}
	std::vector<Link> measuredEvidence;
	if(!buildDepthCornerEvidence(endpointEvidence,receipts,corners,measuredEvidence,reason))
	{
		return false;
	}
	return addLinksAtomicallyImpl(measuredEvidence,sourceResetEpoch,limits,&receipts,&context,reason);
}

bool Rtabmap::addLinksAtomically(
        const std::vector<Link> & evidence,
        unsigned long long sourceResetEpoch,
        const SafeRelocalizationLimits & limits,
        std::string & reason)
{
    return addLinksAtomicallyImpl(evidence, sourceResetEpoch, limits, nullptr, nullptr, reason);
}

bool Rtabmap::addLinksAtomicallyImpl(const std::vector<Link> & evidence,
        unsigned long long sourceResetEpoch, const SafeRelocalizationLimits & limits,
        const std::vector<AcceptedImageReceipt> * receipts, const AcceptedImageSourceContext * context,
        std::string & reason)
{
	reason.clear();
    if(receipts && (!context || !validateAcceptedImagesForTransaction(evidence, *receipts, sourceResetEpoch, *context, reason)))
        return false;
	if(!_rgbdSlamMode || !_memory || !_memory->isIncremental())
	{
		reason = "safe relocalization requires incremental RGBD-SLAM memory";
		return false;
	}
	if(evidence.size() != 3 || sourceResetEpoch == 0 ||
			sourceResetEpoch != _resetEpoch ||
			!std::isfinite(limits.maxHoldoutTranslation) ||
			!std::isfinite(limits.maxHoldoutRotation) ||
			!std::isfinite(limits.minBridgeTranslation) ||
			!std::isfinite(limits.minBridgeRotation) ||
			!std::isfinite(limits.maxAnchorZ) ||
			!std::isfinite(limits.maxAnchorTilt) ||
			!std::isfinite(limits.maxOldPoseTranslation) ||
			!std::isfinite(limits.maxOldPoseRotation) ||
			!std::isfinite(limits.maxNewRigidTranslation) ||
			!std::isfinite(limits.maxNewRigidRotation) ||
			!std::isfinite(limits.maxNewRigidZError) ||
			!std::isfinite(limits.maxNewRigidTiltError) ||
			!std::isfinite(limits.maxGravityResidualIncrease) ||
			!std::isfinite(limits.maxInformationCondition) ||
			limits.maxHoldoutTranslation <= 0.0f ||
			limits.maxHoldoutRotation <= 0.0f ||
			limits.minBridgeTranslation <= 0.0f ||
			limits.minBridgeRotation <= 0.0f ||
			limits.maxAnchorZ <= 0.0f || limits.maxAnchorTilt <= 0.0f ||
			limits.maxOldPoseTranslation <= 0.0f || limits.maxOldPoseRotation <= 0.0f ||
			limits.maxNewRigidTranslation <= 0.0f || limits.maxNewRigidRotation <= 0.0f ||
			limits.maxNewRigidZError <= 0.0f || limits.maxNewRigidTiltError <= 0.0f ||
			limits.maxGravityResidualIncrease < 0.0f ||
			limits.maxInformationCondition < 1.0)
	{
		reason = "safe relocalization transaction or limits are invalid";
		return false;
	}
	if(_safeRelocalizationCommittedEpochs.find(sourceResetEpoch) !=
			_safeRelocalizationCommittedEpochs.end())
	{
		reason = "safe relocalization already committed for this reset epoch";
		return false;
	}
	if(!_safeRelocalizationTrustedSnapshotValid ||
			_safeRelocalizationTrustedPoses.empty() ||
			_safeRelocalizationTrustedComponents.empty())
	{
		reason = "complete valid saved optimized history is unavailable (fail-closed; verify Mem/InitWMWithAllNodes)";
		return false;
	}
	for(std::map<int, Transform>::const_iterator iter=_optimizedPoses.begin();
			iter!=_optimizedPoses.end(); ++iter)
	{
		if(iter->first > 0 && !safeRigidTransform(iter->second))
		{
			reason = "live optimized graph contains an invalid positive-node pose";
			return false;
		}
	}

	std::set<int> fromIds;
	std::set<int> toIds;
	std::set<std::pair<int, int> > pairs;
	int trustedComponent = 0;
	std::vector<Transform> anchorAlignments;
	for(std::vector<Link>::const_iterator iter=evidence.begin(); iter!=evidence.end(); ++iter)
	{
		if(iter->type() != Link::kUserClosure || iter->from() <= 0 || iter->to() <= 0 ||
				iter->from() == iter->to() || !safeRigidTransform(iter->transform()) ||
				!safeInformationMatrix(iter->infMatrix(), limits.maxInformationCondition, reason))
		{
			if(reason.empty())
			{
				reason = "invalid user-closure evidence";
			}
			return false;
		}
		if(!fromIds.insert(iter->from()).second ||
				!toIds.insert(iter->to()).second ||
				!pairs.insert(std::make_pair(iter->from(), iter->to())).second)
		{
			reason = "evidence endpoints are not independent";
			return false;
		}
		if(getSignatureResetEpoch(iter->from()) != sourceResetEpoch ||
				getSignatureResetEpoch(iter->to()) >= sourceResetEpoch ||
				_optimizedPoses.find(iter->from()) == _optimizedPoses.end() ||
				!safeRigidTransform(_optimizedPoses.at(iter->from())) ||
				_safeRelocalizationTrustedPoses.find(iter->to()) == _safeRelocalizationTrustedPoses.end() ||
				_safeRelocalizationTrustedComponents.find(iter->to()) ==
						_safeRelocalizationTrustedComponents.end())
		{
			reason = "evidence is not current-epoch to trusted older history";
			return false;
		}
		const int component = _safeRelocalizationTrustedComponents.at(iter->to());
		if(trustedComponent == 0)
		{
			trustedComponent = component;
		}
		else if(component != trustedComponent)
		{
			reason = "old-map evidence targets do not belong to one trusted component";
			return false;
		}

		// Link convention is from_T_to = map_T_from^-1 * map_T_to.
		// Therefore X mapping the current segment into the old map satisfies
		// X = old_T_to * from_T_to^-1 * current_T_from^-1.
		const Transform alignment =
				_safeRelocalizationTrustedPoses.at(iter->to()) *
				iter->transform().inverse() *
				_optimizedPoses.at(iter->from()).inverse();
		if(!safeRigidTransform(alignment))
		{
			reason = "evidence produced a non-finite current-to-old alignment";
			return false;
		}
		anchorAlignments.push_back(alignment);
	}
	for(unsigned int i=0; i<anchorAlignments.size(); ++i)
	{
		for(unsigned int j=i+1; j<anchorAlignments.size(); ++j)
		{
			const Transform & sourceI = _optimizedPoses.at(evidence[i].from());
			const Transform & sourceJ = _optimizedPoses.at(evidence[j].from());
			const Transform & targetI = _safeRelocalizationTrustedPoses.at(evidence[i].to());
			const Transform & targetJ = _safeRelocalizationTrustedPoses.at(evidence[j].to());
			if((sourceI.getDistance(sourceJ) < limits.minBridgeTranslation &&
					sourceI.getAngle(sourceJ) < limits.minBridgeRotation*M_PI/180.0f) ||
					(targetI.getDistance(targetJ) < limits.minBridgeTranslation &&
					targetI.getAngle(targetJ) < limits.minBridgeRotation*M_PI/180.0f))
			{
				reason = "bridge IDs are distinct but their source or target poses are not geometrically independent";
				return false;
			}
			const Transform delta = anchorAlignments[i].inverse()*anchorAlignments[j];
			if(!safeRigidTransform(delta) ||
					anchorAlignments[i].getDistance(anchorAlignments[j]) > limits.maxHoldoutTranslation ||
					anchorAlignments[i].getAngle(anchorAlignments[j]) > limits.maxHoldoutRotation*M_PI/180.0f ||
					std::fabs(anchorAlignments[i].z()-anchorAlignments[j].z()) > limits.maxAnchorZ ||
					safeTiltError(anchorAlignments[i], anchorAlignments[j]) > limits.maxAnchorTilt*M_PI/180.0f)
			{
				reason = "three anchor alignments disagree in SE3, z or tilt";
				return false;
			}
		}
	}

	std::set<int> oldIds;
	std::set<int> currentIds;
	std::map<int, Transform> initialPoses;
	std::multimap<int, Link> baseConstraints;
	for(std::map<int, int>::const_iterator iter=_safeRelocalizationTrustedComponents.begin();
			iter!=_safeRelocalizationTrustedComponents.end(); ++iter)
	{
		if(iter->second == trustedComponent)
		{
			oldIds.insert(iter->first);
			initialPoses.insert(std::make_pair(
					iter->first, _safeRelocalizationTrustedPoses.at(iter->first)));
		}
	}
	for(std::map<int, Transform>::const_iterator iter=_optimizedPoses.begin();
			iter!=_optimizedPoses.end(); ++iter)
	{
		if(iter->first > 0 && getSignatureResetEpoch(iter->first) == sourceResetEpoch)
		{
			currentIds.insert(iter->first);
			initialPoses.insert(std::make_pair(
					iter->first, iter->second));
		}
	}
	for(std::map<int, unsigned long long>::const_iterator iter=_signatureResetEpochs.begin();
			iter!=_signatureResetEpochs.end(); ++iter)
	{
		if(iter->first > 0 && iter->second == sourceResetEpoch &&
				currentIds.find(iter->first) == currentIds.end())
		{
			reason = "live optimized graph omits a positive node from the current reset epoch";
			return false;
		}
	}
	if(oldIds.size() < 3 || currentIds.size() < 3 || initialPoses.size() != oldIds.size()+currentIds.size())
	{
		reason = "shadow graph lacks a complete old component or current epoch";
		return false;
	}
	// Do not freeze a just-reset component before measured gravity has had
	// enough frames to settle its roll/pitch. Recorded underwater replay showed
	// that the first 68-node Tag opportunity was still converging, while the
	// later 300-node revisit was stable.
	if(currentIds.size() < 200)
	{
		reason = uFormat("current reset component needs gravity stabilization (%u/200 nodes)",
				(unsigned int)currentIds.size());
		return false;
	}
	for(std::multimap<int, Link>::const_iterator iter=_safeRelocalizationTrustedConstraints.begin();
			iter!=_safeRelocalizationTrustedConstraints.end(); ++iter)
	{
		if(oldIds.find(iter->second.from()) != oldIds.end() &&
				oldIds.find(iter->second.to()) != oldIds.end())
		{
			if(!safeRigidTransform(iter->second.transform()) ||
					!safeInformationMatrix(iter->second.infMatrix(),
							limits.maxInformationCondition, reason, false))
			{
				reason = "trusted old graph contains an invalid transform or information matrix";
				return false;
			}
			baseConstraints.insert(*iter);
		}
	}
	for(std::multimap<int, Link>::const_iterator iter=_constraints.begin();
			iter!=_constraints.end(); ++iter)
	{
		// Landmarks use negative IDs and are explicitly ignored by the required
		// Optimizer/LandmarksIgnored=true contract. They must neither enter the
		// shadow graph nor make an otherwise valid positive-node epoch look
		// dangling.
		if(iter->second.from() <= 0 || iter->second.to() <= 0)
		{
			continue;
		}
		const bool fromCurrent = currentIds.find(iter->second.from()) != currentIds.end();
		const bool toCurrent = currentIds.find(iter->second.to()) != currentIds.end();
		if(fromCurrent != toCurrent)
		{
			reason = "current epoch graph contains a dangling or cross-epoch constraint";
			return false;
		}
		if(fromCurrent && toCurrent)
		{
			if(!safeRigidTransform(iter->second.transform()) ||
					!safeInformationMatrix(iter->second.infMatrix(),
							limits.maxInformationCondition, reason, false))
			{
				reason = "current epoch graph contains an invalid transform or information matrix";
				return false;
			}
			baseConstraints.insert(*iter);
		}
	}

	const int rootId = *oldIds.begin();
	std::map<int, Transform> finalPoses;
	std::multimap<int, Link> finalConstraints;
	auto validateShadow = [&](int holdoutIndex,
			std::map<int, Transform> * acceptedPoses,
			std::multimap<int, Link> * acceptedConstraints)->bool
	{
		// A held-out bridge must not influence even the optimizer's initial
		// guess. Seed this fold from the first bridge which it actually contains.
		const unsigned int seedIndex = holdoutIndex == 0?1:0;
		std::map<int, Transform> foldInitialPoses = initialPoses;
		for(std::set<int>::const_iterator id=currentIds.begin(); id!=currentIds.end(); ++id)
		{
			foldInitialPoses.at(*id) = anchorAlignments[seedIndex]*initialPoses.at(*id);
		}
		std::multimap<int, Link> constraints = baseConstraints;
		for(unsigned int i=0; i<evidence.size(); ++i)
		{
			if((int)i != holdoutIndex)
			{
				constraints.insert(std::make_pair(evidence[i].from(), evidence[i]));
			}
		}
		std::unique_ptr<Optimizer> optimizer(Optimizer::create(_parameters));
		if(!optimizer.get() || optimizer->iterations() <= 0 || optimizer->isSlam2d() ||
				!optimizer->landmarksIgnored())
		{
			reason = "safe relocalization requires an iterative 6DoF shadow optimizer with landmarks ignored";
			return false;
		}
		std::map<int, Transform> connectedPoses;
		std::multimap<int, Link> connectedConstraints;
		optimizer->getConnectedGraph(rootId, foldInitialPoses, constraints,
				connectedPoses, connectedConstraints);
		if(connectedPoses.size() != foldInitialPoses.size())
		{
			reason = "two-bridge shadow graph is not fully connected";
			return false;
		}
		double finalError = 0.0;
		int iterationsDone = 0;
		// getConnectedGraph derives traversal poses from constraints; use it only
		// for connectivity/edge selection, then restore the saved optimized old
		// poses plus the explicitly aligned current-segment initial guess.
		for(std::map<int, Transform>::iterator iter=connectedPoses.begin();
				iter!=connectedPoses.end(); ++iter)
		{
			iter->second = foldInitialPoses.at(iter->first);
		}
		OptimizerG2O * fixedOptimizer = dynamic_cast<OptimizerG2O *>(optimizer.get());
		if(!fixedOptimizer)
		{
			reason="fixed-old shadow solve requires the verified g2o backend";
			return false;
		}
		cv::Mat shadowCovariance;
		std::map<int, Transform> optimized = fixedOptimizer->optimizeWithFixedPoses(
				rootId, connectedPoses, connectedConstraints, oldIds, shadowCovariance,
				0, &finalError, &iterationsDone);
		if(optimized.size() != foldInitialPoses.size())
		{
			reason = "shadow graph optimization did not return every pose";
			return false;
		}
		for(std::map<int, Transform>::const_iterator iter=optimized.begin();
				iter!=optimized.end(); ++iter)
		{
			if(!safeRigidTransform(iter->second))
			{
				reason = "shadow optimizer returned a non-finite pose";
				return false;
			}
		}
		graph::MaxGraphErrors errors = graph::computeMaxGraphErrors(optimized, connectedConstraints);
		if(_optimizationMaxError > 0.0f &&
				((errors.linearLink.isValid() && errors.linearRatio > _optimizationMaxError) ||
				 (errors.angularLink.isValid() && errors.angularRatio > _optimizationMaxError)))
		{
			reason = "shadow graph exceeds RGBD/OptimizeMaxError";
			return false;
		}
		for(std::set<int>::const_iterator id=oldIds.begin(); id!=oldIds.end(); ++id)
		{
			if(optimized.at(*id).getDistance(_safeRelocalizationTrustedPoses.at(*id)) >
						limits.maxOldPoseTranslation ||
					optimized.at(*id).getAngle(_safeRelocalizationTrustedPoses.at(*id)) >
						limits.maxOldPoseRotation*M_PI/180.0f)
			{
				reason = "shadow merge deforms the trusted old component";
				return false;
			}
		}
		const int currentReference = evidence.front().from();
		const Transform rigidCorrection = optimized.at(currentReference) *
				foldInitialPoses.at(currentReference).inverse();
		double maxNewRigidTranslation = 0.0;
		double maxNewRigidRotationDeg = 0.0;
		double maxNewRigidZError = 0.0;
		double maxNewRigidTiltErrorDeg = 0.0;
		double maxLocalLinkTranslation = 0.0;
		double maxLocalLinkRotationDeg = 0.0;
		double maxGravityResidualIncreaseDeg = -std::numeric_limits<double>::infinity();
		double gravityResidualBeforeSumDeg = 0.0;
		double gravityResidualAfterSumDeg = 0.0;
		unsigned int gravityResidualCount = 0;
		int maxNewRigidTranslationId = 0;
		int maxNewRigidRotationId = 0;
		int maxNewRigidZErrorId = 0;
		int maxNewRigidTiltErrorId = 0;
		for(std::set<int>::const_iterator id=currentIds.begin(); id!=currentIds.end(); ++id)
		{
			const Transform rigidPrediction = rigidCorrection*foldInitialPoses.at(*id);
			const Transform deformation = rigidPrediction.inverse()*optimized.at(*id);
			if(!safeRigidTransform(deformation))
			{
				reason = uFormat("shadow merge returned non-finite current deformation id=%d", *id);
				return false;
			}
			const double translation = rigidPrediction.getDistance(optimized.at(*id));
			const double rotationDeg = rigidPrediction.getAngle(optimized.at(*id))*180.0/M_PI;
			const double zError = std::fabs(rigidPrediction.z()-optimized.at(*id).z());
			const double tiltErrorDeg = safeTiltError(rigidPrediction, optimized.at(*id))*180.0/M_PI;
			if(translation > maxNewRigidTranslation) {maxNewRigidTranslation=translation; maxNewRigidTranslationId=*id;}
			if(rotationDeg > maxNewRigidRotationDeg) {maxNewRigidRotationDeg=rotationDeg; maxNewRigidRotationId=*id;}
			if(zError > maxNewRigidZError) {maxNewRigidZError=zError; maxNewRigidZErrorId=*id;}
			if(tiltErrorDeg > maxNewRigidTiltErrorDeg) {maxNewRigidTiltErrorDeg=tiltErrorDeg; maxNewRigidTiltErrorId=*id;}
		}
		for(std::multimap<int, Link>::const_iterator link=connectedConstraints.begin();
				link!=connectedConstraints.end(); ++link)
		{
			const int from = link->second.from();
			const int to = link->second.to();
			if(link->second.type() == Link::kGravity && from == to &&
					currentIds.find(from) != currentIds.end())
			{
				const double beforeDeg = safeGravityDirectionError(foldInitialPoses.at(from), link->second.transform())*180.0/M_PI;
				const double afterDeg = safeGravityDirectionError(optimized.at(from), link->second.transform())*180.0/M_PI;
				gravityResidualBeforeSumDeg += beforeDeg;
				gravityResidualAfterSumDeg += afterDeg;
				maxGravityResidualIncreaseDeg = std::max(maxGravityResidualIncreaseDeg, afterDeg-beforeDeg);
				++gravityResidualCount;
			}
			else if(from != to && currentIds.find(from) != currentIds.end() &&
					currentIds.find(to) != currentIds.end())
			{
				const Transform beforeRelative = foldInitialPoses.at(from).inverse()*foldInitialPoses.at(to);
				const Transform afterRelative = optimized.at(from).inverse()*optimized.at(to);
				maxLocalLinkTranslation = std::max(maxLocalLinkTranslation, double(beforeRelative.getDistance(afterRelative)));
				maxLocalLinkRotationDeg = std::max(maxLocalLinkRotationDeg,
						beforeRelative.getAngle(afterRelative)*180.0/M_PI);
			}
		}
		const bool standardCurrentRigidityExceeded =
				maxNewRigidTranslation > limits.maxNewRigidTranslation ||
				maxNewRigidRotationDeg > limits.maxNewRigidRotation ||
				maxNewRigidZError > limits.maxNewRigidZError ||
				maxNewRigidTiltErrorDeg > limits.maxNewRigidTiltError;
		// A gravity solve may smoothly distribute a roll/pitch correction over a
		// newly reset component. Permit that narrow case only when local visual
		// edges remain continuous and every measured-gravity residual improves.
		const bool safeDistributedGravityCorrection =
				gravityResidualCount >= 30 &&
				maxNewRigidTranslation <= std::min(double(limits.maxNewRigidTranslation)*5.0/3.0, 0.050) &&
				maxNewRigidRotationDeg <= std::min(double(limits.maxNewRigidRotation)*7.0/3.0, 7.0) &&
				maxNewRigidZError <= std::min(double(limits.maxNewRigidZError)*2.5, 0.050) &&
				maxNewRigidTiltErrorDeg <= std::min(double(limits.maxNewRigidTiltError)*3.25, 6.5) &&
				maxLocalLinkTranslation <= 0.005 && maxLocalLinkRotationDeg <= 1.75 &&
				gravityResidualAfterSumDeg/gravityResidualCount <= 2.0 &&
				gravityResidualBeforeSumDeg/gravityResidualCount -
						gravityResidualAfterSumDeg/gravityResidualCount >= 3.0 &&
				maxGravityResidualIncreaseDeg <= 0.0;
		if(standardCurrentRigidityExceeded && !safeDistributedGravityCorrection)
		{
			reason = uFormat("shadow current deformation max_translation=%fm(id=%d,limit=%f) max_rotation=%fdeg(id=%d,limit=%f) max_z=%fm(id=%d,limit=%f) max_tilt=%fdeg(id=%d,limit=%f) local_link_max=%fm/%fdeg gravity_mean=%fdeg->%fdeg gravity_max_increase=%fdeg gravity_count=%u",
					maxNewRigidTranslation, maxNewRigidTranslationId, limits.maxNewRigidTranslation,
					maxNewRigidRotationDeg, maxNewRigidRotationId, limits.maxNewRigidRotation,
					maxNewRigidZError, maxNewRigidZErrorId, limits.maxNewRigidZError,
					maxNewRigidTiltErrorDeg, maxNewRigidTiltErrorId, limits.maxNewRigidTiltError,
					maxLocalLinkTranslation, maxLocalLinkRotationDeg,
					gravityResidualCount?gravityResidualBeforeSumDeg/gravityResidualCount:0.0,
					gravityResidualCount?gravityResidualAfterSumDeg/gravityResidualCount:0.0,
					gravityResidualCount?maxGravityResidualIncreaseDeg:0.0, gravityResidualCount);
			return false;
		}
		for(std::multimap<int, Link>::const_iterator link=connectedConstraints.begin();
				link!=connectedConstraints.end(); ++link)
		{
			if(link->second.type() == Link::kGravity && link->second.from() == link->second.to() &&
					foldInitialPoses.find(link->second.from()) != foldInitialPoses.end())
			{
				const double before = safeGravityDirectionError(foldInitialPoses.at(link->second.from()), link->second.transform());
				const double after = safeGravityDirectionError(optimized.at(link->second.from()), link->second.transform());
				if(after > before + limits.maxGravityResidualIncrease*M_PI/180.0f)
				{
					reason = "shadow merge increases a gravity-link tilt residual";
					return false;
				}
			}
		}
		if(holdoutIndex >= 0)
		{
			const Link & holdout = evidence[holdoutIndex];
			const Transform predicted = optimized.at(holdout.from()).inverse()*optimized.at(holdout.to());
			const Transform residual = predicted.inverse()*holdout.transform();
			const Transform optimizedAlignment = optimized.at(holdout.from()) *
					_optimizedPoses.at(holdout.from()).inverse();
			const Transform measuredAlignment = optimized.at(holdout.to()) *
					holdout.transform().inverse() *
					_optimizedPoses.at(holdout.from()).inverse();
			if(!safeRigidTransform(predicted) || !safeRigidTransform(residual) ||
					!safeRigidTransform(optimizedAlignment) || !safeRigidTransform(measuredAlignment) ||
					predicted.getDistance(holdout.transform()) > limits.maxHoldoutTranslation ||
					predicted.getAngle(holdout.transform()) > limits.maxHoldoutRotation*M_PI/180.0f ||
					std::fabs(optimizedAlignment.z()-measuredAlignment.z()) > limits.maxAnchorZ ||
					safeTiltError(optimizedAlignment, measuredAlignment) > limits.maxAnchorTilt*M_PI/180.0f)
			{
				reason = uFormat("leave-one-out holdout %d failed", holdoutIndex);
				return false;
			}
		}
		if(acceptedPoses)
		{
			*acceptedPoses = optimized;
		}
		if(acceptedConstraints)
		{
			// getConnectedGraph() intentionally filters priors/landmarks according
			// to optimizer settings. Preserve the complete live constraint set;
			// only the filtered copy is used by this shadow optimizer.
			*acceptedConstraints = constraints;
		}
		return true;
	};

	// Every bridge must be predicted by an optimization containing the other
	// two. Each fold owns a fresh optimizer, so no incremental backend state can
	// leak between folds. A fourth shadow run with all three bridges produces the
	// pose set which will be committed.
	for(int holdout=0; holdout<3; ++holdout)
	{
		if(!validateShadow(holdout, 0, 0))
		{
			return false;
		}
	}
	if(!validateShadow(-1, &finalPoses, &finalConstraints))
	{
		return false;
	}

    if(receipts && (!context || !validateAcceptedImagesForTransaction(evidence, *receipts, sourceResetEpoch, *context, reason)))
        return false;

	// The shadow phase above did not call any Memory mutator.
	// Commit preparation starts here. All nodes are
	// already resident in WM and every possible addLink() failure condition is checked
	// before the first edge is inserted.  CoreWrapper serializes this callback
	// with all graph-mutating services in one MutuallyExclusive callback group.
	for(std::map<int, Transform>::const_iterator iter=finalPoses.begin();
			iter!=finalPoses.end(); ++iter)
	{
		const Signature * signature = _memory->getSignature(iter->first);
		if(!signature || !safeRigidTransform(signature->getPose()))
		{
			reason = "commit preflight requires the complete shadow graph to already be resident in WM (set Mem/InitWMWithAllNodes=true)";
			return false;
		}
	}
	const Signature * last = _memory->getLastWorkingSignature(true);
	if(!last || getSignatureResetEpoch(last->id()) != sourceResetEpoch ||
			finalPoses.find(last->id()) == finalPoses.end() ||
			!safeRigidTransform(last->getPose()))
	{
		reason = "commit preflight cannot bind map correction to the current-epoch working signature";
		return false;
	}
	const int lastId = last->id();
	const Transform lastRawPose = last->getPose();
	for(std::multimap<int, Link>::const_iterator iter=baseConstraints.begin();
			iter!=baseConstraints.end(); ++iter)
	{
		const Signature * from = _memory->getSignature(iter->second.from());
		bool exactLiveConstraint = false;
		if(from)
		{
			const std::pair<std::multimap<int, Link>::const_iterator,
					std::multimap<int, Link>::const_iterator> range =
					from->getLinks().equal_range(iter->second.to());
			for(std::multimap<int, Link>::const_iterator live=range.first;
					live!=range.second; ++live)
			{
				if(sameConstraintMeasurementUndirected(iter->second, live->second))
				{
					exactLiveConstraint = true;
					break;
				}
			}
		}
		if(!exactLiveConstraint)
		{
			reason = "commit preflight found a removed or changed trusted/current constraint";
			return false;
		}
	}
	for(std::vector<Link>::const_iterator iter=evidence.begin(); iter!=evidence.end(); ++iter)
	{
		const Signature * from = _memory->getSignature(iter->from());
		const Signature * to = _memory->getSignature(iter->to());
		if(!from || !to || from->hasLink(iter->to()) || to->hasLink(iter->from()) ||
				from->getWeight() < 0 || to->getWeight() < 0 ||
				getSignatureResetEpoch(iter->from()) != sourceResetEpoch ||
				getSignatureResetEpoch(iter->to()) >= sourceResetEpoch)
		{
			reason = "commit preflight found a missing, stale or already-linked endpoint";
			return false;
		}
	}
	for(std::vector<Link>::const_iterator iter=evidence.begin(); iter!=evidence.end(); ++iter)
	{
		// After the exhaustive preflight, Memory::addLink(addInDatabase=false)
		// has no remaining false branch: both endpoints are pinned and no other
		// callback can run in this mutually-exclusive transaction.
		if(!_memory->addLink(*iter, false))
		{
			UFATAL("Safe relocalization invariant violated after batch preflight for %d->%d; stopping instead of attempting an inexact rollback", iter->from(), iter->to());
		}
	}

	_optimizedPoses = finalPoses;
	_constraints = finalConstraints;
	_mapCorrection = _optimizedPoses.at(lastId)*lastRawPose.inverse();
	_lastLocalizationPose = _optimizedPoses.at(lastId);
	_memory->save2DMap(cv::Mat(), 0, 0, 0);
	_safeRelocalizationCommittedEpochs.insert(sourceResetEpoch);
	_safeRelocalizationPromotionLimitsValid = true;
	_safeRelocalizationPromotionMaxOldPoseTranslation = limits.maxOldPoseTranslation;
	_safeRelocalizationPromotionMaxOldPoseRotation = limits.maxOldPoseRotation;
	// Freeze exactly the graph revision validated above. Replace the matched old
	// component with its merged result, while retaining other disconnected
	// trusted components verbatim. This is deliberately not a graph-wide union:
	// stale links from the matched component cannot survive the replacement.
	std::set<int> untouchedTrustedIds;
	std::map<int, Transform> promotedTrustedPoses;
	for(std::map<int, int>::const_iterator iter=_safeRelocalizationTrustedComponents.begin();
			iter!=_safeRelocalizationTrustedComponents.end(); ++iter)
	{
		if(iter->second != trustedComponent)
		{
			untouchedTrustedIds.insert(iter->first);
			promotedTrustedPoses.insert(std::make_pair(
					iter->first, _safeRelocalizationTrustedPoses.at(iter->first)));
		}
	}
	promotedTrustedPoses.insert(finalPoses.begin(), finalPoses.end());
	std::multimap<int, Link> promotedTrustedConstraints;
	for(std::multimap<int, Link>::const_iterator iter=_safeRelocalizationTrustedConstraints.begin();
			iter!=_safeRelocalizationTrustedConstraints.end(); ++iter)
	{
		if(untouchedTrustedIds.find(iter->second.from()) != untouchedTrustedIds.end() &&
				untouchedTrustedIds.find(iter->second.to()) != untouchedTrustedIds.end())
		{
			promotedTrustedConstraints.insert(*iter);
		}
	}
	promotedTrustedConstraints.insert(finalConstraints.begin(), finalConstraints.end());
	_safeRelocalizationTrustedPoses.swap(promotedTrustedPoses);
	_safeRelocalizationTrustedConstraints.swap(promotedTrustedConstraints);
	_safeRelocalizationTrustedSnapshotValid = true;
	rebuildSafeRelocalizationTrustedComponents();
	reason = uFormat("committed three bridges after 3-fold leave-one-out for reset epoch %llu; graph-rigidity guard passed (dense plane RMS requires stopped-map validation)", sourceResetEpoch);
	return true;
}

cv::Mat Rtabmap::getInformation(const cv::Mat & covariance) const
{
	cv::Mat information = covariance.inv();
	if(_loopCovLimited)
	{
		const std::vector<double> & odomMaxInf = _memory->getOdomMaxInf();
		if(odomMaxInf.size() == 6)
		{
			for(int i=0; i<6; ++i)
			{
				if(information.at<double>(i,i) > odomMaxInf[i])
				{
					information.at<double>(i,i) = odomMaxInf[i];
				}
			}
		}
	}
	return information;
}

void Rtabmap::addNodesToRepublish(const std::vector<int> & ids)
{
	if(ids.empty())
	{
		_nodesToRepublish.clear();
	}
	else if(_maxRepublished > 0 && _publishLastSignatureData)
	{
		_nodesToRepublish.insert(ids.begin(), ids.end());
	}
	else if(_maxRepublished == 0)
	{
		UWARN("%s=0, so cannot republish the %d requested nodes.", Parameters::kRtabmapMaxRepublished().c_str(), (int)ids.size());
	}
	else //_publishLastSignatureData=false
	{
		UWARN("%s=false, so cannot republish the %d requested nodes.", Parameters::kRtabmapPublishLastSignature().c_str(), (int)ids.size());
	}
}

void Rtabmap::clearPath(int status)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("Path mutation is disabled while safe relocalization owns graph mutations");
		return;
	}
	UINFO("status=%d", status);
	_pathStatus = status;
	_path.clear();
	_pathCurrentIndex=0;
	_pathGoalIndex = 0;
	_pathTransformToGoal.setIdentity();
	_pathUnreachableNodes.clear();
	_pathStuckCount = 0;
	_pathStuckDistance = 0.0f;
	if(_memory)
	{
		_memory->removeAllVirtualLinks();
	}
}

// return true if path is updated
bool Rtabmap::computePath(int targetNode, bool global)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("Path planning is disabled while safe relocalization owns graph mutations");
		return false;
	}
	this->clearPath(0);

	if(targetNode>0)
	{
		UINFO("Planning a path to node %d (global=%d)", targetNode, global?1:0);
	}
	else
	{
		UINFO("Planning a path to landmark %d (global=%d)", -targetNode, global?1:0);
	}

	if(!_rgbdSlamMode)
	{
		UWARN("A path can only be computed in RGBD-SLAM mode");
		return false;
	}

	UTimer totalTimer;
	UTimer timer;
	Transform transformToLandmark = Transform::getIdentity();

	// No need to optimize the graph
	if(_memory)
	{
		int currentNode = 0;
		if(_memory->isIncremental())
		{
			if(!_memory->getLastWorkingSignature(true))
			{
				UWARN("Working memory is empty... cannot compute a path");
				return false;
			}
			currentNode = _memory->getLastWorkingSignature(true)->id();
		}
		else
		{
			if(_lastLocalizationPose.isNull() || _optimizedPoses.empty())
			{
				UWARN("Last localization pose is null or optimized graph is empty... cannot compute a path");
				return false;
			}
			if(_optimizedPoses.begin()->first < 0)
			{
				std::map<int, Transform> poses(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				currentNode = graph::findNearestNode(poses, _lastLocalizationPose);
			}
			else
			{
				currentNode = graph::findNearestNode(_optimizedPoses, _lastLocalizationPose);
			}
		}
		if(currentNode && targetNode)
		{
			std::list<std::pair<int, Transform> > path = graph::computePath(
					currentNode,
					targetNode,
					_memory,
					global,
					false,
					_pathLinearVelocity,
					_pathAngularVelocity);

			//transform in current referential
			Transform t = uValue(_optimizedPoses, currentNode, Transform::getIdentity());
			_path.resize(path.size());
			int oi = 0;
			for(std::list<std::pair<int, Transform> >::iterator iter=path.begin(); iter!=path.end();++iter)
			{
				if(iter->first > 0)
				{
					// just keep nodes in the path
					_path[oi].first = iter->first;
					_path[oi++].second = t * iter->second;
				}
			}
			_path.resize(oi);
			if(!_path.empty() && !path.empty() && path.rbegin()->first < 0)
			{
				transformToLandmark = _path.back().second.inverse() * t * path.rbegin()->second;
			}
		}
		else if(currentNode == 0)
		{
			UWARN("We should be localized before planning.");
		}
	}
	UINFO("Total planning time = %fs (%d nodes, %f m long)", totalTimer.ticks(), (int)_path.size(), graph::computePathLength(_path));

	if(_path.size() == 0)
	{
		_path.clear();
		UWARN("Cannot compute a path!");
		return false;
	}
	else
	{
		UINFO("Path generated! Size=%d", (int)_path.size());
		if(ULogger::level() == ULogger::kInfo)
		{
			std::stringstream stream;
			for(unsigned int i=0; i<_path.size(); ++i)
			{
				stream << _path[i].first;
				if(i+1 < _path.size())
				{
					stream << " ";
				}
			}
			UINFO("Path = [%s]", stream.str().c_str());
		}
		if(_goalsSavedInUserData)
		{
			// set goal to latest signature
			std::string goalStr = uFormat("GOAL:%d", targetNode);

			// use label is exist
			if(_memory->getSignature(targetNode))
			{
				if(!_memory->getSignature(targetNode)->getLabel().empty())
				{
					goalStr = std::string("GOAL:")+_memory->getSignature(targetNode)->getLabel();
				}
			}
			else if(global)
			{
				std::map<int, std::string> labels = _memory->getAllLabels();
				std::map<int, std::string>::iterator iter = labels.find(targetNode);
				if(iter != labels.end() && !iter->second.empty())
				{
					goalStr = std::string("GOAL:")+labels.at(targetNode);
				}
			}
			setUserData(0, cv::Mat(1, int(goalStr.size()+1), CV_8SC1, (void *)goalStr.c_str()).clone());
		}
		_pathTransformToGoal = transformToLandmark;

		updateGoalIndex();
		return _path.size() || _pathStatus > 0;
	}

	return false;
}

bool Rtabmap::computePath(const Transform & targetPose, float tolerance)
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("Path planning is disabled while safe relocalization owns graph mutations");
		return false;
	}
	this->clearPath(0);

	UINFO("Planning a path to pose %s ", targetPose.prettyPrint().c_str());
	if(tolerance < 0.0f)
	{
		tolerance = _localRadius;
	}

	std::list<std::pair<int, Transform> > pathPoses;

	if(!_rgbdSlamMode)
	{
		UWARN("This method can only be used in RGBD-SLAM mode");
		return false;
	}

	//Find the nearest node
	UTimer timer;
	std::map<int, Transform> nodes = _optimizedPoses;
	std::multimap<int, int> links;
	for(std::map<int, Transform>::iterator iter=nodes.upper_bound(0); iter!=nodes.end(); ++iter)
	{
		const Signature * s = _memory->getSignature(iter->first);
		UASSERT(s);
		for(std::map<int, Link>::const_iterator jter=s->getLinks().begin(); jter!=s->getLinks().end(); ++jter)
		{
			// only add links for which poses are in "nodes"
			if(jter->second.from() != jter->second.to() && uContains(nodes, jter->second.to()))
			{
				links.insert(std::make_pair(jter->second.from(), jter->second.to()));
				//links.insert(std::make_pair(jter->second.to(), jter->second.from())); // <-> (commented: already added when iterating in nodes)
			}
		}
	}
	UINFO("Time getting links = %fs", timer.ticks());

	int currentNode = 0;
	if(_memory->isIncremental())
	{
		if(!_memory->getLastWorkingSignature(true))
		{
			UWARN("Working memory is empty... cannot compute a path");
			return false;
		}
		currentNode = _memory->getLastWorkingSignature(true)->id();
	}
	else
	{
		if(_lastLocalizationPose.isNull() || _optimizedPoses.empty())
		{
			UWARN("Last localization pose is null... cannot compute a path");
			return false;
		}
		if(_optimizedPoses.begin()->first < 0)
		{
			std::map<int, Transform> poses(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
			currentNode = graph::findNearestNode(poses, _lastLocalizationPose);
		}
		else
		{
			currentNode = graph::findNearestNode(_optimizedPoses, _lastLocalizationPose);
		}
	}

	int nearestId;
	if(!_lastLocalizationPose.isNull() && _lastLocalizationPose.getDistance(targetPose) < tolerance)
	{
		// target can be reached from the current node
		nearestId = currentNode;
	}
	else
	{
		nearestId = rtabmap::graph::findNearestNode(nodes, targetPose);
	}
	UINFO("Nearest node found=%d ,%fs", nearestId, timer.ticks());
	if(nearestId > 0)
	{
		if(tolerance != 0.0f && targetPose.getDistance(nodes.at(nearestId)) > tolerance)
		{
			UWARN("Cannot plan farther than %f m from the graph! (distance=%f m from node %d)",
					tolerance, targetPose.getDistance(nodes.at(nearestId)), nearestId);
		}
		else
		{
			UINFO("Computing path from location %d to %d", currentNode, nearestId);
			UTimer timer;
			_path = uListToVector(rtabmap::graph::computePath(nodes, links, currentNode, nearestId));
			UINFO("A* time = %fs", timer.ticks());

			if(_path.size() == 0)
			{
				UWARN("Cannot compute a path!");
			}
			else
			{
				UINFO("Path generated! Size=%d", (int)_path.size());
				if(ULogger::level() == ULogger::kInfo)
				{
					std::stringstream stream;
					for(unsigned int i=0; i<_path.size(); ++i)
					{
						stream << _path[i].first;
						if(i+1 < _path.size())
						{
							stream << " ";
						}
					}
					UINFO("Path = [%s]", stream.str().c_str());
				}

				UASSERT(uContains(nodes, _path.back().first));
				_pathTransformToGoal = nodes.at(_path.back().first).inverse() * targetPose;

				updateGoalIndex();

				return true;
			}
		}
	}
	else
	{
		UWARN("Nearest node not found in graph (size=%d) for pose %s", (int)nodes.size(), targetPose.prettyPrint().c_str());
	}

	return false;
}

std::vector<std::pair<int, Transform> > Rtabmap::getPathNextPoses() const
{
	std::vector<std::pair<int, Transform> > poses;
	if(_path.size())
	{
		UASSERT(_pathCurrentIndex < _path.size() && _pathGoalIndex < _path.size());
		poses.resize(_pathGoalIndex-_pathCurrentIndex+1);
		int oi=0;
		for(unsigned int i=_pathCurrentIndex; i<=_pathGoalIndex; ++i)
		{
			std::map<int, Transform>::const_iterator iter = _optimizedPoses.find(_path[i].first);
			if(iter != _optimizedPoses.end())
			{
				poses[oi++] = *iter;
			}
			else
			{
				break;
			}
		}
		poses.resize(oi);
	}
	return poses;
}

std::vector<int> Rtabmap::getPathNextNodes() const
{
	std::vector<int> ids;
	if(_path.size())
	{
		UASSERT(_pathCurrentIndex < _path.size() && _pathGoalIndex < _path.size());
		ids.resize(_pathGoalIndex-_pathCurrentIndex+1);
		int oi = 0;
		for(unsigned int i=_pathCurrentIndex; i<=_pathGoalIndex; ++i)
		{
			std::map<int, Transform>::const_iterator iter = _optimizedPoses.find(_path[i].first);
			if(iter != _optimizedPoses.end())
			{
				ids[oi++] = iter->first;
			}
			else
			{
				break;
			}
		}
		ids.resize(oi);
	}
	return ids;
}

int Rtabmap::getPathCurrentGoalId() const
{
	if(_path.size())
	{
		UASSERT(_pathGoalIndex <= _path.size());
		return _path[_pathGoalIndex].first;
	}
	return 0;
}

void Rtabmap::updateGoalIndex()
{
	if(_safeRelocalizationGuardEnabled)
	{
		UERROR("Goal-path mutation is disabled while safe relocalization owns graph mutations");
		return;
	}
	if(!_rgbdSlamMode)
	{
		UWARN("This method can on be used in RGBD-SLAM mode!");
		return;
	}

	if( _memory && _path.size())
	{
		// remove all previous virtual links
		bool hasIntermediateNodes = false;
		for(unsigned int i=0; i<_pathCurrentIndex && i<_path.size(); ++i)
		{
			const Signature * s = _memory->getSignature(_path[i].first);
			if(s)
			{
				_memory->removeVirtualLinks(s->id());
			}
			if(s->getWeight() == -1)
			{
				hasIntermediateNodes = true;
			}
		}

		// for the current index, only keep the newest virtual link
		// This will make sure that the path is still connected even
		// if the new signature is removed (e.g., because of a small displacement)
		UASSERT(_pathCurrentIndex < _path.size());
		const Signature * currentIndexS = _memory->getSignature(_path[_pathCurrentIndex].first);
		UASSERT_MSG(currentIndexS != 0, uFormat("_path[%d].first=%d", _pathCurrentIndex, _path[_pathCurrentIndex].first).c_str());
		std::multimap<int, Link> links = currentIndexS->getLinks(); // make a copy
		bool latestVirtualLinkFound = false;
		for(std::multimap<int, Link>::reverse_iterator iter=links.rbegin(); iter!=links.rend(); ++iter)
		{
			if(iter->second.type() == Link::kVirtualClosure)
			{
				if(latestVirtualLinkFound)
				{
					_memory->removeLink(currentIndexS->id(), iter->first);
				}
				else
				{
					latestVirtualLinkFound = true;
				}
			}
		}

		// Make sure the next signatures on the path are linked together
		float distanceSoFar = 0.0f;
		for(unsigned int i=_pathCurrentIndex+1;
			i<_path.size() && !hasIntermediateNodes;
			++i)
		{
			if(i>0)
			{
				if(_localRadius > 0.0f)
				{
					distanceSoFar += _path[i-1].second.getDistance(_path[i].second);
				}
				
				if(_path[i].first != _path[i-1].first)
				{
					const Signature * s = _memory->getSignature(_path[i].first);
					if(s)
					{
						if(s->getWeight() == -1)
						{
							hasIntermediateNodes = true;
							break;
						}
						if(!s->hasLink(_path[i-1].first) && _memory->getSignature(_path[i-1].first) != 0)
						{
							Transform virtualLoop = _path[i].second.inverse() * _path[i-1].second;
							if(_safeRelocalizationGuardEnabled &&
									getSignatureResetEpoch(_path[i].first) !=
									getSignatureResetEpoch(_path[i-1].first))
							{
								UINFO("Did not add cross-reset virtual planned-path link %d->%d; guarded transaction is required",
										_path[i].first, _path[i-1].first);
							}
							else
							{
								_memory->addLink(Link(_path[i].first, _path[i-1].first, Link::kVirtualClosure, virtualLoop, cv::Mat::eye(6,6,CV_64FC1)*0.01)); // on the optimized path
								UINFO("Added Virtual link between %d and %d", _path[i-1].first, _path[i].first);
							}
						}
					}
				}

				if(distanceSoFar > _localRadius)
				{
					UDEBUG("Farthest goal=%d : %f m", _path[i].first, distanceSoFar);
					break;
				}
			}
		}

		if(hasIntermediateNodes)
		{
			UERROR("Cannot follow a path with a map containing intermediate nodes (not supported: don't use intermediate nodes if rtabmap's planner has to be used). Aborting current plan!");
			this->clearPath(-1);
			return;
		}

		UDEBUG("current node = %d current goal = %d", _path[_pathCurrentIndex].first, _path[_pathGoalIndex].first);
		Transform currentPose;
		if(_memory->isIncremental())
		{
			if(_memory->getLastWorkingSignature(true) == 0 ||
			   !uContains(_optimizedPoses, _memory->getLastWorkingSignature(true)->id()))
			{
				UERROR("Last node is null in memory or not in optimized poses. Aborting the plan...");
				this->clearPath(-1);
				return;
			}
			currentPose = _optimizedPoses.at(_memory->getLastWorkingSignature(true)->id());
		}
		else
		{
			if(_lastLocalizationPose.isNull())
			{
				UERROR("Last localization pose is null. Aborting the plan...");
				this->clearPath(-1);
				return;
			}
			currentPose = _lastLocalizationPose;
		}

		int goalId = _path.back().first;
		if(uContains(_optimizedPoses, goalId))
		{
			//use local position to know if the goal is reached
			float d = currentPose.getDistance(_optimizedPoses.at(goalId)*_pathTransformToGoal);
			if(d < _goalReachedRadius)
			{
				UINFO("Goal %d reached!", goalId);
				this->clearPath(1);
			}
		}

		if(_path.size())
		{
			//Always check if the farthest node is accessible in local map (max to local space radius if set)
			unsigned int goalIndex = _pathCurrentIndex;
			float distanceFromCurrentNode = 0.0f;
			bool sameGoalIndex = false;
			for(unsigned int i=_pathCurrentIndex+1; i<_path.size(); ++i)
			{
				if(uContains(_optimizedPoses, _path[i].first))
				{
					if(_localRadius > 0.0f)
					{
						distanceFromCurrentNode = _path[_pathCurrentIndex].second.getDistance(_path[i].second);
					}

					if((goalIndex == _pathCurrentIndex && i == _path.size()-1) ||
					   _pathUnreachableNodes.find(i) == _pathUnreachableNodes.end())
					{
						goalIndex = i;
						if(distanceFromCurrentNode > _localRadius)
						{
							break;
						}
					}
				}
				else
				{
					break;
				}
			}
			UASSERT(_pathGoalIndex < _path.size() && goalIndex < _path.size());
			if(_pathGoalIndex != goalIndex)
			{
				UINFO("Updated current goal from %d to %d (%d/%d)",
						(int)_path[_pathGoalIndex].first, _path[goalIndex].first, (int)goalIndex+1, (int)_path.size());
				_pathGoalIndex = goalIndex;
			}
			else
			{
				sameGoalIndex = true;
			}

			// update nearest pose in the path
			unsigned int nearestNodeIndex = 0;
			float distance = -1.0f;
			bool sameCurrentIndex = false;
			UASSERT(_pathGoalIndex < _path.size());
			for(unsigned int i=_pathCurrentIndex; i<=_pathGoalIndex; ++i)
			{
				std::map<int, Transform>::iterator iter = _optimizedPoses.find(_path[i].first);
				if(iter != _optimizedPoses.end())
				{
					float d = currentPose.getDistanceSquared(iter->second);
					if(distance == -1.0f || distance > d)
					{
						distance = d;
						nearestNodeIndex = i;
					}
				}
			}
			if(distance < 0)
			{
				UERROR("The nearest pose on the path not found! Aborting the plan...");
				this->clearPath(-1);
			}
			else
			{
				UDEBUG("Nearest node = %d", _path[nearestNodeIndex].first);
			}
			if(distance >= 0 && nearestNodeIndex != _pathCurrentIndex)
			{
				_pathCurrentIndex = nearestNodeIndex;
				_pathUnreachableNodes.erase(nearestNodeIndex); // if we are on it, it is reachable
			}
			else
			{
				sameCurrentIndex = true;
			}

			bool isStuck = false;
			if(sameGoalIndex && sameCurrentIndex && _pathStuckIterations>0)
			{
				float distanceToCurrentGoal = 0.0f;
				std::map<int, Transform>::iterator iter = _optimizedPoses.find(_path[_pathGoalIndex].first);
				if(iter != _optimizedPoses.end())
				{
					if(_pathGoalIndex == _pathCurrentIndex &&
						_pathGoalIndex == _path.size()-1)
					{
						distanceToCurrentGoal = currentPose.getDistanceSquared(iter->second*_pathTransformToGoal);
					}
					else
					{
						distanceToCurrentGoal = currentPose.getDistanceSquared(iter->second);
					}
				}

				if(distanceToCurrentGoal > 0.0f)
				{
					if(distanceToCurrentGoal >= _pathStuckDistance)
					{
						// we are not approaching the goal
						isStuck = true;
						if(_pathStuckDistance == 0.0f)
						{
							_pathStuckDistance = distanceToCurrentGoal;
						}
					}
				}
				else
				{
					// no nodes available, cannot plan
					isStuck = true;
				}
			}

			if(isStuck && ++_pathStuckCount > _pathStuckIterations)
			{
				UWARN("Current goal %d not reached since %d iterations (\"RGBD/PlanStuckIterations\"=%d), mark that node as unreachable.",
						_path[_pathGoalIndex].first,
						_pathStuckCount,
						_pathStuckIterations);
				_pathStuckCount = 0;
				_pathStuckDistance = 0.0;
				_pathUnreachableNodes.insert(_pathGoalIndex);
				// select previous reachable one
				while(_pathUnreachableNodes.find(_pathGoalIndex) != _pathUnreachableNodes.end())
				{
					if(_pathGoalIndex == 0 || --_pathGoalIndex <= _pathCurrentIndex)
					{
						// plan failed!
						UERROR("No upcoming nodes on the path are reachable! Aborting the plan...");
						this->clearPath(-1);
						return;
					}
				}
			}
			else if(!isStuck)
			{
				_pathStuckCount = 0;
				_pathStuckDistance = 0.0;
			}
		}
	}
}

void Rtabmap::createGlobalScanMap()
{
	UDEBUG("Creating global scan map (if scans are available)");
	_globalScanMap.clear();
	_globalScanMapPoses.clear();
	std::vector<int> scanIndices;
	std::map<int, Transform> scanViewpoints;
	for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
	{
		SensorData data = _memory->getNodeData(iter->first, false, true, false, false);
		if(!data.laserScanCompressed().empty())
		{
			LaserScan scan;
			data.uncompressDataConst(0, 0, &scan, 0, 0, 0, 0);
			if(!scan.empty())
			{
				UDEBUG("Adding scan %d (format=%s, points=%d)", iter->first, scan.formatName().c_str(), scan.size());
				scan = util3d::transformLaserScan(scan, iter->second*scan.localTransform());
				if(_globalScanMap.empty() || _globalScanMap.format() == scan.format())
				{
					_globalScanMap += scan;
					_globalScanMapPoses.insert(*iter);
					scanViewpoints.insert(std::make_pair(iter->first, iter->second * scan.localTransform()));
					scanIndices.resize(_globalScanMap.size(), iter->first);
				}
				else
				{
					UWARN("Incompatible scan formats (%s vs %s), cannot create global scan map.",
							_globalScanMap.formatName().c_str(),
							scan.formatName().c_str());
					_globalScanMap.clear();
					_globalScanMapPoses.clear();
					break;
				}
			}
			else
			{
				UDEBUG("Ignored %d (scan is empty), pose still added.", iter->first);
				_globalScanMapPoses.insert(*iter);
			}
		}
		else
		{
			UDEBUG("Ignored %d (no scan), pose still added.", iter->first);
			_globalScanMapPoses.insert(*iter);
		}
	}
	if(_globalScanMap.size() > 3)
	{
		float voxelSize = 0.0f;
		int normalK = 0;
		float normalRadius = 0.0f;
		Parameters::parse(_parameters, Parameters::kMemLaserScanVoxelSize(), voxelSize);
		Parameters::parse(_parameters, Parameters::kMemLaserScanNormalK(), normalK);
		Parameters::parse(_parameters, Parameters::kMemLaserScanNormalRadius(), normalRadius);

		if(voxelSize > 0.0f)
		{
			LaserScan voxelScan = util3d::commonFiltering(_globalScanMap, 1, 0, 0, voxelSize, normalK, normalRadius);
			if(voxelScan.hasNormals())
			{
				// adjust with point of views
				util3d::adjustNormalsToViewPoints(
						scanViewpoints,
						_globalScanMap,
						scanIndices,
						voxelScan);
			}
			_globalScanMap = voxelScan;
		}

		UINFO("Global scan map has been assembled (size=%d points, %d poses) "
				"for proximity detection (only in localization mode %s=false and with %s=true)",
				(int)_globalScanMap.size(),
				(int)_globalScanMapPoses.size(),
				Parameters::kMemIncrementalMemory().c_str(),
				Parameters::kRGBDProximityGlobalScanMap().c_str());

		//for debugging...
		if(!_globalScanMap.empty() && ULogger::level() == ULogger::kDebug)
		{
			if(!_wDir.empty())
			{
				UWARN("Saving %s/rtabmap_global_scan_map.pcd (only saved when logger level is debug)", _wDir.c_str());
				pcl::PCLPointCloud2::Ptr cloud2 = util3d::laserScanToPointCloud2(_globalScanMap);
				pcl::io::savePCDFile(_wDir+"/rtabmap_global_scan_map.pcd", *cloud2);
			}
			else
			{
				UWARN("%s is enabled and logger is debug, but %s is not set, cannot save global scan map for debugging.",
						Parameters::kRGBDProximityGlobalScanMap().c_str(), Parameters::kRtabmapWorkingDirectory().c_str());
			}
		}
	}
	if(!_globalScanMap.empty() && _globalScanMap.size()<100)
	{
		UWARN("Ignoring global scan map because it is too small (%d points).", (int)_globalScanMap.size());
		_globalScanMap.clear();
		_globalScanMapPoses.clear();
	}
}

} // namespace rtabmap
