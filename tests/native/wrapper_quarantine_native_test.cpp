#include <rtabmap_slam/CoreWrapper.h>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/core/core.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <future>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// Test-only explicit-instantiation access. The production implementation has
// no test hooks; this binary links to the isolated candidate plugin.
template<typename Tag, typename Tag::type Member>
struct PrivateMemberAccess
{
	friend typename Tag::type getPrivateMember(Tag)
	{
		return Member;
	}
};

using Wrapper = rtabmap_slam::CoreWrapper;

struct OdomUpdateTag
{
	using type = bool (Wrapper::*)(const nav_msgs::msg::Odometry &, rclcpp::Time);
	friend type getPrivateMember(OdomUpdateTag);
};
template struct PrivateMemberAccess<OdomUpdateTag, &Wrapper::odomUpdate>;

struct AuthenticateOdomInfoTag
{
	using type = bool (Wrapper::*)(const rtabmap_msgs::msg::OdomInfo &);
	friend type getPrivateMember(AuthenticateOdomInfoTag);
};
template struct PrivateMemberAccess<AuthenticateOdomInfoTag,
		&Wrapper::authenticateResetQuarantineOdomInfo>;

struct ProcessTag
{
	using type = void (Wrapper::*)(
			const rclcpp::Time &,
			rtabmap::SensorData &,
			const rtabmap::Transform &,
			const std::vector<float> &,
			const std::string &,
			const cv::Mat &,
			const rtabmap::OdometryInfo &,
			double, const rtabmap::ImageInputIdentity &);
	friend type getPrivateMember(ProcessTag);
};
template struct PrivateMemberAccess<ProcessTag, &Wrapper::process>;

struct ProcessAsyncTag {
    using type = void (Wrapper::*)();
    friend type getPrivateMember(ProcessAsyncTag);
};
template struct PrivateMemberAccess<ProcessAsyncTag, &Wrapper::processAsync>;
struct SyncMutexTag {
    using type = UMutex Wrapper::*;
    friend type getPrivateMember(SyncMutexTag);
};
template struct PrivateMemberAccess<SyncMutexTag, &Wrapper::syncDataMutex_>;
struct LastPoseMutexTag {
    using type = UMutex Wrapper::*;
    friend type getPrivateMember(LastPoseMutexTag);
};
template struct PrivateMemberAccess<LastPoseMutexTag, &Wrapper::lastPoseMutex_>;
struct JumpGateTag {
    using type = rtabmap_slam::JumpQuarantineGate Wrapper::*;
    friend type getPrivateMember(JumpGateTag);
};
template struct PrivateMemberAccess<JumpGateTag, &Wrapper::jumpQuarantineGate_>;
struct OdomInfoCompromisedTag {
    using type = bool Wrapper::*;
    friend type getPrivateMember(OdomInfoCompromisedTag);
};
template struct PrivateMemberAccess<OdomInfoCompromisedTag, &Wrapper::resetQuarantineOdomInfoPublisherCompromised_>;
struct TrustedPosesTag {
    using type = std::map<int,rtabmap::Transform> rtabmap::Rtabmap::*;
    friend type getPrivateMember(TrustedPosesTag);
};
template struct PrivateMemberAccess<TrustedPosesTag, &rtabmap::Rtabmap::_safeRelocalizationTrustedPoses>;

struct LastPoseTag
{
	using type = rtabmap::Transform Wrapper::*;
	friend type getPrivateMember(LastPoseTag);
};
template struct PrivateMemberAccess<LastPoseTag, &Wrapper::lastPose_>;

struct LastPoseStampTag
{
	using type = rclcpp::Time Wrapper::*;
	friend type getPrivateMember(LastPoseStampTag);
};
template struct PrivateMemberAccess<LastPoseStampTag, &Wrapper::lastPoseStamp_>;

struct RtabmapTag
{
	using type = rtabmap::Rtabmap Wrapper::*;
	friend type getPrivateMember(RtabmapTag);
};
template struct PrivateMemberAccess<RtabmapTag, &Wrapper::rtabmap_>;

struct RecoveryPendingTag
{
	using type = std::atomic<bool> Wrapper::*;
	friend type getPrivateMember(RecoveryPendingTag);
};
template struct PrivateMemberAccess<RecoveryPendingTag,
		&Wrapper::resetQuarantineRecoveryPending_>;

struct SuspectedResetTag
{
	using type = std::atomic<bool> Wrapper::*;
	friend type getPrivateMember(SuspectedResetTag);
};
template struct PrivateMemberAccess<SuspectedResetTag,
		&Wrapper::resetQuarantineSuspectedReset_>;

struct LostStreakTag
{
	using type = std::atomic<int> Wrapper::*;
	friend type getPrivateMember(LostStreakTag);
};
template struct PrivateMemberAccess<LostStreakTag,
		&Wrapper::resetQuarantineLostStreak_>;

struct GraphRevisionTag
{
	using type = unsigned long long Wrapper::*;
	friend type getPrivateMember(GraphRevisionTag);
};
template struct PrivateMemberAccess<GraphRevisionTag,
		&Wrapper::safeRelocalizationGraphRevision_>;

using AuthenticatedOdomSamples = std::map<long long, std::string>;
using AuthenticatedOdomInfoSamples =
		std::map<long long, std::pair<std::string, long long> >;
using AuthenticatedOdomInfoStamps = std::map<long long, long long>;

struct AuthenticatedOdomSamplesTag
{
	using type = AuthenticatedOdomSamples Wrapper::*;
	friend type getPrivateMember(AuthenticatedOdomSamplesTag);
};
template struct PrivateMemberAccess<AuthenticatedOdomSamplesTag,
		&Wrapper::resetQuarantineAuthenticatedOdomSamples_>;

struct AuthenticatedOdomInfoSamplesTag
{
	using type = AuthenticatedOdomInfoSamples Wrapper::*;
	friend type getPrivateMember(AuthenticatedOdomInfoSamplesTag);
};
template struct PrivateMemberAccess<AuthenticatedOdomInfoSamplesTag,
		&Wrapper::resetQuarantineAuthenticatedOdomInfoSamples_>;

struct AuthenticatedOdomInfoStampsTag
{
	using type = AuthenticatedOdomInfoStamps Wrapper::*;
	friend type getPrivateMember(AuthenticatedOdomInfoStampsTag);
};
template struct PrivateMemberAccess<AuthenticatedOdomInfoStampsTag,
		&Wrapper::resetQuarantineAuthenticatedOdomInfoStamps_>;

struct EpochStartStampTag
{
	using type = std::atomic<long long> Wrapper::*;
	friend type getPrivateMember(EpochStartStampTag);
};
template struct PrivateMemberAccess<EpochStartStampTag,
		&Wrapper::resetQuarantineEpochStartStampNs_>;

struct AuthenticatedRgbSamplesTag
{
	using type = AuthenticatedOdomSamples Wrapper::*;
	friend type getPrivateMember(AuthenticatedRgbSamplesTag);
};
template struct PrivateMemberAccess<AuthenticatedRgbSamplesTag,
		&Wrapper::resetQuarantineAuthenticatedRgbSamples_>;

struct AuthenticatedDepthSamplesTag
{
	using type = AuthenticatedOdomSamples Wrapper::*;
	friend type getPrivateMember(AuthenticatedDepthSamplesTag);
};
template struct PrivateMemberAccess<AuthenticatedDepthSamplesTag,
		&Wrapper::resetQuarantineAuthenticatedDepthSamples_>;

struct AuthenticatedCameraInfoSamplesTag
{
	using type = AuthenticatedOdomSamples Wrapper::*;
	friend type getPrivateMember(AuthenticatedCameraInfoSamplesTag);
};
template struct PrivateMemberAccess<AuthenticatedCameraInfoSamplesTag,
		&Wrapper::resetQuarantineAuthenticatedCameraInfoSamples_>;

struct CommonMultiCameraCallbackTag
{
	using type = void (Wrapper::*)(
			const nav_msgs::msg::Odometry::ConstSharedPtr &,
			const rtabmap_msgs::msg::UserData::ConstSharedPtr &,
			const std::vector<cv_bridge::CvImageConstPtr> &,
			const std::vector<cv_bridge::CvImageConstPtr> &,
			const std::vector<sensor_msgs::msg::CameraInfo> &,
			const std::vector<sensor_msgs::msg::CameraInfo> &,
			const sensor_msgs::msg::LaserScan &,
			const sensor_msgs::msg::PointCloud2 &,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr &,
			const std::vector<rtabmap_msgs::msg::GlobalDescriptor> &,
			const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > &,
			const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > &,
			const std::vector<cv::Mat> &);
	friend type getPrivateMember(CommonMultiCameraCallbackTag);
};
template struct PrivateMemberAccess<CommonMultiCameraCallbackTag,
		&Wrapper::commonMultiCameraCallback>;

struct CommonMultiCameraCallbackImplTag
{
	using type = void (Wrapper::*)(
			const std::string &,
			const rtabmap_msgs::msg::UserData::ConstSharedPtr &,
			const std::vector<cv_bridge::CvImageConstPtr> &,
			const std::vector<cv_bridge::CvImageConstPtr> &,
			const std::vector<sensor_msgs::msg::CameraInfo> &,
			const std::vector<sensor_msgs::msg::CameraInfo> &,
			const sensor_msgs::msg::LaserScan &,
			const sensor_msgs::msg::PointCloud2 &,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr &,
			const std::vector<rtabmap_msgs::msg::GlobalDescriptor> &,
			const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > &,
			const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > &,
			const std::vector<cv::Mat> &);
	friend type getPrivateMember(CommonMultiCameraCallbackImplTag);
};
template struct PrivateMemberAccess<CommonMultiCameraCallbackImplTag,
		&Wrapper::commonMultiCameraCallbackImpl>;

struct SafeRelocalizationFatalTag
{
	using type = std::atomic<bool> Wrapper::*;
	friend type getPrivateMember(SafeRelocalizationFatalTag);
};
template struct PrivateMemberAccess<SafeRelocalizationFatalTag,
		&Wrapper::safeRelocalizationFatal_>;

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;

[[noreturn]] void fail(const std::string & message)
{
	throw std::runtime_error(message);
}

void require(bool condition, const std::string & message)
{
	if(!condition)
	{
		fail(message);
	}
}

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

rtabmap::SensorData sensorData(int id, double stamp)
{
	cv::Mat image(32, 32, CV_8UC1);
	cv::Mat depth(32, 32, CV_32FC1, cv::Scalar(0.7f));
	for(int row=0; row<image.rows; ++row)
	{
		for(int col=0; col<image.cols; ++col)
		{
			image.at<unsigned char>(row, col) = static_cast<unsigned char>(
					(row * 17 + col * 29 + id * 13) & 0xff);
		}
	}
	const rtabmap::CameraModel model(
			50.0, 50.0, 15.5, 15.5,
			rtabmap::Transform::getIdentity(), 0.0, cv::Size(32, 32));
	return rtabmap::SensorData(image, depth, model, id, stamp);
}

rclcpp::Time timeFromSeconds(int seconds)
{
	return rclcpp::Time(static_cast<int32_t>(seconds), 0, RCL_ROS_TIME);
}

long long steadyNowNs()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
}

nav_msgs::msg::Odometry odometry(
		int seconds,
		double x,
		double yawDegrees,
		bool highCovariance = false)
{
	nav_msgs::msg::Odometry message;
	message.header.stamp = timeFromSeconds(seconds);
	message.header.frame_id = "odom";
	message.child_frame_id = "base_link";
	message.pose.pose.position.x = x;
	const double half = yawDegrees * kPi / 360.0;
	message.pose.pose.orientation.z = std::sin(half);
	message.pose.pose.orientation.w = std::cos(half);
	for(std::size_t index=0; index<36; ++index)
	{
		message.pose.covariance[index] = 0.0;
		message.twist.covariance[index] = 0.0;
	}
	for(const std::size_t index : {0U, 7U, 14U, 21U, 28U, 35U})
	{
		message.pose.covariance[index] = highCovariance?9999.0:0.01;
		message.twist.covariance[index] = highCovariance?9999.0:0.01;
	}
	return message;
}

rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfo(int seconds, bool lost)
{
	auto message = std::make_shared<rtabmap_msgs::msg::OdomInfo>();
	message->header.stamp = timeFromSeconds(seconds);
	message->lost = lost;
	return message;
}

struct ManagedRgbdFrame
{
	cv_bridge::CvImageConstPtr rgb;
	cv_bridge::CvImageConstPtr depth;
	sensor_msgs::msg::CameraInfo cameraInfo;
};

ManagedRgbdFrame managedRgbdFrame(
		const rclcpp::Time & stamp,
		const std::string & frameId = "zed_left_camera_optical_frame")
{
	std_msgs::msg::Header header;
	header.stamp = stamp;
	header.frame_id = frameId;
	const cv::Mat rgb(32, 32, CV_8UC1, cv::Scalar(17));
	const cv::Mat depth(32, 32, CV_32FC1, cv::Scalar(0.7f));
	ManagedRgbdFrame frame;
	frame.rgb = std::make_shared<cv_bridge::CvImage>(header, "mono8", rgb);
	frame.depth = std::make_shared<cv_bridge::CvImage>(header, "32FC1", depth);
	frame.cameraInfo.header = header;
	frame.cameraInfo.width = 32U;
	frame.cameraInfo.height = 32U;
	frame.cameraInfo.k[0] = 50.0;
	frame.cameraInfo.k[2] = 15.5;
	frame.cameraInfo.k[4] = 50.0;
	frame.cameraInfo.k[5] = 15.5;
	frame.cameraInfo.k[8] = 1.0;
	frame.cameraInfo.p[0] = 50.0;
	frame.cameraInfo.p[2] = 15.5;
	frame.cameraInfo.p[5] = 50.0;
	frame.cameraInfo.p[6] = 15.5;
	frame.cameraInfo.p[10] = 1.0;
	return frame;
}

class ManagedOdomSource
{
public:
	explicit ManagedOdomSource(const std::shared_ptr<Wrapper> & wrapper) :
		wrapper_(wrapper),
		node_(std::make_shared<rclcpp::Node>("private_test_odom_source")),
		odomPublisher_(node_->create_publisher<nav_msgs::msg::Odometry>(
				"reset_quarantine_odom", rclcpp::SensorDataQoS())),
		odomInfoPublisher_(node_->create_publisher<rtabmap_msgs::msg::OdomInfo>(
				"reset_quarantine_odom_info", rclcpp::SensorDataQoS())),
		rgbPublisher_(node_->create_publisher<sensor_msgs::msg::Image>(
				"reset_quarantine_rgb", rclcpp::QoS(10).reliable())),
		depthPublisher_(node_->create_publisher<sensor_msgs::msg::Image>(
				"reset_quarantine_depth", rclcpp::QoS(10).reliable())),
		cameraInfoPublisher_(node_->create_publisher<sensor_msgs::msg::CameraInfo>(
				"reset_quarantine_camera_info", rclcpp::QoS(10).reliable()))
	{
		executor_.add_node(wrapper_);
		executor_.add_node(node_);
		const auto deadline = std::chrono::steady_clock::now() +
				std::chrono::seconds(3);
		while(std::chrono::steady_clock::now() < deadline &&
				(odomPublisher_->get_subscription_count() != 1U ||
				 odomInfoPublisher_->get_subscription_count() != 1U ||
				 rgbPublisher_->get_subscription_count() != 1U ||
				 depthPublisher_->get_subscription_count() != 1U ||
				 cameraInfoPublisher_->get_subscription_count() != 1U))
		{
			executor_.spin_some();
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		require(odomPublisher_->get_subscription_count() == 1U,
				"managed Odometry publisher did not discover exactly one guard subscription");
		require(odomInfoPublisher_->get_subscription_count() == 1U,
				"managed OdomInfo publisher did not discover exactly one guard subscription");
		require(rgbPublisher_->get_subscription_count() == 1U,
				"managed RGB publisher did not discover exactly one guard subscription");
		require(depthPublisher_->get_subscription_count() == 1U,
				"managed depth publisher did not discover exactly one guard subscription");
		require(cameraInfoPublisher_->get_subscription_count() == 1U,
				"managed CameraInfo publisher did not discover exactly one guard subscription");
	}

	~ManagedOdomSource()
	{
		executor_.remove_node(node_);
		executor_.remove_node(wrapper_);
	}

	void authorize(const nav_msgs::msg::Odometry & message)
	{
		const long long stampNs = rclcpp::Time(message.header.stamp).nanoseconds();
		odomPublisher_->publish(message);
		waitForSample(
				wrapper_.get()->*getPrivateMember(AuthenticatedOdomSamplesTag{}),
				stampNs,
				"managed Odometry callback did not authenticate the published sample");
	}

	void introduceConflictingOdomInfoSource()
	{
		auto conflict=node_->create_publisher<rtabmap_msgs::msg::OdomInfo>(
				"reset_quarantine_odom_info",rclcpp::SensorDataQoS());
		const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
		while(std::chrono::steady_clock::now()<deadline &&
				!(wrapper_.get()->*getPrivateMember(OdomInfoCompromisedTag{})))
		{
			conflict->publish(*odomInfo(100,false));
			executor_.spin_some();
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		require(wrapper_.get()->*getPrivateMember(OdomInfoCompromisedTag{}),
				"Second real ROS publisher did not latch source conflict");
	}

	rtabmap_msgs::msg::OdomInfo::SharedPtr pushOdomInfo(int seconds, bool lost, int inliers=0)
	{
		return pushOdomInfo(timeFromSeconds(seconds), lost, inliers);
	}

	rtabmap_msgs::msg::OdomInfo::SharedPtr pushOdomInfo(
			const rclcpp::Time & stamp,
			bool lost, int inliers=0)
	{
		auto message = odomInfo(1, lost);
		message->inliers = inliers;
		message->header.stamp = stamp;
		const long long stampNs = rclcpp::Time(message->header.stamp).nanoseconds();
		odomInfoPublisher_->publish(*message);
		waitForSample(
				wrapper_.get()->*getPrivateMember(AuthenticatedOdomInfoSamplesTag{}),
				stampNs,
				"managed OdomInfo callback did not authenticate the published sample");
		const auto & samples =
				wrapper_.get()->*getPrivateMember(AuthenticatedOdomInfoSamplesTag{});
		const auto found = samples.find(stampNs);
		require(found != samples.end() && !found->second.first.empty() &&
				found->second.second > 0,
				"managed OdomInfo sample did not retain fingerprint and own arrival time");
		return message;
	}

	void authorizeForProcess(const rtabmap_msgs::msg::OdomInfo & message)
	{
		const long long stampNs = rclcpp::Time(message.header.stamp).nanoseconds();
		auto & samples =
				wrapper_.get()->*getPrivateMember(AuthenticatedOdomInfoSamplesTag{});
		const auto found = samples.find(stampNs);
		require(found != samples.end(),
				"exact managed OdomInfo sample disappeared before process admission");
		const long long expectedArrivalSteadyNs = found->second.second;
		require((wrapper_.get()->*getPrivateMember(AuthenticateOdomInfoTag{}))(message),
				"exact managed OdomInfo sample was not admitted for process()");
		require(samples.find(stampNs) == samples.end(),
				"admitted OdomInfo sample was not consumed from the source-bound map");
		const auto & transferred =
				wrapper_.get()->*getPrivateMember(AuthenticatedOdomInfoStampsTag{});
		const auto transferredFound = transferred.find(stampNs);
		require(transferredFound != transferred.end() &&
				transferredFound->second == expectedArrivalSteadyNs,
				"process admission did not transfer the exact sample's own arrival time");
	}

	bool tryAuthorizeForProcess(const rtabmap_msgs::msg::OdomInfo & message)
	{
		return (wrapper_.get()->*getPrivateMember(AuthenticateOdomInfoTag{}))(message);
	}

	void setOdomInfoArrival(
			const rtabmap_msgs::msg::OdomInfo & message,
			long long arrivalSteadyNs)
	{
		const long long stampNs = rclcpp::Time(message.header.stamp).nanoseconds();
		auto & samples =
				wrapper_.get()->*getPrivateMember(AuthenticatedOdomInfoSamplesTag{});
		const auto found = samples.find(stampNs);
		require(found != samples.end(),
				"cannot adjust arrival of a missing OdomInfo sample");
		found->second.second = arrivalSteadyNs;
	}

	bool hasOdomInfoSample(long long stampNs) const
	{
		const auto & samples =
				wrapper_.get()->*getPrivateMember(AuthenticatedOdomInfoSamplesTag{});
		return samples.find(stampNs) != samples.end();
	}

	bool hasTransferredOdomInfo(long long stampNs) const
	{
		const auto & samples =
				wrapper_.get()->*getPrivateMember(AuthenticatedOdomInfoStampsTag{});
		return samples.find(stampNs) != samples.end();
	}

	void authorizeRgbd(
			const cv_bridge::CvImageConstPtr & rgb,
			const cv_bridge::CvImageConstPtr & depth,
			const sensor_msgs::msg::CameraInfo & cameraInfo)
	{
		require(rgb && depth, "managed RGB-D fixture contains a null image");
		const sensor_msgs::msg::Image::SharedPtr rgbMessage = rgb->toImageMsg();
		const sensor_msgs::msg::Image::SharedPtr depthMessage = depth->toImageMsg();
		const long long stampNs = rclcpp::Time(cameraInfo.header.stamp).nanoseconds();
		require(stampNs > 0 &&
				rclcpp::Time(rgbMessage->header.stamp).nanoseconds() == stampNs &&
				rclcpp::Time(depthMessage->header.stamp).nanoseconds() == stampNs,
				"managed RGB-D fixture stamps are not exact");
		rgbPublisher_->publish(*rgbMessage);
		depthPublisher_->publish(*depthMessage);
		cameraInfoPublisher_->publish(cameraInfo);
		waitForSample(
				wrapper_.get()->*getPrivateMember(AuthenticatedRgbSamplesTag{}),
				stampNs,
				"managed RGB callback did not authenticate the published sample");
		waitForSample(
				wrapper_.get()->*getPrivateMember(AuthenticatedDepthSamplesTag{}),
				stampNs,
				"managed depth callback did not authenticate the published sample");
		waitForSample(
				wrapper_.get()->*getPrivateMember(AuthenticatedCameraInfoSamplesTag{}),
				stampNs,
				"managed CameraInfo callback did not authenticate the published sample");
	}

private:
	template<typename SampleMap>
	void waitForSample(
			const SampleMap & samples,
			long long stampNs,
			const std::string & error)
	{
		const auto deadline = std::chrono::steady_clock::now() +
				std::chrono::seconds(3);
		while(std::chrono::steady_clock::now() < deadline &&
				samples.find(stampNs) == samples.end())
		{
			executor_.spin_some();
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}
		require(samples.find(stampNs) != samples.end(), error);
	}

	std::shared_ptr<Wrapper> wrapper_;
	std::shared_ptr<rclcpp::Node> node_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher_;
	rclcpp::Publisher<rtabmap_msgs::msg::OdomInfo>::SharedPtr odomInfoPublisher_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPublisher_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPublisher_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPublisher_;
	rclcpp::executors::SingleThreadedExecutor executor_;
};

bool updateOdom(
		Wrapper & wrapper,
		ManagedOdomSource & source,
		const nav_msgs::msg::Odometry & message)
{
	source.authorize(message);
	return (wrapper.*getPrivateMember(OdomUpdateTag{}))(
			message, rclcpp::Time(message.header.stamp));
}

void processFrame(
		Wrapper & wrapper,
		int id,
		const rclcpp::Time & stamp,
		const rtabmap::Transform & pose)
{
	rtabmap::SensorData data = sensorData(id, stamp.seconds());
	const cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	(wrapper.*getPrivateMember(ProcessTag{}))(
			stamp, data, pose, std::vector<float>(),
			"odom", covariance, rtabmap::OdometryInfo(), 0.0, rtabmap::ImageInputIdentity());
}

void processFrame(
		Wrapper & wrapper,
		int id,
		int seconds,
		const rtabmap::Transform & pose)
{
	processFrame(wrapper, id, timeFromSeconds(seconds), pose);
}

void requirePoseNear(
		const rtabmap::Transform & actual,
		const rtabmap::Transform & expected,
		const std::string & context)
{
	require(!actual.isNull() && !expected.isNull(), context + ": null pose");
	require(actual.getDistance(expected) < 1e-5f &&
			actual.getAngle(expected) < 1e-5f,
			context + ": pose changed unexpectedly");
}

void runOdomInfoArrivalContract(const std::filesystem::path & root)
{
	std::filesystem::create_directories(root);
	rclcpp::NodeOptions options;
	options.parameter_overrides(parameters((root / "arrival_contract.db").string()));
	auto wrapper = std::make_shared<Wrapper>(options);
	ManagedOdomSource source(wrapper);

	auto stale = source.pushOdomInfo(1, false);
	const long long staleStampNs = rclcpp::Time(stale->header.stamp).nanoseconds();
	source.setOdomInfoArrival(*stale, steadyNowNs() - 6000000000LL);
	require(!source.tryAuthorizeForProcess(*stale),
			"OdomInfo whose own arrival exceeded max age was admitted");
	require(!source.hasOdomInfoSample(staleStampNs) &&
			!source.hasTransferredOdomInfo(staleStampNs),
			"stale OdomInfo was not erased or reached the process ledger");

	auto future = source.pushOdomInfo(2, false);
	const long long futureStampNs = rclcpp::Time(future->header.stamp).nanoseconds();
	source.setOdomInfoArrival(*future, steadyNowNs() + 1000000000LL);
	require(!source.tryAuthorizeForProcess(*future),
			"OdomInfo with a future steady-clock arrival was admitted");
	require(!source.hasOdomInfoSample(futureStampNs) &&
			!source.hasTransferredOdomInfo(futureStampNs),
			"future-arrival OdomInfo was not erased or reached the process ledger");

	auto fresh = source.pushOdomInfo(3, false);
	const long long freshStampNs = rclcpp::Time(fresh->header.stamp).nanoseconds();
	source.authorizeForProcess(*fresh);
	require(source.hasTransferredOdomInfo(freshStampNs),
			"fresh exact OdomInfo did not reach the one-shot process ledger");
	require(!source.tryAuthorizeForProcess(*fresh),
			"one exact OdomInfo source sample was admitted twice");
	require(source.hasTransferredOdomInfo(freshStampNs),
			"failed duplicate admission corrupted the first process authorization");

	std::cout << "PASS native OdomInfo arrival contract: exact fingerprint+arrival, "
			"stale/future reject, fresh one-shot transfer" << std::endl;
}

void runManagedNumericContract(const std::filesystem::path & root)
{
	std::filesystem::create_directories(root);
	rclcpp::NodeOptions options;
	options.parameter_overrides(parameters((root / "numeric_contract.db").string()));
	auto wrapper = std::make_shared<Wrapper>(options);
	ManagedOdomSource source(wrapper);
	auto & core = wrapper.get()->*getPrivateMember(RtabmapTag{});
	auto & lastPose = wrapper.get()->*getPrivateMember(LastPoseTag{});
	auto & lastPoseStamp = wrapper.get()->*getPrivateMember(LastPoseStampTag{});
	auto & revision = wrapper.get()->*getPrivateMember(GraphRevisionTag{});
	const auto baselinePose = lastPose;
	const auto baselineStamp = lastPoseStamp.nanoseconds();
	const auto baselineEpoch = core.getResetEpoch();
	const auto baselineGraphSize = core.getLocalOptimizedPoses().size();
	const auto baselineRevision = revision;

	auto rejectWithoutPoseOrGraphMutation = [&](const nav_msgs::msg::Odometry & message,
			const std::string & label)
	{
		require(!(wrapper.get()->*getPrivateMember(OdomUpdateTag{}))(
				message, rclcpp::Time(message.header.stamp)),
				label + " was accepted");
		requirePoseNear(lastPose, baselinePose, label + " mutated lastPose");
		require(lastPoseStamp.nanoseconds() == baselineStamp,
				label + " mutated lastPoseStamp");
		require(core.getResetEpoch() == baselineEpoch &&
				core.getLocalOptimizedPoses().size() == baselineGraphSize &&
				revision == baselineRevision,
				label + " mutated epoch, graph or revision");
	};

	auto invalid = odometry(1, 0.2, 0.0);
	invalid.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
	rejectWithoutPoseOrGraphMutation(invalid, "first-frame NaN position");
	invalid = odometry(2, 0.2, 0.0);
	invalid.pose.pose.position.y = std::numeric_limits<double>::infinity();
	rejectWithoutPoseOrGraphMutation(invalid, "first-frame infinite position");
	invalid = odometry(3, 0.2, 0.0);
	invalid.pose.pose.orientation.w = 2.0;
	rejectWithoutPoseOrGraphMutation(invalid, "first-frame non-unit quaternion");
	invalid = odometry(4, 0.2, 0.0);
	invalid.pose.covariance[17] = std::numeric_limits<double>::quiet_NaN();
	rejectWithoutPoseOrGraphMutation(invalid, "non-leading pose covariance NaN");
	invalid = odometry(5, 0.2, 0.0);
	invalid.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
	rejectWithoutPoseOrGraphMutation(invalid, "twist NaN");

	const auto highCovarianceIdentity = odometry(6, 0.0, 0.0, true);
	source.authorize(highCovarianceIdentity);
	require(!(wrapper.get()->*getPrivateMember(OdomUpdateTag{}))(
			highCovarianceIdentity,
			rclcpp::Time(highCovarianceIdentity.header.stamp)),
			"identity plus BAD_COV without recovery was accepted");
	requirePoseNear(lastPose, baselinePose,
			"identity plus BAD_COV rejection mutated lastPose");
	require(lastPoseStamp.nanoseconds() == baselineStamp &&
			core.getResetEpoch() == baselineEpoch &&
			core.getLocalOptimizedPoses().size() == baselineGraphSize &&
			revision == baselineRevision,
			"identity plus BAD_COV rejection mutated stamp, epoch, graph or revision");

	const auto legalIdentity = odometry(7, 0.0, 0.0, false);
	source.authorize(legalIdentity);
	require((wrapper.get()->*getPrivateMember(OdomUpdateTag{}))(
			legalIdentity, rclcpp::Time(legalIdentity.header.stamp)),
			"legal source-authenticated identity first pose was rejected");
	require(lastPoseStamp.nanoseconds() ==
			rclcpp::Time(legalIdentity.header.stamp).nanoseconds(),
			"legal identity first pose did not advance lastPoseStamp");
	const rtabmap::Transform identityPose = lastPose;
	const long long identityStamp = lastPoseStamp.nanoseconds();
	const unsigned long long identityEpoch = core.getResetEpoch();
	const std::size_t identityGraphSize = core.getLocalOptimizedPoses().size();
	const unsigned long long identityRevision = revision;

	const auto immediateJump = odometry(8, 0.75, 0.0, false);
	require(!updateOdom(*wrapper, source, immediateJump),
			">0.5m jump immediately after a legal identity first pose was accepted");
	requirePoseNear(lastPose, identityPose,
			"post-identity jump rejection mutated lastPose");
	require(lastPoseStamp.nanoseconds() == identityStamp &&
			core.getResetEpoch() == identityEpoch &&
			core.getLocalOptimizedPoses().size() == identityGraphSize &&
			revision == identityRevision,
			"post-identity jump rejection mutated stamp, epoch, graph or revision");

	const auto immediateContinuous = odometry(9, 0.1, 0.0, false);
	require(updateOdom(*wrapper, source, immediateContinuous),
			"small continuous pose immediately after a rejected post-identity jump was rejected");
	requirePoseNear(lastPose,
			rtabmap::Transform(0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
			"small continuous post-identity pose was not committed");
	require(lastPoseStamp.nanoseconds() ==
			rclcpp::Time(immediateContinuous.header.stamp).nanoseconds(),
			"small continuous post-identity pose did not advance lastPoseStamp");
	require(core.getResetEpoch() == identityEpoch &&
			core.getLocalOptimizedPoses().size() == identityGraphSize &&
			revision == identityRevision,
			"small odometry-only post-identity update mutated epoch, graph or revision");

	std::cout << "PASS native managed odometry numeric contract: first-frame "
			"NaN/Inf/non-unit, non-leading covariance NaN, twist NaN and "
			"identity+BAD_COV reject before pose/graph; legal identity accepts, "
			"then immediate jump rejects and continuous motion accepts"
			<< std::endl;
}

void runOutOfOrderAdmissionContract(const std::filesystem::path & root)
{
	std::filesystem::create_directories(root);
	rclcpp::NodeOptions options;
	options.parameter_overrides(parameters((root / "out_of_order.db").string()));
	auto wrapper = std::make_shared<Wrapper>(options);
	ManagedOdomSource source(wrapper);
	auto & core = wrapper.get()->*getPrivateMember(RtabmapTag{});
	auto & lastPose = wrapper.get()->*getPrivateMember(LastPoseTag{});
	auto & lastPoseStamp = wrapper.get()->*getPrivateMember(LastPoseStampTag{});
	auto & revision = wrapper.get()->*getPrivateMember(GraphRevisionTag{});
	auto & odomSamples =
			wrapper.get()->*getPrivateMember(AuthenticatedOdomSamplesTag{});
	auto & odomInfoSamples =
			wrapper.get()->*getPrivateMember(AuthenticatedOdomInfoSamplesTag{});
	auto & rgbSamples =
			wrapper.get()->*getPrivateMember(AuthenticatedRgbSamplesTag{});
	auto & depthSamples =
			wrapper.get()->*getPrivateMember(AuthenticatedDepthSamplesTag{});
	auto & cameraInfoSamples =
			wrapper.get()->*getPrivateMember(AuthenticatedCameraInfoSamplesTag{});

	// Queue t0 source-bound evidence first. It must remain untouched when this
	// delayed synchronized callback arrives after a complete t1 transaction.
	const auto delayedOdomValue = odometry(10, 0.9, 0.0);
	source.authorize(delayedOdomValue);
	auto delayedInfo = source.pushOdomInfo(10, false);
	const long long delayedStampNs =
			rclcpp::Time(delayedOdomValue.header.stamp).nanoseconds();
	const ManagedRgbdFrame delayedRgbd = managedRgbdFrame(timeFromSeconds(10));
	source.authorizeRgbd(
			delayedRgbd.rgb, delayedRgbd.depth, delayedRgbd.cameraInfo);
	require(odomSamples.find(delayedStampNs) != odomSamples.end() &&
			odomInfoSamples.find(delayedStampNs) != odomInfoSamples.end(),
			"failed to stage delayed t0 source-bound evidence");
	const std::string rgbFingerprint = rgbSamples.at(delayedStampNs);
	const std::string depthFingerprint = depthSamples.at(delayedStampNs);
	const std::string cameraInfoFingerprint = cameraInfoSamples.at(delayedStampNs);

	// Admit and commit the newer t1 frame first.
	auto acceptedInfo = source.pushOdomInfo(11, false);
	source.authorizeForProcess(*acceptedInfo);
	require(updateOdom(*wrapper, source, odometry(11, 1.0, 0.0)),
			"newer t1 managed odometry was rejected");
	processFrame(*wrapper, 1, 11,
			rtabmap::Transform(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	require(!core.getLocalOptimizedPoses().empty(),
			"newer t1 frame did not create a graph baseline");

	const rtabmap::Transform baselinePose = lastPose;
	const long long baselineLastPoseStampNs = lastPoseStamp.nanoseconds();
	const unsigned long long baselineEpoch = core.getResetEpoch();
	const std::size_t baselineGraphSize = core.getLocalOptimizedPoses().size();
	const unsigned long long baselineRevision = revision;

	nav_msgs::msg::Odometry::ConstSharedPtr delayedOdom =
			std::make_shared<const nav_msgs::msg::Odometry>(delayedOdomValue);
	rtabmap_msgs::msg::UserData::ConstSharedPtr noUserData;
	const std::vector<cv_bridge::CvImageConstPtr> delayedImages{delayedRgbd.rgb};
	const std::vector<cv_bridge::CvImageConstPtr> delayedDepths{delayedRgbd.depth};
	const std::vector<sensor_msgs::msg::CameraInfo> delayedCameraInfo{
			delayedRgbd.cameraInfo};
	const std::vector<sensor_msgs::msg::CameraInfo> noDepthCameraInfo;
	const sensor_msgs::msg::LaserScan noScan;
	const sensor_msgs::msg::PointCloud2 noCloud;
	const std::vector<rtabmap_msgs::msg::GlobalDescriptor> noGlobalDescriptors;
	const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > noKeyPoints;
	const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > noPoints3d;
	const std::vector<cv::Mat> noDescriptors;
	(wrapper.get()->*getPrivateMember(CommonMultiCameraCallbackTag{}))(
			delayedOdom,
			noUserData,
			delayedImages,
			delayedDepths,
			delayedCameraInfo,
			noDepthCameraInfo,
			noScan,
			noCloud,
			delayedInfo,
			noGlobalDescriptors,
			noKeyPoints,
			noPoints3d,
			noDescriptors);

	// The front monotonic gate must run before every one-shot consumer and
	// before odomUpdate()/process() can roll state back.
	require(odomSamples.find(delayedStampNs) != odomSamples.end(),
			"out-of-order t0 consumed one-shot managed Odometry evidence");
	require(odomInfoSamples.find(delayedStampNs) != odomInfoSamples.end(),
			"out-of-order t0 consumed one-shot managed OdomInfo evidence");
	require(rgbSamples.find(delayedStampNs) != rgbSamples.end() &&
			rgbSamples.at(delayedStampNs) == rgbFingerprint &&
			depthSamples.find(delayedStampNs) != depthSamples.end() &&
			depthSamples.at(delayedStampNs) == depthFingerprint &&
			cameraInfoSamples.find(delayedStampNs) != cameraInfoSamples.end() &&
			cameraInfoSamples.at(delayedStampNs) == cameraInfoFingerprint,
			"out-of-order t0 consumed RGB/depth/CameraInfo evidence");
	requirePoseNear(lastPose, baselinePose,
			"out-of-order t0 rolled lastPose back");
	require(lastPoseStamp.nanoseconds() == baselineLastPoseStampNs,
			"out-of-order t0 rolled lastPoseStamp back");
	require(core.getResetEpoch() == baselineEpoch &&
			core.getLocalOptimizedPoses().size() == baselineGraphSize &&
			revision == baselineRevision,
			"out-of-order t0 mutated epoch, graph or revision");

	std::cout << "PASS native t1->t0 admission ordering: delayed callback "
			"rejected before all one-shot evidence and pose/graph state"
			<< std::endl;
}

void runQuarantineStateMachine(const std::filesystem::path & root)
{
	std::filesystem::create_directories(root);
	rclcpp::NodeOptions options;
	options.parameter_overrides(parameters((root / "quarantine.db").string()));
	auto wrapper = std::make_shared<Wrapper>(options);
	ManagedOdomSource source(wrapper);
	auto & core = wrapper.get()->*getPrivateMember(RtabmapTag{});
	auto & pending = wrapper.get()->*getPrivateMember(RecoveryPendingTag{});
	auto & suspected = wrapper.get()->*getPrivateMember(SuspectedResetTag{});
	auto & lostStreak = wrapper.get()->*getPrivateMember(LostStreakTag{});
	auto & lastPose = wrapper.get()->*getPrivateMember(LastPoseTag{});
	auto & revision = wrapper.get()->*getPrivateMember(GraphRevisionTag{});

	const unsigned long long initialEpoch = core.getResetEpoch();
	auto processInfo = source.pushOdomInfo(1, false);
	source.authorizeForProcess(*processInfo);
	const auto baselineMsg = odometry(1, 1.0, 0.0);
	require(updateOdom(*wrapper, source, baselineMsg),
			"initial odometry was rejected");
	processFrame(*wrapper, 1, 1, rtabmap::Transform(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	require(!core.getLocalOptimizedPoses().empty(),
			"initial valid frame did not create a private graph node");
	require(core.getResetEpoch() == initialEpoch,
			"initial valid frame changed reset epoch");
	const std::size_t initialGraphSize = core.getLocalOptimizedPoses().size();

	const rtabmap::Transform baselinePose = lastPose;
	require(!updateOdom(*wrapper, source, odometry(2, 0.0, 0.0)),
			"unannounced identity odometry was accepted");
	requirePoseNear(lastPose, baselinePose,
			"identity rejection mutated lastPose before graph admission");
	require(suspected.load(std::memory_order_acquire),
			"identity rejection did not latch suspected-reset evidence");
	require(core.getResetEpoch() == initialEpoch &&
			core.getLocalOptimizedPoses().size() == initialGraphSize,
			"identity rejection mutated graph or epoch");

	require(updateOdom(*wrapper, source, odometry(3, 1.1, 2.0)),
			"legitimate continuous odometry did not recover");
	require(!suspected.load(std::memory_order_acquire),
			"legitimate continuous odometry did not clear suspected-reset latch");
	const rtabmap::Transform continuousPose = lastPose;

	require(!updateOdom(*wrapper, source, odometry(4, 1.2, 2.0, true)),
			"unannounced high-covariance odometry was accepted");
	requirePoseNear(lastPose, continuousPose,
			"high-covariance rejection mutated lastPose");
	require(!updateOdom(*wrapper, source, odometry(5, 2.0, 2.0)),
			"unannounced >0.5m odometry jump was accepted");
	requirePoseNear(lastPose, continuousPose,
			"translation-jump rejection mutated lastPose");
	require(!updateOdom(*wrapper, source, odometry(6, 1.1, 40.0)),
			"unannounced >30deg odometry jump was accepted");
	requirePoseNear(lastPose, continuousPose,
			"rotation-jump rejection mutated lastPose");
	require(core.getResetEpoch() == initialEpoch &&
			core.getLocalOptimizedPoses().size() == initialGraphSize,
			"rejected odometry mutated graph or epoch");

	require(updateOdom(*wrapper, source, odometry(7, 1.2, 3.0)),
			"second continuous odometry sample was rejected");
	for(int second=20; second<27; ++second)
	{
		source.pushOdomInfo(second, true);
	}
	require(lostStreak.load(std::memory_order_acquire) == 7,
			"seven unique lost measurements did not produce a seven-count streak");
	processInfo = source.pushOdomInfo(27, false);
	source.authorizeForProcess(*processInfo);
	require(!pending.load(std::memory_order_acquire),
			"seven lost measurements incorrectly armed recovery");
	require(core.getResetEpoch() == initialEpoch,
			"seven lost measurements changed epoch");
	require(updateOdom(*wrapper, source, odometry(27, 1.25, 3.0)),
			"continuous recovery after seven losses was rejected");
	processFrame(*wrapper, 2, 27,
			rtabmap::Transform(1.25f, 0.0f, 0.0f, 0.0f, 0.0f,
					static_cast<float>(3.0 * kPi / 180.0)));
	require(core.getResetEpoch() == initialEpoch,
			"seven-loss valid frame split the epoch");

	for(int second=30; second<38; ++second)
	{
		source.pushOdomInfo(second, true);
	}
	processInfo = source.pushOdomInfo(38, false);
	source.authorizeForProcess(*processInfo);
	require(pending.load(std::memory_order_acquire),
			"eight unique lost measurements plus recovery did not arm boundary");
	const unsigned long long preRecoveryRevision = revision;
	const std::size_t preRecoveryGraphSize = core.getLocalOptimizedPoses().size();
	rtabmap::SensorData invalid;
	const cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	(wrapper.get()->*getPrivateMember(ProcessTag{}))(
			timeFromSeconds(38), invalid, rtabmap::Transform(),
			std::vector<float>(), "odom", covariance,
			rtabmap::OdometryInfo(), 0.0, rtabmap::ImageInputIdentity());
	require(pending.load(std::memory_order_acquire),
			"invalid process input consumed the pending recovery boundary");
	require(core.getResetEpoch() == initialEpoch && revision == preRecoveryRevision &&
			core.getLocalOptimizedPoses().size() == preRecoveryGraphSize,
			"invalid process input mutated epoch, revision or graph");

	// process() consumes the exact OdomInfo admission stamp before checking the
	// data payload, while deliberately retaining the pending reset boundary.
	// A later valid frame therefore needs a fresh source-authenticated stamp.
	const rclcpp::Time delayedPreEpochStamp(38, 500000000, RCL_ROS_TIME);
	auto delayedPreEpochInfo = source.pushOdomInfo(delayedPreEpochStamp, false);
	source.authorizeForProcess(*delayedPreEpochInfo);
	const long long delayedPreEpochStampNs = delayedPreEpochStamp.nanoseconds();
	require(source.hasTransferredOdomInfo(delayedPreEpochStampNs),
			"delayed pre-epoch fixture did not reach process authorization ledger");
	processInfo = source.pushOdomInfo(39, false);
	source.authorizeForProcess(*processInfo);
	const auto authenticatedJump = odometry(39, 10.0, 100.0);
	require(updateOdom(*wrapper, source, authenticatedJump),
			"authenticated recovery large jump was rejected");
	processFrame(*wrapper, 3, 39,
			rtabmap::Transform(10.0f, 0.0f, 0.0f, 0.0f, 0.0f,
					static_cast<float>(100.0 * kPi / 180.0)));
	require(!pending.load(std::memory_order_acquire),
			"valid recovery frame did not consume pending boundary");
	require(core.getResetEpoch() == initialEpoch + 1U,
			"eight-loss recovery did not advance exactly one epoch");
	require(revision > preRecoveryRevision,
			"epoch split did not advance authoritative graph revision");
	auto & epochStartStamp = wrapper.get()->*getPrivateMember(EpochStartStampTag{});
	require(epochStartStamp.load(std::memory_order_acquire) ==
			rclcpp::Time(processInfo->header.stamp).nanoseconds(),
			"recovery split did not publish the exact epoch cutoff stamp");
	const std::size_t graphSizeAfterSplit = core.getLocalOptimizedPoses().size();
	const unsigned long long revisionAfterSplit = revision;
	processFrame(*wrapper, 33, delayedPreEpochStamp,
			rtabmap::Transform(99.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
	require(core.getResetEpoch() == initialEpoch + 1U &&
			core.getLocalOptimizedPoses().size() == graphSizeAfterSplit &&
			revision == revisionAfterSplit,
			"delayed pre-epoch RGB-D frame crossed the epoch cutoff and mutated the graph");

	const unsigned long long recoveredEpoch = core.getResetEpoch();
	processInfo = source.pushOdomInfo(40, false);
	source.authorizeForProcess(*processInfo);
	require(updateOdom(*wrapper, source, odometry(40, 10.05, 101.0)),
			"post-recovery continuous odometry was rejected");
	processFrame(*wrapper, 4, 40,
			rtabmap::Transform(10.05f, 0.0f, 0.0f, 0.0f, 0.0f,
					static_cast<float>(101.0 * kPi / 180.0)));
	require(core.getResetEpoch() == recoveredEpoch,
			"post-recovery continuous frame advanced epoch a second time");
	std::cout << "PASS native odometry quarantine: pre-admission rejects, "
			"7-loss hold, 8-loss authenticated single split, continuous resume"
			<< std::endl;
}

void runConvertedCameraModelExtrinsicFailStop(
		const std::filesystem::path & root)
{
	std::filesystem::create_directories(root);
	rclcpp::NodeOptions options;
	options.parameter_overrides(parameters((root / "converted_extrinsic.db").string()));
	auto wrapper = std::make_shared<Wrapper>(options);
	auto & core = wrapper.get()->*getPrivateMember(RtabmapTag{});
	auto & revision = wrapper.get()->*getPrivateMember(GraphRevisionTag{});
	auto & fatal = wrapper.get()->*getPrivateMember(SafeRelocalizationFatalTag{});
	const std::size_t graphBefore = core.getLocalOptimizedPoses().size();
	const unsigned long long revisionBefore = revision;

	// Calling the implementation directly isolates the post-conversion TOCTOU
	// guard.  The image frame equals base_link, so conversion produces an
	// identity CameraModel transform, deliberately different from the frozen
	// deployment transform in parameters().
	const ManagedRgbdFrame frame = managedRgbdFrame(
			timeFromSeconds(40), "base_link");
	const std::vector<cv_bridge::CvImageConstPtr> images{frame.rgb};
	const std::vector<cv_bridge::CvImageConstPtr> depths{frame.depth};
	const std::vector<sensor_msgs::msg::CameraInfo> cameraInfo{frame.cameraInfo};
	const std::vector<sensor_msgs::msg::CameraInfo> noDepthCameraInfo;
	const rtabmap_msgs::msg::UserData::ConstSharedPtr noUserData;
	const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr noOdomInfo;
	const sensor_msgs::msg::LaserScan noScan;
	const sensor_msgs::msg::PointCloud2 noCloud;
	const std::vector<rtabmap_msgs::msg::GlobalDescriptor> noGlobalDescriptors;
	const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > noKeyPoints;
	const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > noPoints3d;
	const std::vector<cv::Mat> noDescriptors;
	(wrapper.get()->*getPrivateMember(CommonMultiCameraCallbackImplTag{}))(
			std::string(),
			noUserData,
			images,
			depths,
			cameraInfo,
			noDepthCameraInfo,
			noScan,
			noCloud,
			noOdomInfo,
			noGlobalDescriptors,
			noKeyPoints,
			noPoints3d,
			noDescriptors);
	require(fatal.load(std::memory_order_acquire),
			"converted CameraModel extrinsic mismatch did not latch fatal state");
	require(!rclcpp::ok(),
			"converted CameraModel extrinsic mismatch did not request shutdown");
	require(core.getLocalOptimizedPoses().size() == graphBefore &&
			revision == revisionBefore,
			"converted CameraModel extrinsic mismatch mutated graph or revision");
	std::cout << "PASS converted CameraModel extrinsic mismatch fail-stop before graph"
			<< std::endl;
}

} // namespace

void runBusyAndSupersededRecovery(const std::filesystem::path & root)
{
    std::filesystem::create_directories(root);
    auto config=parameters((root/"busy.db").string());
    config.emplace_back("unannounced_jump_quarantine_enabled",true);
    config.emplace_back("reset_quarantine_expected_sensor_frame","base_link");
    config.emplace_back("reset_quarantine_expected_imu_frame","base_link");
    config.emplace_back("reset_quarantine_expected_sensor_to_base","0 0 0 0 0 0 1");
    config.emplace_back("Rtabmap/DetectionRate",std::string("0"));
    config.emplace_back("RGBD/LinearUpdate",std::string("0"));
    config.emplace_back("RGBD/AngularUpdate",std::string("0"));
    rclcpp::NodeOptions options;options.parameter_overrides(config);
    auto wrapper=std::make_shared<Wrapper>(options);
    ManagedOdomSource source(wrapper);
    auto & core=wrapper.get()->*getPrivateMember(RtabmapTag{});
    auto & last=wrapper.get()->*getPrivateMember(LastPoseTag{});
    auto & gate=wrapper.get()->*getPrivateMember(JumpGateTag{});
    const auto initialEpoch=core.getResetEpoch();
    struct Input {
        nav_msgs::msg::Odometry::ConstSharedPtr odom;
        rtabmap_msgs::msg::OdomInfo::SharedPtr info;
        ManagedRgbdFrame rgbd;
    };
    auto prepare=[&](int second,double x) {
        Input input{std::make_shared<const nav_msgs::msg::Odometry>(odometry(second,x,0.)),
            source.pushOdomInfo(second,false,20),managedRgbdFrame(timeFromSeconds(second),"base_link")};
        source.authorize(*input.odom);
        source.authorizeRgbd(input.rgbd.rgb,input.rgbd.depth,input.rgbd.cameraInfo);
        return input;
    };
    auto deliver=[&](const Input & input,bool drain=true) {
        (wrapper.get()->*getPrivateMember(CommonMultiCameraCallbackTag{}))(
            input.odom,{}, {input.rgbd.rgb},{input.rgbd.depth},{input.rgbd.cameraInfo},{},
            sensor_msgs::msg::LaserScan(),sensor_msgs::msg::PointCloud2(),input.info,{},{},{},{});
        if(drain) (wrapper.get()->*getPrivateMember(ProcessAsyncTag{}))();
        require((wrapper.get()->*getPrivateMember(LostStreakTag{})).load()==0 &&
            !(wrapper.get()->*getPrivateMember(RecoveryPendingTag{})).load(),"Busy recovery forged lost state");
    };
    deliver(prepare(1,1.));
    const auto original=core.getLocalOptimizedPoses();
    deliver(prepare(2,2.));deliver(prepare(3,2.02));
    // Real contention from another thread; UMutex is recursive, so locking
    // it in the callback's own thread would NOT reproduce busy rejection.
    {
        struct Busy {
            std::promise<void> entered,release;
            std::thread worker;
            explicit Busy(UMutex & mutex):worker([this,&mutex] {
                UScopeMutex lock(mutex);entered.set_value();release.get_future().wait();
            }) { entered.get_future().wait(); }
            ~Busy() { release.set_value();worker.join(); }
        } busy(wrapper.get()->*getPrivateMember(SyncMutexTag{}));
        deliver(prepare(4,2.04),false);
        deliver(prepare(5,4.),false);
    }
    require(core.getResetEpoch()==initialEpoch && gate.pendingStamp()==0 && std::abs(last.x()-1.f)<1e-5f,
        "BUSY_REGRESSION: busy frame created a recovery boundary/accepted reference without queue ownership");
    deliver(prepare(6,4.02));deliver(prepare(7,4.04));deliver(prepare(8,4.06));
    require(core.getResetEpoch()==initialEpoch+1,"Fresh chain did not recover after busy frames");
    // Authenticate both frames before queueing either: no executor spin can
    // drain frame9 between the two explicit callback invocations.
    auto queued=prepare(9,4.08);auto next=prepare(10,4.1);
    deliver(queued,false);deliver(next,false);
    require(std::abs(last.x()-4.08f)<1e-5f && source.hasOdomInfoSample(timeFromSeconds(10).nanoseconds()),
        "Queued-frame rejection advanced pose or consumed next frame's one-shot evidence");
    (wrapper.get()->*getPrivateMember(ProcessAsyncTag{}))();
    deliver(next);
    require(core.getResetEpoch()==initialEpoch+1 && std::abs(last.x()-4.1f)<1e-5f,
        "Preserved next frame could not be admitted after queue drain");
    deliver(prepare(11,6.));deliver(prepare(12,6.02));deliver(prepare(13,6.04),false);
    rtabmap::SensorData invalid;
    (wrapper.get()->*getPrivateMember(ProcessTag{}))(timeFromSeconds(13),invalid,rtabmap::Transform(),
        {},"odom",cv::Mat::eye(6,6,CV_64FC1)*0.01,rtabmap::OdometryInfo(),0.,rtabmap::ImageInputIdentity());
    (wrapper.get()->*getPrivateMember(ProcessAsyncTag{}))();
    require(gate.pendingStamp()==timeFromSeconds(13).nanoseconds(),"Invalid pending fixture lost boundary");
    auto delayed=prepare(14,6.06);source.authorizeForProcess(*delayed.info);
    deliver(prepare(15,8.));deliver(prepare(16,8.02));
    require(gate.pendingStamp()==timeFromSeconds(13).nanoseconds() && core.getResetEpoch()==initialEpoch+1,
        "Replacement chain armed before three valid probes");
    deliver(prepare(17,8.04),false);
    require(gate.pendingStamp()==timeFromSeconds(17).nanoseconds() && core.getResetEpoch()==initialEpoch+1,
        "SUPERSEDE_REGRESSION: stable newer chain remained pinned to an abandoned pending recovery");
    processFrame(*wrapper,114,14,rtabmap::Transform(6.06f,0.f,0.f,0.f,0.f,0.f));
    require(gate.pendingStamp()==timeFromSeconds(17).nanoseconds() && core.getResetEpoch()==initialEpoch+1,
        "Delayed old-chain payload consumed replacement boundary");
    (wrapper.get()->*getPrivateMember(ProcessAsyncTag{}))();
    require(core.getResetEpoch()==initialEpoch+2 && gate.pendingStamp()==0,"Replacement did not isolate once");
    rtabmap::AcceptedImageReceipt receipt;
    require(core.getAcceptedImageReceipt(core.getLastLocationId(),receipt) &&
        receipt.input.headerStampNs==timeFromSeconds(17).nanoseconds() && receipt.resetEpoch==initialEpoch+2,
        "Replacement receipt does not bind current epoch to the actual new image");
    auto processing=prepare(18,8.06);auto concurrent=prepare(19,8.08);
    deliver(processing,false);
    // Stall the real processAsync transaction on its pose lock, then offer
    // another fully authenticated image. This checks queue ownership during
    // process preflight, not just a mutex substituted for backend work.
    std::unique_lock<UMutex> poseLock(wrapper.get()->*getPrivateMember(LastPoseMutexTag{}));
    auto worker=std::async(std::launch::async,[&] { (wrapper.get()->*getPrivateMember(ProcessAsyncTag{}))(); });
    auto & queueMutex=wrapper.get()->*getPrivateMember(SyncMutexTag{});
    const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    bool processOwnsQueue=false;
    while(std::chrono::steady_clock::now()<until) {
        if(queueMutex.lockTry()!=0) { processOwnsQueue=true;break; }
        queueMutex.unlock();std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    try { if(processOwnsQueue) deliver(concurrent,false); }
    catch(...) { poseLock.unlock();worker.get();throw; }
    poseLock.unlock();worker.get();
    require(processOwnsQueue && source.hasOdomInfoSample(timeFromSeconds(19).nanoseconds()) &&
        std::abs(last.x()-8.06f)<1e-5f,"Concurrent admission crossed an active Core transaction");
    deliver(concurrent);
    require(core.getResetEpoch()==initialEpoch+2,"Concurrent queue drain created an extra epoch");
    const auto & trusted=core.*getPrivateMember(TrustedPosesTag{});
    require(trusted.size()==original.size(),"Busy recovery promoted an uncommitted map");
    for(const auto & item:original) requirePoseNear(trusted.at(item.first),item.second,"Busy recovery moved old trusted graph");
    for(const auto & item:core.getLocalConstraints()) if(item.second.from()>0 && item.second.to()>0)
        require(core.getSignatureResetEpoch(item.second.from())==core.getSignatureResetEpoch(item.second.to()),
            "Busy/replaced recovery created an unverified cross-epoch edge");
    std::cout<<"PASS native busy/superseded recovery: real mutex contention, queued-frame ownership, retained one-shot evidence, stable replacement after invalid frame, stale-chain cutoff, actual concurrent processAsync, frozen old graph, current image receipt"<<std::endl;
}

void runUnannouncedJumpRecovery(const std::filesystem::path & root)
{
    std::filesystem::create_directories(root);
    auto config=parameters((root/"jump.db").string());
    config.emplace_back("unannounced_jump_quarantine_enabled",true);
    config.emplace_back("reset_quarantine_expected_sensor_frame","base_link");
    config.emplace_back("reset_quarantine_expected_imu_frame","base_link");
    config.emplace_back("reset_quarantine_expected_sensor_to_base","0 0 0 0 0 0 1");
    config.emplace_back("Rtabmap/DetectionRate",std::string("0"));
    config.emplace_back("RGBD/LinearUpdate",std::string("0"));
    config.emplace_back("RGBD/AngularUpdate",std::string("0"));
    rclcpp::NodeOptions options;options.parameter_overrides(config);
    auto wrapper=std::make_shared<Wrapper>(options);
    ManagedOdomSource source(wrapper);
    auto & core=wrapper.get()->*getPrivateMember(RtabmapTag{});
    auto & last=wrapper.get()->*getPrivateMember(LastPoseTag{});
    auto & lost=wrapper.get()->*getPrivateMember(LostStreakTag{});
    auto & pending=wrapper.get()->*getPrivateMember(RecoveryPendingTag{});
    auto & jumpGate=wrapper.get()->*getPrivateMember(JumpGateTag{});
    auto & trusted=core.*getPrivateMember(TrustedPosesTag{});
    const auto epoch=core.getResetEpoch();
    auto frame=[&](int second,double x,double yaw,int inliers=20,bool authorizePixels=true,bool drain=true,bool tamperInfo=false) {
        auto msg=std::make_shared<const nav_msgs::msg::Odometry>(odometry(second,x,yaw));
        auto info=source.pushOdomInfo(second,false,inliers);
        if(tamperInfo) ++info->inliers; // copy no longer matches authenticated payload
        source.authorize(*msg);
        auto rgbd=managedRgbdFrame(timeFromSeconds(second),"base_link");
        // Real ROS authentication + actual common callback, not direct state
        // insertion. Synthetic pixels/poses are clearly not field evidence.
        if(authorizePixels) source.authorizeRgbd(rgbd.rgb,rgbd.depth,rgbd.cameraInfo);
        (wrapper.get()->*getPrivateMember(CommonMultiCameraCallbackTag{}))(
            msg,{}, {rgbd.rgb},{rgbd.depth},{rgbd.cameraInfo},{},
            sensor_msgs::msg::LaserScan(),sensor_msgs::msg::PointCloud2(),info,{},{},{},{});
        if(drain) (wrapper.get()->*getPrivateMember(ProcessAsyncTag{}))();
        require(lost.load()==0 && !pending.load(),"Jump recovery forged lost/recovered state");
    };
    frame(1,1.,0.);
    require(!core.getLocalOptimizedPoses().empty(),"Full-envelope initial image not processed");
    frame(2,2.,0.);
    require(core.getResetEpoch()==epoch && std::abs(last.x()-1.f)<1e-5f,"Single spike mutated graph/pose");
    frame(3,1.02,0.);
    require(core.getResetEpoch()==epoch && std::abs(last.x()-1.02f)<1e-5f,"Return to old reference failed");
    const auto oldGraph=core.getLocalOptimizedPoses();
    frame(4,2.,0.);frame(5,2.02,0.);
    require(core.getResetEpoch()==epoch && std::abs(last.x()-1.02f)<1e-5f,"Two probes prematurely admitted jump");
    frame(6,2.03,0.,0); // exact source, insufficient visual quality
    frame(7,2.04,0.);frame(8,2.06,0.);
    require(core.getResetEpoch()==epoch,"Weak OdomInfo did not reset the candidate streak");
    frame(9,2.08,0.,20,true,false);
    require(core.getResetEpoch()==epoch && jumpGate.pendingStamp()==timeFromSeconds(9).nanoseconds(),
        "Candidate admission mutated graph before actual process transaction");
    processFrame(*wrapper,88,8,rtabmap::Transform(2.06f,0.f,0.f,0.f,0.f,0.f));
    require(core.getResetEpoch()==epoch && jumpGate.pendingStamp()!=0,
        "Delayed pre-boundary frame consumed jump isolation");
    rtabmap::SensorData invalid;
    (wrapper.get()->*getPrivateMember(ProcessTag{}))(timeFromSeconds(9),invalid,rtabmap::Transform(),
        {},"odom",cv::Mat::eye(6,6,CV_64FC1)*0.01,rtabmap::OdometryInfo(),0.,rtabmap::ImageInputIdentity());
    (wrapper.get()->*getPrivateMember(ProcessAsyncTag{}))();
    require(core.getResetEpoch()==epoch && jumpGate.pendingStamp()!=0,
        "Invalid payload or consumed authorization lost pending jump boundary");
    frame(10,2.09,0.,0);
    require(core.getResetEpoch()==epoch && jumpGate.pendingStamp()!=0 && std::abs(last.x()-2.08f)<1e-5f,
        "Weak pending sample advanced reference or consumed isolation");
    frame(11,2.1,0.);
    require(core.getResetEpoch()==epoch+1,"Three valid probes did not start exactly one isolated epoch");
    require(trusted.size()==oldGraph.size(),"Old trusted graph was not preserved");
    for(const auto & item:oldGraph) requirePoseNear(trusted.at(item.first),item.second,"Frozen old pose changed");
    rtabmap::AcceptedImageReceipt receipt;
    require(core.getAcceptedImageReceipt(core.getLastLocationId(),receipt),"Recovery image lacks actual Core pixel receipt");
    require(receipt.resetEpoch==epoch+1 && receipt.input.headerStampNs==timeFromSeconds(11).nanoseconds(),
        "Recovery receipt has wrong epoch/stamp");
    frame(12,2.12,0.);
    require(core.getResetEpoch()==epoch+1,"Continuous candidate motion caused another epoch");
    frame(13,3.,0.,20,false);
    require(core.getResetEpoch()==epoch+1 && std::abs(last.x()-2.12f)<1e-5f,"Unauthenticated pixels advanced recovery");
    frame(14,2.12,40.);frame(15,2.14,41.);
    require(core.getResetEpoch()==epoch+1,"Rotation recovery armed too early");
    frame(16,2.16,42.);
    require(core.getResetEpoch()==epoch+2,"Repeated rotation jump did not create protected recovery epoch");
    require(trusted.size()==oldGraph.size(),"Uncommitted candidate was promoted into old trusted map");
    for(const auto & item:oldGraph) requirePoseNear(trusted.at(item.first),item.second,"Repeated jump changed old trusted poses");
    for(const auto & item:core.getLocalConstraints()) {
        if(item.second.from()>0 && item.second.to()>0)
            require(core.getSignatureResetEpoch(item.second.from())==core.getSignatureResetEpoch(item.second.to()),
                "Unverified cross-epoch edge created");
    }
    frame(17,3.,42.,20,true,true,true);
    require(core.getResetEpoch()==epoch+2 && std::abs(last.x()-2.16f)<1e-5f && jumpGate.pendingStamp()==0,
        "Mismatched exact OdomInfo payload advanced jump recovery");
    source.introduceConflictingOdomInfoSource();
    auto conflictingInfo=odomInfo(100,false);
    require(!source.tryAuthorizeForProcess(*conflictingInfo),"Conflicted source still admitted OdomInfo");
    const auto graphSize=core.getLocalOptimizedPoses().size();
    processFrame(*wrapper,100,100,rtabmap::Transform(3.f,0.f,0.f,0.f,0.f,0.f));
    require(core.getResetEpoch()==epoch+2 && core.getLocalOptimizedPoses().size()==graphSize,
        "Compromised source mutated graph or recovery epoch");
    std::cout<<"PASS native unannounced jump: full RGB-D source authentication, transient rejection, low-quality hold, delayed/invalid pending frames, translation/rotation isolation, old poses preserved, Core image receipt, exact payload and second-publisher rejection, no forged lost state or cross-epoch edge"<<std::endl;
}

int main(int argc, char ** argv)
{
	try
	{
		if(argc != 2)
		{
			std::cerr << "usage: wrapper_quarantine_native_test NEW_PRIVATE_DIR"
					<< std::endl;
			return 2;
		}
		const std::filesystem::path root(argv[1]);
		if(root.empty() || root == root.root_path() || std::filesystem::exists(root))
		{
			std::cerr << "private directory must be a new non-root path" << std::endl;
			return 2;
		}
		std::filesystem::create_directories(root);
		rclcpp::init(argc, argv);
		runOdomInfoArrivalContract(root / "odom_info_arrival");
		runManagedNumericContract(root / "numeric_contract");
		runOutOfOrderAdmissionContract(root / "out_of_order_admission");
		runQuarantineStateMachine(root);
		runBusyAndSupersededRecovery(root / "busy_recovery");
		runUnannouncedJumpRecovery(root / "jump_recovery");
		// The real duplicate-publisher guard deliberately shuts down ROS. The
		// independent final fail-stop case needs a fresh context, not a bypass.
		require(!rclcpp::ok(), "Source conflict did not perform expected ROS fail-stop");
		rclcpp::init(argc, argv);
		runConvertedCameraModelExtrinsicFailStop(root / "converted_extrinsic");
		if(rclcpp::ok())
		{
			rclcpp::shutdown();
		}
		return 0;
	}
	catch(const std::exception & exception)
	{
		std::cerr << "FAIL: " << exception.what() << std::endl;
		if(rclcpp::ok())
		{
			rclcpp::shutdown();
		}
		return 1;
	}
}
