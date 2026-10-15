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

#ifndef COREWRAPPER_H_
#define COREWRAPPER_H_

#include <rtabmap_slam/visibility.h>
#include <rtabmap_slam/JumpQuarantineGate.h>
#include <rtabmap_slam/GlobalPublicationGate.h>
#include <rtabmap_slam/MappingContinuityGate.h>
#include <rtabmap_slam/SubmapPublisher.h>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <condition_variable>
#include <mutex>

#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/OdometryInfo.h>

#include "rtabmap_msgs/srv/get_node_data.hpp"
#include "rtabmap_msgs/srv/get_map.hpp"
#include "rtabmap_msgs/srv/get_map2.hpp"
#include "rtabmap_msgs/srv/list_labels.hpp"
#include "rtabmap_msgs/srv/publish_map.hpp"
#include "rtabmap_msgs/srv/set_goal.hpp"
#include "rtabmap_msgs/srv/set_label.hpp"
#include "rtabmap_msgs/srv/remove_label.hpp"
#include "rtabmap_msgs/msg/goal.hpp"
#include "rtabmap_msgs/srv/get_plan.hpp"
#include "rtabmap_sync/CommonDataSubscriber.h"

#include "rtabmap_msgs/msg/odom_info.hpp"
#include "rtabmap_msgs/msg/link.hpp"
#include <atomic>
#include <array>
#include <cstdint>
#include <chrono>
#include <map>
#include <set>
#include <vector>
#include "rtabmap_msgs/msg/info.hpp"
#include "rtabmap_msgs/msg/landmark_detection.hpp"
#include "rtabmap_msgs/msg/landmark_detections.hpp"
#include "rtabmap_msgs/msg/env_sensor.h"
#include "rtabmap_msgs/srv/get_nodes_in_radius.hpp"
#include "rtabmap_msgs/srv/load_database.hpp"
#include "rtabmap_msgs/srv/detect_more_loop_closures.hpp"
#include "rtabmap_msgs/srv/global_bundle_adjustment.hpp"
#include "rtabmap_msgs/srv/cleanup_local_grids.hpp"
#include "rtabmap_msgs/srv/add_link.hpp"

#include "rtabmap_util/MapsManager.h"

#include "rtabmap_util/ULogToRosout.h"

#ifdef WITH_OCTOMAP_MSGS
#include <octomap_msgs/srv/get_octomap.hpp>
#endif

#ifdef WITH_APRILTAG_MSGS
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#endif

#ifdef WITH_ARUCO_MSGS
#include <aruco_msgs/msg/marker_array.hpp>
#endif

#ifdef WITH_ARUCO_OPENCV_MSGS
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#endif

#ifdef WITH_ARUCO_MARKERS_MSGS
#include <aruco_markers_msgs/msg/marker_array.hpp>
#endif

#ifdef WITH_ROS2_ARUCO_INTERFACES
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#endif

#ifdef WITH_NAV2_MSGS
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#endif

//#define WITH_FIDUCIAL_MSGS
#ifdef WITH_FIDUCIAL_MSGS
#include <fiducial_msgs/FiducialTransformArray.h>
#endif

namespace rtabmap {
class StereoDense;
}

namespace rtabmap_slam {

class CoreWrapper : public rclcpp::Node, public rtabmap_sync::CommonDataSubscriber
{
public:
	RTABMAP_SLAM_PUBLIC
	explicit CoreWrapper(const rclcpp::NodeOptions & options);
	virtual ~CoreWrapper();
	bool safeRelocalizationFatal() const
	{
		return safeRelocalizationFatal_.load(std::memory_order_acquire);
	}

#ifdef WITH_NAV2_MSGS
	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;
#endif

private:
	bool odomUpdate(const nav_msgs::msg::Odometry & odomMsg, rclcpp::Time stamp);
	bool odomUpdate(const nav_msgs::msg::Odometry & odomMsg, rclcpp::Time stamp,
			const rtabmap_msgs::msg::OdomInfo * authenticatedRgbdInfo);
	bool odomTFUpdate(const std::string & odomFrameId, const rclcpp::Time & stamp); // TF odom

	// Callback called from sync thread
	virtual void commonMultiCameraCallback(
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & depthCameraInfoMsgs,
				const sensor_msgs::msg::LaserScan & scanMsg,
				const sensor_msgs::msg::PointCloud2 & scan3dMsg,
				const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
				const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_msgs::msg::GlobalDescriptor>(),
				const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > & localKeyPoints = std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> >(),
				const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > & localPoints3d = std::vector<std::vector<rtabmap_msgs::msg::Point3f> >(),
				const std::vector<cv::Mat> & localDescriptors = std::vector<cv::Mat>());
	// Callback called from sync thread
	void commonMultiCameraCallbackImpl(
				const std::string & odomFrameId,
				const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & depthCameraInfoMsgs,
				const sensor_msgs::msg::LaserScan & scan2dMsg,
				const sensor_msgs::msg::PointCloud2 & scan3dMsg,
				const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
				const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & globalDescriptorMsgs,
				const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > & localKeyPoints,
				const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > & localPoints3d,
				const std::vector<cv::Mat> & localDescriptors);
	// Callback called from sync thread
	virtual void commonLaserScanCallback(
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
				const sensor_msgs::msg::LaserScan & scanMsg,
				const sensor_msgs::msg::PointCloud2 & scan3dMsg,
				const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
				const rtabmap_msgs::msg::GlobalDescriptor & globalDescriptor = rtabmap_msgs::msg::GlobalDescriptor());
	// Callback called from sync thread
	virtual void commonOdomCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);

	// Callback called from sync thread
	virtual void commonSensorDataCallback(
			const rtabmap_msgs::msg::SensorData::ConstSharedPtr & sensorDataMsg,
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr & odomInfoMsg);

	void defaultCallback(const sensor_msgs::msg::Image::ConstSharedPtr imageMsg); // no odom

	void userDataAsyncCallback(const rtabmap_msgs::msg::UserData::SharedPtr dataMsg);
	void globalPoseAsyncCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr globalPoseMsg);
	void gpsFixAsyncCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gpsFixMsg);
	void envSensorAsyncCallback(const rtabmap_msgs::msg::EnvSensor::SharedPtr envSensorMsg);
	void landmarkDetectionAsyncCallback(const rtabmap_msgs::msg::LandmarkDetection::SharedPtr landmarkDetection);
	void landmarkDetectionsAsyncCallback(const rtabmap_msgs::msg::LandmarkDetections::SharedPtr landmarkDetections);
#ifdef WITH_APRILTAG_MSGS
	void tagDetectionsAsyncCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
	void apriltagAsyncCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
#endif
#ifdef WITH_ARUCO_MSGS
	void arucoAsyncCallback(const aruco_msgs::msg::MarkerArray::SharedPtr msg);
#endif
#ifdef WITH_ARUCO_OPENCV_MSGS
	void arucoOpencvAsyncCallback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg);
#endif
#ifdef WITH_ARUCO_MARKERS_MSGS
	void arucoMarkersAsyncCallback(const aruco_markers_msgs::msg::MarkerArray::SharedPtr msg);
#endif
#ifdef WITH_ROS2_ARUCO_INTERFACES
	void arucoInterfacesAsyncCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
#endif
#ifdef WITH_FIDUCIAL_MSGS
	void fiducialDetectionsAsyncCallback(const fiducial_msgs::msgs::FiducialTransformArray::SharedPtr fiducialDetections);
#endif
	void imuAsyncCallback(
			const sensor_msgs::msg::Imu::SharedPtr msg,
			const rclcpp::MessageInfo & messageInfo);
	void republishNodeDataCallback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg);
	void interOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void interOdomInfoCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg1, const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr & msg2);
	void resetQuarantineOdomCallback(
			const nav_msgs::msg::Odometry::SharedPtr msg,
			const rclcpp::MessageInfo & messageInfo);
	void resetQuarantineOdomInfoCallback(
			const rtabmap_msgs::msg::OdomInfo::SharedPtr msg,
			const rclcpp::MessageInfo & messageInfo);
	void resetQuarantineRgbCallback(
			const sensor_msgs::msg::Image::SharedPtr msg,
			const rclcpp::MessageInfo & messageInfo);
	void resetQuarantineDepthCallback(
			const sensor_msgs::msg::Image::SharedPtr msg,
			const rclcpp::MessageInfo & messageInfo);
	void resetQuarantineCameraInfoCallback(
			const sensor_msgs::msg::CameraInfo::SharedPtr msg,
			const rclcpp::MessageInfo & messageInfo);
	bool validateResetQuarantinePublisher(
			const std::string & topicName,
			const char * streamName,
			const std::string & expectedNode,
			const std::string & expectedNamespace,
			bool & bound,
			bool & compromised,
			std::array<uint8_t, RMW_GID_STORAGE_SIZE> & boundGid,
			const rmw_gid_t * incomingGid);
	bool observeResetQuarantineOdomInfo(
			const rtabmap_msgs::msg::OdomInfo & msg);
	bool authenticateResetQuarantineOdomInfo(
			const rtabmap_msgs::msg::OdomInfo & msg);
	bool authenticateResetQuarantineRgbd(
			const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
			const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
			const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs);
	bool validateResetQuarantineSensorExtrinsic(
			const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs);
	bool verifySafeRelocalizationDatabasePathBinding();
	void safeRelocalizationProposalCallback(
			const std_msgs::msg::String::SharedPtr msg,
			const rclcpp::MessageInfo & messageInfo);
	void publishResetEpoch();
    rtabmap::ImageInputIdentity captureAuthenticatedImageIdentity(
        const cv_bridge::CvImageConstPtr & image, const sensor_msgs::msg::CameraInfo & info);
    bool queuedImageIdentityStillValid(const rtabmap::ImageInputIdentity & input);
    void publishAcceptedImageReceipt(const rtabmap::AcceptedImageReceipt & receipt);
	std::string publishSafeRelocalizationStatus(
			const std::string & result,
			const std::string & reason,
			const std::string & transactionToken,
			int tagId,
			unsigned long long sourceEpoch,
			unsigned long long observerGraphRevision,
			const std::vector<rtabmap::Link> & evidence,
			bool publish = true);
	void clearSafeRelocalizationTokens();
	void clearSafeRelocalizationProtocolState(bool firstUpdateHandled, bool clearTokens);

	void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

	void goalCommonCallback(int id,
			const std::string & label,
			const std::string & frameId,
			const rtabmap::Transform & pose,
			const rclcpp::Time & stamp,
			double * planningTime = 0);
	void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void goalNodeCallback(const rtabmap_msgs::msg::Goal::SharedPtr msg);
	void updateGoal(const rclcpp::Time & stamp);

	void processAsync();

	void process(
			const rclcpp::Time & stamp,
			rtabmap::SensorData & data,
			const rtabmap::Transform & odom = rtabmap::Transform(),
			const std::vector<float> & odomVelocity = std::vector<float>(),
			const std::string & odomFrameId = "",
			const cv::Mat & odomCovariance = cv::Mat::eye(6,6,CV_64FC1),
			const rtabmap::OdometryInfo & odomInfo = rtabmap::OdometryInfo(),
			double timeMsgConversion = 0.0,
            const rtabmap::ImageInputIdentity & imageIdentity = rtabmap::ImageInputIdentity());
	std::map<int, rtabmap::Transform> filterNodesToAssemble(
			const std::map<int, rtabmap::Transform> & nodes,
			const rtabmap::Transform & currentPose);
	
	void applyParameters();

	void updateRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void resetRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void pauseRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void resumeRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void loadDatabaseCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::LoadDatabase::Request>, std::shared_ptr<rtabmap_msgs::srv::LoadDatabase::Response>);
	void triggerNewMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void backupDatabaseCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void detectMoreLoopClosuresCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::DetectMoreLoopClosures::Request>, std::shared_ptr<rtabmap_msgs::srv::DetectMoreLoopClosures::Response>);
	void globalBundleAdjustmentCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GlobalBundleAdjustment::Request>, std::shared_ptr<rtabmap_msgs::srv::GlobalBundleAdjustment::Response>);
	void cleanupLocalGridsCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::CleanupLocalGrids::Request>, std::shared_ptr<rtabmap_msgs::srv::CleanupLocalGrids::Response>);
	void setModeLocalizationCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setModeMappingCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogDebug(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogInfo(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogWarn(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogError(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void getNodeDataCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GetNodeData::Request>, std::shared_ptr<rtabmap_msgs::srv::GetNodeData::Response>);
	void getMapDataCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GetMap::Request>, std::shared_ptr<rtabmap_msgs::srv::GetMap::Response>);
	void getMapData2Callback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GetMap2::Request>, std::shared_ptr<rtabmap_msgs::srv::GetMap2::Response>);
	void getMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetMap::Request>, std::shared_ptr<nav_msgs::srv::GetMap::Response>);
	void getProbMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetMap::Request>, std::shared_ptr<nav_msgs::srv::GetMap::Response>);
	void publishMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::PublishMap::Request>, std::shared_ptr<rtabmap_msgs::srv::PublishMap::Response>);
	void getPlanCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetPlan::Request>, std::shared_ptr<nav_msgs::srv::GetPlan::Response>);
	void getPlanNodesCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GetPlan::Request>, std::shared_ptr<rtabmap_msgs::srv::GetPlan::Response>);
	void setGoalCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::SetGoal::Request>, std::shared_ptr<rtabmap_msgs::srv::SetGoal::Response>);
	void cancelGoalCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLabelCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::SetLabel::Request>, std::shared_ptr<rtabmap_msgs::srv::SetLabel::Response>);
	void listLabelsCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::ListLabels::Request>, std::shared_ptr<rtabmap_msgs::srv::ListLabels::Response> res);
	void removeLabelCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::RemoveLabel::Request>, std::shared_ptr<rtabmap_msgs::srv::RemoveLabel::Response> res);
	void addLinkCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::AddLink::Request>, std::shared_ptr<rtabmap_msgs::srv::AddLink::Response> res);
	void getNodesInRadiusCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GetNodesInRadius::Request>, std::shared_ptr<rtabmap_msgs::srv::GetNodesInRadius::Response> res);

#ifdef WITH_OCTOMAP_MSGS
	void octomapBinaryCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>, std::shared_ptr<octomap_msgs::srv::GetOctomap::Response>);
	void octomapFullCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>, std::shared_ptr<octomap_msgs::srv::GetOctomap::Response>);
#endif

	void loadParameters(const std::string & configFile, rtabmap::ParametersMap & parameters);
	void saveParameters(const std::string & configFile);

	// Only processingCallbackGroup_ may query Core frame registration.
	bool globalMapPublicationAllowed() const;
	void withFreshGlobalPose(const std::function<void()> & publish);
	GlobalPublicationGate globalPublicationGate_;
	MappingContinuityGate mappingContinuityGate_;
	std::unique_ptr<SubmapPublisher> submapPublisher_; // processing lane only
	rclcpp::TimerBase::SharedPtr submapHeartbeatTimer_;
	bool mappingHasWrittenLocalFrame_=false;
	std::uint64_t mappingWrittenLocalSerial_=0;
	unsigned long long mappingWrittenLocalEpoch_=0;
	void refreshSubmapVisualization(const rclcpp::Time & stamp,bool localHealthy);
	void publishMappingContinuityStatus();
	bool tryMappingVisualRecovery();
	bool mappingRecoveryCommitCurrent(MappingContinuityGate::Ticket ticket, unsigned long long epoch, bool requireProposal);
	bool observeRecoveredMainMap(const rtabmap::SensorData & data, const rtabmap::Transform & odom, long long stampNs);
	void discardPendingMappingRecovery(const std::string & reason);
	bool observePendingMappingRecovery(const rtabmap::SensorData & data, const rtabmap::Transform & odom, long long stampNs);
	bool pendingMappingAlignmentAllowed(const rtabmap::Transform & correction);
	struct PendingMappingRecovery {
		bool active=false,requireProposal=false;
		MappingContinuityGate::Ticket ticket{0,0};
		unsigned long long epoch=0,revision=0;
		rtabmap::Transform correction,lastCorrection;
		rtabmap::Rtabmap::SafeRelocalizationLimits limits;
		std::vector<std::pair<rtabmap::Transform,rtabmap::Transform>> observations; // measured main pose, odom
		std::function<bool(std::string &)> commit;
		std::function<void(bool,const std::string &)> finish;
		std::string token,pendingStatus;
	};
	PendingMappingRecovery pendingMappingRecovery_; // processing callback group only
	int mappingContinuityVoWidth_ = 384;
	int mappingContinuityVoHeight_ = 240;
	std::uint64_t mappingObservationGeneration_ = 0;
	rtabmap::Transform mappingObservationSeedCorrection_;
	rtabmap::Transform mappingObservationLastCorrection_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mappingRecoveryReferencePub_;
	std::mutex mappingRecoveryReferenceMutex_;
	sensor_msgs::msg::Image::SharedPtr mappingLastTrustedImage_;
	int mappingLastTrustedNodeId_ = 0;
	unsigned long long mappingLastTrustedEpoch_ = 0;
	std::uint64_t mappingLastReferencePublishedGeneration_ = 0;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mappingContinuityStatusPub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mappingContinuityTextPub_;
	rclcpp::TimerBase::SharedPtr mappingContinuityTimer_;
	void publishStats(const rclcpp::Time & stamp);
	void publishCurrentGoal(const rclcpp::Time & stamp);
#ifdef WITH_NAV2_MSGS
#ifdef NAV_MSGS_FOXY
	void goalResponseCallback(std::shared_future<GoalHandleNav2::SharedPtr> future);
#else
    void goalResponseCallback(const GoalHandleNav2::SharedPtr & goal_handle);
#endif
	void resultCallback(const GoalHandleNav2::WrappedResult & result);
#endif

	void publishLocalPath(const rclcpp::Time & stamp);
	void publishGlobalPath(const rclcpp::Time & stamp);
	void republishMaps();

private:
	rtabmap::Rtabmap rtabmap_;
	bool paused_;

	UMutex lastPoseMutex_;
	rtabmap::Transform lastPose_;
	rclcpp::Time lastPoseStamp_;
	std::vector<float> lastPoseVelocity_;
	cv::Mat lastPoseCovariance_;
	bool lastPoseIntermediate_;

	rtabmap::Transform currentMetricGoal_;
	rtabmap::Transform lastPublishedMetricGoal_;
	bool latestNodeWasReached_;
	bool pubLocPoseOnlyWhenLocalizing_;
	bool graphLatched_;
	rtabmap::ParametersMap parameters_;
	std::map<std::string, float> rtabmapROSStats_;

	std::string frameId_;
	std::string odomFrameId_;
	std::string mapFrameId_;
	std::string groundTruthFrameId_;
	std::string groundTruthBaseFrameId_;
	std::string configPath_;
	std::string databasePath_;

	double tfDelay;
	double tfTolerance;

	double odomDefaultAngVariance_;
	double odomDefaultLinVariance_;
	double landmarkDefaultAngVariance_;
	double landmarkDefaultLinVariance_;
	double waitForTransform_;
	double stalenessFactor_;
	bool useActionForGoal_;
	bool useSavedMap_;
	bool genScan_;
	double genScanMaxDepth_;
	double genScanMinDepth_;
	bool genDepth_;
	int genDepthDecimation_;
	int genDepthFillHolesSize_;
	int genDepthFillIterations_;
	double genDepthFillHolesError_;
	int scanCloudMaxPoints_;
	bool scanCloudIs2d_;

	rtabmap::Transform mapToOdom_;
	std::mutex mapToOdomMutex_;

	rtabmap_util::MapsManager mapsManager_;

	rclcpp::Publisher<rtabmap_msgs::msg::Info>::SharedPtr infoPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::MapData>::SharedPtr mapDataPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::MapGraph>::SharedPtr mapGraphPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::MapGraph>::SharedPtr odomCachePub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr landmarksPub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr labelsPub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mapPathPub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localGridObstacle_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localGridEmpty_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localGridGround_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localizationPosePub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPoseSub_;

	//Planning stuff
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_;
	rclcpp::Subscription<rtabmap_msgs::msg::Goal>::SharedPtr goalNodeSub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nextMetricGoalPub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goalReachedPub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr globalPathPub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr localPathPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::Path>::SharedPtr globalPathNodesPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::Path>::SharedPtr localPathNodesPub_;
	std::string goalFrameId_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;

	rclcpp::AsyncParametersClient::SharedPtr parametersClient_;
	rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterEventSub_;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr updateSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pauseSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resumeSrv_;
	rclcpp::Service<rtabmap_msgs::srv::LoadDatabase>::SharedPtr loadDatabaseSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr triggerNewMapSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr backupDatabase_;
	rclcpp::Service<rtabmap_msgs::srv::DetectMoreLoopClosures>::SharedPtr detectMoreLoopClosuresSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GlobalBundleAdjustment>::SharedPtr globalBundleAdjustmentSrv_;
	rclcpp::Service<rtabmap_msgs::srv::CleanupLocalGrids>::SharedPtr cleanupLocalGridsSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setModeLocalizationSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setModeMappingSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogDebugSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogInfoSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogWarnSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogErrorSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GetNodeData>::SharedPtr getNodeDataSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GetMap>::SharedPtr getMapDataSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GetMap2>::SharedPtr getMapData2Srv_;
	rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr getMapSrv_;
	rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr getProbMapSrv_;
	rclcpp::Service<rtabmap_msgs::srv::PublishMap>::SharedPtr publishMapDataSrv_;
	rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr getPlanSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GetPlan>::SharedPtr getPlanNodesSrv_;
	rclcpp::Service<rtabmap_msgs::srv::SetGoal>::SharedPtr setGoalSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr cancelGoalSrv_;
	rclcpp::Service<rtabmap_msgs::srv::SetLabel>::SharedPtr setLabelSrv_;
	rclcpp::Service<rtabmap_msgs::srv::ListLabels>::SharedPtr listLabelsSrv_;
	rclcpp::Service<rtabmap_msgs::srv::RemoveLabel>::SharedPtr removeLabelSrv_;
	rclcpp::Service<rtabmap_msgs::srv::AddLink>::SharedPtr addLinkSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GetNodesInRadius>::SharedPtr getNodesInRadiusSrv_;
#ifdef WITH_OCTOMAP_MSGS
	rclcpp::Service<octomap_msgs::srv::GetOctomap>::SharedPtr octomapBinarySrv_;
	rclcpp::Service<octomap_msgs::srv::GetOctomap>::SharedPtr octomapFullSrv_;
#endif
#ifdef WITH_NAV2_MSGS
	rclcpp_action::Client<NavigateToPose>::SharedPtr nav2Client_;
	rclcpp_action::GoalUUID lastGoalSent_;
#endif

	std::thread* transformThread_;
	bool tfThreadRunning_;

	// for loop closure detection only
	image_transport::Subscriber defaultSub_;

	rclcpp::CallbackGroup::SharedPtr userDataAsyncCallbackGroup_;
	rclcpp::Subscription<rtabmap_msgs::msg::UserData>::SharedPtr userDataAsyncSub_;
	cv::Mat userData_;
	UMutex userDataMutex_;

	rclcpp::CallbackGroup::SharedPtr globalPoseAsyncCallbackGroup_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr globalPoseAsyncSub_;
	std::map<double, geometry_msgs::msg::PoseWithCovarianceStamped> globalPoses_;
	UMutex globalPoseMutex_;

	rclcpp::CallbackGroup::SharedPtr gpsAsyncCallbackGroup_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsFixAsyncSub_;
	std::map<double, rtabmap::GPS> gps_;
	UMutex gpsMutex_;

	rclcpp::CallbackGroup::SharedPtr envSensorAsyncCallbackGroup_;
	rclcpp::Subscription<rtabmap_msgs::msg::EnvSensor>::SharedPtr envSensorAsyncSub_;
	rtabmap::EnvSensors envSensors_;
	UMutex envSensorMutex_;

	rclcpp::CallbackGroup::SharedPtr landmarkCallbackGroup_;
	rclcpp::Subscription<rtabmap_msgs::msg::LandmarkDetection>::SharedPtr landmarkDetectionSub_;
	rclcpp::Subscription<rtabmap_msgs::msg::LandmarkDetections>::SharedPtr landmarkDetectionsSub_;
#ifdef WITH_APRILTAG_MSGS
	rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tagDetectionsSub_;
	rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltagSub_;
#endif
#ifdef WITH_ARUCO_MSGS
	rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr arucoSub_;
#endif
#ifdef WITH_ARUCO_OPENCV_MSGS
	rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr arucoOpencvSub_;
#endif
#ifdef WITH_ARUCO_MARKERS_MSGS
	rclcpp::Subscription<aruco_markers_msgs::msg::MarkerArray>::SharedPtr arucoMarkersSub_;
#endif
#ifdef WITH_ROS2_ARUCO_INTERFACES
	rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr arucoInterfacesSub_;
#endif
#ifdef WITH_FIDUCIAL_MSGS
	rclcpp::Subscription<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr fiducialTransfromsSub_;
#endif
	std::map<int, std::pair<geometry_msgs::msg::PoseWithCovarianceStamped, float> > landmarks_; // id, <pose, size>
	UMutex landmarksMutex_;

	rclcpp::CallbackGroup::SharedPtr imuCallbackGroup_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
	std::map<double, rtabmap::Transform> imus_;
	// Same imuMutex_ as imus_; exact managed stamp for each double time bucket.
	std::map<double, long long> resetQuarantineImuStampNsByTime_;
	std::string imuFrameId_;
	UMutex imuMutex_;

	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr republishNodeDataSub_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr interOdomSub_;
	std::list<std::pair<nav_msgs::msg::Odometry, rtabmap_msgs::msg::OdomInfo> > interOdoms_;
	message_filters::Subscriber<nav_msgs::msg::Odometry> interOdomSyncSub_;
	message_filters::Subscriber<rtabmap_msgs::msg::OdomInfo> interOdomInfoSyncSub_;
	typedef message_filters::sync_policies::ExactTime<nav_msgs::msg::Odometry, rtabmap_msgs::msg::OdomInfo> MyExactInterOdomSyncPolicy;
	message_filters::Synchronizer<MyExactInterOdomSyncPolicy> * interOdomSync_;

	bool stereoToDepth_;
	bool odomSensorSync_;
	float rate_;
	bool createIntermediateNodes_;
	int mappingMaxNodes_;
	double mappingAltitudeDelta_;
	bool alreadyRectifiedImages_;
	bool twoDMapping_;
	rclcpp::Time previousStamp_;

	rtabmap_util::ULogToRosout ulogToRosout_;

	class LocalizationStatusTask : public diagnostic_updater::DiagnosticTask
	{
	public:
		LocalizationStatusTask();
		void setLocalizationThreshold(double value);
		void updateStatus(const cv::Mat & covariance, bool twoDMapping);
		void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
	private:
		double localizationThreshold_;
		double localizationError_;
	};
	LocalizationStatusTask localizationDiagnostic_;

	rclcpp::CallbackGroup::SharedPtr processingCallbackGroup_;
	struct SyncData {
		bool valid;
		rclcpp::Time stamp;
		rtabmap::SensorData data;
		rtabmap::Transform odom;
		std::vector<float> odomVelocity;
		std::string odomFrameId;
		cv::Mat odomCovariance;
		rtabmap::OdometryInfo odomInfo;
		double timeMsgConversion;
        rtabmap::ImageInputIdentity imageIdentity;
	};
	rclcpp::TimerBase::SharedPtr syncTimer_;
	SyncData syncData_;
	UMutex syncDataMutex_;
	bool triggerNewMapBeforeNextUpdate_;
	bool resetQuarantineEnabled_;
	int resetQuarantineLostCount_;
	double resetQuarantineOdomInfoMaxAgeSec_;
	double resetQuarantineMaxUnannouncedTranslation_;
	double resetQuarantineMaxUnannouncedRotationDeg_;
	std::string resetQuarantineExpectedOdomInfoNode_;
	std::string resetQuarantineExpectedOdomNamespace_;
	std::string resetQuarantineExpectedSensorNode_;
	std::string resetQuarantineExpectedSensorNamespace_;
	std::string resetQuarantineExpectedImuNode_;
	std::string resetQuarantineExpectedImuNamespace_;
	std::string resetQuarantineExpectedImuFrame_;
	std::string resetQuarantineOdomTopic_;
	std::string resetQuarantineExpectedSensorFrame_;
	std::string resetQuarantineExpectedSensorToBase_;
	rtabmap::Transform resetQuarantineExpectedSensorToBaseTransform_;
	bool resetQuarantineOdomPublisherBound_;
	bool resetQuarantineOdomPublisherCompromised_;
	std::array<uint8_t, RMW_GID_STORAGE_SIZE> resetQuarantineOdomPublisherGid_;
	std::map<long long, std::string> resetQuarantineAuthenticatedOdomSamples_;
	// Full-rate, source-authenticated continuity, independent of RGB-D cadence.
	struct OdomContinuitySample {
		std::string fingerprint;
		unsigned long long chain;
	};
	std::map<long long, OdomContinuitySample> resetQuarantineOdomContinuity_;
	long long resetQuarantineContinuityStampNs_ = 0;
	rtabmap::Transform resetQuarantineContinuityPose_;
	unsigned long long resetQuarantineContinuitySerial_ = 0;
	unsigned long long resetQuarantineContinuityChain_ = 0;
	// Protected by lastPoseMutex_; the cache/stream above use OdomAuthMutex_.
	long long resetQuarantineContinuityAnchorStampNs_ = 0;
	unsigned long long resetQuarantineContinuityAnchorChain_ = 0;
	bool jumpQuarantineEnabled_ = false;
	JumpQuarantineGate jumpQuarantineGate_; // lastPoseMutex_; not a fusion authorization.
	std::mutex resetQuarantineOdomAuthMutex_;
	std::condition_variable resetQuarantineOdomAuthCondition_;
	bool resetQuarantineOdomInfoPublisherBound_;
	bool resetQuarantineOdomInfoPublisherCompromised_;
	std::array<uint8_t, RMW_GID_STORAGE_SIZE> resetQuarantineOdomInfoPublisherGid_;
	// Exact OdomInfo payload fingerprint and its own steady-clock arrival.
	// The per-sample timestamp prevents a newer shadow sample (or a rejected
	// duplicate) from making an older exact sample appear fresh.
	std::map<long long, std::pair<std::string, long long> > resetQuarantineAuthenticatedOdomInfoSamples_;
	std::mutex resetQuarantineOdomInfoAuthMutex_;
	std::condition_variable resetQuarantineOdomInfoAuthCondition_;
	bool resetQuarantineRgbPublisherBound_;
	bool resetQuarantineRgbPublisherCompromised_;
	std::array<uint8_t, RMW_GID_STORAGE_SIZE> resetQuarantineRgbPublisherGid_;
	bool resetQuarantineDepthPublisherBound_;
	bool resetQuarantineDepthPublisherCompromised_;
	std::array<uint8_t, RMW_GID_STORAGE_SIZE> resetQuarantineDepthPublisherGid_;
	bool resetQuarantineCameraInfoPublisherBound_;
	bool resetQuarantineCameraInfoPublisherCompromised_;
	std::array<uint8_t, RMW_GID_STORAGE_SIZE> resetQuarantineCameraInfoPublisherGid_;
	std::map<long long, std::string> resetQuarantineAuthenticatedRgbSamples_;
	std::map<long long, std::string> resetQuarantineAuthenticatedDepthSamples_;
	std::map<long long, std::string> resetQuarantineAuthenticatedCameraInfoSamples_;
	std::mutex resetQuarantineRgbdAuthMutex_;
	std::condition_variable resetQuarantineRgbdAuthCondition_;
	bool resetQuarantineImuPublisherBound_;
	bool resetQuarantineImuPublisherCompromised_;
	std::array<uint8_t, RMW_GID_STORAGE_SIZE> resetQuarantineImuPublisherGid_;
	long long resetQuarantineLastImuStampNs_;
	std::string resetQuarantineLastImuFingerprint_;
	unsigned int resetQuarantineImuEndpointCheckCounter_;
	std::mutex resetQuarantineImuAuthMutex_;
	// Exact sample stamp -> authenticated arrival steady time, transferred by
	// authenticateResetQuarantineOdomInfo() and consumed once by process().
	std::map<long long, long long> resetQuarantineAuthenticatedOdomInfoStamps_;
	std::atomic<int> resetQuarantineLostStreak_;
	std::atomic<bool> resetQuarantineRecoveryPending_;
	std::atomic<bool> resetQuarantineSuspectedReset_;
	std::atomic<bool> resetQuarantineOdomInfoSeen_;
	std::atomic<long long> resetQuarantineLastOdomInfoSteadyNs_;
	std::atomic<long long> resetQuarantineLastOdomInfoStampNs_;
	std::atomic<long long> resetQuarantineLostStartStampNs_;
	std::atomic<long long> resetQuarantineRecoveryStampNs_;
	std::atomic<long long> resetQuarantineEpochStartStampNs_;
	std::atomic<unsigned long long> resetQuarantineEpoch_;
	// Serializes the complete authenticated RGB-D admission envelope.  The
	// stamp/epoch gate at the start of commonMultiCameraCallback() must run
	// before any one-shot RGB-D/OdomInfo evidence or lastPose state is consumed.
	std::mutex resetQuarantineAdmissionMutex_;
	UMutex resetQuarantineStateMutex_;
	rclcpp::Subscription<rtabmap_msgs::msg::OdomInfo>::SharedPtr resetQuarantineOdomInfoSub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr resetQuarantineOdomSub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr resetQuarantineRgbSub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr resetQuarantineDepthSub_;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr resetQuarantineCameraInfoSub_;
	bool safeRelocalizationEnabled_;
	bool safeRelocalizationStartNewMapOnFirstUpdate_;
	bool safeRelocalizationFirstUpdateHandled_;
	double safeRelocalizationTimeoutSec_;
	double safeRelocalizationMaxTranslation_;
	double safeRelocalizationMaxRotation_;
	double safeRelocalizationMinBridgeTranslation_;
	double safeRelocalizationMinBridgeRotation_;
	double safeRelocalizationMaxAnchorZ_;
	double safeRelocalizationMaxAnchorTilt_;
	double safeRelocalizationMaxOldPoseTranslation_;
	double safeRelocalizationMaxOldPoseRotation_;
	double safeRelocalizationMaxNewRigidTranslation_;
	double safeRelocalizationMaxNewRigidRotation_;
	double safeRelocalizationMaxNewRigidZError_;
	double safeRelocalizationMaxNewRigidTiltError_;
	double safeRelocalizationMaxGravityResidualIncrease_;
	double safeRelocalizationMaxInformationCondition_;
	std::string safeRelocalizationDatabaseSha256_;
	std::string safeRelocalizationSessionToken_;
	std::string safeRelocalizationSessionTokenSha256_;
	std::string safeRelocalizationExpectedProposalNode_;
	std::string safeRelocalizationExpectedProposalNamespace_;
	bool safeRelocalizationProposalPublisherBound_;
	bool safeRelocalizationProposalPublisherCompromised_;
	std::array<uint8_t, RMW_GID_STORAGE_SIZE> safeRelocalizationProposalPublisherGid_;
	std::mutex safeRelocalizationProposalAuthMutex_;
	int safeRelocalizationDatabaseFd_;
	uint64_t safeRelocalizationDatabaseDevice_;
	uint64_t safeRelocalizationDatabaseInode_;
	bool safeRelocalizationDatabaseBindingValid_;
	std::atomic<bool> safeRelocalizationFatal_;
	unsigned long long safeRelocalizationGraphRevision_;
	std::set<std::string> safeRelocalizationSeenTokens_;
	// token -> (exact proposal bytes, terminal status JSON). Exact retransmits
	// are idempotent; the same token with different bytes is rejected.
	std::map<std::string, std::pair<std::string, std::string> > safeRelocalizationTerminalTokens_;
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr safeRelocalizationParameterGuardHandle_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safeRelocalizationProposalSub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safeRelocalizationStatusPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr acceptedImageReceiptPub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safeRelocalizationStatePub_;
	rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr resetEpochPub_;
};

}

#endif /* COREWRAPPER_H_ */
