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

#ifndef RTABMAP_H_
#define RTABMAP_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Statistics.h"
#include "rtabmap/core/Link.h"
#include "rtabmap/core/ProgressState.h"
#include "rtabmap/core/Graph.h"

#include <opencv2/core/core.hpp>
#include <list>
#include <map>
#include <stack>
#include <set>
#include <vector>
#include <functional>

namespace rtabmap_slam
{
class CoreWrapper;
}

#include <rtabmap/core/ImageInputReceipt.h>

namespace rtabmap
{

class EpipolarGeometry;
class Memory;
class BayesFilter;
class Signature;
class Optimizer;
class PythonInterface;
class RegistrationInfo;

class RTABMAP_CORE_EXPORT Rtabmap
{
	friend class CrossMapPendingTestAccess;
	friend class ImageReceiptNativeTestAccess;
	friend class rtabmap_slam::CoreWrapper;
public:
	enum VhStrategy {kVhNone, kVhEpipolar, kVhUndef};

public:
	Rtabmap();
	virtual ~Rtabmap();

	/**
	 * @brief Main loop of rtabmap.
	 * @param data Sensor data to process.
	 * @param odomPose Odometry pose, should be non-null for RGB-D SLAM mode.
	 * @param covariance Odometry covariance.
	 * @param externalStats External statistics to be saved in the database for convenience
	 * @return true if data has been added to map.
	 */
	bool process(
			const SensorData & data,
			Transform odomPose,
			const cv::Mat & odomCovariance = cv::Mat::eye(6,6,CV_64FC1),
			const std::vector<float> & odomVelocity = std::vector<float>(),
			const std::map<std::string, float> & externalStats = std::map<std::string, float>());
	// for convenience
	bool process(
			const SensorData & data,
			Transform odomPose,
			float odomLinearVariance,
			float odomAngularVariance,
			const std::vector<float> & odomVelocity = std::vector<float>(),
			const std::map<std::string, float> & externalStats = std::map<std::string, float>());
	// for convenience, loop closure detection only
	bool process(
			const cv::Mat & image,
			int id=0, const std::map<std::string, float> & externalStats = std::map<std::string, float>());

	/**
	 * Initialize Rtabmap with parameters and a database
	 * @param parameters Parameters overriding default parameters and database parameters
	 *                   (@see loadDatabaseParameters)
	 * @param databasePath The database input/output path. If not set, an
	 *                     empty database is used in RAM. If set and the file doesn't exist,
	 *                     it will be created empty. If the database exists, nodes and
	 *                     vocabulary will be loaded in working memory.
	 * @param loadDatabaseParameters If an existing database is used (@see databasePath),
	 *                               the parameters inside are loaded and set to current
	 *                               Rtabmap instance.
	 */
	void init(const ParametersMap & parameters, const std::string & databasePath = "", bool loadDatabaseParameters = false);
	void init(const ParametersMap & parameters, const std::string & databasePath, bool loadDatabaseParameters, bool preboundEmptyIsNew);
	/**
	 * Initialize Rtabmap with parameters from a configuration file and a database
	 * @param configFile Configuration file (*.ini) overriding default parameters and database parameters
	 *                   (@see loadDatabaseParameters)
	 * @param databasePath The database input/output path. If not set, an
	 *                     empty database is used in RAM. If set and the file doesn't exist,
	 *                     it will be created empty. If the database exists, nodes and
	 *                     vocabulary will be loaded in working memory.
	 * @param loadDatabaseParameters If an existing database is used (@see databasePath),
	 *                               the parameters inside are loaded and set to current
	 *                               Rtabmap instance.
	 */
	void init(const std::string & configFile = "", const std::string & databasePath = "", bool loadDatabaseParameters = false);
	void init(const std::string & configFile, const std::string & databasePath, bool loadDatabaseParameters, bool preboundEmptyIsNew);

	/**
	 * Close rtabmap. This will delete rtabmap object if set.
	 * @param databaseSaved true=database saved, false=database discarded.
	 * @param databasePath output database file name, ignored if
	 *                     Db/Sqlite3InMemory=false (opened database is
	 *                     then overwritten).
	 */
	void close(bool databaseSaved = true, const std::string & ouputDatabasePath = "");

	const std::string & getWorkingDir() const {return _wDir;}
	bool isRGBDMode() const { return _rgbdSlamMode; }
	int getLoopClosureId() const {return _loopClosureHypothesis.first;}
	float getLoopClosureValue() const {return _loopClosureHypothesis.second;}
	int getHighestHypothesisId() const {return _highestHypothesis.first;}
	float getHighestHypothesisValue() const {return _highestHypothesis.second;}
	int getLastLocationId() const;
	std::list<int> getWM() const; // working memory
	std::set<int> getSTM() const; // short-term memory
	int getWMSize() const; // working memory size
	int getSTMSize() const; // short-term memory size
	std::map<int, int> getWeights() const;
	int getTotalMemSize() const;
	double getLastProcessTime() const {return _lastProcessTime;};
	bool isInSTM(int locationId) const;
	bool isIDsGenerated() const;
	const Statistics & getStatistics() const;
	bool getAcceptedImageReceipt(int id, AcceptedImageReceipt & output) const;
	const std::map<int, Transform> & getLocalOptimizedPoses() const {return _optimizedPoses;}
	const std::multimap<int, Link> & getLocalConstraints() const {return _constraints;}
	Transform getPose(int locationId) const;
	Transform getMapCorrection() const {return _mapCorrection;}
	const Memory * getMemory() const {return _memory;}
	float getGoalReachedRadius() const {return _goalReachedRadius;}
	float getLocalRadius() const {return _localRadius;}
	const Transform & getLastLocalizationPose() const {return _lastLocalizationPose;}

	float getTimeThreshold() const {return _maxTimeAllowed;} // in ms
	void setTimeThreshold(float maxTimeAllowed); // in ms
	int getMemoryThreshold() const {return _maxMemoryAllowed;} // in nodes
	void setMemoryThreshold(int maxMemoryAllowed); // in nodes

	void setInitialPose(const Transform & initialPose);
	int triggerNewMap();
	bool labelLocation(int id, const std::string & label);
	/**
	 * Set user data. Detect automatically if raw or compressed. If raw, the data is
	 * compressed too. A matrix of type CV_8UC1 with 1 row is considered as compressed.
	 * If you have one dimension unsigned 8 bits raw data, make sure to transpose it
	 * (to have multiple rows instead of multiple columns) in order to be detected as
	 * not compressed.
	 */
	bool setUserData(int id, const cv::Mat & data);
	void generateDOTGraph(const std::string & path, int id=0, int margin=5);
	void exportPoses(
			const std::string & path,
			bool optimized,
			bool global,
			int format // 0=raw, 1=rgbd-slam format, 2=KITTI format, 3=TORO, 4=g2o
	);
	void resetMemory();
	void dumpPrediction() const;
	void dumpData() const;
	void parseParameters(const ParametersMap & parameters);
	const ParametersMap & getParameters() const {return _parameters;}
	void setWorkingDirectory(std::string path);
	void rejectLastLoopClosure();
	void deleteLastLocation();
	void setOptimizedPoses(const std::map<int, Transform> & poses, const std::multimap<int, Link> & constraints);
	Signature getSignatureCopy(int id, bool images, bool scan, bool userData, bool occupancyGrid, bool withWords, bool withGlobalDescriptors) const;
	// Use getGraph() instead with withImages=true, withScan=true, withUserData=true and withGrid=true.
	RTABMAP_DEPRECATED
		void get3DMap(std::map<int, Signature> & signatures,
				std::map<int, Transform> & poses,
				std::multimap<int, Link> & constraints,
				bool optimized,
				bool global) const;
	void getGraph(std::map<int, Transform> & poses,
			std::multimap<int, Link> & constraints,
			bool optimized,
			bool global,
			std::map<int, Signature> * signatures = 0,
			bool withImages = false,
			bool withScan = false,
			bool withUserData = false,
			bool withGrid = false,
			bool withWords = true,
			bool withGlobalDescriptors = true) const;
	std::map<int, Transform> getNodesInRadius(const Transform & pose, float radius, int k=0, std::map<int, float> * distsSqr=0); // If radius=0 and k=0, RGBD/LocalRadius is used. Can return landmarks.
	std::map<int, Transform> getNodesInRadius(int nodeId, float radius, int k=0, std::map<int, float> * distsSqr=0); // If nodeId==0, return poses around latest node. If radius=0 and k=0, RGBD/LocalRadius is used. Can return landmarks and use landmark id (negative) as request.
	int detectMoreLoopClosures(
			float clusterRadiusMax = 0.5f,
			float clusterAngle = M_PI/6.0f,
			int iterations = 1,
			bool intraSession = true,
			bool interSession = true,
			const ProgressState * state = 0,
			float clusterRadiusMin = 0.0f,
			int toFromMapId = -1);
	bool globalBundleAdjustment(
			int optimizerType = 1 /*g2o*/,
			bool rematchFeatures = true,
			int iterations = 0,
			float pixelVariance = 0.0f);
	int cleanupLocalGrids(
			const std::map<int, Transform> & mapPoses,
			const cv::Mat & map,
			float xMin,
			float yMin,
			float cellSize,
			int cropRadius = 1,
			bool filterScans = false);
	int refineLinks();
	bool addLink(const Link & link);
	struct SafeRelocalizationLimits
	{
		float maxHoldoutTranslation;
		float maxHoldoutRotation;
		float minBridgeTranslation;
		float minBridgeRotation;
		float maxAnchorZ;
		float maxAnchorTilt;
		float maxOldPoseTranslation;
		float maxOldPoseRotation;
		float maxNewRigidTranslation;
		float maxNewRigidRotation;
		float maxNewRigidZError;
		float maxNewRigidTiltError;
		float maxGravityResidualIncrease;
		double maxInformationCondition;
	};
	// Process-lifetime latch: once enabled, guarded graph semantics can only be
	// released by destroying the Rtabmap instance.
	void setSafeRelocalizationGuardEnabled(bool enabled);
	unsigned long long getResetEpoch() const {return _resetEpoch;}
	// Coordinate registration only; not a navigation or held-out geometry certificate.
	// Call on the Core processing thread. Adds no data member or virtual ABI change.
	bool isCurrentMapFrameRegistered() const
	{
		return !_safeRelocalizationGuardEnabled || _resetEpoch == 0 ||
				_safeRelocalizationCommittedEpochs.count(_resetEpoch) != 0;
	}
    struct LocalSubmapSnapshot
    {
        int mapId;
        unsigned long long resetEpoch;
        int componentId;
        bool registeredToMain;
        std::map<int,Transform> poses;
        std::multimap<int,Link> links;
    };
    struct RecoveryLocalMotionFault
    {
        unsigned long long resetEpoch=0;
        int fromId=0,toId=0;
    };
    // Core processing lane only. Current unregistered graph, grouped by
    // epoch/map/component; first node is identity, no main-map transform.
    // Relative measured links only; excludes self priors and virtual edges.
    bool getUnregisteredLocalSubmaps(std::vector<LocalSubmapSnapshot> & snapshots,
        std::string & reason) const;
	unsigned long long getSignatureResetEpoch(int id) const;
	cv::Mat getInformation(const cv::Mat & covariance) const;
	void addNodesToRepublish(const std::vector<int> & ids);

	int getPathStatus() const {return _pathStatus;} // -1=failed 0=idle/executing 1=success
	void clearPath(int status); // -1=failed 0=idle/executing 1=success
	bool computePath(int targetNode, bool global);
	bool computePath(const Transform & targetPose, float tolerance = -1.0f); // only in current optimized map, tolerance (m) < 0 means RGBD/LocalRadius, 0 means infinite
	const std::vector<std::pair<int, Transform> > & getPath() const {return _path;}
	std::vector<std::pair<int, Transform> > getPathNextPoses() const;
	std::vector<int> getPathNextNodes() const;
	int getPathCurrentGoalId() const;
	unsigned int getPathCurrentIndex() const {return _pathCurrentIndex;}
	unsigned int getPathCurrentGoalIndex() const {return _pathGoalIndex;}
	const Transform & getPathTransformToGoal() const {return _pathTransformToGoal;}

	std::map<int, Transform> getForwardWMPoses(int fromId, int maxNearestNeighbors, float radius, int maxDiffID) const;
	std::map<int, std::map<int, Transform> > getPaths(const std::map<int, Transform> & poses, const Transform & target, int maxGraphDepth = 0) const;
	void adjustLikelihood(std::map<int, float> & likelihood) const;
	std::pair<int, float> selectHypothesis(const std::map<int, float> & posterior,
											const std::map<int, float> & likelihood) const;

private:
    // Wrapper-owned candidate revalidation; no external transform is accepted.
    // Adds a nonvirtual method only, preserving the existing object layout.
    bool tryVisualRecoveryWithAcceptedImages(const SafeRelocalizationLimits & limits,
        const AcceptedImageSourceContext & context, const std::function<bool()> & commitAllowed,
        std::string & reason);
    bool prepareVisualRecoveryWithAcceptedImages(const SafeRelocalizationLimits & limits,
        const AcceptedImageSourceContext & context, const std::function<bool()> & preparationAllowed,
        Transform & proposedMapCorrection, std::string & reason);
    bool commitVisualRecoveryWithObservedAlignment(const SafeRelocalizationLimits & limits,
        const AcceptedImageSourceContext & context, const std::function<bool(const Transform &)> & commitAllowed,
        std::string & reason);
    bool tryVisualRecoveryImpl(const SafeRelocalizationLimits & limits,
        const AcceptedImageSourceContext & context, const std::function<bool()> & commitAllowed,
        Transform * preparedMapCorrection, const std::function<bool(const Transform &)> * observedAlignmentAllowed,
        std::string & reason);
    bool validateRecoveryCache(const std::set<int> & ids, const std::vector<Link> & evidence,
        const SafeRelocalizationLimits & limits, std::string & reason) const;
    bool findRecoveryLocalMotionFault(const SafeRelocalizationLimits & limits,
        RecoveryLocalMotionFault & fault) const;
    bool validateObservationAgainstReferences(const SensorData & observation, const Transform & predicted,
        const std::map<int,Transform> & references, int excludedId, int requiredAnchor,
        const SafeRelocalizationLimits & limits, Transform & measured, std::string & reason) const;
    bool validateLocalMapObservation(const SensorData & data, const Transform & odomPose,
            int lastLocalNodeId, const SafeRelocalizationLimits & limits,
            Transform & measuredLocalPose, std::string & reason) const;
    bool validateRegisteredMapObservation(const SensorData & data, const Transform & odomPose,
        int lastTrustedNodeId, const SafeRelocalizationLimits & limits,
        Transform & measuredMainPose, std::string & reason) const;
    bool validateProposedMapObservation(const SensorData & data, const Transform & odomPose,
        int lastTrustedNodeId, const Transform & proposedMapCorrection, const SafeRelocalizationLimits & limits,
        Transform & measuredMainPose, std::string & reason) const;
    bool validateSameEpochVisualClosure(int currentId, int oldId, const Transform & proposed,
        std::string & reason) const;
    bool addDepthCornerLinksWithAcceptedImagesAtomically(const std::vector<Link> & endpointEvidence,
        unsigned long long sourceResetEpoch, const SafeRelocalizationLimits & limits,
        const std::vector<AcceptedImageReceipt> & receipts,
        const std::vector<AcceptedTagCornerObservation> & corners,
        const AcceptedImageSourceContext & context, const std::function<bool()> & commitAllowed,
        std::string & reason);
    bool prepareDepthCornerRecoveryWithAcceptedImages(const std::vector<Link> & endpointEvidence,
        unsigned long long sourceResetEpoch, const SafeRelocalizationLimits & limits,
        const std::vector<AcceptedImageReceipt> & receipts,
        const std::vector<AcceptedTagCornerObservation> & corners,
        const AcceptedImageSourceContext & context, const std::function<bool()> & preparationAllowed,
        Transform & proposedMapCorrection, std::string & reason);
    bool commitDepthCornerRecoveryWithObservedAlignment(const std::vector<Link> & endpointEvidence,
        unsigned long long sourceResetEpoch, const SafeRelocalizationLimits & limits,
        const std::vector<AcceptedImageReceipt> & receipts,
        const std::vector<AcceptedTagCornerObservation> & corners,
        const AcceptedImageSourceContext & context, const std::function<bool(const Transform &)> & commitAllowed,
        std::string & reason);
    bool validateAcceptedImagesForTransaction(const std::vector<Link> & evidence,
        const std::vector<AcceptedImageReceipt> & receipts, unsigned long long sourceResetEpoch,
        const AcceptedImageSourceContext & context, std::string & reason) const;
    bool addLinksWithAcceptedImagesAtomically(const std::vector<Link> & evidence,
        unsigned long long sourceResetEpoch, const SafeRelocalizationLimits & limits,
        const std::vector<AcceptedImageReceipt> & receipts, const AcceptedImageSourceContext & context,
        std::string & reason);
	bool addDepthCornerLinksWithAcceptedImagesAtomically(const std::vector<Link> & endpointEvidence,
		unsigned long long sourceResetEpoch, const SafeRelocalizationLimits & limits,
		const std::vector<AcceptedImageReceipt> & receipts,
		const std::vector<AcceptedTagCornerObservation> & corners,
		const AcceptedImageSourceContext & context, std::string & reason);
    bool addLinksAtomicallyImpl(const std::vector<Link> & evidence, unsigned long long sourceResetEpoch,
        const SafeRelocalizationLimits & limits, const std::vector<AcceptedImageReceipt> * receipts,
        const AcceptedImageSourceContext * context, std::string & reason);
    bool addLinksAtomicallyImpl(const std::vector<Link> & evidence, unsigned long long sourceResetEpoch,
        const SafeRelocalizationLimits & limits, const std::vector<AcceptedImageReceipt> * receipts,
        const AcceptedImageSourceContext * context, std::string & reason,
        const std::function<bool()> * commitAllowed);
    bool addLinksAtomicallyImpl(const std::vector<Link> & evidence, unsigned long long sourceResetEpoch,
        const SafeRelocalizationLimits & limits, const std::vector<AcceptedImageReceipt> * receipts,
        const AcceptedImageSourceContext * context, std::string & reason,
        const std::function<bool()> * commitAllowed, Transform * preparedMapCorrection,
        const std::function<bool(const Transform &)> * observedAlignmentAllowed);
	bool buildDepthCornerEvidence(const std::vector<Link> & endpointEvidence,
		const std::vector<AcceptedImageReceipt> & receipts,
		const std::vector<AcceptedTagCornerObservation> & corners,
		std::vector<Link> & measuredEvidence, std::string & reason) const;
	bool addLinksAtomically(
			const std::vector<Link> & evidence,
			unsigned long long sourceResetEpoch,
			const SafeRelocalizationLimits & limits,
			std::string & reason);
	int triggerNewMapImpl();
	// Narrow, wrapper-owned capability used only after the managed odometry
	// recovery state machine has authenticated a graph boundary. Keeping this
	// private prevents same-process callers from advancing the reset epoch while
	// the safe-relocalization guard owns graph mutations.
	int triggerNewMapForContinuation();
	void optimizeCurrentMap(int id,
			bool lookInDatabase,
			std::map<int, Transform> & optimizedPoses,
			cv::Mat & covariance,
			std::multimap<int, Link> * constraints = 0,
			double * error = 0,
			int * iterationsDone = 0) const;
	std::map<int, Transform> optimizeGraph(
			int fromId,
			const std::set<int> & ids,
			const std::map<int, Transform> & guessPoses,
			bool lookInDatabase,
			cv::Mat & covariance,
			std::multimap<int, Link> * constraints = 0,
			double * error = 0,
			int * iterationsDone = 0) const;
	std::list<std::pair<int, int> > repairGraph(
		graph::MaxGraphErrors & maxGraphErrors,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		double & optimizationError,
		int & optimizationIterations,
		cv::Mat & optimizationCovariance);
	void updateGoalIndex();
	bool computePath(int targetNode, std::map<int, Transform> nodes, const std::multimap<int, rtabmap::Link> & constraints);

	void createGlobalScanMap();
	void rememberCurrentGraphForSafeRelocalization();
	bool promoteCommittedEpochForSafeRelocalization(std::string & reason);
	bool validateAndFreezeCommittedEpochForSafeRelocalization(std::string & reason);
	void rebuildSafeRelocalizationTrustedComponents();
    bool collectRawCrossMapLoopEvidence(const Signature & current, const Signature & old,
            unsigned long long currentResetEpoch, unsigned long long oldResetEpoch,
            std::string & reason);
	bool acceptCrossMapLoopEvidence(
			const Signature & current,
			const Signature & old,
			unsigned long long currentResetEpoch,
			unsigned long long oldResetEpoch,
			const Transform & currentToOld,
			const RegistrationInfo & info,
			std::string & reason);

	void setupLogFiles(bool overwrite = false);
	void flushStatisticLogs();

private:
	// Modifiable parameters
	bool _publishStats;
	bool _publishLastSignatureData;
	bool _publishPdf;
	bool _publishLikelihood;
	bool _publishRAMUsage;
	bool _computeRMSE;
	bool _saveWMState;
	float _maxTimeAllowed; // in ms
	unsigned int _maxMemoryAllowed; // signatures count in WM
	float _loopThr;
	float _loopRatio;
	float _aggressiveLoopThr;
	int _virtualPlaceLikelihoodRatio;
	float _maxLoopClosureDistance;
	bool _verifyLoopClosureHypothesis;
	unsigned int _maxRetrieved;
	unsigned int _maxLocalRetrieved;
	unsigned int _maxRepublished;
	bool _rawDataKept;
	bool _statisticLogsBufferedInRAM;
	bool _statisticLogged;
	bool _statisticLoggedHeaders;
	bool _rgbdSlamMode;
	float _rgbdLinearUpdate;
	float _rgbdAngularUpdate;
	float _rgbdLinearSpeedUpdate;
	float _rgbdAngularSpeedUpdate;
	float _newMapOdomChangeDistance;
	bool _neighborLinkRefining;
	bool _proximityByTime;
	bool _proximityBySpace;
	bool _scanMatchingIdsSavedInLinks;
	bool _loopClosureIdentityGuess;
	float _localRadius;
	float _localImmunizationRatio;
	int _proximityMaxGraphDepth;
	int _proximityMaxPaths;
	int _proximityMaxNeighbors;
	float _proximityFilteringRadius;
	bool _proximityRawPosesUsed;
	float _proximityAngle;
	bool _proximityOdomGuess;
	double _proximityMergedScanCovFactor;
	std::string _databasePath;
	bool _optimizeFromGraphEnd;
	float _optimizationMaxError;
	float _optimizationMaxErrorRepairRadius;
	bool _startNewMapOnLoopClosure;
	bool _crossMapLoopPending;
	int _crossMapLoopPendingCount;
	int _crossMapLoopPendingMaxIdSpan;
	int _crossMapLoopMinInliers;
	int _crossMapLoopMinGridCells;
	float _crossMapLoopMin3DSpan;
	float _crossMapLoopMinHessianRatio;
	float _crossMapLoopMaxTranslation;
	float _crossMapLoopMaxRotation;
	float _crossMapLoopMaxNormalizedError;
	float _crossMapLoopMaxFeatureJaccard;
	float _crossMapLoopMinSourceTranslation;
	float _crossMapLoopMinSourceRotation;
	bool _startNewMapOnGoodSignature;
	float _goalReachedRadius; // meters
	bool _goalsSavedInUserData;
	int _pathStuckIterations;
	float _pathLinearVelocity;
	float _pathAngularVelocity;
	bool _forceOdom3doF;
	bool _restartAtOrigin;
	bool _loopCovLimited;
	bool _loopGPS;
	int _maxOdomCacheSize;
	bool _localizationSmoothing;
	double _localizationPriorInf;
	bool _localizationSecondTryWithoutProximityLinks;
	bool _createGlobalScanMap;
	float _markerPriorsLinearVariance;
	float _markerPriorsAngularVariance;

	std::pair<int, float> _loopClosureHypothesis;
	struct CrossMapLoopEvidence
	{
		int newMapId;
		int oldMapId;
		unsigned long long newResetEpoch;
		unsigned long long oldResetEpoch;
		int newId;
		int oldId;
		Transform sourcePose;
		Transform anchor;
		double linearVariance;
		double angularVariance;
		std::set<int> inlierWords;
		double featureJaccardMaximum;
	};
	std::vector<CrossMapLoopEvidence> _crossMapLoopEvidence;
	unsigned long long _resetEpoch;
	std::map<int, unsigned long long> _signatureResetEpochs;
	bool _skipNextOdomResetDetection;
	std::map<int, Transform> _safeRelocalizationTrustedPoses;
	std::multimap<int, Link> _safeRelocalizationTrustedConstraints;
	std::map<int, int> _safeRelocalizationTrustedComponents;
	bool _safeRelocalizationTrustedSnapshotValid;
	std::set<unsigned long long> _safeRelocalizationCommittedEpochs;
	bool _safeRelocalizationPromotionLimitsValid;
	float _safeRelocalizationPromotionMaxOldPoseTranslation;
	float _safeRelocalizationPromotionMaxOldPoseRotation;
	bool _safeRelocalizationGuardEnabled;
	std::pair<int, float> _highestHypothesis;
	double _lastProcessTime;
	bool _someNodesHaveBeenTransferred;
	float _distanceTravelled;
	float _distanceTravelledSinceLastLocalization;
	bool _optimizeFromGraphEndChanged;

	// Abstract classes containing all loop closure
	// strategies for a type of signature or configuration.
	EpipolarGeometry * _epipolarGeometry;
	BayesFilter * _bayesFilter;
	Optimizer * _graphOptimizer;
	ParametersMap _parameters;

	Memory * _memory;

	FILE* _foutFloat;
	FILE* _foutInt;
	std::list<std::string> _bufferedLogsF;
	std::list<std::string> _bufferedLogsI;

	Statistics statistics_;

	std::string _wDir;

	std::map<int, Transform> _optimizedPoses;
	std::multimap<int, Link> _constraints;
	Transform _mapCorrection;
	Transform _mapCorrectionBackup; // used in localization mode when odom is lost
	Transform _lastLocalizationPose; // Corrected odometry pose. In mapping mode, this corresponds to last pose return by getLocalOptimizedPoses().
	int _lastLocalizationNodeId; // for localization mode
	cv::Mat _localizationCovariance;
	std::map<int, std::pair<cv::Point3d, Transform> > _gpsGeocentricCache;
	bool _currentSessionHasGPS;
	LaserScan _globalScanMap;
	std::map<int, Transform> _globalScanMapPoses;
	std::map<int, Transform> _odomCachePoses;       // used in localization mode to reject loop closures
	std::multimap<int, Link> _odomCacheConstraints; // used in localization mode to reject loop closures
	std::map<int, Transform> _markerPriors;
	std::pair<int, int> _lastRejectedLoopClosureIds;

	std::set<int> _nodesToRepublish;

	// Planning stuff
	int _pathStatus;
	std::vector<std::pair<int,Transform> > _path;
	std::set<unsigned int> _pathUnreachableNodes;
	unsigned int _pathCurrentIndex;
	unsigned int _pathGoalIndex;
	Transform _pathTransformToGoal;
	int _pathStuckCount;
	float _pathStuckDistance;

#ifdef RTABMAP_PYTHON
	PythonInterface * _python;
#endif

private:
    bool processWithAuthenticatedImage(const SensorData & data, Transform odomPose,
        const cv::Mat & covariance, const std::vector<float> & velocity,
        const std::map<std::string, float> & externalStats,
        const ImageInputIdentity & input, std::string & receiptReason);
    std::map<int, AcceptedImageReceipt> _acceptedImageReceipts;
    std::int64_t _lastAcceptedImageStampNs = 0;

};

} // namespace rtabmap
#endif /* RTABMAP_H_ */
