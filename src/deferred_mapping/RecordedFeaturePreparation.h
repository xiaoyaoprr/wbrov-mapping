#pragma once
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/Signature.h>
#include <stdexcept>

// Stateless image preparation only: no pose, word IDs, graph or permission is
// reused. RegistrationVis itself extracts the same pixels/depth/features that
// the F2M target would extract, using its own detector in a separate lane.
class RecordedFeaturePreparation {
public:
    explicit RecordedFeaturePreparation(const rtabmap::ParametersMap &p):registration_(checked(p)) {}
    rtabmap::SensorData prepare(const rtabmap::SensorData &raw) {
        rtabmap::Signature empty,target(raw);
        registration_.computeTransformationMod(empty,target);
        // Never pass Signature words into odometry: its matching dictionary
        // and geometry are rebuilt normally for each measurement.
        return target.sensorData();
    }
private:
    static rtabmap::ParametersMap checked(const rtabmap::ParametersMap &p) {
        using P=rtabmap::Parameters;
        int strategy=P::defaultRegStrategy(),cor=P::defaultVisCorType(),decimation=P::defaultOdomImageDecimation();
        bool rectified=P::defaultRtabmapImagesAlreadyRectified();
        P::parse(p,P::kRegStrategy(),strategy);P::parse(p,P::kVisCorType(),cor);
        P::parse(p,P::kOdomImageDecimation(),decimation);P::parse(p,P::kRtabmapImagesAlreadyRectified(),rectified);
        if(strategy!=0 || cor!=0 || decimation!=1 || !rectified)
            throw std::runtime_error("feature preparation requires unchanged rectified RGB-D F2M inputs");
        return p;
    }
    rtabmap::RegistrationVis registration_;
};
