#pragma once
#include <cmath>
#include <cstdint>

// Only reduce redundant Core calls in a registered, writable, healthy map.
// Every recorded frame still runs odometry and the continuity gate. Recovery,
// a new Tag, meaningful cumulative motion, and a one-second revisit always run.
// This selects processing inputs; it cannot grant map-write permission.
inline bool recordedKeyframeDue(std::int64_t source,std::int64_t lastAttempt,
        bool healthyRegistered,bool hasAcceptedReference,bool newTag,
        double translation,double rotationRadians) {
    if(!healthyRegistered || !hasAcceptedReference || newTag || lastAttempt<=0 || source<=lastAttempt)
        return true;
    if(!std::isfinite(translation) || !std::isfinite(rotationRadians))return true;
    return source-lastAttempt>=1000000000LL || translation>=.02 || rotationRadians>=2.*3.14159265358979323846/180.;
}
