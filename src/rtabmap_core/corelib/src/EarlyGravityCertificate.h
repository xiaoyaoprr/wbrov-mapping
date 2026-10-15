#pragma once
#include <cmath>
#include <cstddef>
#include <string>

// Experimental admission certificate for a small graph, not a replacement for
// Core's three-view, 3-fold holdout, fixed-old and strict deformation checks.
// Fifteen seconds is source time, not a frame count or callback wall time.
namespace wbrov {
struct EarlyGravityCertificate {
    std::size_t nodes=0, gravityNodes=0;
    double sourceSpan=0, beforeMeanDeg=0, beforeMaxDeg=0;
    double alignedMeanMaxDeg=0, alignedMaxDeg=0, maxResidualIncreaseDeg=0;
    bool finite=true;
    bool eligible(std::string & reason) const {
        if(!finite || !std::isfinite(sourceSpan) || !std::isfinite(beforeMeanDeg) ||
           !std::isfinite(beforeMaxDeg) || !std::isfinite(alignedMeanMaxDeg) ||
           !std::isfinite(alignedMaxDeg) || !std::isfinite(maxResidualIncreaseDeg)) {
            reason="non-finite early gravity certificate";return false;
        }
        if(nodes<10 || nodes>=200 || sourceSpan<15.0 || gravityNodes*10<nodes*9) {
            reason="insufficient source-time span or measured gravity coverage";return false;
        }
        if(beforeMeanDeg>2.0 || beforeMaxDeg>5.0 || alignedMeanMaxDeg>2.0 ||
           alignedMaxDeg>5.0 || maxResidualIncreaseDeg>0.5) {
            reason="measured gravity is not stable under all three independent alignments";return false;
        }
        reason="source-time and measured-gravity certificate eligible; full shadow checks still required";
        return true;
    }
};
}
