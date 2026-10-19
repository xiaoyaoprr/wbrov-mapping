#include "MappingQualityTrace.h"
#include <cassert>
#include <iostream>
int main() {
    using namespace rtabmap;
    rtabmap_slam::MappingContinuityGate::Evidence e;
    e.lost=false;e.finiteMotion=true;e.dimensionsValid=true;
    e.covarianceFinitePD=true;e.covarianceValid=false;e.visualGeometry=true;
    e.qualityReason="covariance, invalid \"axis\"\nnext";
    const auto json=mappingQualityJson(123,true,e);
    assert(json.find("\"covariance_valid\":false")!=std::string::npos);
    assert(json.find("\"visual_geometry\":true")!=std::string::npos);
    assert(json.find("\"source_ns\":123")!=std::string::npos);
    assert(json.find("covariance, invalid \\\"axis\\\"\\u000anext")!=std::string::npos);
    assert(diagnosticCsvString(e.qualityReason)=="\"covariance, invalid \"\"axis\"\" next\"");
    std::cout<<json<<'\n';
}
