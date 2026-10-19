#pragma once
#include <rtabmap_slam/MappingContinuityGate.h>
#include <iomanip>
#include <sstream>
#include <string>
namespace rtabmap {
inline std::string diagnosticJsonString(const std::string &text) {
    std::ostringstream out;out<<'"';
    for(unsigned char c:text) {
        if(c=='"' || c=='\\')out<<'\\'<<c;
        else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c)<<std::dec;
        else out<<c;
    }
    out<<'"';return out.str();
}
inline std::string diagnosticCsvString(const std::string &text) {
    std::string out="\"";
    for(char c:text){if(c=='"')out+='"';out+=(c=='\r'||c=='\n')?' ':c;}
    return out+'"';
}
inline std::string mappingQualityJson(long long source,bool present,
        const rtabmap_slam::MappingContinuityGate::Evidence &e) {
    std::ostringstream out;out<<std::boolalpha<<"{\"source_ns\":"<<source<<",\"input_present\":"<<present
        <<",\"usable\":"<<e.usable()<<",\"lost\":"<<e.lost<<",\"finite_motion\":"<<e.finiteMotion
        <<",\"dimensions_valid\":"<<e.dimensionsValid<<",\"covariance_finite_pd\":"<<e.covarianceFinitePD
        <<",\"covariance_valid\":"<<e.covarianceValid<<",\"visual_geometry\":"<<e.visualGeometry
        <<",\"depth_geometry\":"<<e.depthGeometry<<",\"measured_registration\":"<<e.measuredRegistration
        <<",\"recovery_geometry\":"<<e.recoveryGeometry<<",\"measured_interval_ns\":"<<e.measuredIntervalNs
        <<",\"reason\":"<<diagnosticJsonString(e.qualityReason)<<'}';return out.str();
}
}
