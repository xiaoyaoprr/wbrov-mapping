// Standalone native probe prepared for the isolated Jetson candidate lane.
// Not built or executed on Windows. Uses real RTABMap inverse/point arithmetic.
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

using namespace rtabmap;
static CameraModel model(const Transform &t) {
    return CameraModel(420., 421., 320., 180., t, 0., cv::Size(640,360));
}
static std::vector<CameraModel> models(int frame) {
    const float d = frame * .0001f;
    return {model(Transform(.12f+d,-.03f,.04f,.01f,.02f,.03f)),
            model(Transform(-.08f,.04f+d,.02f,-.3f,.4f,-.2f)),
            model(Transform(0,0,.01f+d,3.1414f,0,0))};
}
static cv::Point3f point(int i) {
    return cv::Point3f((i%79-39)*.013f,(i%61-30)*.011f,.2f+(i%101)*.009f);
}
static Transform singularNonNull() {
    Transform t = Transform::getIdentity();
    t(2,2) = 0.0f; // Non-null, but determinant zero: CameraModel accepts it.
    assert(!t.isNull() && !t.isInvertible());
    return t;
}
static void equivalence() {
    for(int frame=0; frame<100; ++frame) {
        const auto lastFrameModels = models(frame);
#include "frame_inverse_cache.inc"
        assert(frameCameraInverses.empty() && frameCameraInverseReady.empty());
        for(int i=0; i<900; ++i) {
            const int cameraIndex = (i*7+frame)%3;
            const Transform baseline = lastFrameModels[cameraIndex].localTransform().inverse();
            const Transform &cached = frameCameraInverse(cameraIndex);
            assert(std::memcmp(baseline.data(),cached.data(),12*sizeof(float))==0);
            const cv::Point3f p = point(i);
            const auto a = util3d::transformPoint(p,baseline);
            const auto b = util3d::transformPoint(p,cached);
            assert(std::memcmp(&a.x,&b.x,sizeof(float))==0);
            assert(std::memcmp(&a.y,&b.y,sizeof(float))==0);
            assert(std::memcmp(&a.z,&b.z,sizeof(float))==0);
            const float *storage = cached.data();
            assert(frameCameraInverse(cameraIndex).data()==storage);
        }
    }
    {
        const std::vector<CameraModel> lastFrameModels;
#include "frame_inverse_cache.inc"
        assert(frameCameraInverses.empty() && frameCameraInverseReady.empty());
    }
    {
        const std::vector<CameraModel> lastFrameModels{model(singularNonNull()),model(Transform::getIdentity())};
#include "frame_inverse_cache.inc"
        assert(frameCameraInverse(1).isIdentity()); // Unused singular camera0 must not be inverted.
        assert(frameCameraInverseReady[0]==0 && frameCameraInverseReady[1]==1);
    }
}
static volatile double sink = 0;
static double benchmark(bool cached) {
    double checksum = 0;
    const auto began=std::chrono::steady_clock::now();
    for(int frame=0; frame<400; ++frame) {
        const auto lastFrameModels=models(frame);
#include "frame_inverse_cache.inc"
        for(int i=0; i<600; ++i) {
            const int cameraIndex=i%3;
            const cv::Point3f p=point(i);
            if(cached)checksum+=util3d::transformPoint(p,frameCameraInverse(cameraIndex)).z;
            else checksum+=util3d::transformPoint(p,lastFrameModels[cameraIndex].localTransform().inverse()).z;
        }
    }
    sink=checksum;
    return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
}
int main(int argc,char **argv) {
    // A later runner must require nonzero exit for both singular modes. A fatal
    // assertion/exception remains fail-closed; neither path may accept a pose.
    if(argc==2 && (std::strcmp(argv[1],"singular_baseline")==0 || std::strcmp(argv[1],"singular_candidate")==0)) {
        const std::vector<CameraModel> lastFrameModels{model(singularNonNull())};
#include "frame_inverse_cache.inc"
        if(std::strcmp(argv[1],"singular_candidate")==0)frameCameraInverse(0);
        else lastFrameModels[0].localTransform().inverse();
        return 0; // This is a test failure if reached.
    }
    equivalence();
    std::cout<<"equivalence_passed frames=100 observations=90000\n";
    if(argc==2 && std::strcmp(argv[1],"benchmark")==0) {
        for(int round=0; round<6; ++round) {
            double base=0,candidate=0,baseSum=0,candidateSum=0;
            if(round%2==0) {base=benchmark(false);baseSum=sink;candidate=benchmark(true);candidateSum=sink;}
            else {candidate=benchmark(true);candidateSum=sink;base=benchmark(false);baseSum=sink;}
            assert(std::memcmp(&baseSum,&candidateSum,sizeof(double))==0);
            std::cout<<"round="<<round<<" baseline_ms="<<base<<" candidate_ms="<<candidate<<" checksum="<<baseSum<<std::endl;
        }
    }
}
