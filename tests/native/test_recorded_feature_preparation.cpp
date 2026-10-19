#include "rgbd_spool.hpp"
#include "RecordedFeaturePreparation.h"
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/utilite/ULogger.h>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cstring>
using namespace rtabmap;
static void check(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
static SensorData sensor(const wbrov_spool::Frame &f) {
 const auto &r=f.low;cv::Mat bgr(r.height,r.width,CV_8UC3,const_cast<std::uint8_t*>(r.bgr.data())),mono;
 cv::cvtColor(bgr,mono,cv::COLOR_BGR2GRAY);
 cv::Mat depth(r.height,r.width,CV_32FC1,const_cast<std::uint8_t*>(r.depth.data()));
 CameraModel model(r.K[0],r.K[1],r.K[2],r.K[3],Transform(0,0,0,-.5,.5,-.5,.5),0,cv::Size(r.width,r.height));
 return SensorData(mono,depth.clone(),model,0,f.sourceNs/1e9);
}
static void same(const SensorData &a,const SensorData &b) {
 check(a.keypoints().size()==b.keypoints().size(),"feature count changed");
 check(a.keypoints3D().size()==b.keypoints3D().size(),"depth count changed");
 check(a.descriptors().size()==b.descriptors().size(),"descriptor size changed");
 if(!a.descriptors().empty())check(cv::norm(a.descriptors(),b.descriptors(),cv::NORM_INF)==0,"descriptors changed");
 for(std::size_t i=0;i<a.keypoints().size();++i){
  const auto &x=a.keypoints()[i],&y=b.keypoints()[i];
  check(x.pt==y.pt && x.size==y.size && x.angle==y.angle && x.response==y.response && x.octave==y.octave,"keypoint changed");
 }
 for(std::size_t i=0;i<a.keypoints3D().size();++i)
  check(std::memcmp(&a.keypoints3D()[i],&b.keypoints3D()[i],sizeof(cv::Point3f))==0,"keypoint depth changed");
}
int main(int argc,char **argv)try{
 for(auto change:ParametersMap{{"Reg/Strategy","1"},{"Vis/CorType","1"},{"Odom/ImageDecimation","2"},{"Rtabmap/ImagesAlreadyRectified","false"}}) {
  bool rejected=false;try {RecordedFeaturePreparation invalid(ParametersMap{change});}catch(const std::runtime_error &){rejected=true;}
  check(rejected,"unsupported feature preparation profile accepted");
 }
 check(argc==4,"usage: probe params spool_dir segment_count");cv::setNumThreads(1);ULogger::setLevel(ULogger::kError);
 ParametersMap p;std::ifstream file(argv[1]);std::string line;
 while(std::getline(file,line)){auto n=line.find('=');if(n!=std::string::npos)p[line.substr(0,n)]=line.substr(n+1);}
 RegistrationVis raw(p),reuser(p);RecordedFeaturePreparation preparer(p);double ms=0;int frames=0,empty=0;
 for(int segment=0;segment<std::stoi(argv[3]);++segment){
  char name[64];std::snprintf(name,sizeof(name),"/segment-%08d.frames",segment);
  std::ifstream input(std::string(argv[2])+name,std::ios::binary);char magic[11];input.read(magic,11);
  check(input && std::string(magic,11)=="WBROVRGBD1\n","spool magic");
  while(input.peek()!=std::char_traits<char>::eof()){
   auto frame=wbrov_spool::readFrame(input);auto d=sensor(frame);
   Signature original(d),nothing;cv::setRNGSeed(81018);
   raw.computeTransformationMod(original,nothing);
   Signature target(d);cv::setRNGSeed(81018);
   auto start=std::chrono::steady_clock::now();
   target=Signature(preparer.prepare(d));
   ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
   same(original.sensorData(),target.sensorData());
   Signature again(target.sensorData()),blank;reuser.computeTransformationMod(blank,again);
   same(original.sensorData(),again.sensorData());
   ++frames;if(target.sensorData().keypoints().empty())++empty;
  }
 }
 std::cout<<"{\"passed\":true,\"frames\":"<<frames<<",\"empty\":"<<empty<<",\"preparation_ms\":"<<ms<<",\"mean_ms\":"<<ms/frames<<"}\n";
 return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
