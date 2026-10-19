// Offline front-end comparison only. No ROS publishers, map mutation or hardware command path.
#include <sl/Camera.hpp>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/utilite/ULogger.h>
#include "imu_frame_conversion.hpp"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <array>
#include <chrono>
#include <stdexcept>
int main(int argc,char **argv) try {
 if(argc!=6)return 2;
 const std::string root=argv[1],output=argv[2];int first=std::stoi(argv[3]),last=std::stoi(argv[4]);
 std::ifstream parameters(argv[5]);if(!parameters)throw std::runtime_error("parameter file missing");
 rtabmap::ParametersMap p;std::string line;
 while(std::getline(parameters,line)){auto at=line.find('=');if(at!=std::string::npos)p[line.substr(0,at)]=line.substr(at+1);}
 p["Odom/FillInfoData"]="true";
 ULogger::setLevel(ULogger::kWarning);
 std::array<std::unique_ptr<rtabmap::Odometry>,3> odom;
 for(auto &o:odom)o.reset(rtabmap::Odometry::create(p));
 sl::InitParameters init;sl::RuntimeParameters runtime;
 if(!init.load((root+"/init.json").c_str())||!runtime.load((root+"/runtime.json").c_str()))throw std::runtime_error("saved configuration load failed");
 init.input.setFromSVOFile((root+"/stereo.svo2").c_str());init.svo_real_time_mode=false;
 init.optional_opencv_calibration_file=(root+"/water_calibration.yml").c_str();init.grab_compute_capping_fps=0;
 runtime.enable_depth=true;sl::Camera camera;
 if(camera.open(init)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("SVO open failed");
 if(first<0||last<first||last>=camera.getSVONumberOfFrames())throw std::runtime_error("invalid frame interval");
 const sl::Resolution size(384,240);const auto info=camera.getCameraInformation(size);
 const auto c=info.camera_configuration.calibration_parameters.left_cam;
 const rtabmap::Transform baseFromCamera(0,0,0,-.5,.5,-.5,.5);
 rtabmap::CameraModel model(c.fx,c.fy,c.cx,c.cy,baseFromCamera,0,cv::Size(384,240));
 auto rotation=info.sensors_configuration.camera_imu_transform.getRotationMatrix();
 zed_imu_conversion::Matrix3 cameraFromImu{};for(int i=0;i<9;++i)cameraFromImu[i]=rotation.r[i];
 std::ofstream csv(output+"/odometry.csv"),rejected(output+"/rejected_inputs.csv");
 if(!csv||!rejected)throw std::runtime_error("output unavailable");
 rejected<<"index,source_ns,reason\n";
 csv<<"stream,index,source_ns,lost,inliers,matches,interval_s,compute_ms,x,y,z\n"<<std::setprecision(14);
 sl::Mat pixels,depth;sl::SensorsData sensors;std::array<std::uint64_t,3> selected{};std::uint64_t previous=0;
 std::array<int,3> processed{},lost{};int imuMissing=0;
 const char *names[]={"all_recorded_frames","at_most_20hz","at_most_1hz"};
 const std::uint64_t periods[]={0,50000000,1000000000};camera.setSVOPosition(first);
 for(int index=first;index<=last;++index){
  if(camera.grab(runtime)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("decode/depth failed");
  const auto stamp=camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
  if(previous&&stamp<=previous)throw std::runtime_error("source timestamps not increasing");
  previous=stamp;
  const auto sensorStatus=camera.getSensorsData(sensors,sl::TIME_REFERENCE::IMAGE);
  if(sensorStatus!=sl::ERROR_CODE::SUCCESS||!sensors.imu.is_available){
   ++imuMissing;rejected<<index<<','<<stamp<<",image_synchronized_imu_unavailable_status_"<<static_cast<int>(sensorStatus)<<'\n';
   continue; // Preserve the explicit gap; never synthesize orientation or relabel source time.
  }
  if(camera.retrieveImage(pixels,sl::VIEW::LEFT,sl::MEM::CPU,size)!=sl::ERROR_CODE::SUCCESS||camera.retrieveMeasure(depth,sl::MEASURE::DEPTH,sl::MEM::CPU,size)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("RGB-D retrieval failed");
  cv::Mat rgba(240,384,CV_8UC4,pixels.getPtr<sl::uchar1>(sl::MEM::CPU),pixels.getStepBytes()),gray;
  cv::cvtColor(rgba,gray,cv::COLOR_BGRA2GRAY);
  cv::Mat distance(240,384,CV_32FC1,depth.getPtr<float>(sl::MEM::CPU),depth.getStepBytes());
  auto q=sensors.imu.pose.getOrientation();auto w=sensors.imu.angular_velocity;auto a=sensors.imu.linear_acceleration;
  auto imu=zed_imu_conversion::convert({q.x,q.y,q.z,q.w},cameraFromImu,{w.x,w.y,w.z},{a.x,a.y,a.z},{},{},{});
  auto qi=imu.world_from_camera;
  for(int k=0;k<3;++k){
   if(selected[k]&&stamp-selected[k]<periods[k])continue;
   selected[k]=stamp;
   rtabmap::SensorData data(gray.clone(),distance.clone(),model,index+1,stamp/1e9);
   data.setIMU(rtabmap::IMU(cv::Vec4d(qi.x,qi.y,qi.z,qi.w),cv::Mat(),cv::Vec3d(imu.angular_velocity_camera_rad_s.data()),cv::Mat(),cv::Vec3d(imu.linear_acceleration_camera_m_s2.data()),cv::Mat(),baseFromCamera));
   rtabmap::OdometryInfo measurement;auto begin=std::chrono::steady_clock::now();auto pose=odom[k]->process(data,&measurement);
   double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
   ++processed[k];lost[k]+=pose.isNull();csv<<names[k]<<','<<index<<','<<stamp<<','<<pose.isNull()<<','<<measurement.reg.inliers<<','<<measurement.reg.matches<<','<<measurement.interval<<','<<ms;
   if(pose.isNull())csv<<",,,\n";else csv<<','<<pose.x()<<','<<pose.y()<<','<<pose.z()<<'\n';
  }
  if(index%100==0){csv.flush();std::cout<<"progress="<<index<<'/'<<last<<std::endl;}
 }
 camera.close();csv.close();rejected.close();
 std::cout<<"{\"scope\":\"offline RGB-D/IMU odometry only; no main-map recovery or accuracy acceptance\",\"imu_missing_frames\":"<<imuMissing<<",\"results\":[";
 for(int k=0;k<3;++k)std::cout<<(k?",":"")<<"{\"stream\":\""<<names[k]<<"\",\"processed\":"<<processed[k]<<",\"lost\":"<<lost[k]<<'}';
 std::cout<<"]}"<<std::endl;return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
