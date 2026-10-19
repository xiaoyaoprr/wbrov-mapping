// Offline preparation of a verified closed SVO. The supervisor verifies its
// source hashes before/after this process. No camera, ROS or actuator is opened.
#include "rgbd_spool.hpp"
#include "imu_frame_conversion.hpp"
#include <sl/Camera.hpp>
#include <iostream>
#include <algorithm>
#include <cstring>
#include <limits>
int main(int argc,char **argv) try {
    if(argc!=4)throw std::runtime_error("usage: extract_recorded_rgbd closed_raw_directory first_index last_index");
    const std::filesystem::path root=argv[1];const int first=std::stoi(argv[2]),last=std::stoi(argv[3]);
    sl::InitParameters init;sl::RuntimeParameters runtime;
    if(!init.load((root/"init.json").c_str()) || !runtime.load((root/"runtime.json").c_str()))throw std::runtime_error("recorded settings unavailable");
    init.input.setFromSVOFile((root/"stereo.svo2").c_str());init.svo_real_time_mode=false;init.grab_compute_capping_fps=0;
    init.optional_opencv_calibration_file=(root/"water_calibration.yml").c_str();runtime.enable_depth=true;
    sl::Camera camera;if(camera.open(init)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("recorded depth open failed");
    if(first<0 || last<first || last>=camera.getSVONumberOfFrames())throw std::runtime_error("invalid extraction range");
    wbrov_spool::Writer writer;writer.start((root/"water_calibration.yml").string());
    if(!writer.enabled())throw std::runtime_error("missing output spool directory");
    const auto rotation=camera.getCameraInformation().sensors_configuration.camera_imu_transform.getRotationMatrix();
    zed_imu_conversion::Matrix3 cameraFromImu{};for(int i=0;i<9;++i)cameraFromImu[i]=rotation.r[i];
    sl::Mat rgb,depth;sl::SensorsData sensors;std::uint64_t lastMap=0;
    auto retrieve=[&](int width,int height) {
        const sl::Resolution size(width,height);wbrov_spool::Raster out;out.width=width;out.height=height;
        const auto model=camera.getCameraInformation(size).camera_configuration.calibration_parameters.left_cam;
        out.K={model.fx,model.fy,model.cx,model.cy};out.bgr.resize(width*height*3);out.depth.resize(width*height*4);
        if(camera.retrieveImage(rgb,sl::VIEW::LEFT,sl::MEM::CPU,size)!=sl::ERROR_CODE::SUCCESS ||
           camera.retrieveMeasure(depth,sl::MEASURE::DEPTH,sl::MEM::CPU,size)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("recorded raster unavailable");
        for(int y=0;y<height;++y) {
            const auto *color=rgb.getPtr<sl::uchar1>(sl::MEM::CPU)+y*rgb.getStepBytes();
            const auto *z=depth.getPtr<float>(sl::MEM::CPU)+y*depth.getStepBytes()/4;
            for(int x=0;x<width;++x) {
                const int index=y*width+x;for(int c=0;c<3;++c)out.bgr[3*index+c]=color[4*x+c];
                const double u=(x-model.cx)/model.fx,v=(y-model.cy)/model.fy;
                const double range=z[x]*std::sqrt(1.+u*u+v*v);
                const float value=std::isfinite(z[x]) && range>=.2 && range<=1.5?z[x]:std::numeric_limits<float>::quiet_NaN();
                std::memcpy(out.depth.data()+4*index,&value,4);
            }
        }
        return out;
    };
    camera.setSVOPosition(first);
    for(int index=first;index<=last;++index) {
        if(camera.grab(runtime)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("SVO decode failed");
        wbrov_spool::Frame f;f.sourceNs=camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
        if(camera.getSensorsData(sensors,sl::TIME_REFERENCE::IMAGE)==sl::ERROR_CODE::SUCCESS && sensors.imu.is_available) {
            const auto q=sensors.imu.pose.getOrientation();
            const auto w=sensors.imu.angular_velocity,a=sensors.imu.linear_acceleration;
            const auto c=zed_imu_conversion::convert({q.x,q.y,q.z,q.w},cameraFromImu,{w.x,w.y,w.z},{a.x,a.y,a.z},{},{},{});
            const auto o=c.world_from_camera;f.imuValid=1;f.imu={o.x,o.y,o.z,o.w,c.angular_velocity_camera_rad_s[0],
                c.angular_velocity_camera_rad_s[1],c.angular_velocity_camera_rad_s[2],c.linear_acceleration_camera_m_s2[0],
                c.linear_acceleration_camera_m_s2[1],c.linear_acceleration_camera_m_s2[2]};
            f.low=retrieve(384,240);
            if(!lastMap || f.sourceNs-lastMap>=200000000ULL){f.map=retrieve(960,600);lastMap=f.sourceNs;}
        }
        writer.append(std::move(f));
    }
    writer.finish(true);camera.close();std::cout<<"WBROV_EXTRACTION_COMPLETE "<<last-first+1<<std::endl;return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
