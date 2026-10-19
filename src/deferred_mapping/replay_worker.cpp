#include "DeferredMappingSession.h"
#include "imu_frame_conversion.hpp"
#include "rgbd_spool.hpp"
#include "OrderedProcessingLane.h"
#include "RecordedLookahead.h"
#include "RecordedLookaheadWindow.h"
#include "RecordedFeaturePreparation.h"
#include <rtabmap/utilite/ULogger.h>
#include <sl/Camera.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <openssl/evp.h>
#include <atomic>
#include <future>

using namespace rtabmap;
namespace fs=std::filesystem;
static std::atomic<long long> hashNs{0};
static long long elapsedNs(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-start).count();
}
static std::string sha(const fs::path &p) {
    if(!fs::is_regular_file(p) || fs::is_symlink(p))throw std::runtime_error("non-regular recorded input");
    std::ifstream f(p,std::ios::binary);
    const auto context=std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)>(EVP_MD_CTX_new(),EVP_MD_CTX_free);
    if(!context || EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr)!=1)throw std::runtime_error("input hash initialization failed");
    std::array<unsigned char,1024*1024> b{};
    while(f){f.read(reinterpret_cast<char*>(b.data()),b.size());if(EVP_DigestUpdate(context.get(),b.data(),std::size_t(f.gcount()))!=1)throw std::runtime_error("input hash update failed");}
    if(!f.eof())throw std::runtime_error("recorded input hash read failed");
    std::array<unsigned char,EVP_MAX_MD_SIZE> digest{};unsigned size=0;
    if(EVP_DigestFinal_ex(context.get(),digest.data(),&size)!=1 || size!=32)throw std::runtime_error("input hash finalization failed");
    std::ostringstream hex;hex<<std::hex<<std::setfill('0');for(unsigned i=0;i<size;++i)hex<<std::setw(2)<<unsigned(digest[i]);return hex.str();
}
static ParametersMap parameters(const char *path) {
    std::ifstream f(path);if(!f)throw std::runtime_error("parameter file missing");
    ParametersMap p;std::string line;
    while(std::getline(f,line)){const auto at=line.find('=');if(at!=std::string::npos)p[line.substr(0,at)]=line.substr(at+1);}
    return p;
}
static std::map<std::string,std::string> readPins(const cv::FileNode &pins) {
    if(pins.empty() || !pins.isMap())throw std::runtime_error("recording pins missing");
    std::map<std::string,std::string> result;
    for(auto it=pins.begin();it!=pins.end();++it) {
        const std::string name=(*it).name();
        if(fs::path(name).filename()!=fs::path(name))throw std::runtime_error("invalid recording pin name");
        result[name]=std::string(*it);
    }
    return result;
}
static void checkPins(const fs::path &root,const std::map<std::string,std::string> &pins) {
    const auto started=std::chrono::steady_clock::now();
    for(const auto &pin:pins)if(sha(root/pin.first)!=pin.second)throw std::runtime_error("recorded input changed: "+pin.first);
    hashNs+=elapsedNs(started);
}
static std::string spoolIdentity(const cv::FileNode &command) {
    std::ostringstream out;
    for(const char *key:{"directory","data_file","first_source_ns","last_source_ns"})out<<std::string(command[key])<<'\n';
    out<<int(command["sequence"])<<'\n'<<int(command["frames"])<<'\n';
    for(const auto &pin:readPins(command["sha256"]))out<<pin.first<<'='<<pin.second<<'\n';
    return out.str();
}
static SensorData spoolData(const wbrov_spool::Frame &frame,bool low) {
    const auto &imu=frame.imu;bool finite=true;for(double value:imu)finite=finite && std::isfinite(value);
    const double norm=imu[0]*imu[0]+imu[1]*imu[1]+imu[2]*imu[2]+imu[3]*imu[3];
    if(!frame.imuValid || !finite || std::abs(norm-1.)>.002)throw std::runtime_error("invalid recorded IMU attitude");
    const auto &r=low?frame.low:frame.map;if(!r.width)return SensorData();
    const Transform baseFromCamera(0,0,0,-.5,.5,-.5,.5);
    CameraModel model(r.K[0],r.K[1],r.K[2],r.K[3],baseFromCamera,0,cv::Size(r.width,r.height));
    cv::Mat bgr(r.height,r.width,CV_8UC3,const_cast<std::uint8_t*>(r.bgr.data())),rgb;
    if(low)cv::cvtColor(bgr,rgb,cv::COLOR_BGR2GRAY);else rgb=bgr.clone();
    cv::Mat depth(r.height,r.width,CV_32FC1,const_cast<std::uint8_t*>(r.depth.data()));
    SensorData d(rgb,depth.clone(),model,0,frame.sourceNs/1e9);
    d.setIMU(IMU(cv::Vec4d(imu[0],imu[1],imu[2],imu[3]),cv::Mat(),cv::Vec3d(imu[4],imu[5],imu[6]),cv::Mat(),cv::Vec3d(imu[7],imu[8],imu[9]),cv::Mat(),baseFromCamera));
    return d;
}
int main(int argc,char **argv) try {
    if(argc<6 || argc>8)throw std::runtime_error("usage: replay_worker output odom_parameters map_parameters session_sha256 input_identity [cv_threads [feature_prefetch]]");
    const int cvThreads=argc>=7?std::stoi(argv[6]):1;
    const bool featurePrefetch=argc==8 && std::string(argv[7])=="1";
    if(argc==8 && std::string(argv[7])!="0" && !featurePrefetch)throw std::runtime_error("invalid feature prefetch mode");
    if(cvThreads!=1 && cvThreads!=2 && cvThreads!=4)throw std::runtime_error("invalid OpenCV thread budget");
    cv::setNumThreads(cvThreads);cv::setRNGSeed(81018);ULogger::setLevel(ULogger::kWarning);
    auto odomParams=parameters(argv[2]),mapParams=parameters(argv[3]);
    if(odomParams["Vis/MinInliers"]!="15" || odomParams["Odom/GuessMotion"]!="false" ||
       mapParams["Vis/MinInliers"]!="25")throw std::runtime_error("unapproved geometry parameter profile");
    DeferredMappingSession session(mapParams,odomParams,argv[1],argv[4],argv[5]);
    // Diagnostic only: level alone leaves ULogger in kTypeNoLog.
    ULogger::setType(ULogger::kTypeFile,(fs::path(argv[1])/"core.log").string(),false);
    ULogger::setPrintColored(false);
    OrderedProcessingLane mappingLane([]{cv::setRNGSeed(81018);});
    OrderedProcessingLane odometryLane([]{cv::setRNGSeed(81018);});
    // Keep one serialized owner of F2M, but let the control thread admit the
    // next segment as soon as its predecessor's exact ACK has been checked.
    // Computing every lookahead pose synchronously here used to leave Core
    // idle until the entire following segment had finished odometry.
    auto estimate=[&](SensorData low) {
        auto promise=std::make_shared<std::promise<DeferredMappingSession::MeasuredFrame>>();
        auto ready=promise->get_future();
        odometryLane.submit([&,promise,low=std::move(low)]() mutable {
            try {promise->set_value(session.estimate(std::move(low)));}
            catch(...) {promise->set_exception(std::current_exception());throw;}
        });
        return ready;
    };
    std::unique_ptr<RecordedFeaturePreparation> featurePreparation;
    std::unique_ptr<OrderedProcessingLane> featureLane;
    if(featurePrefetch) {
        featurePreparation.reset(new RecordedFeaturePreparation(odomParams));
        featureLane.reset(new OrderedProcessingLane([]{cv::setRNGSeed(81018);},3));
    }
    RecordedLookaheadWindow<std::future<DeferredMappingSession::MeasuredFrame>> lookahead;
    long long readNs=0,copyNs=0,controlWaitNs=0,drainNs=0,featureNs=0;int prefetchSegments=0;std::size_t maximumLookahead=0;
    struct PreparedFrame {wbrov_spool::Frame frame;SensorData low;long long featureNs=0;};
    auto frames=[&](std::istream &input,int count,bool compressed,bool prepare,auto consume) {
        if(!prepare || !featurePrefetch) {
            for(int i=0;i<count;++i) {
                const auto start=std::chrono::steady_clock::now();auto frame=wbrov_spool::readRecord(input,compressed);readNs+=elapsedNs(start);
                consume(i,frame,SensorData());
            }
            return;
        }
        std::deque<std::future<PreparedFrame>> ready;int read=0;
        auto schedule=[&] {
            const auto start=std::chrono::steady_clock::now();auto frame=wbrov_spool::readRecord(input,compressed);readNs+=elapsedNs(start);
            auto promise=std::make_shared<std::promise<PreparedFrame>>();ready.push_back(promise->get_future());
            featureLane->submit([&,promise,frame=std::move(frame)]() mutable {
                try {
                    PreparedFrame result;result.frame=std::move(frame);
                    if(result.frame.imuValid) {
                        const auto start=std::chrono::steady_clock::now();
                        result.low=featurePreparation->prepare(spoolData(result.frame,true));result.featureNs=elapsedNs(start);
                    }
                    promise->set_value(std::move(result));
                }catch(...){promise->set_exception(std::current_exception());}
            });++read;
        };
        while(read<std::min(count,3))schedule();
        for(int i=0;i<count;++i) {
            auto prepared=ready.front().get();ready.pop_front();
            if(read<count)schedule();
            featureNs+=prepared.featureNs;consume(i,prepared.frame,prepared.low);
        }
    };
    std::string line;
    for(;;) {
        const auto waiting=std::chrono::steady_clock::now();const bool received=bool(std::getline(std::cin,line));
        controlWaitNs+=elapsedNs(waiting);if(!received)break;
        if(line=="CLOSE")break;
        cv::FileStorage command(line,cv::FileStorage::READ);
        if(!command.isOpened())throw std::runtime_error("input descriptor unavailable");
        const fs::path root=std::string(command["directory"]);
        const auto pins=readPins(command["sha256"]);checkPins(root,pins);
        if(std::string(command["kind"])=="rgbd_spool") {
            const int sequence=int(command["sequence"]);
            const bool measuredAhead=lookahead.ready();
            if(measuredAhead)lookahead.requireFront(sequence,spoolIdentity(command.root()));
            std::map<long long,std::vector<AcceptedTagCornerObservation>> tags;
            for(const auto &entry:command["tag_observations"]) {
                AcceptedTagCornerObservation tag;tag.tagId=int(entry["tag_id"]);
                tag.imageSha256=std::string(entry["image_sha256"]);tag.pixelRecordSha256=std::string(entry["pixel_record_sha256"]);
                tag.cornerOrder=std::string(entry["corner_order"]);
                const auto corners=entry["corners_xy"];
                if(corners.size()!=8)throw std::runtime_error("invalid measured corner count");
                for(int i=0;i<8;++i) {tag.cornersXY[i]=double(corners[i]);if(!std::isfinite(tag.cornersXY[i]))throw std::runtime_error("nonfinite recorded corner");}
                tags[std::stoll(std::string(entry["source_ns"]))].push_back(tag);
            }
            const fs::path name=std::string(command["data_file"]);
            if(name.filename()!=name)throw std::runtime_error("invalid spool input name");
            std::ifstream input(root/name,std::ios::binary);const bool compressed=wbrov_spool::readSegmentHeader(input);
            const int count=int(command["frames"]);
            if(count<1 || count>10000)throw std::runtime_error("invalid spool frame count");
            const auto first=std::stoll(std::string(command["first_source_ns"])),last=std::stoll(std::string(command["last_source_ns"]));
            frames(input,count,compressed,!measuredAhead,[&](int i,const auto &frame,const SensorData &preparedLow) {
                if((i==0 && frame.sourceNs!=std::uint64_t(first)) || (i==count-1 && frame.sourceNs!=std::uint64_t(last)))
                    throw std::runtime_error("spool source/index mismatch");
                if(!frame.imuValid){mappingLane.submit([&,stamp=frame.sourceNs]{session.missing(stamp,"capture lacked image-time IMU");});return;}
                auto measured=measuredAhead?lookahead.take(sequence,frame.sourceNs).get():estimate(preparedLow.imageRaw().empty()?spoolData(frame,true):preparedLow).get();
                const auto copying=std::chrono::steady_clock::now();auto map=spoolData(frame,false);copyNs+=elapsedNs(copying);
                mappingLane.submit([&,measured=std::move(measured),map=std::move(map),stamp=frame.sourceNs,observations=tags[frame.sourceNs]]{
                    session.consume(map,stamp,observations,measured);
                });
            });
            if(input.peek()!=std::char_traits<char>::eof())throw std::runtime_error("unaccounted spool suffix");
            if(measuredAhead)lookahead.finish(sequence);
            // Core owns its checkpoint and exact ACK. The frontend can estimate
            // the next SEALED segment while that job finishes. No future Core
            // writes run before the supervisor verifies the prior receipt.
            mappingLane.submit([&,root,pins,sequence=int(command["sequence"]),last] {
                checkPins(root,pins);session.checkpoint();
                if(session.source()!=last)throw std::runtime_error("checkpoint source mismatch");
                std::cout<<"WBROV_SEGMENT_COMPLETE "<<sequence<<' '<<last<<std::endl;
            });
            std::vector<cv::FileNode> following;
            const auto window=command["odometry_lookahead_window"];
            if(!window.empty()) {
                if(!window.isSeq() || window.size()>3)throw std::runtime_error("invalid lookahead window descriptor");
                for(const auto &next:window)following.push_back(next);
            }else if(!command["odometry_lookahead"].empty())following.push_back(command["odometry_lookahead"]);
            int nextSequence=sequence;long long previousLast=last;
            for(const auto &next:following) {
                ++nextSequence;
                const int nextCount=int(next["frames"]);
                const auto nextLast=std::stoll(std::string(next["last_source_ns"]));
                if(std::string(next["kind"])!="rgbd_spool" || int(next["sequence"])!=nextSequence ||
                   nextCount<1 || nextCount>64 || std::stoll(std::string(next["first_source_ns"]))<=previousLast ||
                   nextLast<std::stoll(std::string(next["first_source_ns"])))
                    throw std::runtime_error("invalid odometry lookahead interval");
                const fs::path nextRoot=std::string(next["directory"]),nextName=std::string(next["data_file"]);
                if(nextName.filename()!=nextName)throw std::runtime_error("invalid lookahead filename");
                const auto nextPins=readPins(next["sha256"]);
                for(const char *key:{"metadata.json","init.json","runtime.json","water_calibration.yml","camera_imu_extrinsics.json","sdk_version.txt"})
                    if(nextPins.at(key)!=pins.at(key))throw std::runtime_error("lookahead configuration changed");
                if(lookahead.contains(nextSequence)) {
                    lookahead.require(nextSequence,spoolIdentity(next));previousLast=nextLast;continue;
                }
                ++prefetchSegments;
                checkPins(nextRoot,nextPins);lookahead.begin(sequence,nextSequence,spoolIdentity(next));
                maximumLookahead=std::max(maximumLookahead,lookahead.size());
                std::ifstream future(nextRoot/nextName,std::ios::binary);const bool futureCompressed=wbrov_spool::readSegmentHeader(future);
                long long previous=previousLast;
                frames(future,nextCount,futureCompressed,true,[&](int i,const auto &frame,const SensorData &preparedLow) {
                    if(frame.sourceNs<=std::uint64_t(previous) ||
                       (i==0 && frame.sourceNs!=std::stoull(std::string(next["first_source_ns"]))) ||
                       (i==nextCount-1 && frame.sourceNs!=std::stoull(std::string(next["last_source_ns"]))))
                        throw std::runtime_error("lookahead source mismatch");
                    previous=frame.sourceNs;
                    if(frame.imuValid)lookahead.add(nextSequence,frame.sourceNs,estimate(preparedLow.imageRaw().empty()?spoolData(frame,true):preparedLow));
                });
                if(future.peek()!=std::char_traits<char>::eof())throw std::runtime_error("unaccounted lookahead suffix");
                checkPins(nextRoot,nextPins);
                previousLast=nextLast;
            }
            const auto draining=std::chrono::steady_clock::now();mappingLane.drain();drainNs+=elapsedNs(draining);
            const fs::path timing=fs::path(argv[1])/"frontend_timings.tmp";std::ofstream times(timing);
            times<<"{\"input_read_ms\":"<<readNs/1e6<<",\"map_copy_ms\":"<<copyNs/1e6
                 <<",\"input_hash_ms\":"<<hashNs.load()/1e6<<",\"control_wait_ms\":"<<controlWaitNs/1e6
                 <<",\"mapping_drain_ms\":"<<drainNs/1e6<<",\"feature_preparation_ms\":"<<featureNs/1e6
                 <<",\"feature_prefetch_enabled\":"<<(featurePrefetch?"true":"false")<<",\"prefetched_segments\":"<<prefetchSegments
                 <<",\"maximum_lookahead_segments\":"<<maximumLookahead<<"}\n";
            times.close();if(!times)throw std::runtime_error("frontend timing ledger failed");
            fs::rename(timing,fs::path(argv[1])/"frontend_timings.json");
            continue;
        }
        if(lookahead.ready())throw std::runtime_error("prepared spool replaced by another input kind");
        const std::string expectedVersion=command["sdk_version"];
        if(expectedVersion!=sl::Camera::getSDKVersion().c_str())throw std::runtime_error("recording SDK mismatch");
        const int first=int(command["first_index"]),last=int(command["last_index"]);
        const auto expectedFirst=std::stoll(std::string(command["first_source_ns"]));
        const auto expectedLast=std::stoll(std::string(command["last_source_ns"]));
        sl::InitParameters init;sl::RuntimeParameters runtime;
        if(!init.load((root/"init.json").c_str()) || !runtime.load((root/"runtime.json").c_str()))throw std::runtime_error("saved SDK config failed");
        init.input.setFromSVOFile((root/"stereo.svo2").c_str());init.svo_real_time_mode=false;
        init.optional_opencv_calibration_file=(root/"water_calibration.yml").c_str();init.grab_compute_capping_fps=0;
        runtime.enable_depth=true;sl::Camera camera;
        if(camera.open(init)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("SVO depth open failed");
        if(first<0 || last<first || last>=camera.getSVONumberOfFrames())throw std::runtime_error("invalid recorded interval");
        const Transform baseFromCamera(0,0,0,-.5,.5,-.5,.5);
        const sl::Resolution lowSize(384,240),mapSize(960,600);
        auto model=[&](sl::Resolution size) {
            auto c=camera.getCameraInformation(size).camera_configuration.calibration_parameters.left_cam;
            return CameraModel(c.fx,c.fy,c.cx,c.cy,baseFromCamera,0,cv::Size(size.width,size.height));
        };
        const auto lowModel=model(lowSize),mapModel=model(mapSize);
        const auto rotation=camera.getCameraInformation().sensors_configuration.camera_imu_transform.getRotationMatrix();
        zed_imu_conversion::Matrix3 cameraFromImu{};for(int i=0;i<9;++i)cameraFromImu[i]=rotation.r[i];
        sl::Mat pixels,depth;sl::SensorsData sensors;camera.setSVOPosition(first);
        auto retrieve=[&](sl::Resolution size,const CameraModel &m,long long stamp,bool mono) {
            if(camera.retrieveImage(pixels,sl::VIEW::LEFT,sl::MEM::CPU,size)!=sl::ERROR_CODE::SUCCESS ||
               camera.retrieveMeasure(depth,sl::MEASURE::DEPTH,sl::MEM::CPU,size)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("recorded RGB-D retrieve failed");
            cv::Mat rgba(size.height,size.width,CV_8UC4,pixels.getPtr<sl::uchar1>(sl::MEM::CPU),pixels.getStepBytes()),rgb;
            cv::cvtColor(rgba,rgb,mono?cv::COLOR_BGRA2GRAY:cv::COLOR_BGRA2BGR);
            cv::Mat d(size.height,size.width,CV_32FC1,depth.getPtr<float>(sl::MEM::CPU),depth.getStepBytes());d=d.clone();
            for(int y=0;y<d.rows;++y)for(int x=0;x<d.cols;++x)if(!std::isfinite(d.at<float>(y,x)) || d.at<float>(y,x)<.2f || d.at<float>(y,x)>1.5f)d.at<float>(y,x)=0;
            SensorData data(rgb,d,m,0,stamp/1e9);
            const auto q=sensors.imu.pose.getOrientation();const auto w=sensors.imu.angular_velocity;const auto a=sensors.imu.linear_acceleration;
            const auto converted=zed_imu_conversion::convert({q.x,q.y,q.z,q.w},cameraFromImu,{w.x,w.y,w.z},{a.x,a.y,a.z},{},{},{});
            const auto qi=converted.world_from_camera;
            data.setIMU(IMU(cv::Vec4d(qi.x,qi.y,qi.z,qi.w),cv::Mat(),cv::Vec3d(converted.angular_velocity_camera_rad_s.data()),cv::Mat(),cv::Vec3d(converted.linear_acceleration_camera_m_s2.data()),cv::Mat(),baseFromCamera));
            return data;
        };
        for(int index=first;index<=last;++index) {
            if(camera.grab(runtime)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("recorded frame decode failed");
            const long long stamp=camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
            if((index==first && stamp!=expectedFirst) || (index==last && stamp!=expectedLast))throw std::runtime_error("recording index/source mismatch");
            if(camera.getSensorsData(sensors,sl::TIME_REFERENCE::IMAGE)!=sl::ERROR_CODE::SUCCESS || !sensors.imu.is_available) {
                session.missing(stamp,"image-synchronized IMU unavailable");continue;
            }
            const auto low=retrieve(lowSize,lowModel,stamp,true),map=retrieve(mapSize,mapModel,stamp,false);
            session.consume(map,stamp,{},estimate(low).get());
        }
        camera.close();checkPins(root,pins);session.checkpoint();
        std::cout<<"WBROV_SEGMENT_COMPLETE "<<int(command["sequence"])<<' '<<session.source()<<std::endl;
    }
    if(!std::cin.eof() && !std::cin.good())throw std::runtime_error("worker control input failed");
    if(lookahead.ready())throw std::runtime_error("close before prepared recording input was consumed");
    odometryLane.close();mappingLane.close();session.close();std::cout<<"WBROV_WORKER_CLOSED"<<std::endl;return 0;
}catch(const std::exception &e){std::cerr<<"WBROV_WORKER_FAILED "<<e.what()<<std::endl;return 1;}
