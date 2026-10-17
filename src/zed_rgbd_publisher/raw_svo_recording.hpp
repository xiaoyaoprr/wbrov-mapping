#ifndef WBROV_RAW_SVO_RECORDING_HPP
#define WBROV_RAW_SVO_RECORDING_HPP
#include <sl/Camera.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <memory>
#include <sstream>
#include <chrono>
#include "bounded_async_writer.hpp"

// Recording branches before IMU, depth, odometry and map acceptance. The SDK
// records stereo images and sensor data, not the computed depth/point cloud.
// A closed file is only a candidate: the post-stop verifier checks SVO decoding,
// counts and metadata hashes before reporting a complete recording.
class RawSvoRecording {
public:
  void start(sl::Camera &camera, const sl::InitParameters &init,
             const sl::RuntimeParameters &runtime, const std::string &calibration) {
    const char *directory=std::getenv("WBROV_RAW_RECORDING_DIR");
    if(!directory || !*directory)return;
    root_=directory;
    if(!root_.is_absolute() || std::filesystem::exists(root_))
      throw std::runtime_error("raw recording requires a new absolute directory");
    std::filesystem::create_directory(root_);
    check_space();
    if(calibration.empty())throw std::runtime_error("raw recording requires the external calibration");
    std::filesystem::copy_file(calibration,root_/"water_calibration.yml");
    if(!init.save((root_/"init.json").string().c_str()) ||
       !runtime.save((root_/"runtime.json").string().c_str()))
      throw std::runtime_error("cannot persist SDK depth configuration");
    const sl::RecordingParameters parameters((root_/"stereo.svo2").string().c_str(),sl::SVO_COMPRESSION_MODE::H265);
    if(!parameters.save((root_/"recording.json").string().c_str()))
      throw std::runtime_error("cannot persist SVO recording configuration");
    { std::ofstream version(root_/"sdk_version.txt");version<<sl::Camera::getSDKVersion().c_str()<<'\n';
      if(!version)throw std::runtime_error("cannot persist SDK version"); }
    { const auto info=camera.getCameraInformation();
      const auto transform=info.sensors_configuration.camera_imu_transform;
      const auto q=transform.getOrientation();const auto t=transform.getTranslation();
      std::ofstream f(root_/"camera_imu_extrinsics.json");
      f<<std::setprecision(17)<<"{\"camera_serial\":"<<info.serial_number<<",\"camera_from_imu_quaternion_xyzw\":["
       <<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<"],\"translation_m\":["<<t.x<<','<<t.y<<','<<t.z
       <<"],\"depth_enabled_schedule\":\"odom_due\",\"recording_before_map_gating\":true}\n";
      if(!f)throw std::runtime_error("cannot persist camera/IMU extrinsics"); }
    ledger_.open(root_/"capture.jsonl");
    if(!ledger_)throw std::runtime_error("cannot open recording ledger");
    camera_=&camera;
    write_status("starting",false);
    const auto status=camera.enableRecording(parameters);
    if(status!=sl::ERROR_CODE::SUCCESS) {
      write_status("enable_failed",false);camera_=nullptr;
      throw std::runtime_error(std::string("SVO recording failed: ")+sl::toString(status).c_str());
    }
    enabled_=true;
    write_status("recording",false);
    journal_=std::make_unique<BoundedAsyncWriter<JournalRecord>>(512,[this](const JournalRecord &record) {
      ledger_<<record.line;
      if(!ledger_)throw std::runtime_error("recording ledger write failed");
      if(!record.status.empty()) {
        ledger_.flush();
        if(!ledger_)throw std::runtime_error("recording ledger flush failed");
        check_space();publish_status(record.status,false);
      }
    });
  }
  void grabbed(std::uint64_t stamp) {
    if(!enabled_)return;
    const auto statusBegin=std::chrono::steady_clock::now();
    const auto status=camera_->getRecordingStatus();
    lastSdkStatusMs_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-statusBegin).count();
    ingested_=status.number_frames_ingested;encoded_=status.number_frames_encoded;
    // status refers to the current SDK write attempt: duplicate grabs and
    // asynchronous encoder latency can report false even when every unique
    // source frame is ultimately in the closed SVO. Preserve that diagnostic;
    // verify final indexed coverage against ingestion and unique source stamps.
    if(!status.status)++statusFalseSamples_;
    if(!status.is_recording || status.is_paused)++writeErrors_;
    ++grabs_;
    if(stamp>lastStamp_) {
      if(!firstStamp_)firstStamp_=stamp;
      lastStamp_=stamp;++uniqueStamps_;
    } else if(stamp==lastStamp_)++duplicateGrabs_;
    else ++writeErrors_; // backwards source time cannot be a complete sequence
    std::ostringstream line;
    line<<"{\"stamp_ns\":"<<stamp<<",\"ingested\":"<<ingested_
           <<",\"encoded\":"<<encoded_<<",\"write_ok\":"<<(status.status?"true":"false")
           <<",\"recording\":"<<(status.is_recording?"true":"false")
           <<",\"paused\":"<<(status.is_paused?"true":"false")<<"}\n";
    // No filesystem operation runs on this capture path. Keep SDK calls on
    // the camera thread; only immutable strings cross to the journal worker.
    try {journal_->append({line.str(),grabs_%30==0?status_text("recording",false):std::string()});}
    catch(...) {++writeErrors_;throw;}

  }
  double last_sdk_status_ms() const {return lastSdkStatusMs_;}
  void grab_failed() {if(enabled_)++grabErrors_;}
  void finish(bool clean) {
    if(!enabled_)return;
    // disableRecording closes/flushed the SVO. Capture ledger counters remain
    // pre-close SDK counters and are compared against the decoded file later.
    const auto status=camera_->getRecordingStatus();
    ingested_=status.number_frames_ingested;encoded_=status.number_frames_encoded;
    camera_->disableRecording();enabled_=false;
    std::exception_ptr failure;
    try {if(journal_)journal_->close();}catch(...) {failure=std::current_exception();++writeErrors_;}
    journal_.reset();
    ledger_.flush();
    if(!ledger_)++writeErrors_;
    ledger_.close();
    write_status("closed",clean && !failure && writeErrors_==0);
    if(failure)std::rethrow_exception(failure);
  }
  ~RawSvoRecording() {try {finish(false);}catch(...) {}}
private:
  void check_space() const {
    if(std::filesystem::space(root_).available<2ULL*1024*1024*1024)
      throw std::runtime_error("raw recording stopped: less than 2 GiB reserve");
  }
  struct JournalRecord {std::string line,status;};
  std::string status_text(const char *state,bool clean) const {
    std::ostringstream f;
      f<<"{\"schema\":\"wbrov.raw-recording.v1\",\"state\":\""<<state
       <<"\",\"clean_shutdown\":"<<(clean?"true":"false")
       <<",\"complete\":false,\"requires_post_stop_verification\":true"
       <<",\"first_source_ns\":"<<firstStamp_<<",\"last_source_ns\":"<<lastStamp_
       <<",\"grab_calls\":"<<grabs_<<",\"unique_computed_stamps\":"<<uniqueStamps_
       <<",\"sdk_frames_ingested\":"<<ingested_<<",\"sdk_frames_encoded_before_close\":"<<encoded_
       <<",\"status_false_samples\":"<<statusFalseSamples_<<",\"duplicate_grabs\":"<<duplicateGrabs_
       <<",\"grab_errors\":"<<grabErrors_<<",\"write_errors\":"<<writeErrors_<<"}\n";
    if(!f)throw std::runtime_error("recording status serialization failed");
    return f.str();
  }
  void write_status(const char *state,bool clean) {publish_status(status_text(state,clean),true);}
  void publish_status(const std::string &text,bool durable) {
    const auto temporary=root_/"status.tmp";
    {std::ofstream f(temporary);f<<text;f.flush();
      if(!f)throw std::runtime_error("recording status write failed");}
    if(durable) {
      const int fd=::open(temporary.c_str(),O_RDONLY);
      if(fd<0)throw std::runtime_error("recording status sync open failed");
      const int rc=::fsync(fd);::close(fd);
      if(rc!=0)throw std::runtime_error("recording status sync failed");
    }
    std::filesystem::rename(temporary,root_/"status.json");
  }
  sl::Camera *camera_=nullptr;
  std::filesystem::path root_;
  std::ofstream ledger_;
  std::unique_ptr<BoundedAsyncWriter<JournalRecord>> journal_;
  double lastSdkStatusMs_=0.;
  bool enabled_=false;
  std::uint64_t firstStamp_=0,lastStamp_=0,grabs_=0,uniqueStamps_=0,grabErrors_=0,writeErrors_=0;
  std::uint64_t statusFalseSamples_=0,duplicateGrabs_=0;
  int ingested_=0,encoded_=0;
};
#endif
