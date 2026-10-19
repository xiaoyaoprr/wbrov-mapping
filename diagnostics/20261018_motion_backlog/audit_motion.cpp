// Read-only evidence probe: feature overlap is diagnostic, not a pose or map acceptance test.
#include <sl/Camera.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
struct Frame {std::uint64_t stamp;cv::Mat descriptors;};
int matches(const cv::Mat &a,const cv::Mat &b) {
 if(a.empty()||b.rows<2)return 0;
 std::vector<std::vector<cv::DMatch>> pairs;
 cv::BFMatcher(cv::NORM_HAMMING).knnMatch(a,b,pairs,2);int count=0;
 for(const auto &p:pairs)if(p.size()==2&&p[0].distance<.75*p[1].distance)++count;
 return count;
}
int main(int argc,char **argv) try {
 if(argc!=6)return 2;
 const std::string root=argv[1],out=argv[2];const int start=std::stoi(argv[3]),end=std::stoi(argv[4]);
 const int step=std::stoi(argv[5]);if(start<0||end<start||step<1)return 2;
 sl::InitParameters init;sl::RuntimeParameters runtime;
 if(!init.load((root+"/init.json").c_str())||!runtime.load((root+"/runtime.json").c_str()))throw std::runtime_error("cannot load saved configuration");
 init.input.setFromSVOFile((root+"/stereo.svo2").c_str());init.svo_real_time_mode=false;
 init.optional_opencv_calibration_file=(root+"/water_calibration.yml").c_str();
 init.depth_mode=sl::DEPTH_MODE::NONE;init.grab_compute_capping_fps=0;runtime.enable_depth=false;
 sl::Camera camera;
 if(camera.open(init)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("cannot open closed recording");
 if(end>=camera.getSVONumberOfFrames())throw std::runtime_error("requested interval exceeds recording");
 camera.setSVOPosition(start);sl::Mat pixels;
 auto orb=cv::ORB::create(900);std::deque<Frame> history;
 std::ofstream csv(out+"/frames.csv");if(!csv)throw std::runtime_error("cannot open result");
 csv<<"svo_index,source_ns,source_delta_ms,laplacian_variance,keypoints,adjacent_ratio_matches,one_second_ratio_matches\n"<<std::setprecision(12);
 std::uint64_t previous=0;int decoded=0,analyzed=0;const std::uint64_t targets[]={1791653834500000000ULL,1791653862500000000ULL,1791653875000000000ULL};bool saved[3]={};
 for(int index=start;index<=end;++index) {
  if(camera.grab(runtime)!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("decode failed");
  auto stamp=camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
  if(previous&&stamp<=previous)throw std::runtime_error("non-monotonic source time");
  ++decoded;
  if((index-start)%step!=0){previous=stamp;continue;}
  if(camera.retrieveImage(pixels,sl::VIEW::LEFT,sl::MEM::CPU,sl::Resolution(960,600))!=sl::ERROR_CODE::SUCCESS)throw std::runtime_error("image retrieval failed");
  cv::Mat rgba(600,960,CV_8UC4,pixels.getPtr<sl::uchar1>(sl::MEM::CPU),pixels.getStepBytes());cv::Mat gray,bgr;
  cv::cvtColor(rgba,gray,cv::COLOR_BGRA2GRAY);cv::Mat lap;cv::Laplacian(gray,lap,CV_64F);cv::Scalar mean,stddev;cv::meanStdDev(lap,mean,stddev);
  std::vector<cv::KeyPoint> keys;cv::Mat descriptors;orb->detectAndCompute(gray,cv::noArray(),keys,descriptors);
  const int adjacent=history.empty()?-1:matches(history.back().descriptors,descriptors);int old=-1;
  while(history.size()>1&&history[1].stamp+1000000000ULL<=stamp)history.pop_front();
  if(!history.empty()&&history.front().stamp+1000000000ULL<=stamp)old=matches(history.front().descriptors,descriptors);
  csv<<index<<','<<stamp<<','<<(previous?(stamp-previous)/1e6:0)<<','<<stddev[0]*stddev[0]<<','<<keys.size()<<','<<adjacent<<','<<old<<'\n';
  history.push_back({stamp,descriptors});previous=stamp;++analyzed;
  for(int i=0;i<3;++i)if(!saved[i]&&stamp>=targets[i]){cv::cvtColor(rgba,bgr,cv::COLOR_BGRA2BGR);if(!cv::imwrite(out+"/sample_"+std::to_string(i)+".png",bgr))throw std::runtime_error("sample write failed");saved[i]=true;}
 }
 camera.close();csv.close();
 std::cout<<"{\"decoded\":"<<decoded<<",\"analyzed\":"<<analyzed<<",\"first_index\":"<<start<<",\"last_index\":"<<end<<",\"depth_computed\":false,\"mapping_replayed\":false,\"scope\":\"closed SVO sequential decode and 2D feature-overlap diagnostic only\"}\n";
 return decoded==end-start+1?0:1;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
