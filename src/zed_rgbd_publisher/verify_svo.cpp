#include <sl/Camera.hpp>
#include <iostream>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <algorithm>
int main(int argc,char **argv) {
  if(argc==2 && std::string(argv[1])=="--sdk-version") {std::cout<<sl::Camera::getSDKVersion().c_str()<<'\n';return 0;}
  const bool indexOnly=argc==3 && std::string(argv[2])=="--index-only";
  if(argc!=2 && !indexOnly)return 2;
  sl::Camera camera;sl::InitParameters init;
  init.input.setFromSVOFile(argv[1]);init.svo_real_time_mode=false;
  init.depth_mode=sl::DEPTH_MODE::NONE;
  const auto opened=camera.open(init);
  if(opened!=sl::ERROR_CODE::SUCCESS) {std::cerr<<sl::toString(opened).c_str()<<'\n';return 1;}
  const int expected=camera.getSVONumberOfFrames();int decoded=0;
  std::uint64_t first=0,last=0;
  sl::RuntimeParameters runtime;runtime.enable_depth=false;
  for(;;) {
    const auto status=camera.grab(runtime);
    if(status==sl::ERROR_CODE::END_OF_SVOFILE_REACHED)break;
    if(status!=sl::ERROR_CODE::SUCCESS) {camera.close();return 1;}
    const auto stamp=camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
    if(!stamp || (last && stamp<=last)) {camera.close();return 1;}
    if(!first)first=stamp;
    last=stamp;++decoded;
    if(indexOnly) {
      if(decoded==1 && expected>1)camera.setSVOPosition(expected-1);
      else break;
    }
  }
  camera.close();
  std::cout<<"{\"frames\":"<<expected<<",\"decoded\":"<<decoded
           <<",\"first_source_ns\":"<<first<<",\"last_source_ns\":"<<last<<"}\n";
  return decoded>0 && (indexOnly?decoded==std::min(2,expected):decoded==expected)?0:1;
}
