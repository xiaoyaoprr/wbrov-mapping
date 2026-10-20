#include <rtabmap/core/CameraModel.h>
#include <iostream>
int main(){rtabmap::CameraModel m(420.,421.,320.,180.,rtabmap::Transform::getIdentity(),0.,cv::Size(640,360));std::cout<<"vendor control complete, no candidate cache executed"<<std::endl;return m.isValidForProjection()?0:1;}
