#include <rtabmap/core/ImageInputReceipt.h>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <future>
using namespace rtabmap;
static void require(bool ok){if(!ok)throw std::runtime_error("canonical image digest differs");}
static std::string reference(const cv::Mat &image){
    image_receipt_detail::SafeSha256Stream hash;
    for(int row=0;row<image.rows;++row)hash.update(image.ptr<unsigned char>(row),image.cols*image.elemSize());
    return hash.finish();
}
int main()try {
    cv::setNumThreads(1);cv::setRNGSeed(81018);int cases=0;
    require(canonicalImageSha256(cv::Mat()).empty());
    for(int type:{CV_8UC4,CV_16UC1,CV_32FC1})require(canonicalImageSha256(cv::Mat::zeros(5,7,type)).empty());
    int sizes[3]={2,3,4};require(canonicalImageSha256(cv::Mat(3,sizes,CV_8UC1)).empty());
    const unsigned char abc[]={'a','b','c'};
    require(canonicalImageSha256(cv::Mat(1,3,CV_8UC1,const_cast<unsigned char*>(abc)))==
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    for(int type:{CV_8UC1,CV_8UC3})for(int rows:{1,2,3,16,240,600})for(int cols:{1,3,55,56,63,64,65,384,960}){
        cv::Mat storage(rows,cols+7,type);cv::randu(storage,0,256);
        cv::Mat image=storage(cv::Rect(2,0,cols,rows));
        const auto digest=canonicalImageSha256(image);
        require(digest==reference(image) && digest==canonicalImageSha256(image.clone()));
        storage.col(0).setTo(0);require(canonicalImageSha256(image)==digest); // padding is not an image pixel
        image.ptr<unsigned char>(rows-1)[cols*image.elemSize()-1]^=1;
        require(canonicalImageSha256(image)!=digest && canonicalImageSha256(image)==reference(image));
        ++cases;
    }
    cv::Mat shared(240,384,CV_8UC3);cv::randu(shared,0,256);const auto expected=reference(shared);
    std::vector<std::future<void>> futures;
    for(int i=0;i<4;++i)futures.emplace_back(std::async(std::launch::async,[&]{for(int n=0;n<30;++n)require(canonicalImageSha256(shared)==expected);}));
    for(auto &f:futures)f.get();
    std::cout<<"{\"passed\":true,\"strided_cases\":"<<cases<<",\"concurrent_digests\":120}\n";
    return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
