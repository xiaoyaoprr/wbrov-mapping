#include "ProjectionHammingMatch.h"
#include <opencv2/features2d.hpp>
#include <iostream>
#include <random>
#include <chrono>
#include <stdexcept>
using namespace rtabmap;
static int reference(const cv::Mat &q,const cv::Mat &train,const std::vector<size_t> &ids,
        const std::vector<int> *rows,bool cross,float ratio) {
    cv::Mat copy(ids.size(),train.cols,train.type());
    for(size_t i=0;i<ids.size();++i)train.row(rows?rows->at(ids[i]):int(ids[i])).copyTo(copy.row(i));
    cv::BFMatcher matcher(cv::NORM_HAMMING,cross);
    if(cross){std::vector<cv::DMatch> m;matcher.match(q,copy,m);return m.empty()?-1:int(ids.at(m.front().trainIdx));}
    std::vector<std::vector<cv::DMatch>> m;matcher.knnMatch(q,copy,m,2);
    if(m.size()!=1 || m[0].size()!=2)throw std::runtime_error("reference match count");
    return m[0][0].distance<ratio*m[0][1].distance?int(ids.at(m[0][0].trainIdx)):-1;
}
int main()try {
    cv::setNumThreads(1);cv::setRNGSeed(81018);std::mt19937 random(81018);int cases=0;
    for(int width:{16,32,48,64})for(int trial=0;trial<1000;++trial) {
        cv::Mat padded(70,width+11,CV_8U);cv::randu(padded,0,256);
        cv::Mat data=padded(cv::Rect(3,0,width,70)); // deliberately strided
        if(trial%4==0)data.row(0).copyTo(data.row(1)); // exact ties
        if(trial%5==0)data.setTo(0); // all candidates tied, NNDR must reject
        cv::Mat q=data.row(trial%70);std::vector<size_t> ids;std::vector<int> rows;
        for(int i=0;i<70;++i)rows.push_back(69-i);
        for(int i=0,count=2+random()%60;i<count;++i)ids.push_back(random()%70); // duplicates allowed
        for(auto mapping:{static_cast<const std::vector<int>*>(nullptr),static_cast<const std::vector<int>*>(&rows)})for(bool cross:{false,true})for(float ratio:{.6f,.8f,1.f}) {
            if(reference(q,data,ids,mapping,cross,ratio)!=projectionHammingMatch(q,data,ids,mapping,cross,ratio))
                throw std::runtime_error("projected Hamming selection differs from BFMatcher");
            ++cases;
        }
    }
    cv::Mat data(200,32,CV_8U);cv::randu(data,0,256);std::vector<size_t> ids;
    for(int i=0;i<40;++i)ids.push_back(i*3);
    volatile long total=0;double times[2]{};
    for(int mode=0;mode<2;++mode){auto start=std::chrono::steady_clock::now();
        for(int i=0;i<30000;++i)total+=mode?projectionHammingMatch(data.row(i%200),data,ids,nullptr,false,.8f):reference(data.row(i%200),data,ids,nullptr,false,.8f);
        times[mode]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    }
    std::cout<<"{\"passed\":true,\"cases\":"<<cases<<",\"reference_ms\":"<<times[0]<<",\"direct_ms\":"<<times[1]<<",\"sink\":"<<total<<"}\n";
    return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
