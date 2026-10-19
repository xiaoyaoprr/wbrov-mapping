#pragma once
#include <opencv2/core.hpp>
#include <opencv2/core/hal/hal.hpp>
#include <climits>
#include <vector>

namespace rtabmap {
// Exactly the single-query Hamming BFMatcher used for projected candidates:
// retain candidate order, first-on-tie selection and the same float NNDR test.
// Scan existing descriptor rows instead of copying a temporary matrix and
// constructing a BFMatcher for every image feature. No search radius or
// candidate is changed. Floating point descriptors keep the OpenCV path.
inline int projectionHammingMatch(const cv::Mat &query,const cv::Mat &train,
        const std::vector<size_t> &candidates,const std::vector<int> *projectedRows,
        bool crossCheck,float ratio) {
    CV_Assert(query.type()==CV_8UC1 && train.type()==CV_8UC1 && query.rows==1 &&
        query.cols==train.cols && candidates.size()>=2);
    int best=-1,first=INT_MAX,second=INT_MAX;
    for(size_t i=0;i<candidates.size();++i) {
        const int row=projectedRows?projectedRows->at(candidates[i]):int(candidates[i]);
        CV_Assert(row>=0 && row<train.rows);
        const int distance=cv::hal::normHamming(query.ptr<unsigned char>(),train.ptr<unsigned char>(row),query.cols);
        if(distance<first){second=first;first=distance;best=int(i);}
        else if(distance<second)second=distance;
    }
    // A single query is trivially reciprocal for every candidate's nearest
    // query. Preserve BFMatcher's first minimum when crossCheck is requested.
    return crossCheck || float(first)<ratio*float(second)?int(candidates.at(best)):-1;
}
}
