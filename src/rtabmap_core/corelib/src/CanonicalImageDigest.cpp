#include <rtabmap/core/ImageInputReceipt.h>
#include <openssl/evp.h>
#include <memory>

namespace rtabmap {
std::string canonicalImageSha256(const cv::Mat &image)
{
    if(image.empty() || image.dims!=2 || (image.type()!=CV_8UC1 && image.type()!=CV_8UC3))return {};
    const std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(),EVP_MD_CTX_free);
    if(!context || EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr)!=1)return {};
    const size_t bytes=static_cast<size_t>(image.cols)*image.elemSize();
    for(int row=0;row<image.rows;++row)
        if(EVP_DigestUpdate(context.get(),image.ptr<unsigned char>(row),bytes)!=1)return {};
    unsigned char digest[EVP_MAX_MD_SIZE];unsigned size=0;
    if(EVP_DigestFinal_ex(context.get(),digest,&size)!=1 || size!=32)return {};
    static const char hex[]="0123456789abcdef";
    std::string output(64,'0');
    for(unsigned i=0;i<size;++i){output[2*i]=hex[digest[i]>>4];output[2*i+1]=hex[digest[i]&15];}
    return output;
}
}
