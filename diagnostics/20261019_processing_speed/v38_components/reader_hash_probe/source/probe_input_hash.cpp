#include <openssl/evp.h>
#include <array>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

static std::string digest(const std::filesystem::path &p,int mode) {
    if(!std::filesystem::is_regular_file(p) || std::filesystem::is_symlink(p))throw std::runtime_error("nonregular input");
    std::ifstream input(p,std::ios::binary);
    std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(),EVP_MD_CTX_free);
    if(!context || EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr)!=1)throw std::runtime_error("init");
    auto stream=[&](unsigned char *buffer,std::size_t size) {
        while(input) {input.read(reinterpret_cast<char*>(buffer),size);if(EVP_DigestUpdate(context.get(),buffer,std::size_t(input.gcount()))!=1)throw std::runtime_error("update");}
    };
    if(mode==0){std::array<unsigned char,1024*1024> buffer{};stream(buffer.data(),buffer.size());}
    else if(mode==1){static thread_local std::unique_ptr<unsigned char[]> buffer(new unsigned char[1024*1024]);stream(buffer.get(),1024*1024);}
    else {std::array<unsigned char,64*1024> buffer;stream(buffer.data(),buffer.size());}
    if(!input.eof())throw std::runtime_error("read");
    std::array<unsigned char,EVP_MAX_MD_SIZE> hash{};unsigned length=0;
    if(EVP_DigestFinal_ex(context.get(),hash.data(),&length)!=1 || length!=32)throw std::runtime_error("final");
    std::ostringstream text;text<<std::hex<<std::setfill('0');for(unsigned i=0;i<length;++i)text<<std::setw(2)<<unsigned(hash[i]);return text.str();
}
int main(int argc,char **argv) {
    assert(argc==2);std::ifstream manifest(argv[1]);assert(manifest);std::vector<std::pair<std::string,std::string>> files;
    std::string pin,path;while(manifest>>pin>>path){assert(pin.size()==64);files.emplace_back(path,pin);}
    assert(!files.empty());
    for(int round=0;round<6;++round)for(int slot=0;slot<3;++slot) {
        const int mode=(round+slot)%3;std::uint64_t bytes=0;
        const auto start=std::chrono::steady_clock::now();
        for(int repeat=0;repeat<8;++repeat)for(const auto &entry:files) {
            assert(digest(entry.first,mode)==entry.second);bytes+=std::filesystem::file_size(entry.first);
        }
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        std::cout<<"{\"round\":"<<round<<",\"mode\":"<<mode<<",\"files\":"<<files.size()*8<<",\"bytes\":"<<bytes<<",\"ms\":"<<ms<<",\"sha256_equal\":true}"<<std::endl;
    }
}
