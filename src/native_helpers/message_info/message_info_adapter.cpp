// Private bridge to the public rcl receive API. It takes the message and the
// publisher GID together; no topic-discovery value is substituted for metadata.
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rmw/serialized_message.h>
#include <rmw/types.h>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cstdio>

extern "C" size_t wbrov_gid_size(){return RMW_GID_STORAGE_SIZE;}
extern "C" void wbrov_message_free(void* memory){std::free(memory);}
extern "C" int wbrov_take_with_info(uintptr_t pointer,void** output,size_t* size,
    uint8_t* gid,size_t gid_capacity,int64_t* source_time,int64_t* received_time,
    char* error,size_t error_size){
 if(!pointer||!output||!size||!gid||gid_capacity<RMW_GID_STORAGE_SIZE||!source_time||!received_time||!error||!error_size)return -2;
 *output=nullptr;*size=0;std::memset(gid,0,gid_capacity);*source_time=0;*received_time=0;error[0]=0;
 auto serialized=rmw_get_zero_initialized_serialized_message();auto allocator=rcl_get_default_allocator();
 if(rmw_serialized_message_init(&serialized,0,&allocator)!=RCL_RET_OK){
  std::snprintf(error,error_size,"serialized message allocation failed: %s",rcl_get_error_string().str);rcl_reset_error();return -1;
 }
 rmw_message_info_t info{};
 const auto ret=rcl_take_serialized_message(reinterpret_cast<const rcl_subscription_t*>(pointer),&serialized,&info,nullptr);
 int result=-1;
 if(ret==RCL_RET_SUBSCRIPTION_TAKE_FAILED){result=0;rcl_reset_error();}
 else if(ret!=RCL_RET_OK){std::snprintf(error,error_size,"rcl take failed %d: %s",ret,rcl_get_error_string().str);rcl_reset_error();}
 else if(serialized.buffer_length==0||serialized.buffer_length>16*1024*1024){std::snprintf(error,error_size,"serialized message size invalid");}
 else if(void* memory=std::malloc(serialized.buffer_length)){
  std::memcpy(memory,serialized.buffer,serialized.buffer_length);*output=memory;*size=serialized.buffer_length;
  std::memcpy(gid,info.publisher_gid.data,RMW_GID_STORAGE_SIZE);
  *source_time=info.source_timestamp;*received_time=info.received_timestamp;result=1;
 }else std::snprintf(error,error_size,"message copy allocation failed");
 if(rmw_serialized_message_fini(&serialized)!=RCL_RET_OK){
  if(*output){std::free(*output);*output=nullptr;*size=0;}
  std::snprintf(error,error_size,"serialized message cleanup failed: %s",rcl_get_error_string().str);rcl_reset_error();return -1;
 }
 return result;
}
