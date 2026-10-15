#include <rtabmap_slam/ManagedImuEpochBuffer.h>
#include <cstring>
/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap_slam/CoreWrapper.h"

#include <stdio.h>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cerrno>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <sstream>
#include <sys/stat.h>
#include <thread>
#include <stdexcept>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <opencv2/core/persistence.hpp>

#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/image_encodings.hpp>
#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <pcl_conversions/pcl_conversions.h>
#if PCL_VERSION_COMPARE(>, 1, 12, 0)
#include <pcl/common/io.h>
#else
#include <pcl/io/io.h>
#endif

#include <visualization_msgs/msg/marker_array.hpp>

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>

#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Registration.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/LocalGridMaker.h>
#include <rtabmap/core/Optimizer.h>

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
#include <octomap_msgs/conversions.h>
#include <rtabmap/core/OctoMap.h>
#endif
#endif

#define BAD_COVARIANCE 9999

//msgs
#include "rtabmap_msgs/msg/info.hpp"
#include "rtabmap_msgs/msg/map_data.hpp"
#include "rtabmap_msgs/msg/map_graph.hpp"
#include "rtabmap_msgs/srv/get_map.hpp"
#include "rtabmap_msgs/srv/publish_map.hpp"
#include "rtabmap_msgs/msg/path.hpp"

#include "rtabmap_conversions/MsgConversion.h"

#ifdef PRE_ROS_JAZZY
namespace rclcpp {
	rmw_qos_profile_t ServicesQoS() {return rmw_qos_profile_services_default;}
	rmw_qos_profile_t ParametersQoS() {return rmw_qos_profile_parameters;}
}
#endif

using namespace rtabmap;

namespace rtabmap_slam {

namespace
{

uint32_t safeRotateRight(uint32_t value, unsigned int shift)
{
	return (value >> shift) | (value << (32-shift));
}

std::string safeSha256(const std::string & input)
{
	static const uint32_t constants[64] = {
		0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
		0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
		0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
		0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
		0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
		0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
		0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
		0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U};
	uint32_t state[8] = {
		0x6a09e667U,0xbb67ae85U,0x3c6ef372U,0xa54ff53aU,
		0x510e527fU,0x9b05688cU,0x1f83d9abU,0x5be0cd19U};
	std::vector<unsigned char> bytes(input.begin(), input.end());
	const uint64_t bitLength = (uint64_t)bytes.size()*8ULL;
	bytes.push_back(0x80U);
	while(bytes.size()%64 != 56)
	{
		bytes.push_back(0U);
	}
	for(int shift=56; shift>=0; shift-=8)
	{
		bytes.push_back((unsigned char)((bitLength >> shift)&0xffU));
	}
	for(size_t offset=0; offset<bytes.size(); offset+=64)
	{
		uint32_t words[64];
		for(int i=0; i<16; ++i)
		{
			const size_t p = offset+(size_t)i*4;
			words[i] = ((uint32_t)bytes[p]<<24) | ((uint32_t)bytes[p+1]<<16) |
					((uint32_t)bytes[p+2]<<8) | (uint32_t)bytes[p+3];
		}
		for(int i=16; i<64; ++i)
		{
			const uint32_t s0 = safeRotateRight(words[i-15],7) ^
					safeRotateRight(words[i-15],18) ^ (words[i-15]>>3);
			const uint32_t s1 = safeRotateRight(words[i-2],17) ^
					safeRotateRight(words[i-2],19) ^ (words[i-2]>>10);
			words[i] = words[i-16] + s0 + words[i-7] + s1;
		}
		uint32_t a=state[0], b=state[1], c=state[2], d=state[3];
		uint32_t e=state[4], f=state[5], g=state[6], h=state[7];
		for(int i=0; i<64; ++i)
		{
			const uint32_t s1 = safeRotateRight(e,6) ^ safeRotateRight(e,11) ^ safeRotateRight(e,25);
			const uint32_t choice = (e&f) ^ ((~e)&g);
			const uint32_t temp1 = h+s1+choice+constants[i]+words[i];
			const uint32_t s0 = safeRotateRight(a,2) ^ safeRotateRight(a,13) ^ safeRotateRight(a,22);
			const uint32_t majority = (a&b) ^ (a&c) ^ (b&c);
			const uint32_t temp2 = s0+majority;
			h=g; g=f; f=e; e=d+temp1; d=c; c=b; b=a; a=temp1+temp2;
		}
		state[0]+=a; state[1]+=b; state[2]+=c; state[3]+=d;
		state[4]+=e; state[5]+=f; state[6]+=g; state[7]+=h;
	}
	static const char hex[] = "0123456789abcdef";
	std::string output(64, '0');
	for(int i=0; i<8; ++i)
	{
		for(int byte=0; byte<4; ++byte)
		{
			const unsigned char value = (unsigned char)((state[i]>>(24-byte*8))&0xffU);
			output[(i*4+byte)*2] = hex[value>>4];
			output[(i*4+byte)*2+1] = hex[value&0x0fU];
		}
	}
	return output;
}

class SafeSha256Stream
{
public:
	SafeSha256Stream() : totalBytes_(0), buffered_(0)
	{
		const uint32_t initial[8] = {
			0x6a09e667U,0xbb67ae85U,0x3c6ef372U,0xa54ff53aU,
			0x510e527fU,0x9b05688cU,0x1f83d9abU,0x5be0cd19U};
		std::copy(initial, initial+8, state_);
	}

	void update(const unsigned char * data, size_t size)
	{
		totalBytes_ += (uint64_t)size;
		while(size)
		{
			const size_t take = std::min(size, sizeof(buffer_)-buffered_);
			std::copy(data, data+take, buffer_+buffered_);
			buffered_ += take;
			data += take;
			size -= take;
			if(buffered_ == sizeof(buffer_))
			{
				compress(buffer_);
				buffered_ = 0;
			}
		}
	}

	std::string finish()
	{
		const uint64_t bitLength = totalBytes_*8ULL;
		buffer_[buffered_++] = 0x80U;
		if(buffered_ > 56)
		{
			std::fill(buffer_+buffered_, buffer_+64, 0U);
			compress(buffer_);
			buffered_ = 0;
		}
		std::fill(buffer_+buffered_, buffer_+56, 0U);
		for(int shift=56; shift>=0; shift-=8)
		{
			buffer_[56+(56-shift)/8] = (unsigned char)((bitLength>>shift)&0xffU);
		}
		compress(buffer_);
		static const char hex[] = "0123456789abcdef";
		std::string output(64, '0');
		for(int i=0; i<8; ++i)
		{
			for(int byte=0; byte<4; ++byte)
			{
				const unsigned char value =
						(unsigned char)((state_[i]>>(24-byte*8))&0xffU);
				output[(i*4+byte)*2] = hex[value>>4];
				output[(i*4+byte)*2+1] = hex[value&0x0fU];
			}
		}
		return output;
	}

private:
	void compress(const unsigned char * block)
	{
		static const uint32_t constants[64] = {
			0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
			0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
			0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
			0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
			0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
			0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
			0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
			0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U};
		uint32_t words[64];
		for(int i=0; i<16; ++i)
		{
			const size_t p = (size_t)i*4;
			words[i] = ((uint32_t)block[p]<<24) | ((uint32_t)block[p+1]<<16) |
					((uint32_t)block[p+2]<<8) | (uint32_t)block[p+3];
		}
		for(int i=16; i<64; ++i)
		{
			const uint32_t s0 = safeRotateRight(words[i-15],7) ^
					safeRotateRight(words[i-15],18) ^ (words[i-15]>>3);
			const uint32_t s1 = safeRotateRight(words[i-2],17) ^
					safeRotateRight(words[i-2],19) ^ (words[i-2]>>10);
			words[i] = words[i-16] + s0 + words[i-7] + s1;
		}
		uint32_t a=state_[0], b=state_[1], c=state_[2], d=state_[3];
		uint32_t e=state_[4], f=state_[5], g=state_[6], h=state_[7];
		for(int i=0; i<64; ++i)
		{
			const uint32_t s1 = safeRotateRight(e,6) ^ safeRotateRight(e,11) ^ safeRotateRight(e,25);
			const uint32_t choice = (e&f) ^ ((~e)&g);
			const uint32_t temp1 = h+s1+choice+constants[i]+words[i];
			const uint32_t s0 = safeRotateRight(a,2) ^ safeRotateRight(a,13) ^ safeRotateRight(a,22);
			const uint32_t majority = (a&b) ^ (a&c) ^ (b&c);
			const uint32_t temp2 = s0+majority;
			h=g; g=f; f=e; e=d+temp1; d=c; c=b; b=a; a=temp1+temp2;
		}
		state_[0]+=a; state_[1]+=b; state_[2]+=c; state_[3]+=d;
		state_[4]+=e; state_[5]+=f; state_[6]+=g; state_[7]+=h;
	}

	uint32_t state_[8];
	uint64_t totalBytes_;
	unsigned char buffer_[64];
	size_t buffered_;
};

bool sameDatabaseIdentity(const struct stat & left, const struct stat & right)
{
	return left.st_dev == right.st_dev && left.st_ino == right.st_ino &&
			left.st_mode == right.st_mode && left.st_uid == right.st_uid &&
			left.st_nlink == right.st_nlink && left.st_size == right.st_size &&
			left.st_mtim.tv_sec == right.st_mtim.tv_sec &&
			left.st_mtim.tv_nsec == right.st_mtim.tv_nsec &&
			left.st_ctim.tv_sec == right.st_ctim.tv_sec &&
			left.st_ctim.tv_nsec == right.st_ctim.tv_nsec;
}

bool safeDatabaseBinding(
		const std::string & path,
		const std::string & expectedSha256,
		int & descriptor,
		uint64_t & boundDevice,
		uint64_t & boundInode,
		std::string & reason)
{
	const bool newMapSentinel = expectedSha256 == std::string(64, '0');
	const char * sidecarSuffixes[] = {"-wal", "-shm", "-journal"};
	for(size_t i=0; i<sizeof(sidecarSuffixes)/sizeof(sidecarSuffixes[0]); ++i)
	{
		struct stat sidecarInfo;
		if(::lstat((path+sidecarSuffixes[i]).c_str(), &sidecarInfo) == 0 || errno != ENOENT)
		{
			reason = "database sidecar exists or cannot be excluded";
			return false;
		}
	}

	struct stat pathInfo;
	if(::lstat(path.c_str(), &pathInfo) != 0)
	{
		if(newMapSentinel && errno == ENOENT)
		{
			descriptor = ::open(path.c_str(), O_RDWR | O_CREAT | O_EXCL |
					O_CLOEXEC | O_NOFOLLOW, 0600);
			if(descriptor < 0 || ::fstat(descriptor, &pathInfo) != 0)
			{
				reason = "new-map database inode cannot be created and bound";
				if(descriptor >= 0)
				{
					::close(descriptor);
					descriptor = -1;
				}
				return false;
			}
		}
		else
		{
			reason = "database path is missing or cannot be inspected";
			return false;
		}
	}
	else if(newMapSentinel && descriptor < 0)
	{
		// A zero-length pathname left by another run/user is not proof that this
		// process created and bound it.  Only the missing-path O_EXCL creation
		// above, or a later recheck through our already-held descriptor, is valid.
		reason = "new-map database path already existed before secure binding";
		return false;
	}
	if(!S_ISREG(pathInfo.st_mode) || pathInfo.st_uid != ::geteuid() ||
			pathInfo.st_nlink != 1 || (pathInfo.st_mode & 0022) != 0)
	{
		reason = "database must be an owned single-link regular file not writable by group/other";
		return false;
	}
	if(newMapSentinel && pathInfo.st_size != 0)
	{
		reason = "new-map database sentinel requires a missing or zero-length database";
		return false;
	}
	if(!newMapSentinel && pathInfo.st_size <= 0)
	{
		reason = "continuation database is empty";
		return false;
	}

	if(descriptor < 0)
	{
		descriptor = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
		if(descriptor < 0)
		{
			reason = "database cannot be opened with O_NOFOLLOW";
			return false;
		}
	}
	bool valid = false;
	struct stat openedInfo;
	try
	{
		if(::fstat(descriptor, &openedInfo) != 0 || !sameDatabaseIdentity(pathInfo, openedInfo))
		{
			reason = "database identity changed while opening";
			throw std::runtime_error(reason);
		}
		if((boundDevice != 0 || boundInode != 0) &&
				(boundDevice != (uint64_t)openedInfo.st_dev ||
				 boundInode != (uint64_t)openedInfo.st_ino))
		{
			reason = "database pathname no longer resolves to the held startup inode";
			throw std::runtime_error(reason);
		}
		if(::lseek(descriptor, 0, SEEK_SET) < 0)
		{
			reason = "database descriptor cannot rewind for hashing";
			throw std::runtime_error(reason);
		}
		SafeSha256Stream digest;
		uint64_t total = 0;
		unsigned char buffer[1024*1024];
		while(true)
		{
			const ssize_t count = ::read(descriptor, buffer, sizeof(buffer));
			if(count < 0)
			{
				if(errno == EINTR)
				{
					continue;
				}
				reason = "database read failed";
				throw std::runtime_error(reason);
			}
			if(count == 0)
			{
				break;
			}
			digest.update(buffer, (size_t)count);
			total += (uint64_t)count;
			if(total > (uint64_t)openedInfo.st_size)
			{
				reason = "database grew while hashing";
				throw std::runtime_error(reason);
			}
		}
		struct stat finalOpenedInfo;
		struct stat finalPathInfo;
		if(total != (uint64_t)openedInfo.st_size ||
				::fstat(descriptor, &finalOpenedInfo) != 0 ||
				::lstat(path.c_str(), &finalPathInfo) != 0 ||
				!sameDatabaseIdentity(openedInfo, finalOpenedInfo) ||
				!sameDatabaseIdentity(openedInfo, finalPathInfo))
		{
			reason = "database identity changed while hashing";
			throw std::runtime_error(reason);
		}
		if(!newMapSentinel && digest.finish() != expectedSha256)
		{
			reason = "database SHA256 differs from safe_relocalization_database_sha256";
			throw std::runtime_error(reason);
		}
		boundDevice = (uint64_t)openedInfo.st_dev;
		boundInode = (uint64_t)openedInfo.st_ino;
		valid = true;
	}
	catch(const std::exception &)
	{
		valid = false;
	}
	if(!valid && descriptor >= 0)
	{
		::close(descriptor);
		descriptor = -1;
		boundDevice = 0;
		boundInode = 0;
	}
	return valid;
}

bool safeDatabasePathStillBound(
		const std::string & path,
		int descriptor,
		uint64_t boundDevice,
		uint64_t boundInode)
{
	if(descriptor < 0 || boundDevice == 0 || boundInode == 0)
	{
		return false;
	}
	struct stat descriptorInfo;
	struct stat pathInfo;
	return ::fstat(descriptor, &descriptorInfo) == 0 &&
			::lstat(path.c_str(), &pathInfo) == 0 &&
			S_ISREG(pathInfo.st_mode) && pathInfo.st_uid == ::geteuid() &&
			pathInfo.st_nlink == 1 && (pathInfo.st_mode & 0022) == 0 &&
			(uint64_t)descriptorInfo.st_dev == boundDevice &&
			(uint64_t)descriptorInfo.st_ino == boundInode &&
			(uint64_t)pathInfo.st_dev == boundDevice &&
			(uint64_t)pathInfo.st_ino == boundInode;
}

template<typename T>
void appendManagedOdomValue(std::string & bytes, const T & value)
{
	bytes.append(reinterpret_cast<const char *>(&value), sizeof(T));
}

std::string managedOdomFingerprint(const nav_msgs::msg::Odometry & msg)
{
	std::string bytes;
	bytes.reserve(16U + msg.header.frame_id.size() + msg.child_frame_id.size() +
			(7U+6U+36U+36U)*sizeof(double));
	appendManagedOdomValue(bytes, msg.header.stamp.sec);
	appendManagedOdomValue(bytes, msg.header.stamp.nanosec);
	const uint64_t frameSize = (uint64_t)msg.header.frame_id.size();
	const uint64_t childSize = (uint64_t)msg.child_frame_id.size();
	appendManagedOdomValue(bytes, frameSize);
	bytes.append(msg.header.frame_id);
	appendManagedOdomValue(bytes, childSize);
	bytes.append(msg.child_frame_id);
	const double poseValues[] = {
		msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
		msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z, msg.pose.pose.orientation.w};
	for(size_t i=0; i<sizeof(poseValues)/sizeof(poseValues[0]); ++i)
	{
		appendManagedOdomValue(bytes, poseValues[i]);
	}
	for(size_t i=0; i<msg.pose.covariance.size(); ++i)
	{
		appendManagedOdomValue(bytes, msg.pose.covariance[i]);
	}
	const double twistValues[] = {
		msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
		msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z};
	for(size_t i=0; i<sizeof(twistValues)/sizeof(twistValues[0]); ++i)
	{
		appendManagedOdomValue(bytes, twistValues[i]);
	}
	for(size_t i=0; i<msg.twist.covariance.size(); ++i)
	{
		appendManagedOdomValue(bytes, msg.twist.covariance[i]);
	}
	return safeSha256(bytes);
}

bool managedOdomNumericsValid(const nav_msgs::msg::Odometry & msg)
{
	const double poseValues[] = {
		msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
		msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z, msg.pose.pose.orientation.w};
	const double twistValues[] = {
		msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
		msg.twist.twist.angular.x, msg.twist.twist.angular.y,
		msg.twist.twist.angular.z};
	for(size_t i=0; i<sizeof(poseValues)/sizeof(poseValues[0]); ++i)
	{
		if(!std::isfinite(poseValues[i]))
		{
			return false;
		}
	}
	for(size_t i=0; i<sizeof(twistValues)/sizeof(twistValues[0]); ++i)
	{
		if(!std::isfinite(twistValues[i]))
		{
			return false;
		}
	}
	for(size_t i=0; i<msg.pose.covariance.size(); ++i)
	{
		if(!std::isfinite(msg.pose.covariance[i]) ||
				!std::isfinite(msg.twist.covariance[i]))
		{
			return false;
		}
	}
	const double quaternionNorm = std::sqrt(
			msg.pose.pose.orientation.x*msg.pose.pose.orientation.x +
			msg.pose.pose.orientation.y*msg.pose.pose.orientation.y +
			msg.pose.pose.orientation.z*msg.pose.pose.orientation.z +
			msg.pose.pose.orientation.w*msg.pose.pose.orientation.w);
	return std::isfinite(quaternionNorm) && std::fabs(quaternionNorm-1.0) <= 1e-3;
}

bool managedOdomHasBadCovariance(const nav_msgs::msg::Odometry & msg)
{
	return msg.pose.covariance[0] >= BAD_COVARIANCE &&
			msg.twist.covariance[0] >= BAD_COVARIANCE;
}

std::string managedOdomInfoFingerprint(const rtabmap_msgs::msg::OdomInfo & msg)
{
	rclcpp::Serialization<rtabmap_msgs::msg::OdomInfo> serializer;
	rclcpp::SerializedMessage serialized;
	serializer.serialize_message(&msg, &serialized);
	rcl_serialized_message_t & raw = serialized.get_rcl_serialized_message();
	// CDR alignment padding is not message content. Reserve the exact capacity
	// in the first pass, then zero it before the deterministic second encoding.
	// Every explicit field remains serialized and covered by the fingerprint.
	std::memset(raw.buffer, 0, raw.buffer_capacity);
	serializer.serialize_message(&msg, &serialized);
	return safeSha256(std::string(
			reinterpret_cast<const char *>(raw.buffer), raw.buffer_length));
}

template<typename MessageT>
std::string managedRosMessageFingerprint(const MessageT & msg)
{
	rclcpp::Serialization<MessageT> serializer;
	rclcpp::SerializedMessage serialized;
	serializer.serialize_message(&msg, &serialized);
	rcl_serialized_message_t & raw = serialized.get_rcl_serialized_message();
	// CDR alignment padding is not message content. Reserve the exact capacity
	// in the first pass, then zero it before the deterministic second encoding.
	// Every explicit field remains serialized and covered by the fingerprint.
	std::memset(raw.buffer, 0, raw.buffer_capacity);
	serializer.serialize_message(&msg, &serialized);
	return safeSha256(std::string(
			reinterpret_cast<const char *>(raw.buffer), raw.buffer_length));
}

bool parseUnsignedJson(const cv::FileNode & node, unsigned long long & value)
{
	value = 0;
	if(node.empty())
	{
		return false;
	}
	if(node.isInt() || node.isReal())
	{
		double number = 0.0;
		node >> number;
		if(!std::isfinite(number) || number < 0.0 || number > 9007199254740991.0 ||
				std::floor(number) != number)
		{
			return false;
		}
		value = (unsigned long long)number;
		return true;
	}
	return false;
}

bool readFiniteSequence(const cv::FileNode & node, size_t expected, std::vector<double> & values)
{
	values.clear();
	if(!node.isSeq() || node.size() != expected)
	{
		return false;
	}
	for(cv::FileNodeIterator iter=node.begin(); iter!=node.end(); ++iter)
	{
		if(!(*iter).isInt() && !(*iter).isReal())
		{
			return false;
		}
		double value = 0.0;
		(*iter) >> value;
		if(!std::isfinite(value))
		{
			return false;
		}
		values.push_back(value);
	}
	return true;
}

class StrictJsonCursor
{
	public:
	explicit StrictJsonCursor(const std::string & payload) : payload_(payload), position_(0) {}

	bool parseDocument()
	{
		skipWhitespace();
		if(!parseObject(1))
		{
			return false;
		}
		skipWhitespace();
		return position_ == payload_.size();
	}

	private:
	void skipWhitespace()
	{
		while(position_ < payload_.size() &&
				(payload_[position_] == ' ' || payload_[position_] == '\t' ||
				 payload_[position_] == '\r' || payload_[position_] == '\n'))
		{
			++position_;
		}
	}

	bool consume(char expected)
	{
		if(position_ >= payload_.size() || payload_[position_] != expected)
		{
			return false;
		}
		++position_;
		return true;
	}

	bool parseObject(unsigned int depth)
	{
		if(depth > 16 || !consume('{'))
		{
			return false;
		}
		skipWhitespace();
		if(position_ < payload_.size() && payload_[position_] == '}')
		{
			++position_;
			return true;
		}
		std::set<std::string> keys;
		while(true)
		{
			std::string key;
			if(!parseString(true, &key) || !keys.insert(key).second)
			{
				return false;
			}
			skipWhitespace();
			if(!consume(':'))
			{
				return false;
			}
			skipWhitespace();
			if(!parseValue(depth))
			{
				return false;
			}
			skipWhitespace();
			if(position_ < payload_.size() && payload_[position_] == '}')
			{
				++position_;
				return true;
			}
			if(!consume(','))
			{
				return false;
			}
			skipWhitespace();
			// parseString() below rejects a trailing comma before '}'.
		}
	}

	bool parseArray(unsigned int depth)
	{
		if(depth > 16 || !consume('['))
		{
			return false;
		}
		skipWhitespace();
		if(position_ < payload_.size() && payload_[position_] == ']')
		{
			++position_;
			return true;
		}
		while(true)
		{
			if(!parseValue(depth))
			{
				return false;
			}
			skipWhitespace();
			if(position_ < payload_.size() && payload_[position_] == ']')
			{
				++position_;
				return true;
			}
			if(!consume(','))
			{
				return false;
			}
			skipWhitespace();
			// parseValue() below rejects a trailing comma before ']'.
		}
	}

	bool parseString(bool objectKey, std::string * rawValue)
	{
		if(!consume('"'))
		{
			return false;
		}
		const size_t begin = position_;
		while(position_ < payload_.size())
		{
			const unsigned char value = (unsigned char)payload_[position_++];
			if(value == '"')
			{
				if(rawValue)
				{
					*rawValue = payload_.substr(begin, position_-begin-1);
				}
				return true;
			}
			if(value < 0x20)
			{
				return false;
			}
			if(value == '\\')
			{
				// Canonical protocol keys are unescaped ASCII. Reject escaped keys
				// so duplicate/unknown-field checks cannot be bypassed by aliases.
				if(objectKey || position_ >= payload_.size())
				{
					return false;
				}
				const char escape = payload_[position_++];
				if(escape == 'u')
				{
					for(int i=0; i<4; ++i)
					{
						if(position_ >= payload_.size() ||
								!std::isxdigit((unsigned char)payload_[position_++]))
						{
							return false;
						}
					}
				}
				else if(escape != '"' && escape != '\\' && escape != '/' &&
						escape != 'b' && escape != 'f' && escape != 'n' &&
						escape != 'r' && escape != 't')
				{
					return false;
				}
			}
		}
		return false;
	}

	bool parseNumber()
	{
		const size_t begin = position_;
		bool negative = false;
		if(position_ < payload_.size() && payload_[position_] == '-')
		{
			negative = true;
			++position_;
		}
		const size_t integerBegin = position_;
		if(position_ >= payload_.size())
		{
			return false;
		}
		if(payload_[position_] == '0')
		{
			++position_;
			if(position_ < payload_.size() && payload_[position_] >= '0' && payload_[position_] <= '9')
			{
				return false;
			}
		}
		else if(payload_[position_] >= '1' && payload_[position_] <= '9')
		{
			while(position_ < payload_.size() && payload_[position_] >= '0' && payload_[position_] <= '9')
			{
				++position_;
			}
		}
		else
		{
			return false;
		}
		const size_t integerEnd = position_;
		bool realSyntax = false;
		if(position_ < payload_.size() && payload_[position_] == '.')
		{
			realSyntax = true;
			++position_;
			const size_t fractionBegin = position_;
			while(position_ < payload_.size() && payload_[position_] >= '0' && payload_[position_] <= '9')
			{
				++position_;
			}
			if(position_ == fractionBegin)
			{
				return false;
			}
		}
		if(position_ < payload_.size() && (payload_[position_] == 'e' || payload_[position_] == 'E'))
		{
			realSyntax = true;
			++position_;
			if(position_ < payload_.size() && (payload_[position_] == '+' || payload_[position_] == '-'))
			{
				++position_;
			}
			const size_t exponentBegin = position_;
			while(position_ < payload_.size() && payload_[position_] >= '0' && payload_[position_] <= '9')
			{
				++position_;
			}
			if(position_ == exponentBegin)
			{
				return false;
			}
		}
		if(!realSyntax)
		{
			// OpenCV 4.5 parses integer tokens with strtol(base=0) and stores a
			// 32-bit int. Bound the lexical value before FileStorage sees it so
			// large epoch/revision values cannot wrap and alias a live state.
			const std::string magnitude = payload_.substr(integerBegin, integerEnd-integerBegin);
			const std::string limit = negative?"2147483648":"2147483647";
			if(magnitude.size() > limit.size() ||
					(magnitude.size() == limit.size() && magnitude > limit))
			{
				return false;
			}
		}
		return position_ > begin;
	}

	bool parseValue(unsigned int depth)
	{
		if(position_ >= payload_.size())
		{
			return false;
		}
		if(payload_[position_] == '{')
		{
			return parseObject(depth+1);
		}
		if(payload_[position_] == '[')
		{
			return parseArray(depth+1);
		}
		if(payload_[position_] == '"')
		{
			return parseString(false, 0);
		}
		if(payload_[position_] == '-' ||
				(payload_[position_] >= '0' && payload_[position_] <= '9'))
		{
			return parseNumber();
		}
		// The proposal schema has no Boolean or null values. Rejecting all
		// literals also prevents OpenCV from representing true/false as INT.
		return false;
	}

	const std::string & payload_;
	size_t position_;
};

bool exactlyOneJsonObject(const std::string & payload)
{
	if(payload.empty() || payload.size() > 65536 ||
			payload.find('\0') != std::string::npos)
	{
		return false;
	}
	StrictJsonCursor cursor(payload);
	return cursor.parseDocument();
}

bool hasExactMapKeys(const cv::FileNode & node, const std::set<std::string> & expected)
{
	if(!node.isMap() || node.size() != expected.size())
	{
		return false;
	}
	std::set<std::string> actual;
	for(cv::FileNodeIterator iter=node.begin(); iter!=node.end(); ++iter)
	{
		const std::string name = (*iter).name();
		if(name.empty() || !actual.insert(name).second)
		{
			return false;
		}
	}
	return actual == expected;
}

bool parseReceiptUnsignedString(const cv::FileNode & node, unsigned long long maximum,
        unsigned long long & value, bool positive)
{
    value = 0;
    if(!node.isString()) return false;
    const std::string text = static_cast<std::string>(node);
    if(text.empty() || text.size() > 20 || (text.size() > 1 && text.front() == '0')) return false;
    for(char c : text)
    {
        if(c < '0' || c > '9') return false;
        const unsigned int digit = static_cast<unsigned int>(c-'0');
        if(value > (maximum-digit)/10ULL) return false;
        value = value*10ULL+digit;
    }
    return !positive || value > 0;
}

bool readAcceptedImageReceipt(const cv::FileNode & node, rtabmap::AcceptedImageReceipt & out)
{
    out = rtabmap::AcceptedImageReceipt();
    const std::set<std::string> keys = {
        "schema_version", "producer", "node_id", "map_id", "header_stamp_ns", "reset_epoch",
        "image_width", "image_height", "encoding", "rgb_sha256", "camera_frame", "K", "D",
        "body_from_camera", "rgb_publisher_gid", "camera_info_publisher_gid", "session_token_sha256",
        "database_sha256", "source_scope", "pixel_role", "saved_jpeg_bytes_are_original_pixels"};
    if(!hasExactMapKeys(node, keys)) return false;
    for(const char * key : {"schema_version", "node_id", "map_id", "image_width", "image_height", "saved_jpeg_bytes_are_original_pixels"})
        if(!node[key].isInt()) return false;
    for(const char * key : {"producer", "encoding", "rgb_sha256", "camera_frame", "rgb_publisher_gid",
                           "camera_info_publisher_gid", "session_token_sha256", "database_sha256", "source_scope", "pixel_role"})
        if(!node[key].isString()) return false;
    if(int(node["schema_version"]) != 1 || std::string(node["producer"]) != "wbrov_core_accepted_image_input" ||
       std::string(node["source_scope"]) != "managed_ros_rgbd" ||
       std::string(node["pixel_role"]) != "authenticated_pre_memory_input" ||
       int(node["saved_jpeg_bytes_are_original_pixels"]) != 0) return false;
    out.signatureId = int(node["node_id"]); out.mapId = int(node["map_id"]);
    auto & input = out.input;
    input.width = int(node["image_width"]); input.height = int(node["image_height"]);
    if(out.signatureId <= 0 || out.mapId < 0 || input.width <= 0 || input.height <= 0) return false;
    unsigned long long stamp = 0;
    if(!parseReceiptUnsignedString(node["header_stamp_ns"], static_cast<unsigned long long>(std::numeric_limits<int64_t>::max()), stamp, true) ||
       !parseReceiptUnsignedString(node["reset_epoch"], std::numeric_limits<unsigned long long>::max(), out.resetEpoch, false)) return false;
    input.headerStampNs = static_cast<int64_t>(stamp);
    input.encoding = std::string(node["encoding"]); input.pixelSha256 = std::string(node["rgb_sha256"]);
    input.sensorFrame = std::string(node["camera_frame"]);
    input.rgbPublisherGidHex = std::string(node["rgb_publisher_gid"]);
    input.infoPublisherGidHex = std::string(node["camera_info_publisher_gid"]);
    input.sessionTokenSha256 = std::string(node["session_token_sha256"]);
    input.databaseSha256 = std::string(node["database_sha256"]); input.provenanceScope = "managed_ros_rgbd";
    const size_t gidSize = input.rgbPublisherGidHex.size();
    if((input.encoding != "bgr8" && input.encoding != "mono8") || input.sensorFrame.empty() || input.sensorFrame.size() > 256 ||
       !rtabmap::receiptHex(input.pixelSha256, 64) || !rtabmap::receiptHex(input.sessionTokenSha256, 64) ||
       input.sessionTokenSha256 == std::string(64, '0') || !rtabmap::receiptHex(input.databaseSha256, 64) ||
       gidSize < 32 || gidSize > 64 || gidSize % 2 || !rtabmap::receiptHex(input.rgbPublisherGidHex, gidSize) ||
       !rtabmap::receiptHex(input.infoPublisherGidHex, gidSize) || input.rgbPublisherGidHex == std::string(gidSize, '0') ||
       input.infoPublisherGidHex == std::string(gidSize, '0')) return false;
    std::vector<double> K, D, extrinsic;
    const size_t dsize = node["D"].size();
    if(!readFiniteSequence(node["K"], 9, K) || !readFiniteSequence(node["body_from_camera"], 12, extrinsic) ||
       (dsize != 0 && dsize != 4 && dsize != 5 && dsize != 8 && dsize != 12 && dsize != 14) ||
       !readFiniteSequence(node["D"], dsize, D)) return false;
    for(double d : D) if(d != 0.0) return false;
    std::copy(K.begin(), K.end(), input.K.begin()); input.D = D;
    std::copy(extrinsic.begin(), extrinsic.end(), out.bodyFromCamera.begin());
    return true;
}

bool makeSafeInformationMatrix(
		const std::vector<double> & values,
		double maxCondition,
		cv::Mat & information,
		std::string & reason)
{
	if(values.size() != 36 || !std::isfinite(maxCondition) || maxCondition < 1.0)
	{
		reason = "information matrix must contain 36 finite values";
		return false;
	}
	information = cv::Mat(6, 6, CV_64FC1);
	for(int row=0; row<6; ++row)
	{
		for(int col=0; col<6; ++col)
		{
			const double value = values[row*6+col];
			if(!std::isfinite(value))
			{
				reason = "information matrix is non-finite";
				return false;
			}
			information.at<double>(row,col) = value;
		}
	}
	for(int row=0; row<6; ++row)
	{
		if(information.at<double>(row,row) <= 0.0)
		{
			reason = "information matrix has a non-positive diagonal";
			return false;
		}
		for(int col=0; col<6; ++col)
		{
			if(std::fabs(information.at<double>(row,col)-information.at<double>(col,row)) > 1e-9)
			{
				reason = "information matrix is asymmetric";
				return false;
			}
		}
	}
	cv::Mat eigenvalues;
	if(!cv::eigen(information, eigenvalues) || eigenvalues.total() != 6)
	{
		reason = "information matrix eigendecomposition failed";
		return false;
	}
	const double largest = eigenvalues.at<double>(0);
	const double smallest = eigenvalues.at<double>(5);
	if(!std::isfinite(largest) || !std::isfinite(smallest) ||
			smallest <= 1e-9 || largest > 1000000.0 ||
			largest/smallest > maxCondition)
	{
		reason = "information matrix is not SPD or is ill-conditioned";
		return false;
	}
	return true;
}

bool safeToken(const std::string & token)
{
	if(token.size() < 8 || token.size() > 128)
	{
		return false;
	}
	for(std::string::const_iterator iter=token.begin(); iter!=token.end(); ++iter)
	{
		if(!((*iter >= 'a' && *iter <= 'z') || (*iter >= 'A' && *iter <= 'Z') ||
				(*iter >= '0' && *iter <= '9') || *iter == '-' || *iter == '_' || *iter == '.'))
		{
			return false;
		}
	}
	return true;
}

std::string jsonSafe(std::string value)
{
	for(std::string::iterator iter=value.begin(); iter!=value.end(); ++iter)
	{
		if(*iter == '"' || *iter == '\\' || (unsigned char)*iter < 0x20)
		{
			*iter = '_';
		}
	}
	return value;
}

} // namespace

CoreWrapper::CoreWrapper(const rclcpp::NodeOptions & options) :
		Node("rtabmap", options),
		rtabmap_sync::CommonDataSubscriber(*this, false),
		paused_(false),
		lastPose_(Transform::getIdentity()),
		lastPoseIntermediate_(false),
		latestNodeWasReached_(false),
		pubLocPoseOnlyWhenLocalizing_(false),
		graphLatched_(false),
		frameId_("base_link"),
		odomFrameId_(""),
		mapFrameId_("map"),
		groundTruthFrameId_(""), // e.g., "world"
		groundTruthBaseFrameId_(""), // e.g., "base_link_gt"
		configPath_(""),
		odomDefaultAngVariance_(0.001),
		odomDefaultLinVariance_(0.001),
		landmarkDefaultAngVariance_(0.001),
		landmarkDefaultLinVariance_(0.001),
		waitForTransform_(0.2),// 200 ms
		stalenessFactor_(0.0),
		useActionForGoal_(false),
		useSavedMap_(true),
		genScan_(false),
		genScanMaxDepth_(4.0),
		genScanMinDepth_(0.0),
		genDepth_(false),
		genDepthDecimation_(1),
		genDepthFillHolesSize_(0),
		genDepthFillIterations_(1),
		genDepthFillHolesError_(0.1),
		scanCloudMaxPoints_(0),
		scanCloudIs2d_(false),
		mapToOdom_(rtabmap::Transform::getIdentity()),
		transformThread_(0),
		tfThreadRunning_(false),
		interOdomSync_(0),
		stereoToDepth_(false),
		odomSensorSync_(false),
		rate_(Parameters::defaultRtabmapDetectionRate()),
		createIntermediateNodes_(Parameters::defaultRtabmapCreateIntermediateNodes()),
		mappingMaxNodes_(Parameters::defaultGridGlobalMaxNodes()),
		mappingAltitudeDelta_(Parameters::defaultGridGlobalAltitudeDelta()),
		alreadyRectifiedImages_(Parameters::defaultRtabmapImagesAlreadyRectified()),
		twoDMapping_(Parameters::defaultRegForce3DoF()),
		previousStamp_(0),
		ulogToRosout_(this),
		triggerNewMapBeforeNextUpdate_(false),
		resetQuarantineEnabled_(false),
		resetQuarantineLostCount_(3),
		resetQuarantineOdomInfoMaxAgeSec_(0.5),
		resetQuarantineMaxUnannouncedTranslation_(0.5),
		resetQuarantineMaxUnannouncedRotationDeg_(30.0),
		resetQuarantineExpectedOdomInfoNode_("rtabmap_rgbd_odometry"),
		resetQuarantineExpectedOdomNamespace_("/"),
		resetQuarantineExpectedSensorNode_("zed_rgbd_dual_stream_publisher"),
		resetQuarantineExpectedSensorNamespace_("/"),
		resetQuarantineExpectedImuNode_("zed_rgbd_dual_stream_publisher"),
		resetQuarantineExpectedImuNamespace_("/"),
		resetQuarantineExpectedImuFrame_("zed_left_camera_optical_frame"),
		resetQuarantineOdomTopic_(),
		resetQuarantineExpectedSensorFrame_("zed_left_camera_optical_frame"),
		resetQuarantineExpectedSensorToBase_("0 0 0 -0.5 0.5 -0.5 0.5"),
		resetQuarantineExpectedSensorToBaseTransform_(),
		resetQuarantineOdomPublisherBound_(false),
		resetQuarantineOdomPublisherCompromised_(false),
		resetQuarantineOdomPublisherGid_(),
		resetQuarantineOdomInfoPublisherBound_(false),
		resetQuarantineOdomInfoPublisherCompromised_(false),
		resetQuarantineOdomInfoPublisherGid_(),
		resetQuarantineRgbPublisherBound_(false),
		resetQuarantineRgbPublisherCompromised_(false),
		resetQuarantineRgbPublisherGid_(),
		resetQuarantineDepthPublisherBound_(false),
		resetQuarantineDepthPublisherCompromised_(false),
		resetQuarantineDepthPublisherGid_(),
		resetQuarantineCameraInfoPublisherBound_(false),
		resetQuarantineCameraInfoPublisherCompromised_(false),
		resetQuarantineCameraInfoPublisherGid_(),
		resetQuarantineImuPublisherBound_(false),
		resetQuarantineImuPublisherCompromised_(false),
		resetQuarantineImuPublisherGid_(),
		resetQuarantineLastImuStampNs_(0),
		resetQuarantineLastImuFingerprint_(),
		resetQuarantineImuEndpointCheckCounter_(0),
		resetQuarantineLostStreak_(0),
		resetQuarantineRecoveryPending_(false),
		resetQuarantineSuspectedReset_(false),
		resetQuarantineOdomInfoSeen_(false),
		resetQuarantineLastOdomInfoSteadyNs_(0),
		resetQuarantineLastOdomInfoStampNs_(0),
		resetQuarantineLostStartStampNs_(0),
		resetQuarantineRecoveryStampNs_(0),
		resetQuarantineEpochStartStampNs_(0),
		resetQuarantineEpoch_(0),
		safeRelocalizationEnabled_(false),
		safeRelocalizationStartNewMapOnFirstUpdate_(false),
		safeRelocalizationFirstUpdateHandled_(false),
		safeRelocalizationTimeoutSec_(8.0),
		safeRelocalizationMaxTranslation_(0.08),
		safeRelocalizationMaxRotation_(8.0),
		safeRelocalizationMinBridgeTranslation_(0.02),
		safeRelocalizationMinBridgeRotation_(2.0),
		safeRelocalizationMaxAnchorZ_(0.02),
		safeRelocalizationMaxAnchorTilt_(2.0),
		safeRelocalizationMaxOldPoseTranslation_(0.005),
		safeRelocalizationMaxOldPoseRotation_(0.5),
		safeRelocalizationMaxNewRigidTranslation_(0.03),
		safeRelocalizationMaxNewRigidRotation_(3.0),
		safeRelocalizationMaxNewRigidZError_(0.02),
		safeRelocalizationMaxNewRigidTiltError_(2.0),
		safeRelocalizationMaxGravityResidualIncrease_(0.5),
		safeRelocalizationMaxInformationCondition_(100000000.0),
		safeRelocalizationExpectedProposalNode_("apriltag_safe_relocalizer"),
		safeRelocalizationExpectedProposalNamespace_("/"),
		safeRelocalizationProposalPublisherBound_(false),
		safeRelocalizationProposalPublisherCompromised_(false),
		safeRelocalizationProposalPublisherGid_(),
		safeRelocalizationDatabaseFd_(-1),
		safeRelocalizationDatabaseDevice_(0),
		safeRelocalizationDatabaseInode_(0),
		safeRelocalizationDatabaseBindingValid_(false),
		safeRelocalizationFatal_(false),
		safeRelocalizationGraphRevision_(0)
{
	char * rosHomePath = getenv("ROS_HOME");
	std::string workingDir = rosHomePath?rosHomePath:UDirectory::homeDir()+"/.ros";
	databasePath_ = workingDir+"/"+rtabmap::Parameters::getDefaultDatabaseName();

	mapsManager_.init(*this, this->get_name(), true);

	syncData_.valid = false;

	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
	tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	bool publishTf = true;
	std::string initialPoseStr;
	tfDelay = 0.05; // 20 Hz
	tfTolerance = 0.1; // 100 ms
	std::string odomFrameIdInit;

	configPath_ = this->declare_parameter("config_path", configPath_);
	databasePath_ = this->declare_parameter("database_path", databasePath_);

	frameId_ = this->declare_parameter("frame_id", frameId_);
	this->get_parameter_or("odom_frame_id", odomFrameId_, odomFrameId_); // set to use odom from TF, ros2: declared later in CommonDataSubscriber
	odomFrameIdInit = this->declare_parameter("odom_frame_id_init", odomFrameIdInit); // set to publish map->odom TF before receiving odom topic
	mapFrameId_ = this->declare_parameter("map_frame_id", mapFrameId_);
	groundTruthFrameId_ = this->declare_parameter("ground_truth_frame_id", groundTruthFrameId_);
	groundTruthBaseFrameId_ = this->declare_parameter("ground_truth_base_frame_id", frameId_);

	if(!odomFrameIdInit.empty())
	{
		if(odomFrameId_.empty())
		{
			RCLCPP_INFO(get_logger(), "rtabmap: odom_frame_id_init = %s", odomFrameIdInit.c_str());
			odomFrameId_ = odomFrameIdInit;
		}
		else
		{
			RCLCPP_WARN(get_logger(), "odom_frame_id_init (%s) is ignored if odom_frame_id (%s) is set.", odomFrameIdInit.c_str(), odomFrameId_.c_str());
		}
	}

	int eventLevel = ULogger::kFatal;
	eventLevel = this->declare_parameter("log_to_rosout_level", eventLevel);
	UASSERT(eventLevel >= ULogger::kDebug && eventLevel <= ULogger::kFatal);
	ULogger::setEventLevel((ULogger::Level)eventLevel);

	publishTf = this->declare_parameter("publish_tf", publishTf);
	tfDelay = this->declare_parameter("tf_delay", tfDelay);
	tfTolerance = this->declare_parameter("tf_tolerance", tfTolerance);

	odomDefaultAngVariance_ = this->declare_parameter("odom_tf_angular_variance", odomDefaultAngVariance_);
	odomDefaultLinVariance_ = this->declare_parameter("odom_tf_linear_variance", odomDefaultLinVariance_);

	landmarkDefaultAngVariance_ = this->declare_parameter("landmark_angular_variance", landmarkDefaultAngVariance_);
	landmarkDefaultLinVariance_ = this->declare_parameter("landmark_linear_variance", landmarkDefaultLinVariance_);
	
	pubLocPoseOnlyWhenLocalizing_ = this->declare_parameter("pub_loc_pose_only_when_localizing", pubLocPoseOnlyWhenLocalizing_);
	waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);
	stalenessFactor_ = this->declare_parameter("staleness_factor", stalenessFactor_);
	initialPoseStr = this->declare_parameter("initial_pose", initialPoseStr);
	useActionForGoal_ = this->declare_parameter("use_action_for_goal", useActionForGoal_);
#ifndef WITH_NAV2_MSGS
	if(useActionForGoal_)
	{
		RCLCPP_ERROR(this->get_logger(), "rtabmap: Cannot enable use_action_for_goal because rtabmap_slam is not built with nav2_msgs support.");
		useActionForGoal_ = false;
	}
#endif
	useSavedMap_ = this->declare_parameter("use_saved_map", useSavedMap_);
	genScan_ = this->declare_parameter("gen_scan", genScan_);
	genScanMaxDepth_ = this->declare_parameter("gen_scan_max_depth", genScanMaxDepth_);
	genScanMinDepth_ = this->declare_parameter("gen_scan_min_depth", genScanMinDepth_);
	genDepth_ = this->declare_parameter("gen_depth", genDepth_);
	genDepthDecimation_ = this->declare_parameter("gen_depth_decimation", genDepthDecimation_);
	genDepthFillHolesSize_ = this->declare_parameter("gen_depth_fill_holes_size", genDepthFillHolesSize_);
	genDepthFillIterations_ = this->declare_parameter("gen_depth_fill_iterations", genDepthFillIterations_);
	genDepthFillHolesError_ = this->declare_parameter("gen_depth_fill_holes_error", genDepthFillHolesError_);
	scanCloudMaxPoints_ = this->declare_parameter("scan_cloud_max_points", scanCloudMaxPoints_);
	scanCloudIs2d_ = this->declare_parameter("scan_cloud_is_2d", scanCloudIs2d_);

	stereoToDepth_ = this->declare_parameter("stereo_to_depth", stereoToDepth_);
	odomSensorSync_ = this->declare_parameter("odom_sensor_sync", odomSensorSync_);
	resetQuarantineEnabled_ = this->declare_parameter(
			"reset_quarantine_enabled", resetQuarantineEnabled_);
	resetQuarantineLostCount_ = this->declare_parameter(
			"reset_quarantine_lost_count", resetQuarantineLostCount_);
	resetQuarantineOdomInfoMaxAgeSec_ = this->declare_parameter(
			"reset_quarantine_odom_info_max_age_sec", resetQuarantineOdomInfoMaxAgeSec_);
	resetQuarantineMaxUnannouncedTranslation_ = this->declare_parameter(
			"reset_quarantine_max_unannounced_translation", resetQuarantineMaxUnannouncedTranslation_);
	resetQuarantineMaxUnannouncedRotationDeg_ = this->declare_parameter(
			"reset_quarantine_max_unannounced_rotation_deg", resetQuarantineMaxUnannouncedRotationDeg_);
	resetQuarantineExpectedOdomInfoNode_ = this->declare_parameter(
			"reset_quarantine_expected_odom_info_node", resetQuarantineExpectedOdomInfoNode_);
	resetQuarantineExpectedOdomNamespace_ = this->declare_parameter(
			"reset_quarantine_expected_odom_namespace", resetQuarantineExpectedOdomNamespace_);
	resetQuarantineExpectedSensorNode_ = this->declare_parameter(
			"reset_quarantine_expected_sensor_node", resetQuarantineExpectedSensorNode_);
	resetQuarantineExpectedSensorNamespace_ = this->declare_parameter(
			"reset_quarantine_expected_sensor_namespace", resetQuarantineExpectedSensorNamespace_);
	resetQuarantineExpectedImuNode_ = this->declare_parameter(
			"reset_quarantine_expected_imu_node", resetQuarantineExpectedImuNode_);
	resetQuarantineExpectedImuNamespace_ = this->declare_parameter(
			"reset_quarantine_expected_imu_namespace", resetQuarantineExpectedImuNamespace_);
	resetQuarantineExpectedImuFrame_ = this->declare_parameter(
			"reset_quarantine_expected_imu_frame", resetQuarantineExpectedImuFrame_);
	resetQuarantineExpectedSensorFrame_ = this->declare_parameter(
			"reset_quarantine_expected_sensor_frame", resetQuarantineExpectedSensorFrame_);
	resetQuarantineExpectedSensorToBase_ = this->declare_parameter(
			"reset_quarantine_expected_sensor_to_base", resetQuarantineExpectedSensorToBase_);
	resetQuarantineExpectedSensorToBaseTransform_ =
			rtabmap::Transform::fromString(resetQuarantineExpectedSensorToBase_);
	safeRelocalizationEnabled_ = this->declare_parameter(
			"safe_relocalization_enabled", safeRelocalizationEnabled_);
	safeRelocalizationStartNewMapOnFirstUpdate_ = this->declare_parameter(
			"safe_relocalization_start_new_map_on_first_update",
			safeRelocalizationStartNewMapOnFirstUpdate_);
	safeRelocalizationTimeoutSec_ = this->declare_parameter(
			"safe_relocalization_timeout_sec", safeRelocalizationTimeoutSec_);
	safeRelocalizationMaxTranslation_ = this->declare_parameter(
			"safe_relocalization_max_translation", safeRelocalizationMaxTranslation_);
	safeRelocalizationMaxRotation_ = this->declare_parameter(
			"safe_relocalization_max_rotation", safeRelocalizationMaxRotation_);
	safeRelocalizationMinBridgeTranslation_ = this->declare_parameter(
			"safe_relocalization_min_bridge_translation", safeRelocalizationMinBridgeTranslation_);
	safeRelocalizationMinBridgeRotation_ = this->declare_parameter(
			"safe_relocalization_min_bridge_rotation", safeRelocalizationMinBridgeRotation_);
	safeRelocalizationMaxAnchorZ_ = this->declare_parameter(
			"safe_relocalization_max_anchor_z", safeRelocalizationMaxAnchorZ_);
	safeRelocalizationMaxAnchorTilt_ = this->declare_parameter(
			"safe_relocalization_max_anchor_tilt", safeRelocalizationMaxAnchorTilt_);
	safeRelocalizationMaxOldPoseTranslation_ = this->declare_parameter(
			"safe_relocalization_max_old_pose_translation", safeRelocalizationMaxOldPoseTranslation_);
	safeRelocalizationMaxOldPoseRotation_ = this->declare_parameter(
			"safe_relocalization_max_old_pose_rotation", safeRelocalizationMaxOldPoseRotation_);
	safeRelocalizationMaxNewRigidTranslation_ = this->declare_parameter(
			"safe_relocalization_max_new_rigid_translation", safeRelocalizationMaxNewRigidTranslation_);
	safeRelocalizationMaxNewRigidRotation_ = this->declare_parameter(
			"safe_relocalization_max_new_rigid_rotation", safeRelocalizationMaxNewRigidRotation_);
	safeRelocalizationMaxNewRigidZError_ = this->declare_parameter(
			"safe_relocalization_max_new_rigid_z_error", safeRelocalizationMaxNewRigidZError_);
	safeRelocalizationMaxNewRigidTiltError_ = this->declare_parameter(
			"safe_relocalization_max_new_rigid_tilt_error", safeRelocalizationMaxNewRigidTiltError_);
	safeRelocalizationMaxGravityResidualIncrease_ = this->declare_parameter(
			"safe_relocalization_max_gravity_residual_increase", safeRelocalizationMaxGravityResidualIncrease_);
	safeRelocalizationMaxInformationCondition_ = this->declare_parameter(
			"safe_relocalization_max_information_condition", safeRelocalizationMaxInformationCondition_);
	safeRelocalizationDatabaseSha256_ = this->declare_parameter(
			"safe_relocalization_database_sha256", std::string());
	safeRelocalizationSessionToken_ = this->declare_parameter(
			"safe_relocalization_session_token", std::string());
	safeRelocalizationExpectedProposalNode_ = this->declare_parameter(
			"safe_relocalization_expected_proposal_node", safeRelocalizationExpectedProposalNode_);
	safeRelocalizationExpectedProposalNamespace_ = this->declare_parameter(
			"safe_relocalization_expected_proposal_namespace", safeRelocalizationExpectedProposalNamespace_);
	if(resetQuarantineLostCount_ < 1 || resetQuarantineLostCount_ > 100)
	{
		throw std::runtime_error("reset_quarantine_lost_count must be in [1,100]");
	}
	if(resetQuarantineOdomInfoMaxAgeSec_ < 0.05 || resetQuarantineOdomInfoMaxAgeSec_ > 10.0)
	{
		throw std::runtime_error("reset_quarantine_odom_info_max_age_sec must be in [0.05,10.0]");
	}
	if(!std::isfinite(resetQuarantineMaxUnannouncedTranslation_) ||
			resetQuarantineMaxUnannouncedTranslation_ <= 0.0 ||
			resetQuarantineMaxUnannouncedTranslation_ > 10.0)
	{
		throw std::runtime_error("reset_quarantine_max_unannounced_translation must be in (0,10]");
	}
	if(!std::isfinite(resetQuarantineMaxUnannouncedRotationDeg_) ||
			resetQuarantineMaxUnannouncedRotationDeg_ <= 0.0 ||
			resetQuarantineMaxUnannouncedRotationDeg_ > 180.0)
	{
		throw std::runtime_error("reset_quarantine_max_unannounced_rotation_deg must be in (0,180]");
	}
	if(safeRelocalizationEnabled_ && !resetQuarantineEnabled_)
	{
		throw std::runtime_error(
				"safe_relocalization_enabled requires reset_quarantine_enabled so every odometry input is source-bound");
	}
	if(resetQuarantineEnabled_ &&
			(resetQuarantineExpectedOdomInfoNode_.empty() ||
			 resetQuarantineExpectedOdomInfoNode_.find('/') != std::string::npos ||
			 resetQuarantineExpectedOdomNamespace_.empty() ||
			 resetQuarantineExpectedOdomNamespace_[0] != '/' ||
			 resetQuarantineExpectedSensorNode_.empty() ||
			 resetQuarantineExpectedSensorNode_.find('/') != std::string::npos ||
			 resetQuarantineExpectedSensorNamespace_.empty() ||
			 resetQuarantineExpectedSensorNamespace_[0] != '/' ||
			 resetQuarantineExpectedImuNode_.empty() ||
			 resetQuarantineExpectedImuNode_.find('/') != std::string::npos ||
			 resetQuarantineExpectedImuNamespace_.empty() ||
			 resetQuarantineExpectedImuNamespace_[0] != '/' ||
			 resetQuarantineExpectedImuFrame_.empty() ||
			 resetQuarantineExpectedImuFrame_.find('/') != std::string::npos ||
			 resetQuarantineExpectedImuFrame_ != resetQuarantineExpectedSensorFrame_ ||
			 resetQuarantineExpectedSensorFrame_.empty() ||
			 resetQuarantineExpectedSensorFrame_.find('/') != std::string::npos ||
			 resetQuarantineExpectedSensorToBaseTransform_.isNull()))
	{
		throw std::runtime_error(
				"reset quarantine expected odometry node/namespace identity is invalid");
	}
	if(!std::isfinite(safeRelocalizationTimeoutSec_) ||
			!std::isfinite(safeRelocalizationMaxTranslation_) ||
			!std::isfinite(safeRelocalizationMaxRotation_) ||
			!std::isfinite(safeRelocalizationMinBridgeTranslation_) ||
			!std::isfinite(safeRelocalizationMinBridgeRotation_) ||
			!std::isfinite(safeRelocalizationMaxAnchorZ_) ||
			!std::isfinite(safeRelocalizationMaxAnchorTilt_) ||
			!std::isfinite(safeRelocalizationMaxOldPoseTranslation_) ||
			!std::isfinite(safeRelocalizationMaxOldPoseRotation_) ||
			!std::isfinite(safeRelocalizationMaxNewRigidTranslation_) ||
			!std::isfinite(safeRelocalizationMaxNewRigidRotation_) ||
			!std::isfinite(safeRelocalizationMaxNewRigidZError_) ||
			!std::isfinite(safeRelocalizationMaxNewRigidTiltError_) ||
			!std::isfinite(safeRelocalizationMaxGravityResidualIncrease_) ||
			!std::isfinite(safeRelocalizationMaxInformationCondition_) ||
			safeRelocalizationTimeoutSec_ < 1.0 || safeRelocalizationTimeoutSec_ > 30.0 ||
			safeRelocalizationMaxTranslation_ <= 0.0 || safeRelocalizationMaxTranslation_ > 0.5 ||
			safeRelocalizationMaxRotation_ <= 0.0 || safeRelocalizationMaxRotation_ > 45.0 ||
			safeRelocalizationMinBridgeTranslation_ <= 0.0 || safeRelocalizationMinBridgeTranslation_ > 1.0 ||
			safeRelocalizationMinBridgeRotation_ <= 0.0 || safeRelocalizationMinBridgeRotation_ > 45.0 ||
			safeRelocalizationMaxAnchorZ_ <= 0.0 || safeRelocalizationMaxAnchorZ_ > 0.1 ||
			safeRelocalizationMaxAnchorTilt_ <= 0.0 || safeRelocalizationMaxAnchorTilt_ > 10.0 ||
			safeRelocalizationMaxOldPoseTranslation_ <= 0.0 || safeRelocalizationMaxOldPoseTranslation_ > 0.05 ||
			safeRelocalizationMaxOldPoseRotation_ <= 0.0 || safeRelocalizationMaxOldPoseRotation_ > 5.0 ||
			safeRelocalizationMaxNewRigidTranslation_ <= 0.0 || safeRelocalizationMaxNewRigidTranslation_ > 0.1 ||
			safeRelocalizationMaxNewRigidRotation_ <= 0.0 || safeRelocalizationMaxNewRigidRotation_ > 10.0 ||
			safeRelocalizationMaxNewRigidZError_ <= 0.0 || safeRelocalizationMaxNewRigidZError_ > 0.1 ||
			safeRelocalizationMaxNewRigidTiltError_ <= 0.0 || safeRelocalizationMaxNewRigidTiltError_ > 10.0 ||
			safeRelocalizationMaxGravityResidualIncrease_ < 0.0 || safeRelocalizationMaxGravityResidualIncrease_ > 5.0 ||
			safeRelocalizationMaxInformationCondition_ < 1.0 || safeRelocalizationMaxInformationCondition_ > 1e12)
	{
		throw std::runtime_error("safe relocalization limits are outside their guarded ranges");
	}
	if(safeRelocalizationEnabled_ &&
			(safeRelocalizationDatabaseSha256_.size() != 64 ||
			 safeRelocalizationDatabaseSha256_.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos))
	{
		throw std::runtime_error("safe_relocalization_database_sha256 must be 64 hex (64 zeros for a new map, copied source-DB SHA256 for continuation)");
	}
	if(safeRelocalizationEnabled_ &&
			(safeRelocalizationSessionToken_.size() != 64 ||
			 safeRelocalizationSessionToken_.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos))
	{
		throw std::runtime_error("safe_relocalization_session_token must be a fresh 64-hex lifecycle secret");
	}
	if(safeRelocalizationEnabled_ &&
			(safeRelocalizationExpectedProposalNode_.empty() ||
			 safeRelocalizationExpectedProposalNode_.find('/') != std::string::npos ||
			 safeRelocalizationExpectedProposalNamespace_.empty() ||
			 safeRelocalizationExpectedProposalNamespace_[0] != '/'))
	{
		throw std::runtime_error("safe relocalization expected proposal node/namespace identity is invalid");
	}
	if(safeRelocalizationEnabled_)
	{
		std::transform(safeRelocalizationDatabaseSha256_.begin(), safeRelocalizationDatabaseSha256_.end(),
				safeRelocalizationDatabaseSha256_.begin(), [](unsigned char c){return (char)std::tolower(c);});
		std::transform(safeRelocalizationSessionToken_.begin(), safeRelocalizationSessionToken_.end(),
				safeRelocalizationSessionToken_.begin(), [](unsigned char c){return (char)std::tolower(c);});
		safeRelocalizationSessionTokenSha256_ = safeSha256(safeRelocalizationSessionToken_);
		if(!initialPoseStr.empty())
		{
			throw std::runtime_error(
					"initial_pose is forbidden while safe relocalization owns graph alignment");
		}
	}
	if(resetQuarantineEnabled_)
	{
		resetQuarantineOdomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
				"reset_quarantine_odom",
				rclcpp::SensorDataQoS(),
				std::bind(&CoreWrapper::resetQuarantineOdomCallback, this,
						std::placeholders::_1, std::placeholders::_2));
		// Match the managed high-resolution publisher and synchronized reader.
		// A best-effort duplicate reader can lose fragmented image payloads even
		// while the reliable synchronized reader receives the same exact frame.
		resetQuarantineRgbSub_ = this->create_subscription<sensor_msgs::msg::Image>(
				"reset_quarantine_rgb", rclcpp::QoS(10).reliable(),
				std::bind(&CoreWrapper::resetQuarantineRgbCallback, this,
						std::placeholders::_1, std::placeholders::_2));
		resetQuarantineDepthSub_ = this->create_subscription<sensor_msgs::msg::Image>(
				"reset_quarantine_depth", rclcpp::QoS(10).reliable(),
				std::bind(&CoreWrapper::resetQuarantineDepthCallback, this,
						std::placeholders::_1, std::placeholders::_2));
		resetQuarantineCameraInfoSub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
				"reset_quarantine_camera_info", rclcpp::QoS(10).reliable(),
				std::bind(&CoreWrapper::resetQuarantineCameraInfoCallback, this,
						std::placeholders::_1, std::placeholders::_2));
		resetQuarantineOdomTopic_ = resetQuarantineOdomSub_->get_topic_name();
		resetQuarantineOdomInfoSub_ = this->create_subscription<rtabmap_msgs::msg::OdomInfo>(
				"reset_quarantine_odom_info",
				rclcpp::SensorDataQoS(),
				std::bind(&CoreWrapper::resetQuarantineOdomInfoCallback, this,
						std::placeholders::_1, std::placeholders::_2));
		RCLCPP_WARN(
				this->get_logger(),
				"reset quarantine enabled: lost_count=%d odom_info_max_age=%.3fs expected_odom_node=%s namespace=%s; both /odom and /odom_info source endpoints are bound once and any second publisher fails closed",
				resetQuarantineLostCount_,
				resetQuarantineOdomInfoMaxAgeSec_,
				resetQuarantineExpectedOdomInfoNode_.c_str(),
				resetQuarantineExpectedOdomNamespace_.c_str());
	}

	RCLCPP_INFO(this->get_logger(), "rtabmap: frame_id      = \"%s\"", frameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "rtabmap: odom_frame_id = \"%s\"", odomFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "rtabmap: ground_truth_frame_id = \"%s\" -> ground_truth_base_frame_id = \"%s\"",
			groundTruthFrameId_.c_str(),
			groundTruthBaseFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "rtabmap: map_frame_id  = \"%s\"", mapFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "rtabmap: log_to_rosout_level  = %d", eventLevel);
	RCLCPP_INFO(this->get_logger(), "rtabmap: initial_pose  = \"%s\"", initialPoseStr.c_str());
	RCLCPP_INFO(this->get_logger(), "rtabmap: use_action_for_goal  = %s", useActionForGoal_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "rtabmap: tf_delay      = %f", tfDelay);
	RCLCPP_INFO(this->get_logger(), "rtabmap: tf_tolerance  = %f", tfTolerance);
	RCLCPP_INFO(this->get_logger(), "rtabmap: odom_sensor_sync   = %s", odomSensorSync_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "rtabmap: pub_loc_pose_only_when_localizing = %s", pubLocPoseOnlyWhenLocalizing_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "rtabmap: wait_for_transform = %f", waitForTransform_);
	if(stalenessFactor_!=0.0 && stalenessFactor_ < 1.0) {
		RCLCPP_ERROR(this->get_logger(), "rtabmap: staleness_factor should be 0 (disabled) or >= 1 (value that multiplies the detection update period). Current value is %f, setting it to 0...",
			stalenessFactor_);
		stalenessFactor_ = 0.0;
	}
	RCLCPP_INFO(this->get_logger(), "rtabmap: staleness_factor = %f", stalenessFactor_);
	if(this->isSubscribedToStereo())
	{
		RCLCPP_INFO(this->get_logger(), "rtabmap: stereo_to_depth = %s", stereoToDepth_?"true":"false");
	}

	RCLCPP_INFO(get_logger(), "rtabmap: gen_scan  = %s", genScan_?"true":"false");
	if(genScan_)
	{
		RCLCPP_INFO(get_logger(), "rtabmap: gen_scan_max_depth  = %f", genScanMaxDepth_);
		RCLCPP_INFO(get_logger(), "rtabmap: gen_scan_min_depth  = %f", genScanMinDepth_);
	}

	RCLCPP_INFO(get_logger(), "rtabmap: gen_depth  = %s", genDepth_?"true":"false");
	if(genDepth_)
	{
		RCLCPP_INFO(get_logger(), "rtabmap: gen_depth_decimation        = %d", genDepthDecimation_);
		RCLCPP_INFO(get_logger(), "rtabmap: gen_depth_fill_holes_size   = %d", genDepthFillHolesSize_);
		RCLCPP_INFO(get_logger(), "rtabmap: gen_depth_fill_iterations   = %d", genDepthFillIterations_);
		RCLCPP_INFO(get_logger(), "rtabmap: gen_depth_fill_holes_error  = %f", genDepthFillHolesError_);
	}
	if(this->isSubscribedToScan3d())
	{
		RCLCPP_INFO(get_logger(), "rtabmap: scan_cloud_max_points = %d", scanCloudMaxPoints_);
		RCLCPP_INFO(get_logger(), "rtabmap: scan_cloud_is_2d = %s", scanCloudIs2d_?"true":"false");
	}

	// Create the processing timer  
	processingCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	syncTimer_ = this->create_wall_timer(0s, std::bind(&CoreWrapper::processAsync, this), processingCallbackGroup_);  
	syncTimer_->cancel();

	rclcpp::SubscriptionOptions subOptions;
	subOptions.callback_group = processingCallbackGroup_;
    acceptedImageReceiptPub_ = this->create_publisher<std_msgs::msg::String>(
        "/wbrov/accepted_image_input", rclcpp::QoS(rclcpp::KeepLast(64)).reliable().transient_local());
	resetEpochPub_ = this->create_publisher<std_msgs::msg::UInt64>(
			"/wbrov/reset_epoch", rclcpp::QoS(1).reliable().transient_local());
	if(safeRelocalizationEnabled_)
	{
		safeRelocalizationStatusPub_ = this->create_publisher<std_msgs::msg::String>(
				"/wbrov/safe_relocalization/status", rclcpp::QoS(10).reliable());
		safeRelocalizationStatePub_ = this->create_publisher<std_msgs::msg::String>(
				"/wbrov/safe_relocalization/state", rclcpp::QoS(1).reliable().transient_local());
		safeRelocalizationProposalSub_ = this->create_subscription<std_msgs::msg::String>(
				"/wbrov/safe_relocalization/proposal",
				rclcpp::QoS(10).reliable(),
				std::bind(&CoreWrapper::safeRelocalizationProposalCallback, this,
						std::placeholders::_1, std::placeholders::_2),
				subOptions);
		RCLCPP_WARN(this->get_logger(),
				"safe relocalization enabled: one JSON transaction with exactly three bridges is required; all three leave-one-out folds must pass before commit; database identity=%s (%s)",
				safeRelocalizationDatabaseSha256_.c_str(),
				safeRelocalizationDatabaseSha256_ == std::string(64, '0')?
						"new-map sentinel":"externally hashed continuation source");
	}

	infoPub_ = this->create_publisher<rtabmap_msgs::msg::Info>("info", 1);
	mapDataPub_ = this->create_publisher<rtabmap_msgs::msg::MapData>("mapData", 1);
	mapGraphPub_ = this->create_publisher<rtabmap_msgs::msg::MapGraph>("mapGraph", rclcpp::QoS(1).reliable().durability(mapsManager_.isLatching()?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	odomCachePub_ = this->create_publisher<rtabmap_msgs::msg::MapGraph>("mapOdomCache", 1);
	landmarksPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("landmarks", 1);
	labelsPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("labels", 1);
	mapPathPub_ = this->create_publisher<nav_msgs::msg::Path>("mapPath", 1);
	localGridObstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_grid_obstacle", 1);
	localGridEmpty_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_grid_empty", 1);
	localGridGround_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_grid_ground", 1);
	localizationPosePub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("localization_pose", 1);
	initialPoseSub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 5, std::bind(&CoreWrapper::initialPoseCallback, this, std::placeholders::_1), subOptions);

	// planning topics
	goalSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 5, std::bind(&CoreWrapper::goalCallback, this, std::placeholders::_1), subOptions);
	goalNodeSub_ = this->create_subscription<rtabmap_msgs::msg::Goal>("goal_node", 5, std::bind(&CoreWrapper::goalNodeCallback, this, std::placeholders::_1), subOptions);
	nextMetricGoalPub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_out", 1);
	goalReachedPub_ = this->create_publisher<std_msgs::msg::Bool>("goal_reached", 1);
	globalPathPub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
	localPathPub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 1);
	globalPathNodesPub_ = this->create_publisher<rtabmap_msgs::msg::Path>("global_path_nodes", 1);
	localPathNodesPub_ = this->create_publisher<rtabmap_msgs::msg::Path>("local_path_nodes", 1);

	configPath_ = uReplaceChar(configPath_, '~', UDirectory::homeDir());
	databasePath_ = uReplaceChar(databasePath_, '~', UDirectory::homeDir());
#ifndef _WIN32
	if(configPath_.size() && configPath_.at(0) != '/')
	{
		configPath_ = UDirectory::currentDir(true) + configPath_;
	}
	if(databasePath_.size() && databasePath_.at(0) != '/')
	{
		databasePath_ = UDirectory::currentDir(true) + databasePath_;
	}
#endif

	ParametersMap allParameters = Parameters::getDefaultParameters();
	// remove Odom parameters
	for(ParametersMap::iterator iter=allParameters.begin(); iter!=allParameters.end();)
	{
		if(iter->first.find("Odom") == 0)
		{
			allParameters.erase(iter++);
		}
		else
		{
			++iter;
		}
	}
	uInsert(allParameters, ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "true")); // default true in ROS
	uInsert(allParameters, ParametersPair(Parameters::kRtabmapWorkingDirectory(), workingDir)); // change default to ~/.ros

	// load parameters
	loadParameters(configPath_, parameters_);
	for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end();)
	{
		if(iter->first.find("Odom") == 0)
		{
			parameters_.erase(iter++);
		}
		else
		{
			++iter;
		}
	}

	// declare parameters
	paused_ = this->declare_parameter("is_rtabmap_paused", paused_);
	if(paused_)
	{
		RCLCPP_WARN(get_logger(), "Node paused... don't forget to call service \"resume\" to start rtabmap.");
	}

	const std::map<std::string, rclcpp::ParameterValue> & overrides = this->get_node_parameters_interface()->get_parameter_overrides();
	for(ParametersMap::iterator iter=allParameters.begin(); iter!=allParameters.end(); ++iter)
	{
		std::string vStr = this->declare_parameter(iter->first, iter->second);
		if(overrides.find(iter->first) != overrides.end())
		{
			RCLCPP_INFO(this->get_logger(), "Setting RTAB-Map parameter \"%s\"=\"%s\" (rosparam)", iter->first.c_str(), vStr.c_str());

			if(iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
			{
				vStr = uReplaceChar(vStr, '~', UDirectory::homeDir());
			}
			else if(iter->first.compare(Parameters::kKpDictionaryPath()) == 0)
			{
				vStr = uReplaceChar(vStr, '~', UDirectory::homeDir());
			}
			uInsert(parameters_, ParametersPair(iter->first, vStr));
		}
	}

	//parse input arguments
	std::vector<std::string> tmpList = get_node_options().arguments();
	std::vector<std::string> argList;
	for(unsigned int i=0; i<tmpList.size(); ++i)
	{
	    // Issue with ros2 launch files in which we cannot pass a 
	    // list of strings as argument (they will appear in same string)
	    std::list<std::string> v = uSplit(tmpList[i]);
	    for(std::list<std::string>::iterator iter=v.begin(); iter!=v.end(); ++iter)
	    {
	        argList.push_back(*iter);
	    }
	}
	
	char ** argv = new char*[argList.size()];
	bool deleteDbOnStart = false;
	deleteDbOnStart = this->declare_parameter("delete_db_on_start", deleteDbOnStart);
	for(unsigned int i=0; i<argList.size(); ++i)
	{
		argv[i] = &argList[i].at(0);
		if(strcmp(argv[i], "--delete_db_on_start") == 0 || strcmp(argv[i], "-d") == 0)
		{
			deleteDbOnStart = true;
		}
	}
	rtabmap::ParametersMap parameters = rtabmap::Parameters::parseArguments(argList.size(), argv);
	delete [] argv;
	if(safeRelocalizationEnabled_)
	{
		const bool newMapSentinel =
				safeRelocalizationDatabaseSha256_ == std::string(64, '0');
		const bool databaseExistsAndIsNonEmpty = !databasePath_.empty() &&
				UFile::exists(databasePath_) && UFile::length(databasePath_) > 0;
		if(deleteDbOnStart && !newMapSentinel)
		{
			throw std::runtime_error(
					"safe relocalization requires the all-zero database SHA sentinel when delete_db_on_start creates a new map");
		}
		if(!deleteDbOnStart && newMapSentinel && databaseExistsAndIsNonEmpty)
		{
			throw std::runtime_error(
					"all-zero safe_relocalization_database_sha256 is valid only for a new/empty database");
		}
		if(!newMapSentinel && !databaseExistsAndIsNonEmpty)
		{
			throw std::runtime_error(
					"a continuation database SHA requires an existing non-empty copied source database");
		}
		if(!newMapSentinel && !safeRelocalizationStartNewMapOnFirstUpdate_)
		{
			throw std::runtime_error(
					"safe continuation requires safe_relocalization_start_new_map_on_first_update=true so live nodes cannot share epoch zero with saved history");
		}
		std::string bindingReason;
		if(!safeDatabaseBinding(
				databasePath_, safeRelocalizationDatabaseSha256_,
				safeRelocalizationDatabaseFd_, safeRelocalizationDatabaseDevice_,
				safeRelocalizationDatabaseInode_, bindingReason))
		{
			throw std::runtime_error("safe relocalization database binding failed before Memory open: "+bindingReason);
		}
		// safeDatabaseBinding creates and holds the new zero-length inode itself;
		// unlinking it here would reopen the check/use race.
		if(newMapSentinel)
		{
			deleteDbOnStart = false;
		}
	}
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		uInsert(parameters_, ParametersPair(iter->first, iter->second));
		RCLCPP_INFO(this->get_logger(), "Update RTAB-Map parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
	}

	// Backward compatibility
	for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=Parameters::getRemovedParameters().begin();
		iter!=Parameters::getRemovedParameters().end();
		++iter)
	{
		std::string paramValue;
		rclcpp::Parameter parameter;
		if(get_parameter(iter->first, parameter))
		{
			paramValue = parameter.as_string();
		}
		if(!paramValue.empty())
		{
			if(!iter->second.second.empty() && parameters_.find(iter->second.second)!=parameters_.end())
			{
				RCLCPP_WARN(this->get_logger(), "Rtabmap: Parameter name changed: \"%s\" -> \"%s\". The new parameter is already used with value \"%s\", ignoring the old one with value \"%s\".",
						iter->first.c_str(), iter->second.second.c_str(), parameters_.find(iter->second.second)->second.c_str(), paramValue.c_str());
			}
			else if(iter->second.first)
			{
				// can be migrated
				uInsert(parameters_, ParametersPair(iter->second.second, paramValue));
				RCLCPP_WARN(this->get_logger(), "Rtabmap: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
						iter->first.c_str(), iter->second.second.c_str(), paramValue.c_str());
			}
			else
			{
				if(iter->second.second.empty())
				{
					RCLCPP_ERROR(this->get_logger(), "Rtabmap: Parameter \"%s\" doesn't exist anymore!",
							iter->first.c_str());
				}
				else
				{
					RCLCPP_ERROR(this->get_logger(), "Rtabmap: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
							iter->first.c_str(), iter->second.second.c_str());
				}
			}
		}
	}

	// Backward compatibility (MapsManager)
	mapsManager_.backwardCompatibilityParameters(*this, parameters_);

	int gridSensor = Parameters::defaultGridSensor();
	if((this->isSubscribedToScan2d() || this->isSubscribedToScan3d() || genScan_) && parameters_.find(Parameters::kGridSensor()) == parameters_.end())
	{
		RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to 0 (default 1) as \"subscribe_scan\" or \"subscribe_scan_cloud\" or \"gen_scan\" is "
				"true. The occupancy grid map will be constructed from "
				"laser scans. To get occupancy grid map from cloud projection, set \"%s\" "
				"to true. To suppress this warning, "
				"add <param name=\"%s\" type=\"string\" value=\"0\"/>",
				Parameters::kGridSensor().c_str(),
				Parameters::kGridSensor().c_str(),
				Parameters::kGridSensor().c_str());
		parameters_.insert(ParametersPair(Parameters::kGridSensor(), "0"));
	}
	Parameters::parse(parameters_, Parameters::kGridSensor(), gridSensor);
	if((this->isSubscribedToScan2d() || this->isSubscribedToScan3d() || genScan_) && parameters_.find(Parameters::kGridRangeMax()) == parameters_.end() && gridSensor==0)
	{
		RCLCPP_INFO(this->get_logger(), "Setting \"%s\" parameter to 0 (default %f) as \"subscribe_scan\" or \"subscribe_scan_cloud\" or \"gen_scan\" is true and %s is 0.",
				Parameters::kGridRangeMax().c_str(),
				Parameters::defaultGridRangeMax(),
				Parameters::kGridSensor().c_str());
		parameters_.insert(ParametersPair(Parameters::kGridRangeMax(), "0"));
	}
	if(this->isSubscribedToScan3d() && !scanCloudIs2d_ && parameters_.find(Parameters::kIcpPointToPlaneRadius()) == parameters_.end())
	{
		RCLCPP_INFO(this->get_logger(), "Setting \"%s\" parameter to 0 (default %f) as \"subscribe_scan_cloud\" is true.",
				Parameters::kIcpPointToPlaneRadius().c_str(),
				Parameters::defaultIcpPointToPlaneRadius());
		parameters_.insert(ParametersPair(Parameters::kIcpPointToPlaneRadius(), "0"));
	}
	int regStrategy = Parameters::defaultRegStrategy();
	Parameters::parse(parameters_, Parameters::kRegStrategy(), regStrategy);
	if(parameters_.find(Parameters::kRGBDProximityPathMaxNeighbors()) == parameters_.end() &&
		(regStrategy == Registration::kTypeIcp || regStrategy == Registration::kTypeVisIcp))
	{
		if(this->isSubscribedToScan2d() || (this->isSubscribedToScan3d() && scanCloudIs2d_))
		{
			RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to 10 (default 0) as \"%s\" is "
					"true and \"%s\" uses ICP. Proximity detection by space will be also done by merging close "
					"scans. To disable, set \"%s\" to 0. To suppress this warning, "
					"add <param name=\"%s\" type=\"string\" value=\"10\"/>",
					Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
					this->isSubscribedToScan2d()?"subscribe_scan":"scan_cloud_is_2d",
					Parameters::kRegStrategy().c_str(),
					Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
					Parameters::kRGBDProximityPathMaxNeighbors().c_str());
			parameters_.insert(ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "10"));
		}
		else if(this->isSubscribedToScan3d())
		{
			RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to 1 (default 0) as \"subscribe_scan_cloud\" is "
					"true and \"%s\" uses ICP. To disable, set \"%s\" to 0. To suppress this warning, "
					"add <param name=\"%s\" type=\"string\" value=\"1\"/>",
					Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
					Parameters::kRegStrategy().c_str(),
					Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
					Parameters::kRGBDProximityPathMaxNeighbors().c_str());
			parameters_.insert(ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "1"));
		}
	}

	int estimationType = Parameters::defaultVisEstimationType();
	Parameters::parse(parameters_, Parameters::kVisEstimationType(), estimationType);

	// modify default parameters with those in the database
	if(!deleteDbOnStart && !(safeRelocalizationEnabled_ &&
			safeRelocalizationDatabaseSha256_ == std::string(64, '0')))
	{
		ParametersMap dbParameters;
		rtabmap::DBDriver * driver = rtabmap::DBDriver::create();
		if(driver->openConnection(databasePath_))
		{
			dbParameters = driver->getLastParameters(); // parameter migration is already done
		}
		delete driver;
		for(ParametersMap::iterator iter=dbParameters.begin(); iter!=dbParameters.end(); ++iter)
		{
			if(iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
			{
				// ignore working directory
				continue;
			}
			if(parameters_.find(iter->first) == parameters_.end() &&
				allParameters.find(iter->first) != allParameters.end() &&
				allParameters.find(iter->first)->second.compare(iter->second) !=0)
			{
				RCLCPP_INFO(this->get_logger(), "Update RTAB-Map parameter \"%s\"=\"%s\" from database", iter->first.c_str(), iter->second.c_str());
				parameters_.insert(*iter);
			}
		}
	}
	ParametersMap modifiedParameters = parameters_;
	// Add all other parameters (not copied if already exists)
	parameters_.insert(allParameters.begin(), allParameters.end());
	if(safeRelocalizationEnabled_)
	{
		bool incrementalMemory = false;
		bool initAllNodes = false;
		bool landmarksIgnored = false;
		bool priorsIgnored = false;
		bool loopGPS = true;
		bool force3DoF = false;
		bool reduceGraph = true;
		bool rehearsalUpdatesId = true;
		bool createIntermediateNodes = true;
		float optimizeRepairRadius = 0.0f;
		float rehearsalSimilarity = 0.0f;
		float timeThreshold = -1.0f;
		int optimizerIterations = 0;
		int memoryThreshold = -1;
		Parameters::parse(parameters_, Parameters::kMemIncrementalMemory(), incrementalMemory);
		Parameters::parse(parameters_, Parameters::kMemInitWMWithAllNodes(), initAllNodes);
		Parameters::parse(parameters_, Parameters::kOptimizerLandmarksIgnored(), landmarksIgnored);
		Parameters::parse(parameters_, Parameters::kOptimizerPriorsIgnored(), priorsIgnored);
		Parameters::parse(parameters_, Parameters::kRtabmapLoopGPS(), loopGPS);
		Parameters::parse(parameters_, Parameters::kRegForce3DoF(), force3DoF);
		Parameters::parse(parameters_, Parameters::kMemReduceGraph(), reduceGraph);
		Parameters::parse(parameters_, Parameters::kMemRehearsalIdUpdatedToNewOne(), rehearsalUpdatesId);
		Parameters::parse(parameters_, Parameters::kRGBDOptimizeMaxErrorRepairRadius(), optimizeRepairRadius);
		Parameters::parse(parameters_, Parameters::kMemRehearsalSimilarity(), rehearsalSimilarity);
		Parameters::parse(parameters_, Parameters::kRtabmapTimeThr(), timeThreshold);
		Parameters::parse(parameters_, Parameters::kOptimizerIterations(), optimizerIterations);
		Parameters::parse(parameters_, Parameters::kRtabmapMemoryThr(), memoryThreshold);
		Parameters::parse(parameters_, Parameters::kRtabmapCreateIntermediateNodes(), createIntermediateNodes);
		if(!incrementalMemory || !initAllNodes || !landmarksIgnored || !priorsIgnored || loopGPS || force3DoF ||
				reduceGraph || rehearsalUpdatesId || rehearsalSimilarity != 1.0f ||
				timeThreshold != 0.0f || memoryThreshold != 0 ||
				createIntermediateNodes || odomSensorSync_ ||
				optimizeRepairRadius != 0.0f || optimizerIterations <= 0)
		{
			throw std::runtime_error(
					"safe relocalization requires Mem/IncrementalMemory=true, Mem/InitWMWithAllNodes=true, "
					"Optimizer/LandmarksIgnored=true, Optimizer/PriorsIgnored=true, Rtabmap/LoopGPS=false, Reg/Force3DoF=false, Mem/ReduceGraph=false, "
					"Mem/RehearsalIdUpdatedToNewOne=false, Mem/RehearsalSimilarity=1, "
					"Rtabmap/TimeThr=0, Rtabmap/MemoryThr=0, Rtabmap/CreateIntermediateNodes=false, odom_sensor_sync=false, "
					"RGBD/OptimizeMaxErrorRepairRadius=0 and Optimizer/Iterations>0");
		}
	}

	if(parameters_.find(Parameters::kRtabmapDetectionRate()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kRtabmapDetectionRate(), rate_);
		RCLCPP_INFO(this->get_logger(), "RTAB-Map detection rate = %f Hz", rate_);
	}
	if(parameters_.find(Parameters::kRtabmapCreateIntermediateNodes()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kRtabmapCreateIntermediateNodes(), createIntermediateNodes_);
		if(createIntermediateNodes_)
		{
			RCLCPP_INFO(this->get_logger(), "Create intermediate nodes");
			if(rate_ == 0.0f)
			{
				bool interOdomInfo = false;
				if(get_parameter("subscribe_inter_odom_info", interOdomInfo))
				{
					RCLCPP_INFO(this->get_logger(), "Subscribe to inter odom + info messages");
					interOdomSync_ = new message_filters::Synchronizer<MyExactInterOdomSyncPolicy>(MyExactInterOdomSyncPolicy(100), interOdomSyncSub_, interOdomInfoSyncSub_);
					interOdomSync_->registerCallback(std::bind(&CoreWrapper::interOdomInfoCallback, this, std::placeholders::_1, std::placeholders::_2));
					interOdomSyncSub_.subscribe(this, "inter_odom", RCLCPP_QOS(100, rclcpp::ReliabilityPolicy::SystemDefault));
					interOdomInfoSyncSub_.subscribe(this, "inter_odom_info", RCLCPP_QOS(100, rclcpp::ReliabilityPolicy::SystemDefault));
				}
				else
				{
					RCLCPP_INFO(this->get_logger(), "Subscribe to inter odom messages");
					interOdomSub_ = this->create_subscription<nav_msgs::msg::Odometry>("inter_odom", 100, std::bind(&CoreWrapper::interOdomCallback, this, std::placeholders::_1), subOptions);
				}

			}
		}
	}
	if(parameters_.find(Parameters::kGridGlobalMaxNodes()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kGridGlobalMaxNodes(), mappingMaxNodes_);
		if(mappingMaxNodes_>0)
		{
			RCLCPP_INFO(get_logger(), "Max mapping nodes = %d", mappingMaxNodes_);
		}
	}
	if(parameters_.find(Parameters::kGridGlobalAltitudeDelta()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kGridGlobalAltitudeDelta(), mappingAltitudeDelta_);
		if(mappingAltitudeDelta_>0.0)
		{
			RCLCPP_INFO(this->get_logger(), "Mapping altitude delta = %f", mappingAltitudeDelta_);
		}
	}
	if(parameters_.find(Parameters::kRtabmapImagesAlreadyRectified()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kRtabmapImagesAlreadyRectified(), alreadyRectifiedImages_);
	}
	if(parameters_.find(Parameters::kRegForce3DoF()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kRegForce3DoF(), twoDMapping_);
	}

	if(deleteDbOnStart)
	{
		if(UFile::erase(databasePath_) == 0)
		{
			RCLCPP_INFO(this->get_logger(), "rtabmap: Deleted database \"%s\" (--delete_db_on_start or -d are set).", databasePath_.c_str());
		}
	}

	if(databasePath_.size())
	{
		RCLCPP_INFO(this->get_logger(), "rtabmap: Using database from \"%s\" (%ld MB).", databasePath_.c_str(), UFile::length(databasePath_)/(1024*1024));
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "rtabmap: database_path parameter not set, the map will not be saved.");
	}

	mapsManager_.setParameters(parameters_);

	// Init RTAB-Map
	if(safeRelocalizationEnabled_)
	{
		// Revalidate immediately before the driver/Memory opens the path. This
		// catches an atomic pathname replacement after argument processing.
		safeRelocalizationDatabaseBindingValid_ = false;
		std::string bindingReason;
		if(!safeDatabaseBinding(
				databasePath_, safeRelocalizationDatabaseSha256_,
				safeRelocalizationDatabaseFd_, safeRelocalizationDatabaseDevice_,
				safeRelocalizationDatabaseInode_, bindingReason))
		{
			throw std::runtime_error("safe relocalization database binding changed before Memory open: "+bindingReason);
		}
	}
	const bool safeNewMapSentinel = safeRelocalizationEnabled_ &&
			safeRelocalizationDatabaseSha256_ == std::string(64, '0');
	// The safe new-map sentinel is already a path-bound zero-length inode.
	// Skip Rtabmap::init's generic pre-open parameter migration; the non-public
	// preboundEmptyIsNew call capability reaches Memory/DBDriver and creates the
	// schema on that same inode.
	rtabmap_.init(parameters_, databasePath_, !safeNewMapSentinel, safeNewMapSentinel);
	if(safeRelocalizationEnabled_)
	{
		if(!safeDatabasePathStillBound(
				databasePath_, safeRelocalizationDatabaseFd_,
				safeRelocalizationDatabaseDevice_, safeRelocalizationDatabaseInode_))
		{
			rtabmap_.close(false);
			::close(safeRelocalizationDatabaseFd_);
			safeRelocalizationDatabaseFd_ = -1;
			throw std::runtime_error(
					"safe relocalization database pathname changed while Memory opened it");
		}
		safeRelocalizationDatabaseBindingValid_ = true;
	}
	rtabmap_.setSafeRelocalizationGuardEnabled(safeRelocalizationEnabled_);
	++safeRelocalizationGraphRevision_;
	resetQuarantineEpoch_.store(rtabmap_.getResetEpoch(), std::memory_order_release);
	publishResetEpoch();

	if(rtabmap_.getMemory())
	{
		if(useSavedMap_)
		{
			float xMin, yMin, gridCellSize;
			cv::Mat map = rtabmap_.getMemory()->load2DMap(xMin, yMin, gridCellSize);
			if(!map.empty())
			{
				RCLCPP_INFO(this->get_logger(), "rtabmap: 2D occupancy grid map loaded (%dx%d).", map.cols, map.rows);
				mapsManager_.set2DMap(map, xMin, yMin, gridCellSize, rtabmap_.getLocalOptimizedPoses(), rtabmap_.getMemory());
			}
		}
		if(rtabmap_.getMemory()->getWorkingMem().size()>1)
		{
			RCLCPP_INFO(get_logger(), "rtabmap: Working Memory = %d, Local map = %d.",
					(int)rtabmap_.getMemory()->getWorkingMem().size()-1,
					(int)rtabmap_.getLocalOptimizedPoses().size());
		}

		if(databasePath_.size())
		{
			RCLCPP_INFO(get_logger(), "rtabmap: Database version = \"%s\".", rtabmap_.getMemory()->getDatabaseVersion().c_str());
		}

		if(rtabmap_.getMemory()->isIncremental())
		{
			RCLCPP_INFO(get_logger(), "rtabmap: SLAM mode (%s=true)", Parameters::kMemIncrementalMemory().c_str());
		}
		else
		{
			RCLCPP_INFO(get_logger(), "rtabmap: Localization mode (%s=false)", Parameters::kMemIncrementalMemory().c_str());
		}
	}

	// setup services
	const std::string servicePrefix = get_name() + std::string("/");
	updateSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "update_parameters", std::bind(&CoreWrapper::updateRtabmapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	resetSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "reset", std::bind(&CoreWrapper::resetRtabmapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	pauseSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "pause", std::bind(&CoreWrapper::pauseRtabmapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	resumeSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "resume", std::bind(&CoreWrapper::resumeRtabmapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	loadDatabaseSrv_ = this->create_service<rtabmap_msgs::srv::LoadDatabase>(servicePrefix + "load_database", std::bind(&CoreWrapper::loadDatabaseCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	triggerNewMapSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "trigger_new_map", std::bind(&CoreWrapper::triggerNewMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	backupDatabase_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "backup", std::bind(&CoreWrapper::backupDatabaseCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	detectMoreLoopClosuresSrv_ = this->create_service<rtabmap_msgs::srv::DetectMoreLoopClosures>(servicePrefix + "detect_more_loop_closures", std::bind(&CoreWrapper::detectMoreLoopClosuresCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	globalBundleAdjustmentSrv_ = this->create_service<rtabmap_msgs::srv::GlobalBundleAdjustment>(servicePrefix + "global_bundle_adjustment", std::bind(&CoreWrapper::globalBundleAdjustmentCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	cleanupLocalGridsSrv_ = this->create_service<rtabmap_msgs::srv::CleanupLocalGrids>(servicePrefix + "cleanup_local_grids", std::bind(&CoreWrapper::cleanupLocalGridsCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	setModeLocalizationSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "set_mode_localization", std::bind(&CoreWrapper::setModeLocalizationCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	setModeMappingSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "set_mode_mapping", std::bind(&CoreWrapper::setModeMappingCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	getNodeDataSrv_ = this->create_service<rtabmap_msgs::srv::GetNodeData>(servicePrefix + "get_node_data", std::bind(&CoreWrapper::getNodeDataCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	getMapDataSrv_ = this->create_service<rtabmap_msgs::srv::GetMap>(servicePrefix + "get_map_data", std::bind(&CoreWrapper::getMapDataCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	getMapData2Srv_ = this->create_service<rtabmap_msgs::srv::GetMap2>(servicePrefix + "get_map_data2", std::bind(&CoreWrapper::getMapData2Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	getMapSrv_ = this->create_service<nav_msgs::srv::GetMap>(servicePrefix + "get_map", std::bind(&CoreWrapper::getMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	getProbMapSrv_ = this->create_service<nav_msgs::srv::GetMap>(servicePrefix + "get_prob_map", std::bind(&CoreWrapper::getProbMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	publishMapDataSrv_ = this->create_service<rtabmap_msgs::srv::PublishMap>(servicePrefix + "publish_map", std::bind(&CoreWrapper::publishMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	getPlanSrv_ = this->create_service<nav_msgs::srv::GetPlan>(servicePrefix + "get_plan", std::bind(&CoreWrapper::getPlanCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	getPlanNodesSrv_ = this->create_service<rtabmap_msgs::srv::GetPlan>(servicePrefix + "get_plan_nodes", std::bind(&CoreWrapper::getPlanNodesCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	setGoalSrv_ = this->create_service<rtabmap_msgs::srv::SetGoal>(servicePrefix + "set_goal", std::bind(&CoreWrapper::setGoalCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	cancelGoalSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "cancel_goal", std::bind(&CoreWrapper::cancelGoalCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	setLabelSrv_ = this->create_service<rtabmap_msgs::srv::SetLabel>(servicePrefix + "set_label", std::bind(&CoreWrapper::setLabelCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	listLabelsSrv_ = this->create_service<rtabmap_msgs::srv::ListLabels>(servicePrefix + "list_labels", std::bind(&CoreWrapper::listLabelsCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	removeLabelSrv_ = this->create_service<rtabmap_msgs::srv::RemoveLabel>(servicePrefix + "remove_label", std::bind(&CoreWrapper::removeLabelCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	addLinkSrv_ = this->create_service<rtabmap_msgs::srv::AddLink>(servicePrefix + "add_link", std::bind(&CoreWrapper::addLinkCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	getNodesInRadiusSrv_ = this->create_service<rtabmap_msgs::srv::GetNodesInRadius>(servicePrefix + "get_nodes_in_radius", std::bind(&CoreWrapper::getNodesInRadiusCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	octomapBinarySrv_ = this->create_service<octomap_msgs::srv::GetOctomap>(servicePrefix + "octomap_binary", std::bind(&CoreWrapper::octomapBinaryCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	octomapFullSrv_ = this->create_service<octomap_msgs::srv::GetOctomap>(servicePrefix + "octomap_full", std::bind(&CoreWrapper::octomapFullCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
#endif
#endif
	//private services
	setLogDebugSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "log_debug", std::bind(&CoreWrapper::setLogDebug, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	setLogInfoSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "log_info", std::bind(&CoreWrapper::setLogInfo, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	setLogWarnSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "log_warning", std::bind(&CoreWrapper::setLogWarn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);
	setLogErrorSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "log_error", std::bind(&CoreWrapper::setLogError, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rclcpp::ServicesQoS(), processingCallbackGroup_);

	int optimizeIterations = 0;
	Parameters::parse(parameters_, Parameters::kOptimizerIterations(), optimizeIterations);
	if(publishTf && optimizeIterations != 0)
	{
		tfThreadRunning_ = true;
		transformThread_ = new std::thread([&](){
			if(tfDelay == 0)
				return;
			rclcpp::Rate r(1.0 / tfDelay);
			while(tfThreadRunning_)
			{
				mapToOdomMutex_.lock();
				if(!odomFrameId_.empty())
				{
					geometry_msgs::msg::TransformStamped msg;
					rtabmap_conversions::transformToGeometryMsg(mapToOdom_, msg.transform);
					msg.child_frame_id = odomFrameId_;
					msg.header.frame_id = mapFrameId_;
					msg.header.stamp = now() + rclcpp::Duration::from_seconds(tfTolerance);
					tfBroadcaster_->sendTransform(msg);
				}
				mapToOdomMutex_.unlock();
				try {
					r.sleep();
				}
				catch(std::exception & e) {
					if(rclcpp::ok()) {
						RCLCPP_ERROR(this->get_logger(),
							"Could not sleep: \"%s\", TF \"%s\"->\"%s\" won't be published anymore!",
							e.what(), mapFrameId_.c_str(), odomFrameId_.c_str());
					} // else: the node may have been shutdown
					break;
				}
			}
		});
	}
	else if(publishTf)
	{
		RCLCPP_WARN(this->get_logger(), "Graph optimization is disabled (%s=0), the tf between frame \"%s\" and odometry frame will not be published. You can safely ignore this warning if you are using map_optimizer node.",
				Parameters::kOptimizerIterations().c_str(), mapFrameId_.c_str());
	}

	std::vector<diagnostic_updater::DiagnosticTask*> tasks;
	double localizationThreshold = 0.0f;
	localizationThreshold = this->declare_parameter("loc_thr", localizationThreshold);
	if(rtabmap_.getMemory() && !rtabmap_.getMemory()->isIncremental() && localizationThreshold > 0.0)
	{
		RCLCPP_INFO(this->get_logger(), "rtabmap: loc_thr  = %f", localizationThreshold);
		localizationDiagnostic_.setLocalizationThreshold(localizationThreshold);
		tasks.push_back(&localizationDiagnostic_);
	}

	RCLCPP_INFO(this->get_logger(), "Setup callbacks");
	setupCallbacks(*this, tasks); // do it at the end
	if((resetQuarantineEnabled_ || safeRelocalizationEnabled_) &&
			!this->isSubscribedToOdom())
	{
		throw std::runtime_error(
				"guarded mapping requires subscribe_odom so every graph pose can match a source-bound odometry message");
	}
	if((resetQuarantineEnabled_ || safeRelocalizationEnabled_) &&
			(!this->isDataSubscribed() || !this->isSubscribedToDepth()))
	{
		throw std::runtime_error(
				"guarded mapping requires the supported exact RGB-D subscription; image-only/default callbacks are forbidden");
	}
	if(safeRelocalizationEnabled_ && this->isSubscribedToSensorData())
	{
		throw std::runtime_error(
				"safe relocalization forbids subscribe_sensor_data because embedded global-pose/GPS priors are not trusted");
	}
	if(!this->isDataSubscribed())
	{
		bool isRGBD = uStr2Bool(parameters_.at(Parameters::kRGBDEnabled()).c_str());
		if(isRGBD)
		{
			RCLCPP_WARN(this->get_logger(), "ROS param subscribe_depth, subscribe_stereo and subscribe_rgbd are false, but RTAB-Map "
					  "parameter \"%s\" is true! Please set subscribe_depth, subscribe_stereo or subscribe_rgbd "
					  "to true to use rtabmap node for RGB-D SLAM, set \"%s\" to false for loop closure "
					  "detection on images-only or set subscribe_rgb to true to localize a single RGB camera against pre-built 3D map.",
					  Parameters::kRGBDEnabled().c_str(),
					  Parameters::kRGBDEnabled().c_str());
		}
		std::string imageTopic = this->get_node_topics_interface()->resolve_topic_name("image"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
#ifdef PRE_ROS_LYRICAL
		image_transport::TransportHints hints(this); // using "image_transport" parameter
		defaultSub_ = image_transport::create_subscription(this, imageTopic, std::bind(&CoreWrapper::defaultCallback, this, std::placeholders::_1), hints.getTransport(), rclcpp::QoS(this->getTopicQueueSize()).reliability((rmw_qos_reliability_policy_t)qosImage_).get_rmw_qos_profile(), subOptions);
#else
		image_transport::TransportHints hints(*this); // using "image_transport" parameter
		defaultSub_ = image_transport::create_subscription(*this, imageTopic, std::bind(&CoreWrapper::defaultCallback, this, std::placeholders::_1), hints.getTransport(), rclcpp::QoS(this->getTopicQueueSize()).reliability((rmw_qos_reliability_policy_t)qosImage_), subOptions);
#endif

		RCLCPP_INFO(this->get_logger(), "\n%s subscribed to:\n   %s", get_name(), defaultSub_.getTopic().c_str());
	}
	else if(!this->isSubscribedToDepth() &&
			!this->isSubscribedToStereo() &&
			!this->isSubscribedToRGBD() &&
			!this->isSubscribedToRGB() &&
			(this->isSubscribedToScan2d() || this->isSubscribedToScan3d() || this->isSubscribedToOdom()) &&
			!this->isSubscribedToSensorData())
	{
		RCLCPP_WARN(this->get_logger(), "There is no image subscription, bag-of-words loop closure detection will be disabled...");
		int kpMaxFeatures = Parameters::defaultKpMaxFeatures();
		int registrationStrategy = Parameters::defaultRegStrategy();
		Parameters::parse(parameters_, Parameters::kKpMaxFeatures(), kpMaxFeatures);
		Parameters::parse(parameters_, Parameters::kRegStrategy(), registrationStrategy);
		bool updateParams = false;
		if(kpMaxFeatures!= -1)
		{
			uInsert(parameters_, ParametersPair(Parameters::kKpMaxFeatures(), "-1"));
			RCLCPP_WARN(this->get_logger(), "Setting %s=-1 (bag-of-words disabled)", Parameters::kKpMaxFeatures().c_str());
			updateParams = true;
		}
		if((this->isSubscribedToScan2d() || this->isSubscribedToScan3d()) && registrationStrategy != 1)
		{
			uInsert(parameters_, ParametersPair(Parameters::kRegStrategy(), "1"));
			RCLCPP_WARN(this->get_logger(), "Setting %s=1 (ICP)", Parameters::kRegStrategy().c_str());
			updateParams = true;

			if(modifiedParameters.find(Parameters::kRGBDProximityPathMaxNeighbors()) == modifiedParameters.end())
			{
				if(this->isSubscribedToScan2d())
				{
					RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to 10 (default 0) as \"subscribe_scan\" is "
							"true and \"%s\" uses ICP. Proximity detection by space will be also done by merging close "
							"scans. To disable, set \"%s\" to 0. To suppress this warning, "
							"add <param name=\"%s\" type=\"string\" value=\"10\"/>",
							Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
							Parameters::kRegStrategy().c_str(),
							Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
							Parameters::kRGBDProximityPathMaxNeighbors().c_str());
					uInsert(parameters_, ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "10"));
				}
				else if(this->isSubscribedToScan3d())
				{
					RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to 1 (default 0) as \"subscribe_scan_cloud\" is "
							"true and \"%s\" uses ICP. To disable, set \"%s\" to 0. To suppress this warning, "
							"add <param name=\"%s\" type=\"string\" value=\"1\"/>",
							Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
							Parameters::kRegStrategy().c_str(),
							Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
							Parameters::kRGBDProximityPathMaxNeighbors().c_str());
					uInsert(parameters_, ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "1"));
				}
			}
		}
		if(updateParams)
		{
			rtabmap_.parseParameters(parameters_);
		}
	}
	
	if(!this->isSubscribedToOdom() && odomFrameId_.empty())
	{
		bool isRGBD = uStr2Bool(parameters_.at(Parameters::kRGBDEnabled()).c_str());
		if(isRGBD) {
			RCLCPP_ERROR(this->get_logger(), "\"subscribe_odom\" or \"odom_frame_id\" should be used when \"%s\" is enabled!", Parameters::kRGBDEnabled().c_str());
		}
	}

	// Set initial pose if set
	if(!initialPoseStr.empty())
	{
		Transform intialPose = Transform::fromString(initialPoseStr);
		if(!intialPose.isNull())
		{
			RCLCPP_INFO(this->get_logger(), "Setting initial pose: \"%s\"", intialPose.prettyPrint().c_str());
			rtabmap_.setInitialPose(intialPose);
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Invalid initial_pose: \"%s\"", initialPoseStr.c_str());
		}
	}

	// Update declared parameters
	std::vector<rclcpp::Parameter> rosParameters;
	for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		rosParameters.push_back(rclcpp::Parameter(iter->first, iter->second));
	}
	this->set_parameters(rosParameters);
	if(safeRelocalizationEnabled_)
	{
		safeRelocalizationParameterGuardHandle_ = this->add_on_set_parameters_callback(
				[this](const std::vector<rclcpp::Parameter> & updates)
				{
					rcl_interfaces::msg::SetParametersResult result;
					result.successful = true;
					ParametersMap candidate = parameters_;
					for(std::vector<rclcpp::Parameter>::const_iterator iter=updates.begin();
							iter!=updates.end(); ++iter)
					{
						if(iter->get_name().find("safe_relocalization_") == 0 ||
								iter->get_name().find("reset_quarantine_") == 0 ||
								iter->get_name() == "odom_sensor_sync")
						{
							result.successful = false;
							result.reason = "safe_relocalization_* and reset_quarantine_* parameters are immutable after startup";
							return result;
						}
						if(candidate.find(iter->get_name()) != candidate.end())
						{
							result.successful = false;
							result.reason = "all RTAB-Map core parameters are immutable while safe relocalization is enabled";
							return result;
						}
					}
					bool incrementalMemory = false;
					bool initAllNodes = false;
					bool landmarksIgnored = false;
					bool priorsIgnored = false;
					bool loopGPS = true;
					bool force3DoF = false;
					bool reduceGraph = true;
					bool rehearsalUpdatesId = true;
					bool createIntermediateNodes = true;
					float optimizeRepairRadius = 0.0f;
					float rehearsalSimilarity = 0.0f;
					float timeThreshold = -1.0f;
					int optimizerIterations = 0;
					int memoryThreshold = -1;
					Parameters::parse(candidate, Parameters::kMemIncrementalMemory(), incrementalMemory);
					Parameters::parse(candidate, Parameters::kMemInitWMWithAllNodes(), initAllNodes);
					Parameters::parse(candidate, Parameters::kOptimizerLandmarksIgnored(), landmarksIgnored);
					Parameters::parse(candidate, Parameters::kOptimizerPriorsIgnored(), priorsIgnored);
					Parameters::parse(candidate, Parameters::kRtabmapLoopGPS(), loopGPS);
					Parameters::parse(candidate, Parameters::kRegForce3DoF(), force3DoF);
					Parameters::parse(candidate, Parameters::kMemReduceGraph(), reduceGraph);
					Parameters::parse(candidate, Parameters::kMemRehearsalIdUpdatedToNewOne(), rehearsalUpdatesId);
					Parameters::parse(candidate, Parameters::kRGBDOptimizeMaxErrorRepairRadius(), optimizeRepairRadius);
					Parameters::parse(candidate, Parameters::kMemRehearsalSimilarity(), rehearsalSimilarity);
					Parameters::parse(candidate, Parameters::kRtabmapTimeThr(), timeThreshold);
					Parameters::parse(candidate, Parameters::kOptimizerIterations(), optimizerIterations);
					Parameters::parse(candidate, Parameters::kRtabmapMemoryThr(), memoryThreshold);
					Parameters::parse(candidate, Parameters::kRtabmapCreateIntermediateNodes(), createIntermediateNodes);
					if(!incrementalMemory || !initAllNodes || !landmarksIgnored || !priorsIgnored || loopGPS || force3DoF ||
							reduceGraph || rehearsalUpdatesId || rehearsalSimilarity != 1.0f ||
							timeThreshold != 0.0f || memoryThreshold != 0 ||
							createIntermediateNodes || odomSensorSync_ ||
							optimizeRepairRadius != 0.0f || optimizerIterations <= 0)
					{
						result.successful = false;
						result.reason = "safe relocalization requires immutable full-memory mapping with rehearsal/reduction disabled, ignored landmarks and priors, GPS loops off, 6DoF, repair radius zero and positive optimizer iterations";
					}
					return result;
				});
	}

	// Setup callback groups for any subscriptions that should not be affected by main processing thread.
	userDataAsyncCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	globalPoseAsyncCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	gpsAsyncCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	landmarkCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	imuCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	envSensorAsyncCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	rclcpp::SubscriptionOptions userDataAsyncSubOptions;
	rclcpp::SubscriptionOptions globalPoseAsyncSubOptions;
	rclcpp::SubscriptionOptions gpsAsyncSubOptions;
	rclcpp::SubscriptionOptions landmarkSubOptions;
	rclcpp::SubscriptionOptions imuSubOptions;
	rclcpp::SubscriptionOptions envSensorAsyncSubOptions;
	userDataAsyncSubOptions.callback_group = userDataAsyncCallbackGroup_;
	globalPoseAsyncSubOptions.callback_group = globalPoseAsyncCallbackGroup_;
	gpsAsyncSubOptions.callback_group = gpsAsyncCallbackGroup_;
	landmarkSubOptions.callback_group = landmarkCallbackGroup_;
	imuSubOptions.callback_group = imuCallbackGroup_;
	envSensorAsyncSubOptions.callback_group = envSensorAsyncCallbackGroup_;

	int qosGPS = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	int qosIMU = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	int qosEnvSensor = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	qosGPS = this->declare_parameter("qos_gps", qosGPS);
	qosIMU = this->declare_parameter("qos_imu", qosIMU);
	qosEnvSensor = this->declare_parameter("qos_env_sensor", qosEnvSensor);
	userDataAsyncSub_ = this->create_subscription<rtabmap_msgs::msg::UserData>("user_data_async", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosUserData_), std::bind(&CoreWrapper::userDataAsyncCallback, this, std::placeholders::_1), userDataAsyncSubOptions);
	globalPoseAsyncSub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("global_pose", 1, std::bind(&CoreWrapper::globalPoseAsyncCallback, this, std::placeholders::_1), globalPoseAsyncSubOptions);
	gpsFixAsyncSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosGPS), std::bind(&CoreWrapper::gpsFixAsyncCallback, this, std::placeholders::_1), gpsAsyncSubOptions);
	envSensorAsyncSub_ = this->create_subscription<rtabmap_msgs::msg::EnvSensor>("env_sensor", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosEnvSensor), std::bind(&CoreWrapper::envSensorAsyncCallback, this, std::placeholders::_1), envSensorAsyncSubOptions);
	// Safe relocalization accepts AprilTag evidence only through the authenticated
	// three-view proposal transaction. Native landmark subscriptions would let
	// an unauthenticated topic persist negative-ID poses and constraints first.
	if(!safeRelocalizationEnabled_)
	{
	landmarkDetectionSub_ = this->create_subscription<rtabmap_msgs::msg::LandmarkDetection>("landmark_detection", 1, std::bind(&CoreWrapper::landmarkDetectionAsyncCallback, this, std::placeholders::_1), landmarkSubOptions);
	landmarkDetectionsSub_ = this->create_subscription<rtabmap_msgs::msg::LandmarkDetections>("landmark_detections", 1, std::bind(&CoreWrapper::landmarkDetectionsAsyncCallback, this, std::placeholders::_1), landmarkSubOptions);
#ifdef WITH_APRILTAG_MSGS
	tagDetectionsSub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("tag_detections", 5, std::bind(&CoreWrapper::tagDetectionsAsyncCallback, this, std::placeholders::_1), landmarkSubOptions);
	apriltagSub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("apriltag/detections", 5, std::bind(&CoreWrapper::apriltagAsyncCallback, this, std::placeholders::_1), landmarkSubOptions);
#endif
#ifdef WITH_ARUCO_MSGS
	arucoSub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>("aruco/detections", 5, std::bind(&CoreWrapper::arucoAsyncCallback, this, std::placeholders::_1), landmarkSubOptions);
#endif
#ifdef WITH_ARUCO_OPENCV_MSGS
	arucoOpencvSub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>("aruco_opencv/detections", 5, std::bind(&CoreWrapper::arucoOpencvAsyncCallback, this, std::placeholders::_1), landmarkSubOptions);
#endif
#ifdef WITH_ARUCO_MARKERS_MSGS
	arucoMarkersSub_ = this->create_subscription<aruco_markers_msgs::msg::MarkerArray>("aruco_markers/detections", 5, std::bind(&CoreWrapper::arucoMarkersAsyncCallback, this, std::placeholders::_1), landmarkSubOptions);
#endif
#ifdef WITH_ROS2_ARUCO_INTERFACES
	arucoInterfacesSub_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_interfaces/detections", 5, std::bind(&CoreWrapper::arucoInterfacesAsyncCallback, this, std::placeholders::_1), landmarkSubOptions);
#endif
#ifdef WITH_FIDUCIAL_MSGS
	fiducialTransfromsSub_ = this->create_subscription<fiducial_msgs::msg::FiducialTransformArray>("fiducial_transforms", 5, std::bind(&CoreWrapper::fiducialDetectionsAsyncCallback, this, std::placeholders::_1), landmarkSubOptions);
#endif
	}
	imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(100).reliability((rmw_qos_reliability_policy_t)qosIMU), std::bind(&CoreWrapper::imuAsyncCallback, this, std::placeholders::_1, std::placeholders::_2), imuSubOptions);
	republishNodeDataSub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(servicePrefix+"republish_node_data", 1, std::bind(&CoreWrapper::republishNodeDataCallback, this, std::placeholders::_1), subOptions);
	parametersClient_ = std::make_shared<rclcpp::AsyncParametersClient>(this, std::string(), rclcpp::ParametersQoS(), processingCallbackGroup_);

	auto on_parameter_event_callback =
			[this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
			{
				// /parameter_events is an ordinary DDS topic: event->node is not an
				// authenticated publisher identity.  In safe mode, accepting an event
				// which merely claims our node name would bypass the on-set callback
				// and mutate the graph contract without a parameter-service request.
				if(safeRelocalizationEnabled_)
				{
					return;
				}
				std::string ns = get_namespace();
				if(ns != "/")
				{
					ns += "/";
				}
				if(event->node.compare(ns+get_name()) != 0)
				{
					return;
				}
				RCLCPP_INFO(this->get_logger(), "Parameters event received!");
				if(event->changed_parameters.size())
				{
					for(size_t i=0; i<event->changed_parameters.size(); ++i)
					{
						std::string key = event->changed_parameters[i].name;
						if(parameters_.find(key) != parameters_.end())
						{
							std::string vStr;
							switch(event->changed_parameters[i].value.type) {
							    case 1:
							        vStr = uBool2Str(event->changed_parameters[i].value.bool_value);
							        break;
							    case 2:
							        vStr = uNumber2Str((int)event->changed_parameters[i].value.integer_value);
							        break;
							    case 3:
							        vStr = uNumber2Str(event->changed_parameters[i].value.double_value);
							        break;
							    case 4:
							        vStr = event->changed_parameters[i].value.string_value;
							        break;
							    default:
							        RCLCPP_WARN(this->get_logger(), "Parameter type %d received for parameter %s is not supported, use string type.",
							                (int)event->changed_parameters[i].value.type, key.c_str());
							        continue;
							}
							RCLCPP_INFO(this->get_logger(), "Setting RTAB-Map parameter \"%s\"=\"%s\"", key.c_str(), vStr.c_str());
							parameters_.at(key) = vStr;
						}
					}
					applyParameters();
				}
			};

	// Setup callback for changes to parameters.
	rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> paramOptions;
	paramOptions.callback_group = processingCallbackGroup_;
	if(!safeRelocalizationEnabled_)
	{
		parameterEventSub_ = parametersClient_->on_parameter_event(
			on_parameter_event_callback,
			rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events)),
			paramOptions);
	}
}

CoreWrapper::~CoreWrapper()
{
	if(transformThread_)
	{
		tfThreadRunning_ = false;
		transformThread_->join();
		delete transformThread_;
	}

	this->saveParameters(configPath_);

	const bool fatalSafeRelocalization =
			safeRelocalizationFatal_.load(std::memory_order_acquire);
	printf("rtabmap: %s database/long-term memory... (located at %s)\n",
			fatalSafeRelocalization?"Discarding unsafe in-memory changes from":"Saving",
			databasePath_.c_str());
	bool saveDatabase = !fatalSafeRelocalization;
	if(saveDatabase && rtabmap_.getMemory())
	{
		if(!rtabmap_.getMemory()->isReadOnly())
		{
			// save the grid map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
			if(!pixels.empty())
			{
				printf("rtabmap: 2D occupancy grid map saved.\n");
				rtabmap_.getMemory()->save2DMap(pixels, xMin, yMin, gridCellSize);
			}
		}
		else
		{
			printf("rtabmap: Database is read-only, the current state of the memory is not saved.\n");
			saveDatabase = false;
		}
	}

	rtabmap_.close(saveDatabase);
	if(safeRelocalizationDatabaseFd_ >= 0)
	{
		::close(safeRelocalizationDatabaseFd_);
		safeRelocalizationDatabaseFd_ = -1;
	}
	printf("rtabmap: Database/long-term memory close done (save=%s, located at %s, %ld MB)\n",
			saveDatabase?"true":"false", databasePath_.c_str(),
			UFile::length(databasePath_)/(1024*1024));

	delete interOdomSync_;
}

void CoreWrapper::loadParameters(const std::string & configFile, ParametersMap & parameters)
{
	if(!configFile.empty())
	{
		RCLCPP_INFO(this->get_logger(), "Loading parameters from %s", configFile.c_str());
		if(!UFile::exists(configFile.c_str()))
		{
			RCLCPP_WARN(this->get_logger(), "Config file doesn't exist! It will be generated...");
		}
		Parameters::readINI(configFile.c_str(), parameters);
	}
}

void CoreWrapper::saveParameters(const std::string & configFile)
{
	if(!configFile.empty())
	{
		printf("Saving parameters to %s\n", configFile.c_str());

		if(!UFile::exists(configFile.c_str()))
		{
			printf("Config file doesn't exist, a new one will be created.\n");
		}
		Parameters::writeINI(configFile.c_str(), parameters_);
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "Parameters are not saved (No configuration file provided...)");
	}
}

void CoreWrapper::defaultCallback(const sensor_msgs::msg::Image::ConstSharedPtr imageMsg)
{
	if(resetQuarantineEnabled_ || safeRelocalizationEnabled_)
	{
		RCLCPP_FATAL(this->get_logger(),
				"guarded mapping fail-stop: image-only default callback bypassed the authenticated RGB-D envelope");
		safeRelocalizationFatal_.store(true, std::memory_order_release);
		rclcpp::shutdown();
		return;
	}
	if(!paused_)
	{
		rclcpp::Time stamp = imageMsg->header.stamp;
		if(stamp.seconds() == 0.0)
		{
			RCLCPP_WARN(this->get_logger(), "A null stamp has been detected in the input topic. Make sure the stamp is set.");
			return;
		}

		if(rate_>0.0f)
		{
			if(previousStamp_.seconds() > 0.0 && stamp.seconds() > previousStamp_.seconds() && stamp - previousStamp_ < rclcpp::Duration::from_seconds(1.0f/rate_))
			{
				return;
			}
		}
		previousStamp_ = stamp;

		if(!(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
		{
			RCLCPP_ERROR(this->get_logger(), "Input type must be image=mono8,mono16,rgb8,bgr8");
			return;
		}

		cv_bridge::CvImageConstPtr ptrImage;
		if(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		   imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			ptrImage = cv_bridge::toCvShare(imageMsg, "mono8");
		}
		else
		{
			ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		}

		// process data
		UTimer timer;
		if(rtabmap_.isIDsGenerated())
		{
			if(!rtabmap_.process(ptrImage->image.clone()))
			{
				RCLCPP_WARN(this->get_logger(), "RTAB-Map could not process the data received!");
			}
			else
			{
				this->publishStats(now());
			}
		}
		else if(!rtabmap_.isIDsGenerated())
		{
			RCLCPP_WARN(this->get_logger(), "Ignoring received image because its sequence ID=0. Please "
					 "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
					 "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
					 "when you need to have IDs output of RTAB-map synchronised with the source "
					 "image sequence ID.");
		}
		RCLCPP_INFO(this->get_logger(), "rtabmap: Update rate=%fs, Limit=%fs, Processing time = %fs (%d local nodes)",
				1.0f/rate_,
				rtabmap_.getTimeThreshold()/1000.0f,
				timer.ticks(),
				rtabmap_.getWMSize()+rtabmap_.getSTMSize());
	}
}

bool CoreWrapper::odomUpdate(const nav_msgs::msg::Odometry & odomMsg, rclcpp::Time stamp)
{
	unsigned long long authenticatedContinuityChain = 0;
	if(!paused_)
	{
		if(resetQuarantineEnabled_ || safeRelocalizationEnabled_)
		{
			const long long messageStampNs = rclcpp::Time(odomMsg.header.stamp).nanoseconds();
			const long long synchronizedStampNs = stamp.nanoseconds();
			if(!managedOdomNumericsValid(odomMsg))
			{
				RCLCPP_ERROR(this->get_logger(),
						"reset quarantine rejected managed odometry with non-finite pose/twist/covariance or non-unit quaternion before graph update");
				return false;
			}
			const std::string fingerprint = managedOdomFingerprint(odomMsg);
			std::unique_lock<std::mutex> authLock(resetQuarantineOdomAuthMutex_);
			if(messageStampNs > 0 && messageStampNs == synchronizedStampNs &&
					!resetQuarantineOdomPublisherCompromised_)
			{
				resetQuarantineOdomAuthCondition_.wait_for(
						authLock,
						std::chrono::milliseconds(100),
						[this, messageStampNs]()
						{
							return resetQuarantineOdomPublisherCompromised_ ||
									resetQuarantineAuthenticatedOdomSamples_.find(messageStampNs) !=
											resetQuarantineAuthenticatedOdomSamples_.end();
						});
			}
			std::map<long long, std::string>::iterator authenticated =
					resetQuarantineAuthenticatedOdomSamples_.find(messageStampNs);
			if(resetQuarantineOdomPublisherCompromised_ ||
					messageStampNs <= 0 || messageStampNs != synchronizedStampNs ||
					odomMsg.header.frame_id != "odom" ||
					odomMsg.child_frame_id != frameId_ ||
					authenticated == resetQuarantineAuthenticatedOdomSamples_.end() ||
					authenticated->second != fingerprint)
			{
				RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
						"reset quarantine rejected odometry without an exact synchronized stamp, canonical frames and one source-bound managed publisher (message_stamp_ns=%lld synchronized_stamp_ns=%lld frame=%s child=%s compromised=%s)",
						messageStampNs, synchronizedStampNs,
						odomMsg.header.frame_id.c_str(), odomMsg.child_frame_id.c_str(),
						resetQuarantineOdomPublisherCompromised_?"true":"false");
				return false;
			}
			if(managedOdomHasBadCovariance(odomMsg) &&
					!resetQuarantineRecoveryPending_.load(std::memory_order_acquire))
			{
				resetQuarantineAuthenticatedOdomSamples_.erase(authenticated);
				RCLCPP_ERROR(this->get_logger(),
						"reset quarantine rejected high-covariance managed odometry without an authenticated lost/recovered boundary");
				return false;
			}
			const auto continuity = resetQuarantineOdomContinuity_.find(messageStampNs);
			if(continuity != resetQuarantineOdomContinuity_.end() &&
					continuity->second.fingerprint == fingerprint)
			{
				authenticatedContinuityChain = continuity->second.chain;
			}
			resetQuarantineAuthenticatedOdomSamples_.erase(authenticated);
		}
		Transform odom = rtabmap_conversions::transformFromPoseMsg(odomMsg.pose.pose);
		if(!odom.isNull())
		{
			// In guarded mode the source-bound Odometry message is the sole pose
			// authority.  Looking the pose up again through TF would allow a
			// conflicting broadcaster to replace the authenticated sample after
			// the source/stamp checks above.
			if(!resetQuarantineEnabled_ && !safeRelocalizationEnabled_)
			{
				Transform odomTF;
				if(stamp.seconds() != 0.0) {
					odomTF = rtabmap_conversions::getTransform(odomMsg.header.frame_id, frameId_, stamp, *tfBuffer_, waitForTransform_);
				}
				if(odomTF.isNull())
				{
					static bool shown = false;
					if(!shown)
					{
						RCLCPP_WARN(this->get_logger(), "We received odometry message, but we cannot get the "
								"corresponding TF %s->%s at data stamp %fs (odom msg stamp is %fs). Make sure TF of odometry is "
								"also published to get more accurate pose estimation. This "
								"warning is only printed once.", odomMsg.header.frame_id.c_str(), frameId_.c_str(), stamp.seconds(), rtabmap_conversions::timestampFromROS(odomMsg.header.stamp));
						shown = true;
					}
					stamp = odomMsg.header.stamp;
				}
				else
				{
					odom = odomTF;
				}
			}
		}

		UScopeMutex lock(lastPoseMutex_);
		if((resetQuarantineEnabled_ || safeRelocalizationEnabled_) &&
				lastPoseStamp_.nanoseconds() > 0 && !odom.isNull())
		{
			const Transform delta = lastPose_.inverse() * odom;
			const double deltaTranslation = delta.getNorm();
			const double deltaRotationDeg = delta.getAngle(rtabmap::Transform::getIdentity()) * 180.0 / M_PI;
			const bool highCovariance = managedOdomHasBadCovariance(odomMsg);
			// A sparse mapping frame can accumulate seconds of normal motion.
			// Only bridge that displacement when both exact endpoints belong to
			// the same source-bound, step-checked chain. A rejected jump starts a new
			// chain and cannot advance this accepted reference to launder itself.
			const bool continuous = authenticatedContinuityChain != 0 &&
					stamp.nanoseconds() > lastPoseStamp_.nanoseconds() &&
					resetQuarantineContinuityAnchorStampNs_ == lastPoseStamp_.nanoseconds() &&
					authenticatedContinuityChain == resetQuarantineContinuityAnchorChain_;
			const bool excessiveDelta = deltaTranslation > resetQuarantineMaxUnannouncedTranslation_ ||
					deltaRotationDeg > resetQuarantineMaxUnannouncedRotationDeg_;
			const bool unexpectedReset = odom.isIdentity() || highCovariance ||
					!std::isfinite(deltaTranslation) || !std::isfinite(deltaRotationDeg) ||
					(excessiveDelta && !continuous);
			if(excessiveDelta && continuous && !unexpectedReset)
			{
				RCLCPP_INFO(this->get_logger(),
						"reset quarantine admitted continuous full-rate odometry across sparse mapping frames (dt=%.3fs translation=%.6f rotation_deg=%.6f chain=%llu)",
						(stamp-lastPoseStamp_).seconds(), deltaTranslation, deltaRotationDeg,
						authenticatedContinuityChain);
			}
			if(unexpectedReset &&
					!resetQuarantineRecoveryPending_.load(std::memory_order_acquire))
			{
				resetQuarantineSuspectedReset_.store(true, std::memory_order_release);
				RCLCPP_ERROR(this->get_logger(),
						"reset quarantine rejected unannounced odometry reset/jump before graph update (identity=%s high_covariance=%s translation=%.6f limit=%.6f rotation_deg=%.6f limit=%.6f); waiting for an authenticated lost/recovered boundary or a continuous pose",
						odom.isIdentity()?"true":"false", highCovariance?"true":"false",
						deltaTranslation, resetQuarantineMaxUnannouncedTranslation_,
						deltaRotationDeg, resetQuarantineMaxUnannouncedRotationDeg_);
				return false;
			}
			if(!unexpectedReset)
			{
				resetQuarantineSuspectedReset_.store(false, std::memory_order_release);
			}
		}

		// The legacy one-sample reset/staleness path is incompatible with reset
		// quarantine: it can split the graph before the unique-stamp lost streak
		// and the first valid recovery frame have been established.  Safe
		// relocalization therefore owns every reset boundary while either guard
		// is enabled.
		if(!resetQuarantineEnabled_ && !safeRelocalizationEnabled_ &&
				!lastPose_.isIdentity() && !odom.isNull() &&
				(odom.isIdentity() ||
				 (odomMsg.pose.covariance[0] >= BAD_COVARIANCE &&
				  odomMsg.twist.covariance[0] >= BAD_COVARIANCE)))
		{
			UWARN("Odometry is reset (identity pose or high variance (%f) detected). Increment map id!", MAX(odomMsg.pose.covariance[0], odomMsg.twist.covariance[0]));
			triggerNewMapBeforeNextUpdate_ = true;
			lastPoseCovariance_ = cv::Mat();
		} 
		else if(!resetQuarantineEnabled_ && !safeRelocalizationEnabled_ &&
				stalenessFactor_>0.0 && 
			    previousStamp_.seconds() > 0.0 && 
				rate_>0.0f && 
				(stamp - previousStamp_).seconds() > stalenessFactor_/rate_)
		{
			UWARN("The time difference (%f s) between the new timestamp received (%f) and "
				"the previous one (%f) is way over than the expected update period (%s=%f Hz) "
				"%f x staleness_factor (%f) = %f s. Triggering a new map! Set staleness_factor to 0 "
				"to avoid triggering a new map when this happens.",
				(stamp - previousStamp_).seconds(),
				stamp.seconds(),
				previousStamp_.seconds(),
				Parameters::kRtabmapDetectionRate().c_str(),
				rate_,
				1.0f/rate_,
				stalenessFactor_,
				stalenessFactor_/rate_);
			triggerNewMapBeforeNextUpdate_ = true;
			lastPoseCovariance_ = cv::Mat();
		}

		lastPoseIntermediate_ = false;
		lastPose_ = odom;
		lastPoseStamp_ = stamp;
		resetQuarantineContinuityAnchorStampNs_ = stamp.nanoseconds();
		resetQuarantineContinuityAnchorChain_ = authenticatedContinuityChain;
		lastPoseVelocity_.resize(6);
		lastPoseVelocity_[0] = odomMsg.twist.twist.linear.x;
		lastPoseVelocity_[1] = odomMsg.twist.twist.linear.y;
		lastPoseVelocity_[2] = odomMsg.twist.twist.linear.z;
		lastPoseVelocity_[3] = odomMsg.twist.twist.angular.x;
		lastPoseVelocity_[4] = odomMsg.twist.twist.angular.y;
		lastPoseVelocity_[5] = odomMsg.twist.twist.angular.z;

		// Only update variance if odom is not null
		if(!odom.isNull())
		{
			cv::Mat covariance;
			double variance = odomMsg.twist.covariance[0];
			if(variance == BAD_COVARIANCE || variance <= 0.0f)
			{
				//use the one of the pose
				covariance = cv::Mat(6,6,CV_64FC1, (void*)odomMsg.pose.covariance.data()).clone();
				covariance /= 2.0;
			}
			else
			{
				covariance = cv::Mat(6,6,CV_64FC1, (void*)odomMsg.twist.covariance.data()).clone();
			}

			if(uIsFinite(covariance.at<double>(0,0)) &&
				covariance.at<double>(0,0) != 1.0 &&
				covariance.at<double>(0,0)>0.0)
			{
				// Use largest covariance error (to be independent of the odometry frame rate)
				if(lastPoseCovariance_.empty() || covariance.at<double>(0,0) > lastPoseCovariance_.at<double>(0,0))
				{
					lastPoseCovariance_ = covariance;
				}
			}
		}

		// Throttle
		bool ignoreFrame = false;
		if(stamp.seconds() == 0.0)
		{
			RCLCPP_WARN(this->get_logger(), "A null stamp has been detected in the input topics. Make sure the stamp in all input topics is set.");
			ignoreFrame = true;
		}
		if(rate_>0.0f)
		{
			if(previousStamp_.seconds() > 0.0 && stamp.seconds() > previousStamp_.seconds() && stamp.seconds() - previousStamp_.seconds() < 1.0f/rate_)
			{
				ignoreFrame = true;
			}
		}
		if(ignoreFrame)
		{
			if(createIntermediateNodes_)
			{
				lastPoseIntermediate_ = true;
			}
			else
			{
				return false;
			}
		}

		return true;
	}
	return false;
}

bool CoreWrapper::odomTFUpdate(const std::string & odomFrameId, const rclcpp::Time & stamp)
{
	if(!paused_)
	{
		if(resetQuarantineEnabled_ || safeRelocalizationEnabled_)
		{
			RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
					"reset quarantine rejects TF-only odometry because it has no source-bound message identity");
			return false;
		}
		// Odom TF ready?
		Transform odom = rtabmap_conversions::getTransform(odomFrameId, frameId_, stamp, *tfBuffer_, waitForTransform_);
		if(odom.isNull())
		{
			return false;
		}

		UScopeMutex lock(lastPoseMutex_);
		if((resetQuarantineEnabled_ || safeRelocalizationEnabled_) &&
				!lastPose_.isIdentity() && !odom.isNull())
		{
			const Transform delta = lastPose_.inverse() * odom;
			const double deltaTranslation = delta.getNorm();
			const double deltaRotationDeg = delta.getAngle(rtabmap::Transform::getIdentity()) * 180.0 / M_PI;
			const bool unexpectedReset = odom.isIdentity() ||
					!std::isfinite(deltaTranslation) || !std::isfinite(deltaRotationDeg) ||
					deltaTranslation > resetQuarantineMaxUnannouncedTranslation_ ||
					deltaRotationDeg > resetQuarantineMaxUnannouncedRotationDeg_;
			if(unexpectedReset &&
					!resetQuarantineRecoveryPending_.load(std::memory_order_acquire))
			{
				resetQuarantineSuspectedReset_.store(true, std::memory_order_release);
				RCLCPP_ERROR(this->get_logger(),
						"reset quarantine rejected unannounced TF odometry reset/jump before graph update (identity=%s translation=%.6f limit=%.6f rotation_deg=%.6f limit=%.6f); waiting for an authenticated lost/recovered boundary or a continuous pose",
						odom.isIdentity()?"true":"false", deltaTranslation,
						resetQuarantineMaxUnannouncedTranslation_, deltaRotationDeg,
						resetQuarantineMaxUnannouncedRotationDeg_);
				return false;
			}
			if(!unexpectedReset)
			{
				resetQuarantineSuspectedReset_.store(false, std::memory_order_release);
			}
		}

		// See odomUpdate(): guarded sessions may advance an epoch only through
		// the reset-quarantine lost/recovered state machine.
		if(!resetQuarantineEnabled_ && !safeRelocalizationEnabled_ &&
				!lastPose_.isIdentity() && odom.isIdentity())
		{
			UWARN("Odometry is reset (identity pose detected). Increment map id!");
			triggerNewMapBeforeNextUpdate_ = true;
			lastPoseCovariance_ = cv::Mat();
		}
		else if(!resetQuarantineEnabled_ && !safeRelocalizationEnabled_ &&
				stalenessFactor_>0.0 && 
			    previousStamp_.seconds() > 0.0 && 
				rate_>0.0f && 
				(stamp - previousStamp_).seconds() > stalenessFactor_/rate_)
		{
			UWARN("The time difference (%f s) between the new timestamp received (%f) and "
				"the previous one (%f) is way over than the expected update period (%s=%f Hz) "
				"%f x staleness_factor (%f) = %f s. Triggering a new map! Set staleness_factor to 0 "
				"to avoid triggering a new map when this happens.",
				(stamp - previousStamp_).seconds(),
				stamp.seconds(),
				previousStamp_.seconds(),
				Parameters::kRtabmapDetectionRate().c_str(),
				rate_,
				1.0f/rate_,
				stalenessFactor_,
				stalenessFactor_/rate_);
			triggerNewMapBeforeNextUpdate_ = true;
			lastPoseCovariance_ = cv::Mat();
		}

		lastPoseIntermediate_ = false;
		lastPose_ = odom;
		lastPoseStamp_ = stamp;
		lastPoseVelocity_.clear();

		bool ignoreFrame = false;
		if(stamp.seconds() == 0.0)
		{
			RCLCPP_WARN(this->get_logger(), "A null stamp has been detected in the input topics. Make sure the stamp in all input topics is set.");
			ignoreFrame = true;
		}
		if(rate_>0.0f)
		{
			if(previousStamp_.seconds() > 0.0 && stamp.seconds() > previousStamp_.seconds() && stamp.seconds() - previousStamp_.seconds() < 1.0f/rate_)
			{
				ignoreFrame = true;
			}
		}
		if(ignoreFrame)
		{
			if(createIntermediateNodes_)
			{
				lastPoseIntermediate_ = true;
			}
			else
			{
				return false;
			}
		}

		return true;
	}
	return false;
}

void CoreWrapper::commonMultiCameraCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & depthCameraInfoMsgs,
		const sensor_msgs::msg::LaserScan & scan2dMsg,
		const sensor_msgs::msg::PointCloud2 & scan3dMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
		const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & globalDescriptorMsgs,
		const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > & localKeyPoints,
		const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > & localPoints3d,
		const std::vector<cv::Mat> & localDescriptors)
{
	// Guarded callbacks can otherwise overtake each other in a multithreaded
	// executor.  Keep the complete one-shot admission transaction serialized so
	// a delayed t0 callback cannot consume authenticated evidence or roll
	// lastPose back after t1 has already been admitted.
	std::unique_lock<std::mutex> admissionLock(
			resetQuarantineAdmissionMutex_, std::defer_lock);
	if(resetQuarantineEnabled_ || safeRelocalizationEnabled_)
	{
		admissionLock.lock();
		if(!odomMsg)
		{
			RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
					"guarded RGB-D admission requires synchronized managed odometry");
			return;
		}
		const long long admissionStampNs =
				rclcpp::Time(odomMsg->header.stamp).nanoseconds();
		const long long epochStartStampNs =
				resetQuarantineEpochStartStampNs_.load(std::memory_order_acquire);
		long long lastPoseStampNs = 0;
		{
			UScopeMutex lastPoseLock(lastPoseMutex_);
			lastPoseStampNs = lastPoseStamp_.nanoseconds();
		}
		if(admissionStampNs <= 0 ||
				(epochStartStampNs > 0 && admissionStampNs < epochStartStampNs) ||
				(lastPoseStampNs > 0 && admissionStampNs <= lastPoseStampNs))
		{
			RCLCPP_ERROR_THROTTLE(
					this->get_logger(), *this->get_clock(), 5000,
					"guarded RGB-D admission rejected a null, delayed pre-epoch or non-monotonic callback before consuming any one-shot evidence (stamp_ns=%lld epoch_start_ns=%lld last_pose_stamp_ns=%lld)",
					admissionStampNs, epochStartStampNs, lastPoseStampNs);
			return;
		}
		// The independent full-rate subscription owns publisher/GID validation
		// and the lost-streak state machine.  The synchronized copy must match one
		// exact authenticated payload before this image can enter the graph.
		if(!odomInfoMsg)
		{
			RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
					"reset quarantine rejected an RGB-D update without synchronized OdomInfo");
			return;
		}
		if(!authenticateResetQuarantineRgbd(imageMsgs, depthMsgs, cameraInfoMsgs))
		{
			return;
		}
		if(!validateResetQuarantineSensorExtrinsic(cameraInfoMsgs))
		{
			return;
		}
		if(!authenticateResetQuarantineOdomInfo(*odomInfoMsg))
		{
			return;
		}
	}

	std::string odomFrameId;
	if(odomMsg.get())
	{
		odomFrameId = odomMsg->header.frame_id;
		if(!scan2dMsg.ranges.empty())
		{
			if(!odomUpdate(*odomMsg, scan2dMsg.header.stamp))
			{
				return;
			}
		}
		else if(!scan3dMsg.data.empty())
		{
			if(!odomUpdate(*odomMsg, scan3dMsg.header.stamp))
			{
				return;
			}
		}
		else if(cameraInfoMsgs.size() == 0 || !odomUpdate(*odomMsg, cameraInfoMsgs[0].header.stamp))
		{
			return;
		}
	}
	else
	{
		mapToOdomMutex_.lock();
		odomFrameId = odomFrameId_;
		mapToOdomMutex_.unlock();
		if(odomFrameId.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "This callback cannot be used without \"subscribe_odom\" or \"odom_frame_id\" set.");
			return;
		}
		if(!scan2dMsg.ranges.empty())
		{
			if(!odomTFUpdate(odomFrameId, scan2dMsg.header.stamp))
			{
				return;
			}
		}
		else if(!scan3dMsg.data.empty())
		{
			if(!odomTFUpdate(odomFrameId, scan3dMsg.header.stamp))
			{
				return;
			}
		}
		else if(cameraInfoMsgs.size() == 0 || !odomTFUpdate(odomFrameId, cameraInfoMsgs[0].header.stamp))
		{
			return;
		}
	}

	if(syncTimer_->is_canceled() && syncDataMutex_.lockTry() == 0)
	{
		UScopeMutex lock(lastPoseMutex_);
		commonMultiCameraCallbackImpl(odomFrameId,
				userDataMsg,
				imageMsgs,
				depthMsgs,
				cameraInfoMsgs,
				depthCameraInfoMsgs,
				scan2dMsg,
				scan3dMsg,
				odomInfoMsg,
				globalDescriptorMsgs,
				localKeyPoints,
				localPoints3d,
				localDescriptors);
		
		if(syncData_.valid) {
            if(resetQuarantineEnabled_ && safeRelocalizationEnabled_ &&
               imageMsgs.size() == 1U && cameraInfoMsgs.size() == 1U)
                syncData_.imageIdentity = captureAuthenticatedImageIdentity(imageMsgs.front(), cameraInfoMsgs.front());
			syncTimer_->reset();
		}
		syncDataMutex_.unlock();
	}
}

void CoreWrapper::commonMultiCameraCallbackImpl(
		const std::string & odomFrameId,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & depthCameraInfoMsgs,
		const sensor_msgs::msg::LaserScan & scan2dMsg,
		const sensor_msgs::msg::PointCloud2 & scan3dMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
		const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & globalDescriptorMsgs,
		const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > & localKeyPointsMsgs,
		const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > & localPoints3dMsgs,
		const std::vector<cv::Mat> & localDescriptorsMsgs)
{
	UTimer timerConversion;
	cv::Mat rgb;
	cv::Mat depth;
	std::vector<rtabmap::CameraModel> cameraModels;
	std::vector<rtabmap::StereoCameraModel> stereoCameraModels;
	std::vector<cv::KeyPoint> keypoints;
	std::vector<cv::Point3f> points;
	cv::Mat descriptors;

	if(!rtabmap_conversions::convertRGBDMsgs(
			imageMsgs,
			depthMsgs,
			cameraInfoMsgs,
			depthCameraInfoMsgs,
			frameId_,
			odomSensorSync_?odomFrameId:"",
			lastPoseStamp_,
			rgb,
			depth,
			cameraModels,
			stereoCameraModels,
			*tfBuffer_,
			waitForTransform_,
			alreadyRectifiedImages_,
			localKeyPointsMsgs,
			localPoints3dMsgs,
			localDescriptorsMsgs,
			&keypoints,
			&points,
			&descriptors))
	{
		RCLCPP_ERROR(this->get_logger(), "Could not convert rgb/depth msgs! Aborting rtabmap update...");
		return;
	}
	// convertRGBDMsgs performs its own TF lookup. A concurrent TF update could
	// otherwise race the pre-conversion deployment-extrinsic check. Verify the
	// actual CameraModel transforms that are about to enter SensorData/Memory.
	if(resetQuarantineEnabled_ || safeRelocalizationEnabled_)
	{
		auto matchesFrozenSensorExtrinsic = [this](const Transform & converted) -> bool
		{
			if(converted.isNull())
			{
				return false;
			}
			const Transform delta =
					resetQuarantineExpectedSensorToBaseTransform_.inverse() * converted;
			const double translationError = delta.getNorm();
			const double rotationError = delta.getAngle(rtabmap::Transform::getIdentity());
			return std::isfinite(translationError) && std::isfinite(rotationError) &&
					translationError <= 1e-6 && rotationError <= 1e-6;
		};
		bool convertedExtrinsicValid = !cameraModels.empty() || !stereoCameraModels.empty();
		for(size_t i=0; convertedExtrinsicValid && i<cameraModels.size(); ++i)
		{
			convertedExtrinsicValid = matchesFrozenSensorExtrinsic(cameraModels[i].localTransform());
		}
		for(size_t i=0; convertedExtrinsicValid && i<stereoCameraModels.size(); ++i)
		{
			convertedExtrinsicValid = matchesFrozenSensorExtrinsic(stereoCameraModels[i].localTransform());
		}
		if(!convertedExtrinsicValid)
		{
			RCLCPP_FATAL(this->get_logger(),
					"reset quarantine fail-stop: converted CameraModel extrinsic differs from the frozen deployment value");
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
			return;
		}
	}
	UDEBUG("cameraModels=%ld stereoCameraModels=%ld", cameraModels.size(), stereoCameraModels.size());
	UDEBUG("rgb=%dx%d(type=%d), depth/right=%dx%d(type=%d)", rgb.rows, rgb.cols, rgb.type(), depth.rows, depth.cols, depth.type());

	if(stereoCameraModels.size() && stereoToDepth_)
	{
		UASSERT(depth.type() == CV_8UC1);
		cv::Mat leftMono;
		if(rgb.channels() == 3)
		{
			cv::cvtColor(rgb, leftMono, CV_BGR2GRAY);
		}
		else
		{
			leftMono = rgb;
		}
		cv::Mat rightMono = depth;
		depth = cv::Mat();

		UASSERT(int((leftMono.cols/stereoCameraModels.size())*stereoCameraModels.size()) == leftMono.cols);
		UASSERT(int((rightMono.cols/stereoCameraModels.size())*stereoCameraModels.size()) == rightMono.cols);
		int subImageWidth = leftMono.cols/stereoCameraModels.size();
		for(size_t i=0; i<stereoCameraModels.size(); ++i)
		{
			cv::Mat left(leftMono, cv::Rect(subImageWidth*i, 0, subImageWidth, leftMono.rows));
			cv::Mat right(rightMono, cv::Rect(subImageWidth*i, 0, subImageWidth, rightMono.rows));

			// cv::stereoBM() see "$ rosrun rtabmap_ros rtabmap --params | grep StereoBM" for parameters
			cv::Mat disparity = rtabmap::util2d::disparityFromStereoImages(
					left,
					right,
					parameters_);
			if(disparity.empty())
			{
				RCLCPP_ERROR(this->get_logger(), "Could not compute disparity image (\"stereo_to_depth\" is true)!");
				return;
			}
			cv::Mat subDepth = rtabmap::util2d::depthFromDisparity(
							disparity,
							stereoCameraModels[i].left().fx(),
							stereoCameraModels[i].baseline());

			if(subDepth.empty())
			{
				RCLCPP_ERROR(this->get_logger(), "Could not compute depth image (\"stereo_to_depth\" is true)!");
				return;
			}
			UASSERT(subDepth.type() == CV_16UC1 || subDepth.type() == CV_32FC1);

			if(depth.empty())
			{
				depth = cv::Mat(subDepth.rows, subDepth.cols*stereoCameraModels.size(), subDepth.type());
			}

			if(subDepth.type() == depth.type())
			{
				subDepth.copyTo(cv::Mat(depth, cv::Rect(i*subDepth.cols, 0, subDepth.cols, subDepth.rows)));
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Some Depth images are not the same type!");
				return;
			}

			cameraModels.push_back(stereoCameraModels[i].left());
		}
		stereoCameraModels.clear();
	}

	UASSERT(uContains(parameters_, rtabmap::Parameters::kMemSaveDepth16Format()));
	if(!depth.empty() && depth.type() == CV_32FC1 && uStr2Bool(parameters_.at(Parameters::kMemSaveDepth16Format())))
	{
		depth = rtabmap::util2d::cvtDepthFromFloat(depth);
		static bool shown = false;
		if(!shown)
		{
			RCLCPP_WARN(this->get_logger(), "Save depth data to 16 bits format: depth type detected is "
				  "32FC1, use 16UC1 depth format to avoid this conversion "
				  "(or set parameter \"Mem/SaveDepth16Format=false\" to use "
				  "32bits format). This message is only printed once...");
			shown = true;
		}
	}

	LaserScan scan;
	bool genMaxScanPts = 0;
	if(scan2dMsg.ranges.empty() && scan3dMsg.data.empty() && !depth.empty() && stereoCameraModels.empty() && genScan_)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud2d(new pcl::PointCloud<pcl::PointXYZ>);
		*scanCloud2d = util3d::laserScanFromDepthImages(
				depth,
				cameraModels,
				genScanMaxDepth_,
				genScanMinDepth_);
		genMaxScanPts += depth.cols;
		scan = LaserScan(rtabmap::util3d::laserScan2dFromPointCloud(*scanCloud2d), 0, genScanMaxDepth_);
	}
	else if(!scan2dMsg.ranges.empty())
	{
		if(!rtabmap_conversions::convertScanMsg(
				scan2dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scan,
				*tfBuffer_,
				waitForTransform_,
				// backward compatibility, project 2D scan in /base_link frame
				rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0))
		{
			RCLCPP_ERROR(this->get_logger(), "Could not convert laser scan msg! Aborting rtabmap update...");
			return;
		}
	}
	else if(!scan3dMsg.data.empty())
	{
		if(!rtabmap_conversions::convertScan3dMsg(
				scan3dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scan,
				*tfBuffer_,
				waitForTransform_,
				scanCloudMaxPoints_,
				0,
				scanCloudIs2d_))
		{
			RCLCPP_ERROR(this->get_logger(), "Could not convert 3d laser scan msg! Aborting rtabmap update...");
			return;
		}

		RCLCPP_DEBUG(get_logger(), "%d %d %d %d", rgb.empty()?1:0, depth.empty()?1:0, scan.isEmpty()?1:0, genDepth_?1:0);
		if(!rgb.empty() && depth.empty() && !scan.isEmpty() && genDepth_)
		{
			for(size_t i=0; i<cameraModels.size(); ++i)
			{
				rtabmap::CameraModel model = cameraModels[i];
				if(genDepthDecimation_ > 1)
				{
					if(model.imageWidth()%genDepthDecimation_ == 0 && model.imageHeight()%genDepthDecimation_ == 0)
					{
						model = model.scaled(1.0f/float(genDepthDecimation_));
					}
					else
					{
						RCLCPP_ERROR(get_logger(), "decimation (%d) not valid for image size %dx%d! Aborting depth generation from scan...",
								genDepthDecimation_,
								model.imageWidth(),
								model.imageHeight());
						depth = cv::Mat();
						break;
					}
				}

				cv::Mat depthProjected = util3d::projectCloudToCamera(
						model.imageSize(),
						model.K(),
						scan.data(),
						scan.localTransform().inverse()*model.localTransform());

				if(genDepthFillHolesSize_ > 0 && genDepthFillIterations_ > 0)
				{
					for(int i=0; i<genDepthFillIterations_;++i)
					{
						depthProjected = util2d::fillDepthHoles(
								depthProjected,
								genDepthFillHolesSize_,
								genDepthFillHolesError_);
					}
				}

				if(depth.empty())
				{
					depth = cv::Mat::zeros(model.imageHeight(), model.imageWidth()*cameraModels.size(), CV_32FC1);
				}
				depthProjected.copyTo(depth.colRange(i*model.imageWidth(), (i+1)*model.imageWidth()));
			}
		}
	}

	cv::Mat userData;
	if(userDataMsg.get())
	{
		userData = rtabmap_conversions::userDataFromROS(*userDataMsg);
		UScopeMutex lock(userDataMutex_);
		if(!userData_.empty())
		{
			RCLCPP_WARN(this->get_logger(), "Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
			userData_ = cv::Mat();
		}
	}
	else
	{
		UScopeMutex lock(userDataMutex_);
		userData = userData_;
		userData_ = cv::Mat();
	}

	if(!stereoCameraModels.empty())
	{
		syncData_.data = SensorData(
				scan,
				rgb,
				depth,
				stereoCameraModels,
				lastPoseIntermediate_?-1:0,
				rtabmap_conversions::timestampFromROS(lastPoseStamp_),
				userData);
	}
	else
	{
		syncData_.data = SensorData(
				scan,
				rgb,
				depth,
				cameraModels,
				lastPoseIntermediate_?-1:0,
				rtabmap_conversions::timestampFromROS(lastPoseStamp_),
				userData);
	}

	OdometryInfo odomInfo;
	if(odomInfoMsg.get())
	{
		odomInfo = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg);
	}

	if(!globalDescriptorMsgs.empty())
	{
		syncData_.data.setGlobalDescriptors(rtabmap_conversions::globalDescriptorsFromROS(globalDescriptorMsgs));
	}

	if(!keypoints.empty())
	{
		UASSERT(points.empty() || points.size() == keypoints.size());
		UASSERT(descriptors.empty() || descriptors.rows == (int)keypoints.size());
		syncData_.data.setFeatures(keypoints, points, descriptors);
	}

	syncData_.imageIdentity = rtabmap::ImageInputIdentity();
	syncData_.valid = true;
	syncData_.stamp = lastPoseStamp_;
	syncData_.odom = lastPose_;
	syncData_.odomVelocity = lastPoseVelocity_;
	syncData_.odomFrameId = odomFrameId;
	syncData_.odomCovariance = lastPoseCovariance_;
	syncData_.odomInfo = odomInfo;
	syncData_.timeMsgConversion = timerConversion.ticks();

	if(!lastPoseIntermediate_)
	{
		previousStamp_ = lastPoseStamp_;
	}

	lastPoseCovariance_ = cv::Mat();
}

void CoreWrapper::commonLaserScanCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
		const sensor_msgs::msg::LaserScan & scan2dMsg,
		const sensor_msgs::msg::PointCloud2 & scan3dMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
		const rtabmap_msgs::msg::GlobalDescriptor & globalDescriptor)
{
	UTimer timerConversion;
	std::string odomFrameId;
	if(odomMsg.get())
	{
		odomFrameId = odomMsg->header.frame_id;
		if(!scan2dMsg.ranges.empty())
		{
			if(!odomUpdate(*odomMsg, scan2dMsg.header.stamp))
			{
				return;
			}
		}
		else if(!scan3dMsg.data.empty())
		{
			if(!odomUpdate(*odomMsg, scan3dMsg.header.stamp))
			{
				return;
			}
		}
		else
		{
			return;
		}
	}
	else
	{
		mapToOdomMutex_.lock();
		odomFrameId = odomFrameId_;
		mapToOdomMutex_.unlock();
		if(odomFrameId.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "This callback cannot be used without \"subscribe_odom\" or \"odom_frame_id\" set.");
			return;
		}
		if(!scan2dMsg.ranges.empty())
		{
			if(!odomTFUpdate(odomFrameId, scan2dMsg.header.stamp))
			{
				return;
			}
		}
		else if(!scan3dMsg.data.empty())
		{
			if(!odomTFUpdate(odomFrameId, scan3dMsg.header.stamp))
			{
				return;
			}
		}
		else
		{
			return;
		}
	}

	if(syncTimer_->is_canceled() && syncDataMutex_.lockTry() == 0)
	{
		UScopeMutex lock(lastPoseMutex_);
		LaserScan scan;
		if(!scan2dMsg.ranges.empty())
		{
			if(!rtabmap_conversions::convertScanMsg(
					scan2dMsg,
					frameId_,
					odomSensorSync_?odomFrameId:"",
					lastPoseStamp_,
					scan,
					*tfBuffer_,
					waitForTransform_,
					// backward compatibility, project 2D scan in /base_link frame
					rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not convert laser scan msg! Aborting rtabmap update...");
				return;
			}
		}
		else if(!scan3dMsg.data.empty())
		{
			if(!rtabmap_conversions::convertScan3dMsg(
					scan3dMsg,
					frameId_,
					odomSensorSync_?odomFrameId:"",
					lastPoseStamp_,
					scan,
					*tfBuffer_,
					waitForTransform_,
					scanCloudMaxPoints_,
					0,
					scanCloudIs2d_))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not convert 3d laser scan msg! Aborting rtabmap update...");
				return;
			}
		}

		cv::Mat userData;
		if(userDataMsg.get())
		{
			userData = rtabmap_conversions::userDataFromROS(*userDataMsg);
			UScopeMutex lock(userDataMutex_);
			if(!userData_.empty())
			{
				RCLCPP_WARN(this->get_logger(), "Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
				userData_ = cv::Mat();
			}
		}
		else
		{
			UScopeMutex lock(userDataMutex_);
			userData = userData_;
			userData_ = cv::Mat();
		}

		syncData_.data = SensorData(
				scan,
				cv::Mat(),
				cv::Mat(),
				rtabmap::CameraModel(),
				lastPoseIntermediate_?-1:0,
				rtabmap_conversions::timestampFromROS(lastPoseStamp_),
				userData);

		OdometryInfo odomInfo;
		if(odomInfoMsg.get())
		{
			odomInfo = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg, true);
		}

		if(!globalDescriptor.data.empty())
		{
			syncData_.data.addGlobalDescriptor(rtabmap_conversions::globalDescriptorFromROS(globalDescriptor));
		}

		syncData_.imageIdentity = rtabmap::ImageInputIdentity();
	syncData_.valid = true;
		syncData_.stamp = lastPoseStamp_;
		syncData_.odom = lastPose_;
		syncData_.odomVelocity = lastPoseVelocity_;
		syncData_.odomFrameId = odomFrameId;
		syncData_.odomCovariance = lastPoseCovariance_;
		syncData_.odomInfo = odomInfo;
		syncData_.timeMsgConversion = timerConversion.ticks();

		if(!lastPoseIntermediate_)
		{
			previousStamp_ = lastPoseStamp_;
		}

		lastPoseCovariance_ = cv::Mat();

		syncTimer_->reset();
		syncDataMutex_.unlock();
	}
}

void CoreWrapper::commonOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	UTimer timerConversion;
	UASSERT(odomMsg.get());
	std::string odomFrameId = odomMsg->header.frame_id;
	if(!odomUpdate(*odomMsg, odomMsg->header.stamp))
	{
		return;
	}

	if(syncTimer_->is_canceled() && syncDataMutex_.lockTry() == 0)
	{
		UScopeMutex lock(lastPoseMutex_);
		cv::Mat userData;
		if(userDataMsg.get())
		{
			userData = rtabmap_conversions::userDataFromROS(*userDataMsg);
			UScopeMutex lock(userDataMutex_);
			if(!userData_.empty())
			{
				RCLCPP_WARN(this->get_logger(), "Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
				userData_ = cv::Mat();
			}
		}
		else
		{
			UScopeMutex lock(userDataMutex_);
			userData = userData_;
			userData_ = cv::Mat();
		}

		syncData_.data = SensorData(
				cv::Mat(),
				cv::Mat(),
				rtabmap::CameraModel(),
				lastPoseIntermediate_?-1:0,
				rtabmap_conversions::timestampFromROS(lastPoseStamp_),
				userData);

		OdometryInfo odomInfo;
		if(odomInfoMsg.get())
		{
			odomInfo = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg, true);
		}

		syncData_.imageIdentity = rtabmap::ImageInputIdentity();
	syncData_.valid = true;
		syncData_.stamp = lastPoseStamp_;
		syncData_.odom = lastPose_;
		syncData_.odomVelocity = lastPoseVelocity_;
		syncData_.odomFrameId = odomFrameId;
		syncData_.odomCovariance = lastPoseCovariance_;
		syncData_.odomInfo = odomInfo;
		syncData_.timeMsgConversion = timerConversion.ticks();

		if(!lastPoseIntermediate_)
		{
			previousStamp_ = lastPoseStamp_;
		}

		lastPoseCovariance_ = cv::Mat();

		syncTimer_->reset();
		syncDataMutex_.unlock();
	}
}

void CoreWrapper::commonSensorDataCallback(
		const rtabmap_msgs::msg::SensorData::ConstSharedPtr & sensorDataMsg,
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	UTimer timerConversion;
	UASSERT(sensorDataMsg.get());
	std::string odomFrameId;
	if(odomMsg.get())
	{
		odomFrameId = odomMsg->header.frame_id;
		if(!odomUpdate(*odomMsg, sensorDataMsg->header.stamp))
		{
			return;
		}
	}
	else
	{
		mapToOdomMutex_.lock();
		odomFrameId = odomFrameId_;
		mapToOdomMutex_.unlock();
		if(odomFrameId.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "This callback cannot be used without \"subscribe_odom\" or \"odom_frame_id\" set.");
			return;
		}
		if(!odomTFUpdate(odomFrameId, sensorDataMsg->header.stamp))
		{
			return;
		}
	}

	if(syncTimer_->is_canceled() && syncDataMutex_.lockTry() == 0)
	{
		UScopeMutex lock(lastPoseMutex_);
		SensorData convertedData = rtabmap_conversions::sensorDataFromROS(*sensorDataMsg);
		if(safeRelocalizationEnabled_ &&
				(!convertedData.globalPose().isNull() || convertedData.gps().stamp() > 0.0))
		{
			RCLCPP_ERROR(this->get_logger(),
					"Ignoring SensorData carrying embedded global-pose/GPS priors while safe relocalization is enabled");
			syncDataMutex_.unlock();
			return;
		}
		syncData_.data = convertedData;
		syncData_.data.setId(lastPoseIntermediate_?-1:0);

		OdometryInfo odomInfo;
		if(odomInfoMsg.get())
		{
			odomInfo = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg, true);
		}

		syncData_.imageIdentity = rtabmap::ImageInputIdentity();
	syncData_.valid = true;
		syncData_.stamp = lastPoseStamp_;
		syncData_.odom = lastPose_;
		syncData_.odomVelocity = lastPoseVelocity_;
		syncData_.odomFrameId = odomFrameId;
		syncData_.odomCovariance = lastPoseCovariance_;
		syncData_.odomInfo = odomInfo;
		syncData_.timeMsgConversion = timerConversion.ticks();

		if(!lastPoseIntermediate_)
		{
			previousStamp_ = lastPoseStamp_;
		}

		lastPoseCovariance_ = cv::Mat();

		syncTimer_->reset();
		syncDataMutex_.unlock();
	}
}

void CoreWrapper::processAsync()
{
	UScopeMutex lock(syncDataMutex_);

	if(syncData_.valid)
	{
		// Never consume a pending legacy boundary before a valid synchronized
		// update.  In guarded sessions this flag should be unreachable, but clear
		// it fail-closed instead of allowing a one-frame bypass of quarantine.
		if(triggerNewMapBeforeNextUpdate_)
		{
			if(resetQuarantineEnabled_ || safeRelocalizationEnabled_)
			{
				RCLCPP_ERROR(this->get_logger(),
						"discarding legacy new-map trigger while safe reset ownership is enabled");
				triggerNewMapBeforeNextUpdate_ = false;
			}
			else
			{
				rtabmap_.triggerNewMapForContinuation();
				clearSafeRelocalizationTokens();
				resetQuarantineEpoch_.store(rtabmap_.getResetEpoch(), std::memory_order_release);
				++safeRelocalizationGraphRevision_;
				publishResetEpoch();
				triggerNewMapBeforeNextUpdate_ = false;
			}
		}
		process(syncData_.stamp,
				syncData_.data,
				syncData_.odom,
				syncData_.odomVelocity,
				syncData_.odomFrameId,
				syncData_.odomCovariance,
				syncData_.odomInfo,
				syncData_.timeMsgConversion, syncData_.imageIdentity);
        syncData_.imageIdentity = rtabmap::ImageInputIdentity();
		syncData_.valid=false;
	}
	syncTimer_->cancel();
}

void CoreWrapper::process(
		const rclcpp::Time & stamp,
		SensorData & data,
		const Transform & odom,
		const std::vector<float> & odomVelocityIn,
		const std::string & odomFrameId,
		const cv::Mat & odomCovariance,
		const OdometryInfo & odomInfo,
		double timeMsgConversion, const rtabmap::ImageInputIdentity & imageIdentity)
{
	UTimer timer;
	if(safeRelocalizationEnabled_ && !verifySafeRelocalizationDatabasePathBinding())
	{
		return;
	}
	bool resetBoundaryTriggeredThisUpdate = false;
	if(resetQuarantineEnabled_)
	{
		const long long lastOdomInfoStampNs = resetQuarantineLastOdomInfoStampNs_.load(
				std::memory_order_acquire);
		const long long processStampNs = stamp.nanoseconds();
		const long long epochStartStampNs = resetQuarantineEpochStartStampNs_.load(
				std::memory_order_acquire);
		if(epochStartStampNs > 0 && processStampNs < epochStartStampNs)
		{
			RCLCPP_ERROR_THROTTLE(
					this->get_logger(), *this->get_clock(), 5000,
					"reset quarantine rejected a delayed pre-epoch RGB-D frame before any recovery/continuation state could be consumed (process_stamp_ns=%lld epoch_start_stamp_ns=%lld)",
					processStampNs, epochStartStampNs);
			return;
		}
		const long long nowSteadyNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::steady_clock::now().time_since_epoch()).count();
		const double odomInfoStampDeltaSec =
				lastOdomInfoStampNs > 0 && processStampNs > 0?
				double(processStampNs - lastOdomInfoStampNs) / 1000000000.0 :
				std::numeric_limits<double>::infinity();
		bool exactAuthenticatedOdomInfo = false;
		long long exactOdomInfoArrivalSteadyNs = 0;
		bool odomInfoSourceCompromised = false;
		{
			std::lock_guard<std::mutex> authLock(resetQuarantineOdomInfoAuthMutex_);
			odomInfoSourceCompromised = resetQuarantineOdomInfoPublisherCompromised_;
		}
		{
			UScopeMutex sourceLock(resetQuarantineStateMutex_);
			std::map<long long, long long>::iterator authenticated =
					resetQuarantineAuthenticatedOdomInfoStamps_.find(processStampNs);
			if(!odomInfoSourceCompromised &&
					authenticated != resetQuarantineAuthenticatedOdomInfoStamps_.end())
			{
				exactAuthenticatedOdomInfo = true;
				exactOdomInfoArrivalSteadyNs = authenticated->second;
				resetQuarantineAuthenticatedOdomInfoStamps_.erase(authenticated);
			}
		}
		const double exactOdomInfoAgeSec = exactOdomInfoArrivalSteadyNs > 0?
				double(nowSteadyNs - exactOdomInfoArrivalSteadyNs) / 1000000000.0 :
				std::numeric_limits<double>::infinity();
		if(!resetQuarantineOdomInfoSeen_.load(std::memory_order_acquire) ||
				odomInfoSourceCompromised ||
				!exactAuthenticatedOdomInfo ||
				exactOdomInfoAgeSec < 0.0 ||
				exactOdomInfoAgeSec > resetQuarantineOdomInfoMaxAgeSec_)
		{
			RCLCPP_ERROR_THROTTLE(
					this->get_logger(),
					*this->get_clock(),
					5000,
					"reset quarantine odom-info monitor is missing, compromised, arrival-stale or lacks an exact authenticated process stamp (arrival_age=%.3fs latest_stamp_delta=%.3fs limit=%.3fs process_stamp_ns=%lld latest_odom_info_stamp_ns=%lld exact=%s compromised=%s); refusing graph updates",
					exactOdomInfoAgeSec,
					odomInfoStampDeltaSec,
					resetQuarantineOdomInfoMaxAgeSec_,
					processStampNs,
					lastOdomInfoStampNs,
					exactAuthenticatedOdomInfo?"true":"false",
					odomInfoSourceCompromised?"true":"false");
			return;
		}
		if(odom.isNull() || !data.isValid())
		{
			RCLCPP_ERROR_THROTTLE(
					this->get_logger(),
					*this->get_clock(),
					5000,
					"reset quarantine refuses to consume a pending recovery on invalid RGB-D/odometry data");
			return;
		}
		if(createIntermediateNodes_ || !interOdoms_.empty())
		{
			RCLCPP_FATAL(this->get_logger(),
					"reset quarantine fail-stop: unauthenticated intermediate odometry is enabled or queued");
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
			return;
		}
		const long long recoveryStampNs = resetQuarantineRecoveryStampNs_.load(
				std::memory_order_acquire);
		if(resetQuarantineRecoveryPending_.load(std::memory_order_acquire) &&
				(processStampNs <= 0 || recoveryStampNs <= 0 || processStampNs < recoveryStampNs))
		{
			RCLCPP_WARN(
					this->get_logger(),
					"reset quarantine rejects a pre-recovery or unstamped RGB-D update without consuming the pending boundary (process_stamp_ns=%lld recovery_stamp_ns=%lld)",
					processStampNs,
					recoveryStampNs);
			return;
		}
		bool expectedRecoveryPending = true;
		if(resetQuarantineRecoveryPending_.compare_exchange_strong(
				expectedRecoveryPending,
				false,
				std::memory_order_acq_rel,
				std::memory_order_acquire))
		{
			resetQuarantineSuspectedReset_.store(false, std::memory_order_release);
			if(!rtabmap_.getLocalOptimizedPoses().empty())
			{
				const int newMapId = rtabmap_.triggerNewMapForContinuation();
				if(newMapId < 0)
				{
					RCLCPP_FATAL(this->get_logger(),
							"reset quarantine fail-stop: trusted graph promotion rejected the recovery boundary transaction");
					safeRelocalizationFatal_.store(true, std::memory_order_release);
					rclcpp::shutdown();
					return;
				}
				resetBoundaryTriggeredThisUpdate = true;
				{
					std::lock_guard<std::mutex> imuAuthLock(resetQuarantineImuAuthMutex_);
					UScopeMutex imuLock(imuMutex_);
					resetQuarantineEpochStartStampNs_.store(
							processStampNs, std::memory_order_release);
					managed_imu::cut(imus_, resetQuarantineImuStampNsByTime_, processStampNs);
					if(imus_.empty()) imuFrameId_.clear();
				}
				clearSafeRelocalizationTokens();
				const unsigned long long epoch = rtabmap_.getResetEpoch();
				resetQuarantineEpoch_.store(epoch, std::memory_order_release);
				++safeRelocalizationGraphRevision_;
				publishResetEpoch();
				RCLCPP_WARN(
						this->get_logger(),
					"reset quarantine: recovery epoch=%llu triggered map_id=%d on first valid RGB-D update at-or-after recovery stamp (process_stamp_ns=%lld recovery_stamp_ns=%lld lost_start_stamp_ns=%lld); no neighbor edge crosses the lost boundary",
					epoch,
					newMapId,
					processStampNs,
					recoveryStampNs,
					resetQuarantineLostStartStampNs_.load(std::memory_order_acquire));
			}
			else
			{
				RCLCPP_WARN(
						this->get_logger(),
						"reset quarantine observed a lost/recovered sequence before a graph existed; no map boundary was required");
			}
		}
	}
	// Continuation is a graph mutation, so it must happen only after the same
	// exact-stamp OdomInfo, source, RGB-D and odometry gates that protect a
	// normal update.  A missing/invalid first callback must leave the epoch,
	// revision and first-update latch untouched.
	if(safeRelocalizationStartNewMapOnFirstUpdate_ &&
			!safeRelocalizationFirstUpdateHandled_)
	{
		if(!resetBoundaryTriggeredThisUpdate &&
				!rtabmap_.getLocalOptimizedPoses().empty())
		{
			const int newMapId = rtabmap_.triggerNewMapForContinuation();
			if(newMapId < 0)
			{
				RCLCPP_FATAL(this->get_logger(),
						"safe relocalization fail-stop: trusted graph promotion rejected the startup continuation transaction");
				safeRelocalizationFatal_.store(true, std::memory_order_release);
				rclcpp::shutdown();
				return;
			}
			resetQuarantineRecoveryPending_.store(false, std::memory_order_release);
			{
				std::lock_guard<std::mutex> imuAuthLock(resetQuarantineImuAuthMutex_);
				UScopeMutex imuLock(imuMutex_);
				resetQuarantineEpochStartStampNs_.store(
						stamp.nanoseconds(), std::memory_order_release);
				managed_imu::cut(imus_, resetQuarantineImuStampNsByTime_, stamp.nanoseconds());
				if(imus_.empty()) imuFrameId_.clear();
			}
			clearSafeRelocalizationTokens();
			resetQuarantineEpoch_.store(rtabmap_.getResetEpoch(), std::memory_order_release);
			++safeRelocalizationGraphRevision_;
			publishResetEpoch();
			RCLCPP_WARN(this->get_logger(),
					"safe relocalization continuation started a new map_id=%d reset_epoch=%llu immediately before the first authenticated RGB-D update",
					newMapId, rtabmap_.getResetEpoch());
		}
		safeRelocalizationFirstUpdateHandled_ = true;
	}
	if(rtabmap_.isIDsGenerated() || data.id() > 0)
	{
		// Add intermediate nodes?
		for(std::list<std::pair<nav_msgs::msg::Odometry, rtabmap_msgs::msg::OdomInfo> >::iterator iter=interOdoms_.begin(); iter!=interOdoms_.end();)
		{
			if(rclcpp::Time(iter->first.header.stamp.sec, iter->first.header.stamp.nanosec) < stamp)
			{
				Transform interOdom;
				if(!rtabmap_.getLocalOptimizedPoses().empty())
				{
					// add intermediate poses only if the current local graph is not empty
					interOdom = rtabmap_conversions::transformFromPoseMsg(iter->first.pose.pose);
				}
				if(!interOdom.isNull())
				{
					cv::Mat covariance;
					double variance = iter->first.twist.covariance[0];
					if(variance == BAD_COVARIANCE || variance <= 0.0f)
					{
						//use the one of the pose
						covariance = cv::Mat(6,6,CV_64FC1, (void*)iter->first.pose.covariance.data()).clone();
						covariance /= 2.0;
					}
					else
					{
						covariance = cv::Mat(6,6,CV_64FC1, (void*)iter->first.twist.covariance.data()).clone();
					}
					if(!uIsFinite(covariance.at<double>(0,0)) || covariance.at<double>(0,0)<=0.0f)
					{
						covariance = cv::Mat::eye(6,6,CV_64FC1);
						if(odomDefaultLinVariance_ > 0.0f)
						{
							covariance.at<double>(0,0) = odomDefaultLinVariance_;
							covariance.at<double>(1,1) = odomDefaultLinVariance_;
							covariance.at<double>(2,2) = odomDefaultLinVariance_;
						}
						if(odomDefaultAngVariance_ > 0.0f)
						{
							covariance.at<double>(3,3) = odomDefaultAngVariance_;
							covariance.at<double>(4,4) = odomDefaultAngVariance_;
							covariance.at<double>(5,5) = odomDefaultAngVariance_;
						}
					}
					else if(twoDMapping_)
					{
						// If 2d mapping, make sure all diagonal values of the covariance that even not used are not null.
						covariance.at<double>(2,2) = uIsFinite(covariance.at<double>(2,2)) && covariance.at<double>(2,2)!=0?covariance.at<double>(2,2):1;
						covariance.at<double>(3,3) = uIsFinite(covariance.at<double>(3,3)) && covariance.at<double>(3,3)!=0?covariance.at<double>(3,3):1;
						covariance.at<double>(4,4) = uIsFinite(covariance.at<double>(4,4)) && covariance.at<double>(4,4)!=0?covariance.at<double>(4,4):1;
					}

					SensorData interData(cv::Mat(), cv::Mat(), rtabmap::CameraModel(), -1, rtabmap_conversions::timestampFromROS(iter->first.header.stamp));
					Transform gt;
					if(!groundTruthFrameId_.empty())
					{
						gt = rtabmap_conversions::getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, iter->first.header.stamp, *tfBuffer_, waitForTransform_);
					}
					interData.setGroundTruth(gt);

					std::map<std::string, float> externalStats;
					std::vector<float> odomVelocity;
					if(iter->second.time_estimation != 0.0f)
					{
						OdometryInfo info = rtabmap_conversions::odomInfoFromROS(iter->second, true);
						externalStats = rtabmap_conversions::odomInfoToStatistics(info);

						if(info.interval>0.0)
						{
							odomVelocity.resize(6);
							float x,y,z,roll,pitch,yaw;
							info.transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
							odomVelocity[0] = x/info.interval;
							odomVelocity[1] = y/info.interval;
							odomVelocity[2] = z/info.interval;
							odomVelocity[3] = roll/info.interval;
							odomVelocity[4] = pitch/info.interval;
							odomVelocity[5] = yaw/info.interval;
						}
					}
					if(odomVelocity.empty())
					{
						odomVelocity.resize(6);
						odomVelocity[0] = iter->first.twist.twist.linear.x;
						odomVelocity[1] = iter->first.twist.twist.linear.y;
						odomVelocity[2] = iter->first.twist.twist.linear.z;
						odomVelocity[3] = iter->first.twist.twist.angular.x;
						odomVelocity[4] = iter->first.twist.twist.angular.y;
						odomVelocity[5] = iter->first.twist.twist.angular.z;
					}

					const unsigned long long epochBeforeIntermediate = rtabmap_.getResetEpoch();
					const bool intermediateProcessed =
							rtabmap_.process(interData, interOdom, covariance, odomVelocity, externalStats);
					const bool intermediateEpochChanged =
							epochBeforeIntermediate != rtabmap_.getResetEpoch();
					if(intermediateProcessed)
					{
						// Intermediate signatures are graph revisions too. Publish the
						// authoritative revision before any later proposal callback can run,
						// even if the main RGB-D update below is ultimately rejected.
						++safeRelocalizationGraphRevision_;
						publishResetEpoch();
					}
					else if(intermediateEpochChanged)
					{
						clearSafeRelocalizationTokens();
						resetQuarantineEpoch_.store(rtabmap_.getResetEpoch(), std::memory_order_release);
						++safeRelocalizationGraphRevision_;
						publishResetEpoch();
					}
				}
				interOdoms_.erase(iter++);
			}
			else if(iter->first.header.stamp == stamp)
			{
				interOdoms_.erase(iter++);
				break;
			}
			else
			{
				break;
			}
		}

		//Add async stuff
		Transform groundTruthPose;
		if(!groundTruthFrameId_.empty())
		{
			groundTruthPose = rtabmap_conversions::getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, stamp, *tfBuffer_, waitForTransform_);
		}
		data.setGroundTruth(groundTruthPose);

		//global pose
		geometry_msgs::msg::PoseWithCovarianceStamped globalPoseMsg;
		globalPoseMsg.header.stamp = rclcpp::Time(0);
		{
			UScopeMutex lock(globalPoseMutex_);
			if(!globalPoses_.empty())
			{
				auto iter = rtabmap_conversions::getClosestIterator<double, geometry_msgs::msg::PoseWithCovarianceStamped>(globalPoses_, data.stamp());
				// Check if it is not too old
				if(rate_ == 0 || fabs(iter->first - data.stamp()) < 1.0/rate_)
				{
					globalPoseMsg = iter->second;
				}
				else
				{
					RCLCPP_WARN(this->get_logger(), "Ignoring global pose with stamp %f because it should be inside the update period (%f) of the current data stamp (%f).",
						iter->first,
						1.0/rate_,
						data.stamp());
				}
				globalPoses_.clear();
			}
		}
		if(globalPoseMsg.header.stamp.sec != 0 || globalPoseMsg.header.stamp.nanosec != 0)
		{
			// assume sensor is fixed
			Transform sensorToBase = rtabmap_conversions::getTransform(
					globalPoseMsg.header.frame_id,
					frameId_,
					stamp,
					*tfBuffer_,
					waitForTransform_);
			if(!sensorToBase.isNull())
			{
				Transform globalPose = rtabmap_conversions::transformFromPoseMsg(globalPoseMsg.pose.pose);
				globalPose *= sensorToBase; // transform global pose from sensor frame to robot base frame

				// Correction of the global pose accounting the odometry movement since we received it
				Transform correction = rtabmap_conversions::getMovingTransform(
						frameId_,
						odomFrameId,
						stamp,
						rclcpp::Time(globalPoseMsg.header.stamp.sec, globalPoseMsg.header.stamp.nanosec),
						*tfBuffer_,
						waitForTransform_);
				if(!correction.isNull())
				{
					globalPose *= correction;
				}
				else
				{
					RCLCPP_WARN(this->get_logger(), "Could not adjust global pose accordingly to latest odometry pose. "
							"If odometry is small since it received the global pose and "
							"covariance is large, this should not be a problem.");
				}
				cv::Mat globalPoseCovariance = cv::Mat(6,6, CV_64FC1, (void*)globalPoseMsg.pose.covariance.data()).clone();
				data.setGlobalPose(globalPose, globalPoseCovariance);
			}
		}

		{
			UScopeMutex lock(gpsMutex_);
			if(!gps_.empty())
			{
				std::map<double, rtabmap::GPS>::const_iterator iter = rtabmap_conversions::getClosestIterator<double, rtabmap::GPS>(gps_, data.stamp());
				// Check if it is not too old
				if(rate_ == 0 || fabs(iter->first - data.stamp()) < 1.0/rate_)
				{
					data.setGPS(iter->second);
				}
				else
				{
					RCLCPP_WARN(this->get_logger(), "Ignoring GPS with stamp %f because it should be inside the update period (%f) of the current data stamp (%f).",
						iter->first,
						1.0/rate_,
						data.stamp());
				}
				gps_.clear();
			}
		}
		

		// Safe relocalization accepts tag evidence only through the authenticated
		// three-view transaction. Always clear any stale native cache, and never
		// convert it into SensorData while the guard is enabled.
		Landmarks landmarks;
		landmarksMutex_.lock();
		if(!safeRelocalizationEnabled_)
		{
			landmarks = rtabmap_conversions::landmarksFromROS(
					landmarks_,
					frameId_,
					odomFrameId,
					stamp,
					*tfBuffer_,
					waitForTransform_,
					landmarkDefaultLinVariance_,
					landmarkDefaultAngVariance_);
		}
		landmarks_.clear();
		landmarksMutex_.unlock();
		if(!landmarks.empty())
		{
			data.setLandmarks(landmarks);
		}

		// Env sensors
		{
			UScopeMutex lock(envSensorMutex_);
			if(!envSensors_.empty())
			{
				data.setEnvSensors(envSensors_);
				envSensors_.clear();
			}
		}

		// IMU
		imuMutex_.lock();
		if(!imus_.empty())
		{
			Transform t = Transform::getTransform(imus_, data.stamp());
			if(!t.isNull())
			{
				imuMutex_.unlock();
				// get local transform
				rtabmap::Transform localTransform;
				if(resetQuarantineEnabled_)
				{
					// The managed contract already proves that IMU and camera use the
					// same frozen optical frame. Never re-read a mutable TF for graph input.
					localTransform = resetQuarantineExpectedSensorToBaseTransform_;
				}
				else if(frameId_.compare(imuFrameId_) != 0)
				{
					localTransform = rtabmap_conversions::getTransform(frameId_, imuFrameId_, rtabmap_conversions::timestampToROS(data.stamp()), *tfBuffer_, waitForTransform_);
				}
				else
				{
					localTransform = rtabmap::Transform::getIdentity();
				}

				if(!localTransform.isNull())
				{
					Eigen::Quaterniond q = t.getQuaterniond();
					data.setIMU(IMU(cv::Vec4d(q.x(), q.y(), q.z(), q.w()), cv::Mat::eye(3,3,CV_64FC1),
							cv::Vec3d(), cv::Mat(),
							cv::Vec3d(), cv::Mat(),
							localTransform));
				}
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "We are receiving imu data (buffer=%d), but cannot interpolate "
						"imu transform at time %f (latest imu received with stamp %f). IMU won't be added to graph.",
						(int)imus_.size(), data.stamp(), imus_.rbegin()->first);
						imuMutex_.unlock();
			}
		}
		else
		{
			imuMutex_.unlock();
		}

		double timeRtabmap = 0.0;
		double timeUpdateMaps = 0.0;
		double timePublishMaps = 0.0;

		cv::Mat covariance = odomCovariance;
		if(covariance.empty() || !uIsFinite(covariance.at<double>(0,0)) || covariance.at<double>(0,0)<=0.0f)
		{
			covariance = cv::Mat::eye(6,6,CV_64FC1);
			if(odomDefaultLinVariance_ > 0.0f)
			{
				covariance.at<double>(0,0) = odomDefaultLinVariance_;
				covariance.at<double>(1,1) = odomDefaultLinVariance_;
				covariance.at<double>(2,2) = odomDefaultLinVariance_;
			}
			if(odomDefaultAngVariance_ > 0.0f)
			{
				covariance.at<double>(3,3) = odomDefaultAngVariance_;
				covariance.at<double>(4,4) = odomDefaultAngVariance_;
				covariance.at<double>(5,5) = odomDefaultAngVariance_;
			}
		}
		else if(twoDMapping_)
		{
			// If 2d mapping, make sure all diagonal values of the covariance that even not used are not null.
			covariance.at<double>(2,2) = uIsFinite(covariance.at<double>(2,2)) && covariance.at<double>(2,2)!=0?covariance.at<double>(2,2):1;
			covariance.at<double>(3,3) = uIsFinite(covariance.at<double>(3,3)) && covariance.at<double>(3,3)!=0?covariance.at<double>(3,3):1;
			covariance.at<double>(4,4) = uIsFinite(covariance.at<double>(4,4)) && covariance.at<double>(4,4)!=0?covariance.at<double>(4,4):1;
		}

		std::map<std::string, float> externalStats;
		std::vector<float> odomVelocity;
		if(odomInfo.timeEstimation != 0.0f)
		{
			externalStats = rtabmap_conversions::odomInfoToStatistics(odomInfo);

			if(odomInfo.interval>0.0)
			{
				odomVelocity.resize(6);
				float x,y,z,roll,pitch,yaw;
				odomInfo.transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
				odomVelocity[0] = x/odomInfo.interval;
				odomVelocity[1] = y/odomInfo.interval;
				odomVelocity[2] = z/odomInfo.interval;
				odomVelocity[3] = roll/odomInfo.interval;
				odomVelocity[4] = pitch/odomInfo.interval;
				odomVelocity[5] = yaw/odomInfo.interval;
			}
		}
		if(odomVelocity.empty())
		{
			odomVelocity = odomVelocityIn;
		}
		if(rtabmapROSStats_.size())
		{
			externalStats.insert(rtabmapROSStats_.begin(), rtabmapROSStats_.end());
			rtabmapROSStats_.clear();
		}

		timeMsgConversion += timer.ticks();
		const unsigned long long epochBeforeProcess = rtabmap_.getResetEpoch();
        const bool receiptSourceValid = imageIdentity.headerStampNs == stamp.nanoseconds() &&
                queuedImageIdentityStillValid(imageIdentity);
        std::string receiptReason;
		const bool processSucceeded = receiptSourceValid ?
                rtabmap_.processWithAuthenticatedImage(data, odom, covariance, odomVelocity, externalStats, imageIdentity, receiptReason) :
				rtabmap_.process(data, odom, covariance, odomVelocity, externalStats);
		const bool processEpochChanged = epochBeforeProcess != rtabmap_.getResetEpoch();
		if(processSucceeded)
		{
			if(safeRelocalizationEnabled_)
			{
				std::string freezeReason;
				if(!rtabmap_.validateAndFreezeCommittedEpochForSafeRelocalization(freezeReason))
				{
					RCLCPP_FATAL(this->get_logger(),
							"safe relocalization stopped before map publication/save because a committed epoch violated its frozen-graph invariant: %s",
							freezeReason.c_str());
					safeRelocalizationFatal_.store(true, std::memory_order_release);
					rclcpp::shutdown();
					return;
				}
			}
			if(resetQuarantineEpoch_.load(std::memory_order_acquire) != rtabmap_.getResetEpoch())
			{
				clearSafeRelocalizationTokens();
			}
			resetQuarantineEpoch_.store(rtabmap_.getResetEpoch(), std::memory_order_release);
            if(receiptSourceValid)
            {
                rtabmap::AcceptedImageReceipt receipt;
                if(rtabmap_.getAcceptedImageReceipt(rtabmap_.getLastLocationId(), receipt) &&
                   receipt.input.headerStampNs == imageIdentity.headerStampNs &&
                   receipt.input.pixelSha256 == imageIdentity.pixelSha256)
                    publishAcceptedImageReceipt(receipt);
                else RCLCPP_DEBUG(this->get_logger(), "No accepted-image receipt: %s", receiptReason.c_str());
            }
			timeRtabmap = timer.ticks();
			mapToOdomMutex_.lock();
			mapToOdom_ = rtabmap_.getMapCorrection();
			Transform mapToOdomSafe = mapToOdom_.clone();
			if(!odomFrameId.empty() && !odomFrameId_.empty() && odomFrameId_.compare(odomFrameId)!=0)
			{
				RCLCPP_ERROR(get_logger(), "Odometry received doesn't have same frame_id "
						  "than the one previously set (old=%s, new=%s). "
						  "Are there multiple nodes publishing on same odometry topic name? "
						  "The new frame_id is now used.", odomFrameId_.c_str(), odomFrameId.c_str());
			}

			odomFrameId_ = odomFrameId;
			mapToOdomMutex_.unlock();

			if(data.id() < 0)
			{
				RCLCPP_INFO(this->get_logger(), "Intermediate node added");
			}
			else
			{
				if(localizationPosePub_->get_subscription_count())
				{
				    bool localized = rtabmap_.getStatistics().loopClosureId()!=0 ||
							rtabmap_.getStatistics().proximityDetectionId()!=0 ||
							static_cast<int>(uValue(rtabmap_.getStatistics().data(), rtabmap::Statistics::kLoopLandmark_detected(), 0.0f))!=0;

                    if(localized || !pubLocPoseOnlyWhenLocalizing_)
					{
						geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
					    poseMsg.header.frame_id = mapFrameId_;
					    poseMsg.header.stamp = stamp;
						rtabmap_conversions::transformToPoseMsg(mapToOdomSafe*odom, poseMsg.pose.pose);
						if(!rtabmap_.getStatistics().localizationCovariance().empty())
						{
							const cv::Mat & cov = rtabmap_.getStatistics().localizationCovariance();
							memcpy(poseMsg.pose.covariance.data(), cov.data, cov.total()*sizeof(double));
						}
						else
						{
							// Not yet localized, publish large covariance
							poseMsg.pose.covariance.data()[0] = 9999;
							poseMsg.pose.covariance.data()[7] = 9999;
							poseMsg.pose.covariance.data()[14] = twoDMapping_?rtabmap::Registration::COVARIANCE_LINEAR_EPSILON:9999;
							poseMsg.pose.covariance.data()[21] = twoDMapping_?rtabmap::Registration::COVARIANCE_ANGULAR_EPSILON:9999;
							poseMsg.pose.covariance.data()[28] = twoDMapping_?rtabmap::Registration::COVARIANCE_ANGULAR_EPSILON:9999;
							poseMsg.pose.covariance.data()[35] = 9999;
						}
						localizationPosePub_->publish(poseMsg);
					}
				}
				std::map<int, rtabmap::Transform> filteredPoses(rtabmap_.getLocalOptimizedPoses().lower_bound(1), rtabmap_.getLocalOptimizedPoses().end());

				// create a tmp signature with latest sensory data if latest signature was ignored
				std::map<int, rtabmap::Signature> tmpSignature;
				if(rtabmap_.getMemory() == 0 ||
					filteredPoses.size() == 0 ||
					rtabmap_.getMemory()->getLastSignatureId() != filteredPoses.rbegin()->first ||
					rtabmap_.getMemory()->getLastWorkingSignature(false) == 0 ||
					rtabmap_.getMemory()->getLastWorkingSignature(false)->sensorData().gridCellSize() == 0 ||
					(!mapsManager_.getLocalMapMaker()->isGridFromDepth() && data.laserScanRaw().is2d())) // 2d laser scan would fill empty space for latest data
				{
					SensorData tmpData = data;
					tmpData.setId(0);
					tmpSignature.insert(std::make_pair(0, Signature(0, -1, 0, data.stamp(), "", odom, Transform(), tmpData)));
					filteredPoses.insert(std::make_pair(0, mapToOdomSafe*odom));
				}

				if((mappingMaxNodes_ > 0 || mappingAltitudeDelta_>0.0) && filteredPoses.size()>1)
				{
					std::map<int, Transform> nearestPoses = filterNodesToAssemble(filteredPoses, mapToOdomSafe*odom);
					//add latest/zero and make sure those on a planned path are not filtered
					std::set<int> onPath;
					if(rtabmap_.getPath().size())
					{
						std::vector<int> nextNodes = rtabmap_.getPathNextNodes();
						onPath.insert(nextNodes.begin(), nextNodes.end());
					}
					for(std::map<int, Transform>::iterator iter=filteredPoses.begin(); iter!=filteredPoses.end(); ++iter)
					{
						if(iter->first == 0 || onPath.find(iter->first) != onPath.end())
						{
							nearestPoses.insert(*iter);
						}
						else if(onPath.empty())
						{
							break;
						}
					}

					filteredPoses = nearestPoses;
				}

				// Update maps
				filteredPoses = mapsManager_.updateMapCaches(
						filteredPoses,
						rtabmap_.getMemory(),
						false,
						false,
						tmpSignature);

				timeUpdateMaps = timer.ticks();

				mapsManager_.publishMaps(filteredPoses, stamp, mapFrameId_);

				// Publish local graph, info
				this->publishStats(stamp);

				// update goal if planning is enabled
				if(!currentMetricGoal_.isNull())
				{
					if(rtabmap_.getPath().size() == 0)
					{
						// Don't send status yet if nav2 actionlib is used unless it failed,
						// let nav2 finish reaching the goal
#ifdef WITH_NAV2_MSGS
						if(nav2Client_ == 0 || rtabmap_.getPathStatus() <= 0)
#else
						if(rtabmap_.getPathStatus() <= 0)
#endif
						{
							if(rtabmap_.getPathStatus() > 0)
							{
								// Goal reached
								RCLCPP_INFO(this->get_logger(), "Planning: Publishing goal reached!");
							}
							else if(rtabmap_.getPathStatus() <= 0)
							{
								RCLCPP_WARN(this->get_logger(), "Planning: Plan failed!");
#ifdef WITH_NAV2_MSGS
								if(nav2Client_.get()!=NULL && nav2Client_->action_server_is_ready())
								{
									nav2Client_->async_cancel_all_goals();
								}
#endif
							}

							if(goalReachedPub_->get_subscription_count())
							{
								std_msgs::msg::Bool result;
								result.data = rtabmap_.getPathStatus() > 0;
								goalReachedPub_->publish(result);
							}
							currentMetricGoal_.setNull();
							lastPublishedMetricGoal_.setNull();
							goalFrameId_.clear();
							latestNodeWasReached_ = false;
						}
					}
					else
					{
						currentMetricGoal_ = rtabmap_.getPose(rtabmap_.getPathCurrentGoalId());
						if(!currentMetricGoal_.isNull())
						{
							// Adjust the target pose relative to last node
							if(rtabmap_.getPathCurrentGoalId() == rtabmap_.getPath().back().first && rtabmap_.getLocalOptimizedPoses().size())
							{
								if(latestNodeWasReached_ ||
								   rtabmap_.getLastLocalizationPose().getDistance(currentMetricGoal_) < rtabmap_.getLocalRadius())
								{
									latestNodeWasReached_ = true;
									Transform goalLocalTransform = Transform::getIdentity();
									if(!goalFrameId_.empty() && goalFrameId_.compare(frameId_) != 0)
									{
										Transform localT = rtabmap_conversions::getTransform(frameId_, goalFrameId_, now(), *tfBuffer_, waitForTransform_);
										if(!localT.isNull())
										{
											goalLocalTransform = localT.inverse().to3DoF();
										}
									}
									currentMetricGoal_ *= rtabmap_.getPathTransformToGoal()*goalLocalTransform;
								}
							}

							// publish next goal with updated currentMetricGoal_
							publishCurrentGoal(stamp);

							// publish local path
							publishLocalPath(stamp);

							// publish global path
							publishGlobalPath(stamp);
						}
						else
						{
							RCLCPP_ERROR(this->get_logger(), "Planning: Local map broken, current goal id=%d (the robot may have moved to far from planned nodes)",
									rtabmap_.getPathCurrentGoalId());
							rtabmap_.clearPath(-1);
							if(goalReachedPub_->get_subscription_count())
							{
								std_msgs::msg::Bool result;
								result.data = false;
								goalReachedPub_->publish(result);
							}
							currentMetricGoal_.setNull();
							lastPublishedMetricGoal_.setNull();
							goalFrameId_.clear();
							latestNodeWasReached_ = false;
						}
					}
				}

				timePublishMaps = timer.ticks();
			}

			// If not intermediate node
			if(data.id() >= 0)
			{
				localizationDiagnostic_.updateStatus(rtabmap_.getStatistics().localizationCovariance(), twoDMapping_);
				tick(stamp, rate_>0?rate_:1000.0/(timeMsgConversion+timeRtabmap+timeUpdateMaps+timePublishMaps));
			}
			// Publish a revision only after map->odom, enabled map caches and their
			// publications have observed the accepted graph update.
			++safeRelocalizationGraphRevision_;
			publishResetEpoch();
		}
		else
		{
			if(processEpochChanged)
			{
				clearSafeRelocalizationTokens();
				resetQuarantineEpoch_.store(rtabmap_.getResetEpoch(), std::memory_order_release);
				++safeRelocalizationGraphRevision_;
				publishResetEpoch();
			}
			timeRtabmap = timer.ticks();
		}
		RCLCPP_INFO(this->get_logger(), "rtabmap (%d): Rate=%.2fs, Limit=%.3fs, Conversion=%.4fs, RTAB-Map=%.4fs, Maps update=%.4fs pub=%.4fs delay=%.4fs (local map=%d, WM=%d)",
				rtabmap_.getLastLocationId(),
				rate_>0?1.0f/rate_:0,
				rtabmap_.getTimeThreshold()/1000.0f,
				timeMsgConversion,
				timeRtabmap,
				timeUpdateMaps,
				timePublishMaps,
				(now() - stamp).seconds(),
				(int)rtabmap_.getLocalOptimizedPoses().size(),
				rtabmap_.getWMSize()+rtabmap_.getSTMSize());
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/HasSubscribers/"), mapsManager_.hasSubscribers()?1:0));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeMsgConversion/ms"), timeMsgConversion*1000.0f));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeRtabmap/ms"), timeRtabmap*1000.0f));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeUpdatingMaps/ms"), timeUpdateMaps*1000.0f));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimePublishing/ms"), timePublishMaps*1000.0f));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeTotal/ms"), (timeMsgConversion+timeRtabmap+timeUpdateMaps+timePublishMaps)*1000.0f));
	}
	else if(!rtabmap_.isIDsGenerated())
	{
		RCLCPP_WARN(this->get_logger(), "Ignoring received image because its sequence ID=0. Please "
				 "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
				 "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
				 "when you need to have IDs output of RTAB-map synchronized with the source "
				 "image sequence ID.");
	}
}

std::map<int, Transform> CoreWrapper::filterNodesToAssemble(
		const std::map<int, Transform> & nodes,
		const Transform & currentPose)
{
	std::map<int, Transform> output;
	if(mappingMaxNodes_ > 0)
	{
		std::map<int, float> nodesDist = graph::findNearestNodes(currentPose, nodes, 0, 0, mappingMaxNodes_);
		for(std::map<int, float>::iterator iter=nodesDist.begin(); iter!=nodesDist.end(); ++iter)
		{
			if(mappingAltitudeDelta_<=0.0 ||
			   fabs(nodes.at(iter->first).z()-currentPose.z())<mappingAltitudeDelta_)
			{
				output.insert(*nodes.find(iter->first));
			}
		}
	}
	else // mappingAltitudeDelta_>0.0
	{
		for(std::map<int, Transform>::const_iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
		{
			if(fabs(iter->second.z()-currentPose.z())<mappingAltitudeDelta_)
			{
				output.insert(*iter);
			}
		}
	}
	return output;
}

void CoreWrapper::userDataAsyncCallback(const rtabmap_msgs::msg::UserData::SharedPtr dataMsg)
{
	if(!paused_)
	{
		UScopeMutex lock(userDataMutex_);
		static bool warningShow = false;
		if(!userData_.empty() && !warningShow)
		{
			RCLCPP_WARN(this->get_logger(), "Overwriting previous user data set. When asynchronous user "
					"data input topic rate is higher than "
					"map update rate (current %s=%f), only latest data is saved "
					"in the next node created. This message will is shown only once.",
					Parameters::kRtabmapDetectionRate().c_str(), rate_);
			warningShow = true;
		}
		userData_ = rtabmap_conversions::userDataFromROS(*dataMsg);
	}
}

void CoreWrapper::globalPoseAsyncCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr globalPoseMsg)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring global_pose while safe relocalization owns absolute graph alignment");
		return;
	}
	if(!paused_)
	{
		UScopeMutex lock(globalPoseMutex_);
		globalPoses_.insert(std::make_pair(rtabmap_conversions::timestampFromROS(globalPoseMsg->header.stamp), *globalPoseMsg));
		if(globalPoses_.size() > 1000)
		{
			globalPoses_.erase(globalPoses_.begin());
		}
	}
}

void CoreWrapper::gpsFixAsyncCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gpsFixMsg)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring gps/fix while safe relocalization owns absolute graph alignment");
		return;
	}
	if(!paused_)
	{
		double error = 10.0;
		if(gpsFixMsg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
		{
			double variance = uMax3(gpsFixMsg->position_covariance.at(0), gpsFixMsg->position_covariance.at(4), gpsFixMsg->position_covariance.at(8));
			if(variance>0.0)
			{
				error = sqrt(variance);
			}
		}

		rtabmap::GPS gps(
			rtabmap_conversions::timestampFromROS(gpsFixMsg->header.stamp),
			gpsFixMsg->longitude,
			gpsFixMsg->latitude,
			gpsFixMsg->altitude,
			error,
			0);

		UScopeMutex lock(gpsMutex_);
		gps_.insert(std::make_pair(gps.stamp(), gps));
		if(gps_.size() > 1000)
		{
			gps_.erase(gps_.begin());
		}
	}
}

void CoreWrapper::envSensorAsyncCallback(const rtabmap_msgs::msg::EnvSensor::SharedPtr envSensorMsg)
{
	if(!paused_)
	{
		// Can only insert one value for each type per node, keep the most recent
		EnvSensor value = rtabmap_conversions::envSensorFromROS(*envSensorMsg);
		UScopeMutex lock(envSensorMutex_);
		uInsert(envSensors_, std::make_pair(value.type(), value));
	}
}

void CoreWrapper::landmarkDetectionAsyncCallback(const rtabmap_msgs::msg::LandmarkDetection::SharedPtr landmarkDetection)
{
	if(safeRelocalizationEnabled_)
	{
		UScopeMutex lock(landmarksMutex_);
		landmarks_.clear();
		return;
	}
	if(!paused_)
	{
		geometry_msgs::msg::PoseWithCovarianceStamped p;
		p.header = landmarkDetection->header;
		p.pose = landmarkDetection->pose;
		UScopeMutex lock(landmarksMutex_);
		uInsert(landmarks_,
			std::make_pair(landmarkDetection->id,
				std::make_pair(p, landmarkDetection->size)));
	}
}

void CoreWrapper::landmarkDetectionsAsyncCallback(const rtabmap_msgs::msg::LandmarkDetections::SharedPtr landmarkDetections)
{
	if(safeRelocalizationEnabled_)
	{
		UScopeMutex lock(landmarksMutex_);
		landmarks_.clear();
		return;
	}
	if(!paused_)
	{
		UScopeMutex lock(landmarksMutex_);
		for(unsigned int i=0; i<landmarkDetections->landmarks.size(); ++i)
		{
			geometry_msgs::msg::PoseWithCovarianceStamped p;
			p.header = landmarkDetections->landmarks[i].header;
			p.pose = landmarkDetections->landmarks[i].pose;
			uInsert(landmarks_,
				std::make_pair(landmarkDetections->landmarks[i].id,
					std::make_pair(p, landmarkDetections->landmarks[i].size)));
		}
	}
}

#ifdef WITH_APRILTAG_MSGS
void CoreWrapper::tagDetectionsAsyncCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
	if(!paused_)
	{
		static bool warningShow = false;
		if(!warningShow) {
			RCLCPP_WARN(this->get_logger(), "\"tag_detections\" input topic name for apriltag_msgs is deprecated, remap \"apriltag\" input topic name instead. This message is only printed once.");
			warningShow = true;
		}
		apriltagAsyncCallback(msg);
	}
}
void CoreWrapper::apriltagAsyncCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
	if(!paused_)
	{
		UScopeMutex lock(landmarksMutex_);
		for(unsigned int i=0; i<msg->detections.size(); ++i)
		{
			std::string tagFrameId = msg->detections[i].family+":"+uNumber2Str(msg->detections[i].id);
			Transform camToTag = rtabmap_conversions::getTransform(
				msg->header.frame_id, // e.g., camera_optical_frame
				tagFrameId,                     // e.g., tag36h11:42
				msg->header.stamp,
				*tfBuffer_,
				waitForTransform_);
			if(camToTag.isNull())
			{
				RCLCPP_WARN(get_logger(), "Could not get TF between %s and %s frames for tag detection %d.",
					frameId_.c_str(),
					tagFrameId.c_str(),
					msg->detections[i].id);
					continue;
			}

			geometry_msgs::msg::PoseWithCovarianceStamped p;
			rtabmap_conversions::transformToPoseMsg(camToTag, p.pose.pose);
			p.header = msg->header;
			
			uInsert(landmarks_,
					std::make_pair(msg->detections[i].id,
							std::make_pair(p, 0.0f)));
		}
	}
}
#endif

#ifdef WITH_ARUCO_MSGS
void CoreWrapper::arucoAsyncCallback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
{
	if(!paused_)
	{
		UScopeMutex lock(landmarksMutex_);
		for(unsigned int i=0; i<msg->markers.size(); ++i)
		{
			geometry_msgs::msg::PoseWithCovarianceStamped p;
			p.pose = msg->markers[i].pose;
			p.header = msg->markers[i].header;

			uInsert(landmarks_,
					std::make_pair((int)msg->markers[i].id,
							std::make_pair(p, 0.0f)));
		}
	}
}
#endif

#ifdef WITH_ARUCO_OPENCV_MSGS
void CoreWrapper::arucoOpencvAsyncCallback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
{
	if(!paused_)
	{
		UScopeMutex lock(landmarksMutex_);
		for(unsigned int i=0; i<msg->markers.size(); ++i)
		{
			geometry_msgs::msg::PoseWithCovarianceStamped p;
			p.pose.pose = msg->markers[i].pose;
			p.header = msg->header;
			
			uInsert(landmarks_,
					std::make_pair((int)msg->markers[i].marker_id,
							std::make_pair(p, 0.0f)));
		}
	}
}
#endif

#ifdef WITH_ARUCO_MARKERS_MSGS
void CoreWrapper::arucoMarkersAsyncCallback(const aruco_markers_msgs::msg::MarkerArray::SharedPtr msg)
{
	if(!paused_)
	{
		UScopeMutex lock(landmarksMutex_);
		for(unsigned int i=0; i<msg->markers.size(); ++i)
		{
			geometry_msgs::msg::PoseWithCovarianceStamped p;
			p.pose.pose = msg->markers[i].pose.pose;
			p.header = msg->markers[i].pose.header;
			
			uInsert(landmarks_,
					std::make_pair((int)msg->markers[i].id,
							std::make_pair(p, 0.0f)));
		}
	}
}
#endif

#ifdef WITH_ROS2_ARUCO_INTERFACES
void CoreWrapper::arucoInterfacesAsyncCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
	if(!paused_)
	{
		UScopeMutex lock(landmarksMutex_);
		UASSERT(msg->marker_ids.size() == msg->poses.size());
		for(unsigned int i=0; i<msg->marker_ids.size(); ++i)
		{
			geometry_msgs::msg::PoseWithCovarianceStamped p;
			p.pose.pose = msg->poses[i];
			p.header = msg->header;
			
			uInsert(landmarks_,
					std::make_pair((int)msg->marker_ids[i],
							std::make_pair(p, 0.0f)));
		}
	}
}
#endif

#ifdef WITH_FIDUCIAL_MSGS
void CoreWrapper::fiducialDetectionsAsyncCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr fiducialDetections)
{
	if(!paused_)
	{
		UScopeMutex lock(landmarksMutex_);
		for(unsigned int i=0; i<fiducialDetections.transforms.size(); ++i)
		{
			geometry_msgs::PoseWithCovarianceStamped p;
			p.pose.pose.orientation = fiducialDetections.transforms[i].transform.rotation;
			p.pose.pose.position.x = fiducialDetections.transforms[i].transform.translation.x;
			p.pose.pose.position.y = fiducialDetections.transforms[i].transform.translation.y;
			p.pose.pose.position.z = fiducialDetections.transforms[i].transform.translation.z;
			p.header = fiducialDetections.header;
			uInsert(landmarks_,
					std::make_pair(fiducialDetections.transforms[i].fiducial_id,
							std::make_pair(p, 0.0f)));
		}
	}
}
#endif

void CoreWrapper::imuAsyncCallback(
		const sensor_msgs::msg::Imu::SharedPtr msg,
		const rclcpp::MessageInfo & messageInfo)
{
	if(!msg)
	{
		return;
	}
	if(resetQuarantineEnabled_)
	{
		const rmw_gid_t & incomingGid =
				messageInfo.get_rmw_message_info().publisher_gid;
		std::lock_guard<std::mutex> authLock(resetQuarantineImuAuthMutex_);
		const bool recheckEndpoint = !resetQuarantineImuPublisherBound_ ||
				(++resetQuarantineImuEndpointCheckCounter_ % 256U) == 0U;
		bool sourceValid = true;
		if(recheckEndpoint)
		{
			sourceValid = validateResetQuarantinePublisher(
					imuSub_->get_topic_name(), "IMU",
					resetQuarantineExpectedImuNode_, resetQuarantineExpectedImuNamespace_,
					resetQuarantineImuPublisherBound_, resetQuarantineImuPublisherCompromised_,
					resetQuarantineImuPublisherGid_, &incomingGid);
		}
		else if(!std::equal(resetQuarantineImuPublisherGid_.begin(),
				resetQuarantineImuPublisherGid_.end(), incomingGid.data))
		{
			resetQuarantineImuPublisherCompromised_ = true;
			sourceValid = false;
			RCLCPP_FATAL(this->get_logger(),
					"reset quarantine fail-stop: IMU publisher GID changed during the run");
		}
		if(!sourceValid)
		{
			if(resetQuarantineImuPublisherCompromised_)
			{
				safeRelocalizationFatal_.store(true, std::memory_order_release);
				rclcpp::shutdown();
			}
			return;
		}
		const long long stampNs = rclcpp::Time(msg->header.stamp).nanoseconds();
		const double quaternionNorm = std::sqrt(
				msg->orientation.x * msg->orientation.x +
				msg->orientation.y * msg->orientation.y +
				msg->orientation.z * msg->orientation.z +
				msg->orientation.w * msg->orientation.w);
		const bool finite =
				std::isfinite(msg->orientation.x) && std::isfinite(msg->orientation.y) &&
				std::isfinite(msg->orientation.z) && std::isfinite(msg->orientation.w) &&
				std::isfinite(msg->angular_velocity.x) && std::isfinite(msg->angular_velocity.y) &&
				std::isfinite(msg->angular_velocity.z) &&
				std::isfinite(msg->linear_acceleration.x) && std::isfinite(msg->linear_acceleration.y) &&
				std::isfinite(msg->linear_acceleration.z) &&
				msg->orientation_covariance[0] != -1.0;
		bool finiteCovariances = true;
		for(size_t i=0; i<9U; ++i)
		{
			finiteCovariances = finiteCovariances &&
					std::isfinite(msg->orientation_covariance[i]) &&
					std::isfinite(msg->angular_velocity_covariance[i]) &&
					std::isfinite(msg->linear_acceleration_covariance[i]);
		}
		if(stampNs <= 0 || msg->header.frame_id != resetQuarantineExpectedImuFrame_ ||
				!finite || !finiteCovariances || !std::isfinite(quaternionNorm) ||
				std::fabs(quaternionNorm - 1.0) > 1e-3)
		{
			resetQuarantineImuPublisherCompromised_ = true;
			RCLCPP_FATAL(this->get_logger(),
					"reset quarantine fail-stop: invalid IMU identity/stamp/payload");
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
			return;
		}
		const std::string fingerprint = managedRosMessageFingerprint(*msg);
		if(stampNs == resetQuarantineLastImuStampNs_)
		{
			if(resetQuarantineLastImuFingerprint_ != fingerprint)
			{
				resetQuarantineImuPublisherCompromised_ = true;
				RCLCPP_FATAL(this->get_logger(),
						"reset quarantine fail-stop: one managed IMU stamp carried two payloads");
				safeRelocalizationFatal_.store(true, std::memory_order_release);
				rclcpp::shutdown();
			}
			return;
		}
		if(stampNs < resetQuarantineLastImuStampNs_)
		{
			RCLCPP_WARN(this->get_logger(),
					"reset quarantine rejected out-of-order IMU stamp_ns=%lld latest_ns=%lld",
					stampNs, resetQuarantineLastImuStampNs_);
			return;
		}
		resetQuarantineLastImuStampNs_ = stampNs;
		resetQuarantineLastImuFingerprint_ = fingerprint;
		if(paused_)
		{
			return;
		}
		// Keep auth->IMU as the single lock order shared with an epoch cut.
		// A callback authenticated before a cut is either inserted before the
		// cut-and-prune, or observes the new cutoff and is rejected afterward.
		UScopeMutex imuLock(imuMutex_);
		const long long epochStartNs =
				resetQuarantineEpochStartStampNs_.load(std::memory_order_acquire);
		if(epochStartNs > 0 && stampNs < epochStartNs)
		{
			RCLCPP_WARN(this->get_logger(),
					"reset quarantine rejected pre-epoch IMU stamp_ns=%lld cutoff_ns=%lld",
					stampNs, epochStartNs);
			return;
		}
		Transform orientation(0,0,0, msg->orientation.x, msg->orientation.y,
				msg->orientation.z, msg->orientation.w);
		managed_imu::insert(imus_, resetQuarantineImuStampNsByTime_,
				rtabmap_conversions::timestampFromROS(msg->header.stamp), stampNs, orientation);
		imuFrameId_ = msg->header.frame_id;
		return;
	}
	if(!resetQuarantineEnabled_ && !paused_)
	{
		if(msg->orientation.x == 0 && msg->orientation.y == 0 && msg->orientation.z == 0 && msg->orientation.w == 0)
		{
			UERROR("IMU received doesn't have orientation set, it is ignored.");
		}
		else
		{
			UScopeMutex lock(imuMutex_);
			Transform orientation(0,0,0, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
			imus_.insert(std::make_pair(rtabmap_conversions::timestampFromROS(msg->header.stamp), orientation));
			if(imus_.size() > 1000)
			{
				imus_.erase(imus_.begin());
			}
			if(!imuFrameId_.empty() && imuFrameId_.compare(msg->header.frame_id) != 0)
			{
				RCLCPP_ERROR(get_logger(), "IMU frame_id has changed from %s to %s! Are "
						"multiple nodes publishing "
						"on same topic %s? IMU buffer is cleared!",
						imuFrameId_.c_str(),
						msg->header.frame_id.c_str(),
						imuSub_->get_topic_name());
				imus_.clear();
				resetQuarantineImuStampNsByTime_.clear();
				imuFrameId_.clear();
			}
			else
			{
				imuFrameId_ = msg->header.frame_id;
			}
		}
	}
}

void CoreWrapper::republishNodeDataCallback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg)
{
	rtabmap_.addNodesToRepublish(msg->data);
}

void CoreWrapper::interOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	if(!paused_)
	{
		interOdoms_.push_back(std::make_pair(*msg, rtabmap_msgs::msg::OdomInfo()));
	}
}

void CoreWrapper::interOdomInfoCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg1, const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr & msg2)
{
	if(!paused_)
	{
		interOdoms_.push_back(std::make_pair(*msg1, *msg2));
	}
}

bool CoreWrapper::validateResetQuarantinePublisher(
		const std::string & topicName,
		const char * streamName,
		const std::string & expectedNodeName,
		const std::string & expectedNodeNamespace,
		bool & bound,
		bool & compromised,
		std::array<uint8_t, RMW_GID_STORAGE_SIZE> & boundGid,
		const rmw_gid_t * incomingGid)
{
	if(compromised)
	{
		return false;
	}
	const std::vector<rclcpp::TopicEndpointInfo> publishers =
			this->get_publishers_info_by_topic(topicName);
	if(publishers.size() != 1U)
	{
		if(publishers.size() > 1U)
		{
			compromised = true;
			RCLCPP_FATAL(this->get_logger(),
					"reset quarantine fail-stop: expected exactly one managed %s publisher on %s, found %zu",
					streamName, topicName.c_str(), publishers.size());
		}
		return false;
	}
	const auto & endpointGid = publishers.front().endpoint_gid();
	const bool endpointGidNonzero = std::any_of(
			endpointGid.begin(), endpointGid.end(), [](uint8_t value){return value != 0U;});
	const bool incomingGidNonzero = incomingGid != 0 && std::any_of(
			incomingGid->data, incomingGid->data + RMW_GID_STORAGE_SIZE,
			[](uint8_t value){return value != 0U;});
	const bool expectedNode =
			publishers.front().node_name() == expectedNodeName &&
			publishers.front().node_namespace() == expectedNodeNamespace;
	const bool incomingMatches = incomingGidNonzero && endpointGidNonzero &&
			(endpointGid.size() == RMW_GID_STORAGE_SIZE &&
			 std::equal(endpointGid.begin(), endpointGid.end(), incomingGid->data));
	const bool pendingName = publishers.front().node_name() == "_NODE_NAME_UNKNOWN_";
	const bool pendingNamespace = publishers.front().node_namespace() == "_NODE_NAMESPACE_UNKNOWN_";
	if(!bound && incomingMatches && (pendingName || pendingNamespace) &&
			(pendingName || publishers.front().node_name() == expectedNodeName) &&
			(pendingNamespace || publishers.front().node_namespace() == expectedNodeNamespace))
	{
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
				"reset quarantine waiting for ROS graph identity on %s; sample discarded", topicName.c_str());
		return false;
	}
	if(!expectedNode || !incomingMatches || endpointGid.size() != RMW_GID_STORAGE_SIZE)
	{
		compromised = true;
		RCLCPP_FATAL(this->get_logger(),
				"reset quarantine fail-stop: %s source endpoint or node identity mismatch (topic=%s node=%s namespace=%s expected=%s%s)",
				streamName, topicName.c_str(), publishers.front().node_name().c_str(),
				publishers.front().node_namespace().c_str(),
				expectedNodeNamespace.c_str(), expectedNodeName.c_str());
		return false;
	}
	if(!bound)
	{
		std::copy(endpointGid.begin(), endpointGid.end(), boundGid.begin());
		bound = true;
	}
	else if(!std::equal(boundGid.begin(), boundGid.end(), endpointGid.begin()))
	{
		compromised = true;
		RCLCPP_FATAL(this->get_logger(),
				"reset quarantine fail-stop: managed %s publisher GID changed during the run",
				streamName);
		return false;
	}
	return true;
}

bool CoreWrapper::validateResetQuarantineSensorExtrinsic(
		const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs)
{
	if(cameraInfoMsgs.empty())
	{
		RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
				"reset quarantine rejected RGB-D input without CameraInfo");
		return false;
	}
	const rclcpp::Time stamp(cameraInfoMsgs.front().header.stamp);
	if(stamp.nanoseconds() <= 0)
	{
		RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
				"reset quarantine rejected CameraInfo with a null measurement stamp");
		return false;
	}
	for(size_t i=0; i<cameraInfoMsgs.size(); ++i)
	{
		if(cameraInfoMsgs[i].header.frame_id != resetQuarantineExpectedSensorFrame_ ||
				rclcpp::Time(cameraInfoMsgs[i].header.stamp).nanoseconds() != stamp.nanoseconds())
		{
			RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
					"reset quarantine rejected CameraInfo identity/stamp mismatch at index=%zu (frame=%s expected=%s)",
					i, cameraInfoMsgs[i].header.frame_id.c_str(),
					resetQuarantineExpectedSensorFrame_.c_str());
			return false;
		}
	}
	const Transform measured = rtabmap_conversions::getTransform(
			frameId_, resetQuarantineExpectedSensorFrame_, stamp,
			*tfBuffer_, waitForTransform_);
	if(measured.isNull())
	{
		RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
				"reset quarantine rejected RGB-D input because the frozen camera extrinsic is unavailable");
		return false;
	}
	const Transform delta = resetQuarantineExpectedSensorToBaseTransform_.inverse() * measured;
	const double translationError = delta.getNorm();
	const double rotationError = delta.getAngle(rtabmap::Transform::getIdentity());
	if(!std::isfinite(translationError) || !std::isfinite(rotationError) ||
			translationError > 1e-6 || rotationError > 1e-6)
	{
		RCLCPP_FATAL(this->get_logger(),
				"reset quarantine fail-stop: camera extrinsic changed from the frozen deployment value (translation_error=%.9f rotation_error_rad=%.9f)",
				translationError, rotationError);
		safeRelocalizationFatal_.store(true, std::memory_order_release);
		rclcpp::shutdown();
		return false;
	}
	return true;
}

bool CoreWrapper::verifySafeRelocalizationDatabasePathBinding()
{
	if(!safeRelocalizationEnabled_)
	{
		return true;
	}
	if(safeRelocalizationDatabaseBindingValid_ &&
			safeDatabasePathStillBound(
				databasePath_, safeRelocalizationDatabaseFd_,
				safeRelocalizationDatabaseDevice_, safeRelocalizationDatabaseInode_))
	{
		return true;
	}
	safeRelocalizationDatabaseBindingValid_ = false;
	RCLCPP_FATAL(this->get_logger(),
			"safe relocalization fail-stop: database pathname no longer resolves to the held startup inode");
	safeRelocalizationFatal_.store(true, std::memory_order_release);
	rclcpp::shutdown();
	return false;
}

void CoreWrapper::resetQuarantineOdomCallback(
		const nav_msgs::msg::Odometry::SharedPtr msg,
		const rclcpp::MessageInfo & messageInfo)
{
	if(!msg || !resetQuarantineEnabled_)
	{
		return;
	}
	const rmw_gid_t & incomingGid =
			messageInfo.get_rmw_message_info().publisher_gid;
	{
		std::unique_lock<std::mutex> authLock(resetQuarantineOdomAuthMutex_);
		if(!validateResetQuarantinePublisher(
				resetQuarantineOdomTopic_,
				"Odometry",
				resetQuarantineExpectedOdomInfoNode_,
				resetQuarantineExpectedOdomNamespace_,
				resetQuarantineOdomPublisherBound_,
				resetQuarantineOdomPublisherCompromised_,
				resetQuarantineOdomPublisherGid_,
				&incomingGid))
		{
			if(resetQuarantineOdomPublisherCompromised_)
			{
				safeRelocalizationFatal_.store(true, std::memory_order_release);
				rclcpp::shutdown();
			}
			authLock.unlock();
			resetQuarantineOdomAuthCondition_.notify_all();
			return;
		}
		const long long stampNs = rclcpp::Time(msg->header.stamp).nanoseconds();
		if(stampNs <= 0)
		{
			return;
		}
		if(!managedOdomNumericsValid(*msg))
		{
			resetQuarantineOdomPublisherCompromised_ = true;
			RCLCPP_FATAL(this->get_logger(),
					"reset quarantine fail-stop: managed Odometry contained non-finite pose/twist/covariance or a non-unit quaternion");
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
			authLock.unlock();
			resetQuarantineOdomAuthCondition_.notify_all();
			return;
		}
		const std::string fingerprint = managedOdomFingerprint(*msg);
		std::map<long long, std::string>::const_iterator existing =
				resetQuarantineAuthenticatedOdomSamples_.find(stampNs);
		if(existing != resetQuarantineAuthenticatedOdomSamples_.end() &&
				existing->second != fingerprint)
		{
			resetQuarantineOdomPublisherCompromised_ = true;
			RCLCPP_FATAL(this->get_logger(),
					"reset quarantine fail-stop: one managed Odometry stamp carried two payloads");
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
		}
		else
		{
			// Keep fingerprints after the one-shot admission cache is consumed:
			// duplicates cannot build continuity and conflicting payloads fail stop.
			const auto prior = resetQuarantineOdomContinuity_.find(stampNs);
			if(prior != resetQuarantineOdomContinuity_.end() && prior->second.fingerprint != fingerprint)
			{
				resetQuarantineOdomPublisherCompromised_ = true;
				RCLCPP_FATAL(this->get_logger(), "reset quarantine fail-stop: conflicting historical Odometry continuity payload");
				safeRelocalizationFatal_.store(true, std::memory_order_release);
				rclcpp::shutdown();
				authLock.unlock();
				resetQuarantineOdomAuthCondition_.notify_all();
				return;
			}
			if(stampNs > resetQuarantineContinuityStampNs_)
			{
				const Transform pose = rtabmap_conversions::transformFromPoseMsg(msg->pose.pose);
				const bool valid = msg->header.frame_id == "odom" && msg->child_frame_id == frameId_ &&
						!pose.isNull() && !pose.isIdentity() && !managedOdomHasBadCovariance(*msg);
				// A scheduling / input pause does not change the odometry frame.
				// Check the complete displacement between observed samples, without
				// interpolation and without scaling the spatial limits by elapsed time.
				// Exact source/payload admission and trusted lost/recovery remain separate.
				const long long previousStampNs = resetQuarantineContinuityStampNs_;
				const auto previousChain = resetQuarantineContinuityChain_;
				const double gapSeconds = previousStampNs > 0 ? (stampNs-previousStampNs)*1e-9 : 0.0;
				double distance = std::numeric_limits<double>::quiet_NaN();
				double angle = std::numeric_limits<double>::quiet_NaN();
				bool linked = valid && previousChain != 0;
				if(linked)
				{
					const Transform step = resetQuarantineContinuityPose_.inverse()*pose;
					distance = step.getNorm();
					angle = step.getAngle(Transform::getIdentity())*180.0/M_PI;
					linked = std::isfinite(distance) && std::isfinite(angle) &&
							distance <= resetQuarantineMaxUnannouncedTranslation_ &&
							angle <= resetQuarantineMaxUnannouncedRotationDeg_;
				}
				if(!linked)
				{
					++resetQuarantineContinuitySerial_;
					resetQuarantineContinuityChain_ = valid ? resetQuarantineContinuitySerial_ : 0;
					if(previousStampNs > 0)
					{
						RCLCPP_WARN(this->get_logger(),
								"reset quarantine odometry continuity broken (reason=%s previous_stamp_ns=%lld stamp_ns=%lld gap_s=%.9f step_translation=%.6f step_rotation_deg=%.6f previous_chain=%llu chain=%llu)",
								!valid ? "invalid_pose_frames_or_covariance" :
								previousChain == 0 ? "previous_sample_invalid" : "step_exceeds_spatial_gate",
								previousStampNs, stampNs, gapSeconds, distance, angle,
								previousChain, resetQuarantineContinuityChain_);
					}
				}
				else if(gapSeconds > 0.5)
				{
					RCLCPP_INFO(this->get_logger(),
							"reset quarantine odometry pause with bounded observed step (previous_stamp_ns=%lld stamp_ns=%lld gap_s=%.9f step_translation=%.6f step_rotation_deg=%.6f chain=%llu)",
							previousStampNs, stampNs, gapSeconds, distance, angle, previousChain);
				}
				resetQuarantineContinuityStampNs_ = stampNs;
				resetQuarantineContinuityPose_ = pose;
				resetQuarantineOdomContinuity_[stampNs] = {fingerprint, resetQuarantineContinuityChain_};
				while(resetQuarantineOdomContinuity_.size() > 256U)
				{
					resetQuarantineOdomContinuity_.erase(resetQuarantineOdomContinuity_.begin());
				}
			}
			resetQuarantineAuthenticatedOdomSamples_[stampNs] = fingerprint;
			while(resetQuarantineAuthenticatedOdomSamples_.size() > 256U)
			{
				resetQuarantineAuthenticatedOdomSamples_.erase(
						resetQuarantineAuthenticatedOdomSamples_.begin());
			}
		}
	}
	resetQuarantineOdomAuthCondition_.notify_all();
}

void CoreWrapper::resetQuarantineOdomInfoCallback(
		const rtabmap_msgs::msg::OdomInfo::SharedPtr msg,
		const rclcpp::MessageInfo & messageInfo)
{
	if(!msg || !resetQuarantineEnabled_)
	{
		return;
	}
	const rmw_gid_t & incomingGid =
			messageInfo.get_rmw_message_info().publisher_gid;
	std::unique_lock<std::mutex> authLock(resetQuarantineOdomInfoAuthMutex_);
	const std::string topicName = resetQuarantineOdomInfoSub_->get_topic_name();
	if(!validateResetQuarantinePublisher(
			topicName,
			"OdomInfo",
			resetQuarantineExpectedOdomInfoNode_,
			resetQuarantineExpectedOdomNamespace_,
			resetQuarantineOdomInfoPublisherBound_,
			resetQuarantineOdomInfoPublisherCompromised_,
			resetQuarantineOdomInfoPublisherGid_,
			&incomingGid))
	{
		if(resetQuarantineOdomInfoPublisherCompromised_)
		{
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
		}
		authLock.unlock();
		resetQuarantineOdomInfoAuthCondition_.notify_all();
		return;
	}
	const long long stampNs = rclcpp::Time(msg->header.stamp).nanoseconds();
	if(stampNs <= 0)
	{
		RCLCPP_WARN(this->get_logger(),
				"reset quarantine ignored OdomInfo with invalid measurement stamp_ns=%lld",
				stampNs);
		return;
	}
	const std::string fingerprint = managedOdomInfoFingerprint(*msg);
	std::map<long long, std::pair<std::string, long long> >::const_iterator existing =
			resetQuarantineAuthenticatedOdomInfoSamples_.find(stampNs);
	if(existing != resetQuarantineAuthenticatedOdomInfoSamples_.end() &&
			existing->second.first != fingerprint)
	{
		resetQuarantineOdomInfoPublisherCompromised_ = true;
		RCLCPP_FATAL(this->get_logger(),
				"reset quarantine fail-stop: one managed OdomInfo stamp carried two payloads");
		safeRelocalizationFatal_.store(true, std::memory_order_release);
		rclcpp::shutdown();
		authLock.unlock();
		resetQuarantineOdomInfoAuthCondition_.notify_all();
		return;
	}
	// Advance the full-rate lost/recovery state before making this exact sample
	// visible to the synchronized RGB-D callback.  This ordering prevents a
	// callback race from processing a recovery frame against the previous epoch.
	if(!observeResetQuarantineOdomInfo(*msg))
	{
		return;
	}
	const long long arrivalSteadyNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
	resetQuarantineAuthenticatedOdomInfoSamples_[stampNs] =
			std::make_pair(fingerprint, arrivalSteadyNs);
	while(resetQuarantineAuthenticatedOdomInfoSamples_.size() > 256U)
	{
		resetQuarantineAuthenticatedOdomInfoSamples_.erase(
				resetQuarantineAuthenticatedOdomInfoSamples_.begin());
	}
	authLock.unlock();
	resetQuarantineOdomInfoAuthCondition_.notify_all();
}

void CoreWrapper::resetQuarantineRgbCallback(
		const sensor_msgs::msg::Image::SharedPtr msg,
		const rclcpp::MessageInfo & messageInfo)
{
	if(!msg || !resetQuarantineEnabled_)
	{
		return;
	}
	const rmw_gid_t & incomingGid = messageInfo.get_rmw_message_info().publisher_gid;
	std::lock_guard<std::mutex> authLock(resetQuarantineRgbdAuthMutex_);
	if(!validateResetQuarantinePublisher(
			resetQuarantineRgbSub_->get_topic_name(), "RGB image",
			resetQuarantineExpectedSensorNode_, resetQuarantineExpectedSensorNamespace_,
			resetQuarantineRgbPublisherBound_, resetQuarantineRgbPublisherCompromised_,
			resetQuarantineRgbPublisherGid_, &incomingGid))
	{
		if(resetQuarantineRgbPublisherCompromised_)
		{
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
		}
		resetQuarantineRgbdAuthCondition_.notify_all();
		return;
	}
	const long long stampNs = rclcpp::Time(msg->header.stamp).nanoseconds();
	if(stampNs <= 0 || msg->header.frame_id != resetQuarantineExpectedSensorFrame_)
	{
		return;
	}
	const std::string fingerprint = managedRosMessageFingerprint(*msg);
	std::map<long long, std::string>::const_iterator existing =
			resetQuarantineAuthenticatedRgbSamples_.find(stampNs);
	if(existing != resetQuarantineAuthenticatedRgbSamples_.end() &&
			existing->second != fingerprint)
	{
		resetQuarantineRgbPublisherCompromised_ = true;
		RCLCPP_FATAL(this->get_logger(),
				"reset quarantine fail-stop: one managed RGB stamp carried two payloads");
		safeRelocalizationFatal_.store(true, std::memory_order_release);
		rclcpp::shutdown();
	}
	else
	{
		resetQuarantineAuthenticatedRgbSamples_[stampNs] = fingerprint;
		while(resetQuarantineAuthenticatedRgbSamples_.size() > 64U)
		{
			resetQuarantineAuthenticatedRgbSamples_.erase(
					resetQuarantineAuthenticatedRgbSamples_.begin());
		}
	}
	resetQuarantineRgbdAuthCondition_.notify_all();
}

void CoreWrapper::resetQuarantineDepthCallback(
		const sensor_msgs::msg::Image::SharedPtr msg,
		const rclcpp::MessageInfo & messageInfo)
{
	if(!msg || !resetQuarantineEnabled_)
	{
		return;
	}
	const rmw_gid_t & incomingGid = messageInfo.get_rmw_message_info().publisher_gid;
	std::lock_guard<std::mutex> authLock(resetQuarantineRgbdAuthMutex_);
	if(!validateResetQuarantinePublisher(
			resetQuarantineDepthSub_->get_topic_name(), "depth image",
			resetQuarantineExpectedSensorNode_, resetQuarantineExpectedSensorNamespace_,
			resetQuarantineDepthPublisherBound_, resetQuarantineDepthPublisherCompromised_,
			resetQuarantineDepthPublisherGid_, &incomingGid))
	{
		if(resetQuarantineDepthPublisherCompromised_)
		{
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
		}
		resetQuarantineRgbdAuthCondition_.notify_all();
		return;
	}
	const long long stampNs = rclcpp::Time(msg->header.stamp).nanoseconds();
	if(stampNs <= 0 || msg->header.frame_id != resetQuarantineExpectedSensorFrame_)
	{
		return;
	}
	const std::string fingerprint = managedRosMessageFingerprint(*msg);
	std::map<long long, std::string>::const_iterator existing =
			resetQuarantineAuthenticatedDepthSamples_.find(stampNs);
	if(existing != resetQuarantineAuthenticatedDepthSamples_.end() &&
			existing->second != fingerprint)
	{
		resetQuarantineDepthPublisherCompromised_ = true;
		RCLCPP_FATAL(this->get_logger(),
				"reset quarantine fail-stop: one managed depth stamp carried two payloads");
		safeRelocalizationFatal_.store(true, std::memory_order_release);
		rclcpp::shutdown();
	}
	else
	{
		resetQuarantineAuthenticatedDepthSamples_[stampNs] = fingerprint;
		while(resetQuarantineAuthenticatedDepthSamples_.size() > 64U)
		{
			resetQuarantineAuthenticatedDepthSamples_.erase(
					resetQuarantineAuthenticatedDepthSamples_.begin());
		}
	}
	resetQuarantineRgbdAuthCondition_.notify_all();
}

void CoreWrapper::resetQuarantineCameraInfoCallback(
		const sensor_msgs::msg::CameraInfo::SharedPtr msg,
		const rclcpp::MessageInfo & messageInfo)
{
	if(!msg || !resetQuarantineEnabled_)
	{
		return;
	}
	const rmw_gid_t & incomingGid = messageInfo.get_rmw_message_info().publisher_gid;
	std::lock_guard<std::mutex> authLock(resetQuarantineRgbdAuthMutex_);
	if(!validateResetQuarantinePublisher(
			resetQuarantineCameraInfoSub_->get_topic_name(), "CameraInfo",
			resetQuarantineExpectedSensorNode_, resetQuarantineExpectedSensorNamespace_,
			resetQuarantineCameraInfoPublisherBound_,
			resetQuarantineCameraInfoPublisherCompromised_,
			resetQuarantineCameraInfoPublisherGid_, &incomingGid))
	{
		if(resetQuarantineCameraInfoPublisherCompromised_)
		{
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
		}
		resetQuarantineRgbdAuthCondition_.notify_all();
		return;
	}
	const long long stampNs = rclcpp::Time(msg->header.stamp).nanoseconds();
	if(stampNs <= 0 || msg->header.frame_id != resetQuarantineExpectedSensorFrame_)
	{
		return;
	}
	const std::string fingerprint = managedRosMessageFingerprint(*msg);
	std::map<long long, std::string>::const_iterator existing =
			resetQuarantineAuthenticatedCameraInfoSamples_.find(stampNs);
	if(existing != resetQuarantineAuthenticatedCameraInfoSamples_.end() &&
			existing->second != fingerprint)
	{
		resetQuarantineCameraInfoPublisherCompromised_ = true;
		RCLCPP_FATAL(this->get_logger(),
				"reset quarantine fail-stop: one managed CameraInfo stamp carried two payloads");
		safeRelocalizationFatal_.store(true, std::memory_order_release);
		rclcpp::shutdown();
	}
	else
	{
		resetQuarantineAuthenticatedCameraInfoSamples_[stampNs] = fingerprint;
		while(resetQuarantineAuthenticatedCameraInfoSamples_.size() > 64U)
		{
			resetQuarantineAuthenticatedCameraInfoSamples_.erase(
					resetQuarantineAuthenticatedCameraInfoSamples_.begin());
		}
	}
	resetQuarantineRgbdAuthCondition_.notify_all();
}

bool CoreWrapper::authenticateResetQuarantineRgbd(
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs)
{
	if(imageMsgs.size() != 1U || depthMsgs.size() != 1U || cameraInfoMsgs.size() != 1U ||
			!imageMsgs.front() || !depthMsgs.front())
	{
		RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
				"reset quarantine requires exactly one RGB, depth and CameraInfo sample");
		return false;
	}
	const sensor_msgs::msg::Image::SharedPtr rgb = imageMsgs.front()->toImageMsg();
	const sensor_msgs::msg::Image::SharedPtr depth = depthMsgs.front()->toImageMsg();
	const long long stampNs = rclcpp::Time(cameraInfoMsgs.front().header.stamp).nanoseconds();
	if(!rgb || !depth || stampNs <= 0 ||
			rclcpp::Time(rgb->header.stamp).nanoseconds() != stampNs ||
			rclcpp::Time(depth->header.stamp).nanoseconds() != stampNs)
	{
		RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
				"reset quarantine rejected non-exact RGB-D-CameraInfo stamps");
		return false;
	}
	const std::string rgbFingerprint = managedRosMessageFingerprint(*rgb);
	const std::string depthFingerprint = managedRosMessageFingerprint(*depth);
	const std::string infoFingerprint = managedRosMessageFingerprint(cameraInfoMsgs.front());
	std::unique_lock<std::mutex> authLock(resetQuarantineRgbdAuthMutex_);
	resetQuarantineRgbdAuthCondition_.wait_for(
			authLock, std::chrono::milliseconds(100), [&](){
				return resetQuarantineRgbPublisherCompromised_ ||
						resetQuarantineDepthPublisherCompromised_ ||
						resetQuarantineCameraInfoPublisherCompromised_ ||
						(resetQuarantineAuthenticatedRgbSamples_.count(stampNs) != 0U &&
						 resetQuarantineAuthenticatedDepthSamples_.count(stampNs) != 0U &&
						 resetQuarantineAuthenticatedCameraInfoSamples_.count(stampNs) != 0U);
			});
	if(resetQuarantineRgbPublisherCompromised_ ||
			resetQuarantineDepthPublisherCompromised_ ||
			resetQuarantineCameraInfoPublisherCompromised_)
	{
		return false;
	}
	const std::map<long long, std::string>::const_iterator rgbFound =
			resetQuarantineAuthenticatedRgbSamples_.find(stampNs);
	const std::map<long long, std::string>::const_iterator depthFound =
			resetQuarantineAuthenticatedDepthSamples_.find(stampNs);
	const std::map<long long, std::string>::const_iterator infoFound =
			resetQuarantineAuthenticatedCameraInfoSamples_.find(stampNs);
	if(rgbFound == resetQuarantineAuthenticatedRgbSamples_.end() ||
			depthFound == resetQuarantineAuthenticatedDepthSamples_.end() ||
			infoFound == resetQuarantineAuthenticatedCameraInfoSamples_.end() ||
			rgbFound->second != rgbFingerprint ||
			depthFound->second != depthFingerprint ||
			infoFound->second != infoFingerprint)
	{
		RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
				"reset quarantine rejected RGB-D input without three exact authenticated payloads (stamp_ns=%lld present_rgb=%d present_depth=%d present_info=%d match_rgb=%d match_depth=%d match_info=%d)",
				stampNs,
				rgbFound != resetQuarantineAuthenticatedRgbSamples_.end(),
				depthFound != resetQuarantineAuthenticatedDepthSamples_.end(),
				infoFound != resetQuarantineAuthenticatedCameraInfoSamples_.end(),
				rgbFound != resetQuarantineAuthenticatedRgbSamples_.end() && rgbFound->second == rgbFingerprint,
				depthFound != resetQuarantineAuthenticatedDepthSamples_.end() && depthFound->second == depthFingerprint,
				infoFound != resetQuarantineAuthenticatedCameraInfoSamples_.end() && infoFound->second == infoFingerprint);
		return false;
	}
	resetQuarantineAuthenticatedRgbSamples_.erase(stampNs);
	resetQuarantineAuthenticatedDepthSamples_.erase(stampNs);
	resetQuarantineAuthenticatedCameraInfoSamples_.erase(stampNs);
	return true;
}

bool CoreWrapper::observeResetQuarantineOdomInfo(
		const rtabmap_msgs::msg::OdomInfo & msg)
{
	if(!resetQuarantineEnabled_)
	{
		return false;
	}
	UScopeMutex stateLock(resetQuarantineStateMutex_);
	const long long nowSteadyNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
	const long long stampNs = rclcpp::Time(msg.header.stamp).nanoseconds();
	if(stampNs <= 0)
	{
		RCLCPP_WARN(this->get_logger(),
				"reset quarantine ignored OdomInfo with invalid measurement stamp_ns=%lld",
				stampNs);
		return false;
	}
	long long previousStampNs = resetQuarantineLastOdomInfoStampNs_.load(
			std::memory_order_acquire);
	while(stampNs > previousStampNs &&
			!resetQuarantineLastOdomInfoStampNs_.compare_exchange_weak(
					previousStampNs,
					stampNs,
					std::memory_order_acq_rel,
					std::memory_order_acquire))
	{
	}
	if(stampNs <= previousStampNs)
	{
		RCLCPP_WARN(this->get_logger(),
				"reset quarantine ignored duplicate/out-of-order OdomInfo stamp_ns=%lld last_unique_stamp_ns=%lld lost=%s",
				stampNs,
				previousStampNs,
				msg.lost?"true":"false");
		return false;
	}
	// Only an accepted new measurement advances arrival diagnostics. Rejected
	// duplicates/out-of-order packets must never refresh freshness.
	resetQuarantineLastOdomInfoSteadyNs_.store(nowSteadyNs, std::memory_order_release);
	resetQuarantineOdomInfoSeen_.store(true, std::memory_order_release);
	if(msg.lost)
	{
		int previous = resetQuarantineLostStreak_.load(std::memory_order_relaxed);
		if(previous == 0)
		{
			resetQuarantineLostStartStampNs_.store(stampNs, std::memory_order_release);
		}
		while(previous < 100000 &&
				!resetQuarantineLostStreak_.compare_exchange_weak(
						previous,
						previous + 1,
						std::memory_order_release,
						std::memory_order_relaxed))
		{
		}
		return true;
	}

	const int lostCount = resetQuarantineLostStreak_.exchange(
			0, std::memory_order_acq_rel);
	if(lostCount >= resetQuarantineLostCount_)
	{
		resetQuarantineRecoveryStampNs_.store(stampNs, std::memory_order_release);
		resetQuarantineRecoveryPending_.store(true, std::memory_order_release);
		RCLCPP_WARN(
				this->get_logger(),
				"reset quarantine armed after %d unique-stamp lost odometry samples (lost_start_stamp_ns=%lld recovery_stamp_ns=%lld); first valid RGB-D stamp at-or-after recovery starts an isolated map epoch",
				lostCount,
				resetQuarantineLostStartStampNs_.load(std::memory_order_acquire),
				stampNs);
	}
	else
	{
		resetQuarantineLostStartStampNs_.store(0, std::memory_order_release);
	}
	return true;
}

bool CoreWrapper::authenticateResetQuarantineOdomInfo(
		const rtabmap_msgs::msg::OdomInfo & msg)
{
	const long long stampNs = rclcpp::Time(msg.header.stamp).nanoseconds();
	const std::string fingerprint = managedOdomInfoFingerprint(msg);
	std::unique_lock<std::mutex> authLock(resetQuarantineOdomInfoAuthMutex_);
	if(stampNs > 0 && !resetQuarantineOdomInfoPublisherCompromised_)
	{
		resetQuarantineOdomInfoAuthCondition_.wait_for(
				authLock,
				std::chrono::milliseconds(100),
				[this, stampNs]()
				{
					return resetQuarantineOdomInfoPublisherCompromised_ ||
							resetQuarantineAuthenticatedOdomInfoSamples_.find(stampNs) !=
									resetQuarantineAuthenticatedOdomInfoSamples_.end();
				});
	}
	std::map<long long, std::pair<std::string, long long> >::iterator authenticated =
			resetQuarantineAuthenticatedOdomInfoSamples_.find(stampNs);
	if(resetQuarantineOdomInfoPublisherCompromised_ || stampNs <= 0 ||
			authenticated == resetQuarantineAuthenticatedOdomInfoSamples_.end() ||
			authenticated->second.first != fingerprint)
	{
		RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
				"reset quarantine rejected synchronized OdomInfo without one exact source-bound full-rate sample (stamp_ns=%lld compromised=%s)",
				stampNs,
				resetQuarantineOdomInfoPublisherCompromised_?"true":"false");
		return false;
	}
	const long long nowSteadyNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
	const long long exactArrivalSteadyNs = authenticated->second.second;
	const double exactAgeSec = exactArrivalSteadyNs > 0?
			double(nowSteadyNs - exactArrivalSteadyNs) / 1000000000.0 :
			std::numeric_limits<double>::infinity();
	if(exactAgeSec < 0.0 || exactAgeSec > resetQuarantineOdomInfoMaxAgeSec_)
	{
		resetQuarantineAuthenticatedOdomInfoSamples_.erase(authenticated);
		RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
				"reset quarantine rejected an exact OdomInfo sample using its own stale/future arrival time (stamp_ns=%lld age=%.3fs limit=%.3fs)",
				stampNs, exactAgeSec, resetQuarantineOdomInfoMaxAgeSec_);
		return false;
	}
	resetQuarantineAuthenticatedOdomInfoSamples_.erase(authenticated);
	authLock.unlock();
	{
		UScopeMutex stateLock(resetQuarantineStateMutex_);
		resetQuarantineAuthenticatedOdomInfoStamps_[stampNs] = exactArrivalSteadyNs;
		while(resetQuarantineAuthenticatedOdomInfoStamps_.size() > 128U)
		{
			resetQuarantineAuthenticatedOdomInfoStamps_.erase(
					resetQuarantineAuthenticatedOdomInfoStamps_.begin());
		}
	}
	return true;
}


void CoreWrapper::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(),
				"Ignoring /initialpose: safe relocalization owns graph alignment through guarded transactions");
		return;
	}
	Transform mapToPose = Transform::getIdentity();
	if(msg->header.frame_id.empty())
	{
		RCLCPP_WARN(this->get_logger(), "Received initialpose doesn't have frame_id set, assuming it is in %s frame.", mapFrameId_.c_str());
	}
	else if(msg->header.frame_id != mapFrameId_)
	{
		mapToPose = rtabmap_conversions::getTransform(mapFrameId_, msg->header.frame_id, msg->header.stamp, *tfBuffer_, waitForTransform_);
		if(mapToPose.isNull())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to transform initialpose from frame %s to map frame %s", msg->header.frame_id.c_str(), mapFrameId_.c_str());
			return;
		}
	}

	Transform initialPose = rtabmap_conversions::transformFromPoseMsg(msg->pose.pose);
	if(initialPose.isNull())
	{
		RCLCPP_ERROR(this->get_logger(), "initialpose received is null!");
		return;
	}
	if(mapToPose.isIdentity())
	{
		RCLCPP_INFO(this->get_logger(), "initialpose received: %s", initialPose.prettyPrint().c_str());
		rtabmap_.setInitialPose(initialPose);
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "initialpose received: %s in %s frame, transformed to %s in %s frame.",
			initialPose.prettyPrint().c_str(),
			msg->header.frame_id.c_str(),
			(mapToPose * initialPose).prettyPrint().c_str(),
			mapFrameId_.c_str());
		rtabmap_.setInitialPose(mapToPose*initialPose);
	}
	++safeRelocalizationGraphRevision_;
	publishResetEpoch();
}

void CoreWrapper::goalCommonCallback(
		int id,
		const std::string & label,
		const std::string & frameId,
		const Transform & pose,
		const rclcpp::Time & stamp,
		double * planningTime)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring planning goal: guarded relocalization mode forbids virtual-link mutation");
		if(planningTime)
		{
			*planningTime = 0.0;
		}
		return;
	}
	UTimer timer;

	if(id == 0 && !label.empty() && rtabmap_.getMemory())
	{
		id = rtabmap_.getMemory()->getSignatureIdByLabel(label);
	}

	if(id > 0)
	{
		RCLCPP_INFO(this->get_logger(), "Planning: set goal to node %d", id);
	}
	else if(id < 0)
	{
		RCLCPP_INFO(this->get_logger(), "Planning: set goal to landmark %d", id);
	}
	else if(!pose.isNull())
	{
		RCLCPP_INFO(this->get_logger(), "Planning: set goal %s", pose.prettyPrint().c_str());
	}

	if(planningTime)
	{
		*planningTime = 0.0;
	}

	bool success = false;
	if((id != 0 && rtabmap_.computePath(id, true)) ||
	   (!pose.isNull() && rtabmap_.computePath(pose)))
	{
		if(planningTime)
		{
			*planningTime = timer.elapsed();
		}
		RCLCPP_INFO(this->get_logger(), "Planning: Time computing path = %f s", timer.ticks());
		const std::vector<std::pair<int, Transform> > & poses = rtabmap_.getPath();

		currentMetricGoal_.setNull();
		lastPublishedMetricGoal_.setNull();
		goalFrameId_.clear();
		latestNodeWasReached_ = false;
		if(poses.size() == 0)
		{
			RCLCPP_WARN(this->get_logger(), "Planning: Goal already reached (RGBD/GoalReachedRadius=%fm).",
					rtabmap_.getGoalReachedRadius());
			rtabmap_.clearPath(1);
			if(goalReachedPub_->get_subscription_count())
			{
				std_msgs::msg::Bool result;
				result.data = true;
				goalReachedPub_->publish(result);
			}
			success = true;
		}
		else
		{
			currentMetricGoal_ = rtabmap_.getPose(rtabmap_.getPathCurrentGoalId());
			if(!currentMetricGoal_.isNull())
			{
				RCLCPP_INFO(this->get_logger(), "Planning: Path successfully created (size=%d)", (int)poses.size());
				goalFrameId_ = frameId;

				// Adjust the target pose relative to last node
				if(rtabmap_.getPathCurrentGoalId() == rtabmap_.getPath().back().first && rtabmap_.getLocalOptimizedPoses().size())
				{
					if(rtabmap_.getLastLocalizationPose().getDistance(currentMetricGoal_) < rtabmap_.getLocalRadius())
					{
						latestNodeWasReached_ = true;
						Transform goalLocalTransform = Transform::getIdentity();
						if(!goalFrameId_.empty() && goalFrameId_.compare(frameId_) != 0)
						{
							Transform localT = rtabmap_conversions::getTransform(frameId_, goalFrameId_, stamp, *tfBuffer_, waitForTransform_);
							if(!localT.isNull())
							{
								goalLocalTransform = localT.inverse().to3DoF();
							}
						}
						currentMetricGoal_ *= rtabmap_.getPathTransformToGoal() * goalLocalTransform;
					}
				}

				publishCurrentGoal(stamp);
				publishLocalPath(stamp);
				publishGlobalPath(stamp);

				// Just output the path on screen
				std::stringstream stream;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					if(iter != poses.begin())
					{
						stream << " ";
					}
					stream << iter->first;
				}
				RCLCPP_INFO(this->get_logger(), "Global path: [%s]", stream.str().c_str());
				success=true;
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Pose of node %d not found!? Cannot send a metric goal...", rtabmap_.getPathCurrentGoalId());
			}
		}
	}
	else if(!label.empty())
	{
		RCLCPP_ERROR(this->get_logger(), "Planning: Node with label \"%s\" not found!", label.c_str());
	}
	else if(pose.isNull())
	{
		if(id > 0)
		{
			RCLCPP_ERROR(this->get_logger(), "Planning: Could not plan to node %d! The node is not in map's graph (look for warnings before this message for more details).", id);
		}
		else if(id < 0)
		{
			RCLCPP_ERROR(this->get_logger(), "Planning: Could not plan to landmark %d! The landmark is not in map's graph (look for warnings before this message for more details).", id);
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Planning: Node id should be > 0 !");
		}
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "Planning: A node near the goal's pose not found! The pose may be to far from the graph (RGBD/LocalRadius=%f m)", rtabmap_.getLocalRadius());
	}

	if(!success)
	{
		rtabmap_.clearPath(-1);
		if(goalReachedPub_->get_subscription_count())
		{
			std_msgs::msg::Bool result;
			result.data = false;
			goalReachedPub_->publish(result);
		}
	}
}

void CoreWrapper::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring goal topic: guarded relocalization mode forbids path mutation");
		return;
	}
	Transform targetPose = rtabmap_conversions::transformFromPoseMsg(msg->pose, true);

	// transform goal in /map frame
	if(!msg->header.frame_id.empty() && mapFrameId_.compare(msg->header.frame_id) != 0)
	{
		Transform t = rtabmap_conversions::getTransform(mapFrameId_, msg->header.frame_id, msg->header.stamp, *tfBuffer_, waitForTransform_);
		if(t.isNull())
		{
			RCLCPP_ERROR(this->get_logger(), "Cannot transform goal pose from \"%s\" frame to \"%s\" frame!",
					msg->header.frame_id.c_str(), mapFrameId_.c_str());
			if(goalReachedPub_->get_subscription_count())
			{
				std_msgs::msg::Bool result;
				result.data = false;
				goalReachedPub_->publish(result);
			}
			return;
		}
		targetPose = t * targetPose;
	}
	// else assume map frame if not set

	goalCommonCallback(0, "", "", targetPose, msg->header.stamp);
}

void CoreWrapper::goalNodeCallback(const rtabmap_msgs::msg::Goal::SharedPtr msg)
{
	if(msg->node_id == 0 && msg->node_label.empty())
	{
		RCLCPP_ERROR(this->get_logger(), "Node id or label should be set!");
		if(goalReachedPub_->get_subscription_count())
		{
			std_msgs::msg::Bool result;
			result.data = false;
			goalReachedPub_->publish(result);
		}
		return;
	}
	goalCommonCallback(msg->node_id, msg->node_label, msg->frame_id, Transform(), msg->header.stamp);
}

void CoreWrapper::updateRtabmapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(),
				"Ignoring external /update_parameters while safe relocalization owns immutable graph parameters");
		return;
	}
	for(rtabmap::ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		std::string paramValue;
		rclcpp::Parameter parameter;
		if(get_parameter(iter->first, parameter))
		{
			paramValue = parameter.as_string();
			if(paramValue.compare(iter->second)!=0)
			{
				RCLCPP_INFO(get_logger(), "Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), paramValue.c_str());
				iter->second = paramValue;
			}
		}
	}
	applyParameters();
}

void CoreWrapper::applyParameters()
{
	RCLCPP_INFO(get_logger(), "rtabmap: Updating parameters");
	if(safeRelocalizationEnabled_)
	{
		bool incrementalMemory = false;
		bool initAllNodes = false;
		bool landmarksIgnored = false;
		bool priorsIgnored = false;
		bool loopGPS = true;
		bool force3DoF = false;
		float optimizeRepairRadius = 0.0f;
		float rehearsalSimilarity = 0.0f;
		float timeThreshold = -1.0f;
		int optimizerIterations = 0;
		int memoryThreshold = -1;
		bool reduceGraph = true;
		bool rehearsalUpdatesId = true;
		bool createIntermediateNodes = true;
		Parameters::parse(parameters_, Parameters::kMemIncrementalMemory(), incrementalMemory);
		Parameters::parse(parameters_, Parameters::kMemInitWMWithAllNodes(), initAllNodes);
		Parameters::parse(parameters_, Parameters::kOptimizerLandmarksIgnored(), landmarksIgnored);
		Parameters::parse(parameters_, Parameters::kOptimizerPriorsIgnored(), priorsIgnored);
		Parameters::parse(parameters_, Parameters::kRtabmapLoopGPS(), loopGPS);
		Parameters::parse(parameters_, Parameters::kRegForce3DoF(), force3DoF);
		Parameters::parse(parameters_, Parameters::kRGBDOptimizeMaxErrorRepairRadius(), optimizeRepairRadius);
		Parameters::parse(parameters_, Parameters::kMemReduceGraph(), reduceGraph);
		Parameters::parse(parameters_, Parameters::kMemRehearsalIdUpdatedToNewOne(), rehearsalUpdatesId);
		Parameters::parse(parameters_, Parameters::kMemRehearsalSimilarity(), rehearsalSimilarity);
		Parameters::parse(parameters_, Parameters::kRtabmapTimeThr(), timeThreshold);
		Parameters::parse(parameters_, Parameters::kOptimizerIterations(), optimizerIterations);
		Parameters::parse(parameters_, Parameters::kRtabmapMemoryThr(), memoryThreshold);
		Parameters::parse(parameters_, Parameters::kRtabmapCreateIntermediateNodes(), createIntermediateNodes);
		if(!incrementalMemory || !initAllNodes || !landmarksIgnored || !priorsIgnored || loopGPS || force3DoF ||
				reduceGraph || rehearsalUpdatesId || rehearsalSimilarity != 1.0f ||
				timeThreshold != 0.0f || memoryThreshold != 0 ||
				createIntermediateNodes || odomSensorSync_ ||
				optimizeRepairRadius != 0.0f || optimizerIterations <= 0)
		{
			RCLCPP_ERROR(get_logger(),
					"Rejecting RTAB-Map parameter update: safe relocalization requires immutable full-memory mapping with rehearsal/reduction and time/memory eviction disabled, ignored landmarks and priors, GPS loops off, 6DoF, repair radius zero and positive optimizer iterations");
			++safeRelocalizationGraphRevision_;
			publishResetEpoch();
			return;
		}
	}
	if(parameters_.find(Parameters::kRtabmapDetectionRate()) != parameters_.end())
	{
		rate_ = uStr2Float(parameters_.at(Parameters::kRtabmapDetectionRate()));
		RCLCPP_INFO(get_logger(), "RTAB-Map rate detection = %f Hz", rate_);
	}
	if(parameters_.find(Parameters::kRtabmapCreateIntermediateNodes()) != parameters_.end())
	{
		createIntermediateNodes_ = uStr2Bool(parameters_.at(Parameters::kRtabmapCreateIntermediateNodes()));
		RCLCPP_INFO(get_logger(), "Create intermediate nodes = %s", createIntermediateNodes_?"true":"false");
	}
	if(parameters_.find(Parameters::kGridGlobalMaxNodes()) != parameters_.end())
	{
		mappingMaxNodes_ = uStr2Int(parameters_.at(Parameters::kGridGlobalMaxNodes()));
		RCLCPP_INFO(get_logger(), "Max mapping nodes = %d", mappingMaxNodes_);
	}
	if(parameters_.find(Parameters::kGridGlobalAltitudeDelta()) != parameters_.end())
	{
		mappingAltitudeDelta_ = uStr2Float(parameters_.at(Parameters::kGridGlobalAltitudeDelta()));
		RCLCPP_INFO(get_logger(), "Mapping altitude delta = %f", mappingAltitudeDelta_);
	}
	if(parameters_.find(Parameters::kRtabmapImagesAlreadyRectified()) != parameters_.end())
	{
		alreadyRectifiedImages_ = uStr2Bool(parameters_.at(Parameters::kRtabmapImagesAlreadyRectified()));
		RCLCPP_INFO(get_logger(), "Already rectified images = %s", alreadyRectifiedImages_?"true":"false");
	}
	if(parameters_.find(Parameters::kRegForce3DoF()) != parameters_.end())
	{
		twoDMapping_= uStr2Bool(parameters_.at(Parameters::kRegForce3DoF()));
		RCLCPP_INFO(get_logger(), "2D mapping = %s", twoDMapping_?"true":"false");
	}
	rtabmap_.parseParameters(parameters_);
	// Don't reset map in localization mode
	if(rtabmap_.getMemory()->isIncremental()) {
		mapsManager_.setParameters(parameters_);
	}
	// Optimizer/registration settings are part of the transaction snapshot.
	++safeRelocalizationGraphRevision_;
	publishResetEpoch();
}

void CoreWrapper::resetRtabmapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring external /reset while safe relocalization owns graph lifecycle");
		return;
	}
	RCLCPP_INFO(this->get_logger(), "rtabmap: Reset");
	rtabmap_.resetMemory();

	lastPoseMutex_.lock();
	lastPoseCovariance_ = cv::Mat();
	lastPose_.setIdentity();
	lastPoseStamp_ = rclcpp::Time();
	lastPoseVelocity_.clear();
	lastPoseIntermediate_ = false;
	lastPoseMutex_.unlock();

	currentMetricGoal_.setNull();
	lastPublishedMetricGoal_.setNull();
	goalFrameId_.clear();
	latestNodeWasReached_ = false;
	graphLatched_ = false;
	mapsManager_.clear();
	previousStamp_ = rclcpp::Time(0);
	globalPoses_.clear();
	gps_.clear();
	landmarksMutex_.lock();
	landmarks_.clear();
	landmarksMutex_.unlock();
	userDataMutex_.lock();
	userData_ = cv::Mat();
	userDataMutex_.unlock();
	envSensorMutex_.lock();
	envSensors_.clear();
	envSensorMutex_.unlock();
	imuMutex_.lock();
	imus_.clear();
	resetQuarantineImuStampNsByTime_.clear();
	imuFrameId_.clear();
	imuMutex_.unlock();
	interOdoms_.clear();
	mapToOdomMutex_.lock();
	mapToOdom_.setIdentity();
	mapToOdomMutex_.unlock();
	// resetMemory() creates a new-map lifecycle. Only the all-zero sentinel is
	// still a valid launch-time database binding after this destructive reset.
	safeRelocalizationDatabaseBindingValid_ =
			safeRelocalizationDatabaseSha256_ == std::string(64, '0');
	clearSafeRelocalizationProtocolState(true, true);
}

void CoreWrapper::pauseRtabmapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(paused_)
	{
		RCLCPP_WARN(this->get_logger(), "rtabmap: Already paused!");
	}
	else
	{
		paused_ = true;
		RCLCPP_INFO(this->get_logger(), "rtabmap: paused!");
		set_parameter(rclcpp::Parameter("is_rtabmap_paused", true));
	}
}

void CoreWrapper::resumeRtabmapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(!paused_)
	{
		RCLCPP_WARN(this->get_logger(), "rtabmap: Already running!");
	}
	else
	{
		paused_ = false;
		RCLCPP_INFO(this->get_logger(), "rtabmap: resumed!");
		set_parameter(rclcpp::Parameter("is_rtabmap_paused", false));
	}
}

void CoreWrapper::loadDatabaseCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::LoadDatabase::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::LoadDatabase::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(get_logger(), "Ignoring /load_database while safe relocalization is bound to one database digest");
		return;
	}
	RCLCPP_INFO(get_logger(), "LoadDatabase: Loading database (%s, clear=%s)...", req->database_path.c_str(), req->clear?"true":"false");
	std::string newDatabasePath = uReplaceChar(req->database_path, '~', UDirectory::homeDir());
	std::string dir = UDirectory::getDir(newDatabasePath);
	if(!UDirectory::exists(dir))
	{
		RCLCPP_ERROR(get_logger(), "Directory %s doesn't exist! Cannot load database \"%s\"", newDatabasePath.c_str(), dir.c_str());
		return;
	}

	if(UFile::exists(newDatabasePath) && req->clear)
	{
		UFile::erase(newDatabasePath);
	}

	// Close old database
	RCLCPP_INFO(get_logger(), "LoadDatabase: Saving current map (%s)...", databasePath_.c_str());
	bool saveDatabase = true;
	if(rtabmap_.getMemory())
	{
		if(!rtabmap_.getMemory()->isReadOnly())
		{
			// save the grid map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
			if(!pixels.empty())
			{
				printf("rtabmap: 2D occupancy grid map saved.\n");
				rtabmap_.getMemory()->save2DMap(pixels, xMin, yMin, gridCellSize);
			}
		}
		else
		{
			printf("rtabmap: Database is read-only, the current state of the memory is not saved.\n");
			saveDatabase = false;
		}
	}
	rtabmap_.close(saveDatabase);
	RCLCPP_INFO(get_logger(), "LoadDatabase: Saving current map (%s, %ld MB)... done!", databasePath_.c_str(), UFile::length(databasePath_)/(1024*1024));

	lastPoseMutex_.lock();
	lastPoseCovariance_ = cv::Mat();
	lastPose_.setIdentity();
	lastPoseStamp_ = rclcpp::Time();
	lastPoseVelocity_.clear();
	lastPoseIntermediate_ = false;
	lastPoseMutex_.unlock();
	
	currentMetricGoal_.setNull();
	lastPublishedMetricGoal_.setNull();
	goalFrameId_.clear();
	latestNodeWasReached_ = false;
	graphLatched_ = false;
	mapsManager_.clear();
	previousStamp_ = rclcpp::Time(0);
	globalPoses_.clear();
	gps_.clear();
	landmarksMutex_.lock();
	landmarks_.clear();
	landmarksMutex_.unlock();
	userDataMutex_.lock();
	userData_ = cv::Mat();
	userDataMutex_.unlock();
	envSensorMutex_.lock();
	envSensors_.clear();
	envSensorMutex_.unlock();
	imuMutex_.lock();
	imus_.clear();
	resetQuarantineImuStampNsByTime_.clear();
	imuFrameId_.clear();
	imuMutex_.unlock();
	interOdoms_.clear();
	mapToOdomMutex_.lock();
	mapToOdom_.setIdentity();
	mapToOdomMutex_.unlock();

	// Open new database
	databasePath_ = newDatabasePath;

	//Warn if database's parameters are different than the current ones we are using
	if(!req->clear && UFile::exists(databasePath_))
	{
		ParametersMap dbParameters;
		rtabmap::DBDriver * driver = rtabmap::DBDriver::create();
		std::string databaseVersion = "0.0.0";
		if(driver->openConnection(databasePath_))
		{
			dbParameters = driver->getLastParameters(); // parameter migration is already done
			databaseVersion = driver->getDatabaseVersion();
		}
		delete driver;
		for(ParametersMap::iterator iter=dbParameters.begin(); iter!=dbParameters.end(); ++iter)
		{
			if(iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
			{
				// ignore working directory
				continue;
			}
			if(iter->first.find("Odom") == 0)
			{
				// ignore odometry params
				continue;
			}
			if(parameters_.find(iter->first) == parameters_.end())
			{
				RCLCPP_WARN(get_logger(), "RTAB-Map parameter \"%s\" from database (%s, version \"%s\") doesn't exist "
						"in current rtabmap version (\"%s\"). The parameter is ignored.",
						iter->first.c_str(),
						iter->second.c_str(),
						databaseVersion.c_str(),
						RTABMAP_VERSION);
			}
			else if(parameters_.find(iter->first)->second.compare(iter->second) !=0)
			{
				RCLCPP_WARN(get_logger(), "RTAB-Map parameter \"%s\" from database (%s, version=\"%s\") is different "
						"from the current used one (%s, version=\"%s\"). We still keep the "
						"current parameter value (%s). If you want to switch between databases "
						"with different configurations, restart rtabmap node instead of using this service.",
						iter->first.c_str(),
						iter->second.c_str(),
						databaseVersion.c_str(),
						parameters_.find(iter->first)->second.c_str(),
						RTABMAP_VERSION,
						parameters_.find(iter->first)->second.c_str());
			}
		}
	}

	RCLCPP_INFO(get_logger(), "LoadDatabase: Loading database...");
	rtabmap_.init(parameters_, databasePath_);
	rtabmap_.setSafeRelocalizationGuardEnabled(safeRelocalizationEnabled_);
	// The core cannot calculate a cryptographic SHA of a live SQLite file.
	// Loading another DB invalidates the launch-time external binding until a
	// restart supplies the new database SHA explicitly.
	safeRelocalizationDatabaseBindingValid_ = false;
	clearSafeRelocalizationProtocolState(false, true);
	RCLCPP_INFO(get_logger(), "LoadDatabase: Loading database... done!");

	if(rtabmap_.getMemory())
	{
		if(useSavedMap_ && !rtabmap_.getMemory()->isIncremental())
		{
			float xMin, yMin, gridCellSize;
			cv::Mat map = rtabmap_.getMemory()->load2DMap(xMin, yMin, gridCellSize);
			if(!map.empty())
			{
				RCLCPP_INFO(get_logger(), "LoadDatabase: 2D occupancy grid map loaded (%dx%d).", map.cols, map.rows);
				mapsManager_.set2DMap(map, xMin, yMin, gridCellSize, rtabmap_.getLocalOptimizedPoses(), rtabmap_.getMemory());
			}
		}

		if(rtabmap_.getMemory()->getWorkingMem().size()>1)
		{
			RCLCPP_INFO(get_logger(), "LoadDatabase: Working Memory = %d, Local map = %d.",
					(int)rtabmap_.getMemory()->getWorkingMem().size()-1,
					(int)rtabmap_.getLocalOptimizedPoses().size());
		}

		if(databasePath_.size())
		{
			RCLCPP_INFO(get_logger(), "LoadDatabase: Database version = \"%s\".", rtabmap_.getMemory()->getDatabaseVersion().c_str());
		}

		if(rtabmap_.getMemory()->isIncremental())
		{
			RCLCPP_INFO(get_logger(), "LoadDatabase: SLAM mode (%s=true)", Parameters::kMemIncrementalMemory().c_str());
		}
		else
		{
			RCLCPP_INFO(get_logger(), "LoadDatabase: Localization mode (%s=false)", Parameters::kMemIncrementalMemory().c_str());
		}
	}
}

void CoreWrapper::triggerNewMapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(),
				"Ignoring external /trigger_new_map: safe relocalization owns reset epochs");
		return;
	}
	RCLCPP_INFO(this->get_logger(), "rtabmap: Trigger new map");
	rtabmap_.triggerNewMapForContinuation();
	clearSafeRelocalizationTokens();
	resetQuarantineEpoch_.store(rtabmap_.getResetEpoch(), std::memory_order_release);
	++safeRelocalizationGraphRevision_;
	publishResetEpoch();
}

void CoreWrapper::backupDatabaseCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring /backup while the managed launcher owns database save and export");
		return;
	}
	RCLCPP_INFO(this->get_logger(), "Backup: Saving memory...");
	bool saveDatabase = true;
	if(rtabmap_.getMemory())
	{
		if(!rtabmap_.getMemory()->isReadOnly())
		{
			// save the grid map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
			if(!pixels.empty())
			{
				printf("rtabmap: 2D occupancy grid map saved.\n");
				rtabmap_.getMemory()->save2DMap(pixels, xMin, yMin, gridCellSize);
			}
		}
		else
		{
			printf("rtabmap: Database is read-only, the current state of the memory is not saved.\n");
			saveDatabase = false;
		}
	}
	rtabmap_.close(saveDatabase);
	RCLCPP_INFO(this->get_logger(), "Backup: Saving memory... done!");

	lastPoseMutex_.lock();
	lastPoseCovariance_ = cv::Mat();
	lastPose_.setIdentity();
	lastPoseStamp_ = rclcpp::Time();
	lastPoseVelocity_.clear();
	lastPoseIntermediate_ = false;
	lastPoseMutex_.unlock();

	currentMetricGoal_.setNull();
	lastPublishedMetricGoal_.setNull();
	goalFrameId_.clear();
	latestNodeWasReached_ = false;
	graphLatched_ = false;
	userDataMutex_.lock();
	userData_ = cv::Mat();
	userDataMutex_.unlock();
	globalPoses_.clear();
	gps_.clear();
	envSensorMutex_.lock();
	envSensors_.clear();
	envSensorMutex_.unlock();
	landmarksMutex_.lock();
	landmarks_.clear();
	landmarksMutex_.unlock();

	RCLCPP_INFO(this->get_logger(), "Backup: Saving \"%s\" to \"%s\"...", databasePath_.c_str(), (databasePath_+".back").c_str());
	UFile::copy(databasePath_, databasePath_+".back");
	RCLCPP_INFO(this->get_logger(), "Backup: Saving \"%s\" to \"%s\"... done!", databasePath_.c_str(), (databasePath_+".back").c_str());

	RCLCPP_INFO(this->get_logger(), "Backup: Reloading memory...");
	rtabmap_.init(parameters_, databasePath_);
	rtabmap_.setSafeRelocalizationGuardEnabled(safeRelocalizationEnabled_);
	// Saving/reopening may change both the SQLite bytes and saved optimized
	// graph. The launch-time external SHA binding cannot remain authoritative.
	safeRelocalizationDatabaseBindingValid_ = false;
	clearSafeRelocalizationProtocolState(true, true);
	RCLCPP_INFO(this->get_logger(), "Backup: Reloading memory... done!");
}

void CoreWrapper::republishMaps()
{
	rclcpp::Time stamp = now();
	mapsManager_.publishMaps(rtabmap_.getLocalOptimizedPoses(), stamp, mapFrameId_);

	if(mapDataPub_->get_subscription_count()>0)
	{
		rtabmap_msgs::msg::MapData::UniquePtr msg(new rtabmap_msgs::msg::MapData);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		rtabmap_conversions::mapDataToROS(
			rtabmap_.getLocalOptimizedPoses(),
			rtabmap_.getLocalConstraints(),
			std::map<int, Signature>(),
			rtabmap_.getMapCorrection(),
			*msg);

		mapDataPub_->publish(std::move(msg));
	}

	if(mapGraphPub_->get_subscription_count()>0)
	{
		rtabmap_msgs::msg::MapGraph::UniquePtr msg(new rtabmap_msgs::msg::MapGraph);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		rtabmap_conversions::mapGraphToROS(
			rtabmap_.getLocalOptimizedPoses(),
			rtabmap_.getLocalConstraints(),
			rtabmap_.getMapCorrection(),
			*msg);

		mapGraphPub_->publish(std::move(msg));
	}
}

void CoreWrapper::detectMoreLoopClosuresCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::DetectMoreLoopClosures::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::DetectMoreLoopClosures::Response> res)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(get_logger(), "Ignoring /detect_more_loop_closures while safe relocalization owns cross-reset links");
		return;
	}
	RCLCPP_WARN(get_logger(), "Detect more loop closures service called");

	UTimer timer;
	float clusterRadiusMax = 1;
	float clusterRadiusMin = 0;
	float clusterAngle = 0;
	int iterations = 1;
	bool intraSession = true;
	bool interSession = true;
	if(req->cluster_radius_max > 0.0f)
	{
		clusterRadiusMax = req->cluster_radius_max;
	}
	if(req->cluster_radius_min >= 0.0f)
	{
		clusterRadiusMin = req->cluster_radius_min;
	}
	if(req->cluster_angle >= 0.0f)
	{
		clusterAngle = req->cluster_angle;
	}
	if(req->iterations >= 1.0f)
	{
		iterations = (int)req->iterations;
	}
	if(req->intra_only)
	{
		interSession = false;
	}
	else if(req->inter_only)
	{
		intraSession = false;
	}
	RCLCPP_WARN(get_logger(), "Post-Processing service called: Detecting more loop closures "
			"(max radius=%f, min radius=%f, angle=%f, iterations=%d, intra=%s, inter=%s)...",
			clusterRadiusMax,
			clusterRadiusMin,
			clusterAngle,
			iterations,
			intraSession?"true":"false",
			interSession?"true":"false");
	res->detected = rtabmap_.detectMoreLoopClosures(
			clusterRadiusMax,
			clusterAngle*M_PI/180.0,
			iterations,
			intraSession,
			interSession,
			0,
			clusterRadiusMin);
	if(res->detected<0)
	{
		RCLCPP_ERROR(get_logger(), "Post-Processing: Detecting more loop closures failed!");
	}
	else
	{
		RCLCPP_WARN(get_logger(), "Post-Processing: Detected %d loop closures! (%fs)", res->detected, timer.ticks());

		if(res->detected>0)
		{
			republishMaps();
			++safeRelocalizationGraphRevision_;
			publishResetEpoch();
		}
	}
}

void CoreWrapper::cleanupLocalGridsCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::CleanupLocalGrids::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::CleanupLocalGrids::Response> res)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(get_logger(),
				"Ignoring /cleanup_local_grids: safe relocalization owns persistent map mutations");
		res->modified = -1;
		return;
	}
	RCLCPP_WARN(get_logger(), "Cleanup local grids service called");
	UTimer timer;
	int radius = 1;
	bool filterScans = false;
	if(req->radius > 1.0f)
	{
		radius = (int)req->radius;
	}
	filterScans = req->filter_scans;
	float xMin, yMin, gridCellSize;
	cv::Mat map = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
	if(map.empty())
	{
		RCLCPP_ERROR(get_logger(), "Post-Processing: Cleanup local grids failed! There is no optimized map.");
	}
	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	RCLCPP_WARN(get_logger(), "Post-Processing: Cleanup local grids... (radius=%d, filter scans=%s)",
			radius,
			filterScans?"true":"false");
	res->modified = rtabmap_.cleanupLocalGrids(poses, map, xMin, yMin, gridCellSize, radius, filterScans);
	if(res->modified<0)
	{
		RCLCPP_ERROR(get_logger(), "Post-Processing: Cleanup local grids failed!");
	}
	else
	{
		if(filterScans)
		{
			RCLCPP_WARN(get_logger(), "Post-Processing: %d grids and scans modified! (%fs)", res->modified, timer.ticks());
		}
		else
		{
			RCLCPP_WARN(get_logger(), "Post-Processing: %d grids modified! (%fs)", res->modified, timer.ticks());
		}
		if(res->modified > 0)
		{
			// We should update MapsManager's cache with the modifications
			mapsManager_.clear();
			mapsManager_.set2DMap(map, xMin, yMin, gridCellSize, rtabmap_.getLocalOptimizedPoses(), rtabmap_.getMemory());

			republishMaps();
		}
	}
}
void CoreWrapper::globalBundleAdjustmentCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::GlobalBundleAdjustment::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::GlobalBundleAdjustment::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(get_logger(), "Ignoring /global_bundle_adjustment while guarded transaction snapshots are active");
		return;
	}
	RCLCPP_WARN(get_logger(), "Global bundle adjustment service called");

	UTimer timer;
	int optimizer = (int)Optimizer::kTypeG2O; // g2o
	int iterations = Parameters::defaultOptimizerIterations();
	float pixelVariance = Parameters::defaultg2oPixelVariance();
	bool rematchFeatures = true;
	Parameters::parse(parameters_, Parameters::kOptimizerIterations(), iterations);
	Parameters::parse(parameters_, Parameters::kg2oPixelVariance(), pixelVariance);
	if(req->type == 1.0f)
	{
		optimizer = (int)Optimizer::kTypeCVSBA;
	}
	if(req->iterations >= 1.0f)
	{
		iterations = req->iterations;
	}
	if(req->pixel_variance > 0.0f)
	{
		pixelVariance = req->pixel_variance;
	}
	rematchFeatures = !req->voc_matches;

	RCLCPP_WARN(get_logger(), "Post-Processing: Global Bundle Adjustment... "
			"(Optimizer=%s, iterations=%d, pixel variance=%f, rematch=%s)...",
			optimizer==Optimizer::kTypeG2O?"g2o":"cvsba",
			iterations,
			pixelVariance,
			rematchFeatures?"true":"false");
	bool success = rtabmap_.globalBundleAdjustment((Optimizer::Type)optimizer, rematchFeatures, iterations, pixelVariance);
	if(!success)
	{
		RCLCPP_ERROR(get_logger(), "Post-Processing: Global Bundle Adjustment failed!");
	}
	else
	{
		RCLCPP_WARN(get_logger(), "Post-Processing: Global Bundle Adjustment... done! (%fs)", timer.ticks());
		republishMaps();
		++safeRelocalizationGraphRevision_;
		publishResetEpoch();
	}
}

void CoreWrapper::setModeLocalizationCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring localization-mode switch: guarded relocalization requires incremental mapping");
		return;
	}
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set localization mode");
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
	rtabmap_.parseParameters(parameters);
	RCLCPP_INFO(this->get_logger(), "rtabmap: Localization mode enabled!");
	++safeRelocalizationGraphRevision_;
	publishResetEpoch();
}

void CoreWrapper::setModeMappingCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring mapping-mode switch: guarded relocalization mode is immutable");
		return;
	}
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set mapping mode");
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
	rtabmap_.parseParameters(parameters);
	RCLCPP_INFO(this->get_logger(), "rtabmap: Mapping mode enabled!");
	++safeRelocalizationGraphRevision_;
	publishResetEpoch();
}

void CoreWrapper::setLogDebug(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set log level to Debug");
	ULogger::setLevel(ULogger::kDebug);
}
void CoreWrapper::setLogInfo(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set log level to Info");
	ULogger::setLevel(ULogger::kInfo);
}
void CoreWrapper::setLogWarn(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set log level to Warning");
	ULogger::setLevel(ULogger::kWarning);
}
void CoreWrapper::setLogError(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set log level to Error");
	ULogger::setLevel(ULogger::kError);
}

void CoreWrapper::getNodeDataCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::GetNodeData::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::GetNodeData::Response> res)
{
	RCLCPP_INFO(get_logger(), "rtabmap: Getting node data (%d node(s), images=%s scan=%s grid=%s user_data=%s)...",
			(int)req->ids.size(),
			req->images?"true":"false",
			req->scan?"true":"false",
			req->grid?"true":"false",
			req->user_data?"true":"false");

	if(req->ids.empty() && rtabmap_.getMemory() && rtabmap_.getMemory()->getLastWorkingSignature(true))
	{
		req->ids.push_back(rtabmap_.getMemory()->getLastWorkingSignature(true)->id());
	}
	for(size_t i=0; i<req->ids.size(); ++i)
	{
		int id = req->ids[i];
		Signature s = rtabmap_.getSignatureCopy(id, req->images, req->scan, req->user_data, req->grid, true, true);

		if(s.id()>0)
		{
			rtabmap_msgs::msg::Node msg;
			rtabmap_conversions::nodeToROS(s, msg);
			res->data.push_back(msg);
		}
	}
}

void CoreWrapper::getMapDataCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::GetMap::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::GetMap::Response> res)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Getting map (global=%s optimized=%s graphOnly=%s)...",
			req->global_map?"true":"false",
			req->optimized?"true":"false",
			req->graph_only?"true":"false");
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, rtabmap::Link> constraints;

	rtabmap_.getGraph(
			poses,
			constraints,
			req->optimized,
			req->global_map,
			&signatures,
			!req->graph_only,
			!req->graph_only,
			!req->graph_only,
			!req->graph_only,
			!req->graph_only,
			!req->graph_only);

	mapToOdomMutex_.lock();
	Transform mapToOdomSafe = mapToOdom_.clone();
	mapToOdomMutex_.unlock();

	//RGB-D SLAM data
	rtabmap_conversions::mapDataToROS(poses,
		constraints,
		signatures,
		mapToOdomSafe,
		res->data);

	res->data.header.stamp = now();
	res->data.header.frame_id = mapFrameId_;
	RCLCPP_INFO(this->get_logger(), "rtabmap: Getting map (global=%s optimized=%s graphOnly=%s)...done!",
			req->global_map?"true":"false",
			req->optimized?"true":"false",
			req->graph_only?"true":"false");
}

void CoreWrapper::getMapData2Callback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::GetMap2::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::GetMap2::Response> res)
{
	RCLCPP_INFO(get_logger(), "rtabmap: Getting map (global=%s optimized=%s with_images=%s with_scans=%s with_user_data=%s with_grids=%s with_words=%s with_global_descriptors=%s)...",
			req->global_map?"true":"false",
			req->optimized?"true":"false",
			req->with_images?"true":"false",
			req->with_scans?"true":"false",
			req->with_user_data?"true":"false",
			req->with_grids?"true":"false",
			req->with_words?"true":"false",
			req->with_global_descriptors?"true":"false");
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, rtabmap::Link> constraints;

	rtabmap_.getGraph(
			poses,
			constraints,
			req->optimized,
			req->global_map,
			&signatures,
			req->with_images,
			req->with_scans,
			req->with_user_data,
			req->with_grids,
			req->with_words,
			req->with_global_descriptors);

	mapToOdomMutex_.lock();
	Transform mapToOdomSafe = mapToOdom_.clone();
	mapToOdomMutex_.unlock();

	//RGB-D SLAM data
	rtabmap_conversions::mapDataToROS(poses,
		constraints,
		signatures,
		mapToOdomSafe,
		res->data);

	res->data.header.stamp = now();
	res->data.header.frame_id = mapFrameId_;
}

void CoreWrapper::getMapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
		std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
	// Make sure grid map cache is up to date (in case there is no subscriber on map topics)
	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), true, false);

	// create the grid map
	float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
	cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);

	if(!pixels.empty())
	{
		//init
		res->map.info.resolution = gridCellSize;
		res->map.info.origin.position.x = 0.0;
		res->map.info.origin.position.y = 0.0;
		res->map.info.origin.position.z = 0.0;
		res->map.info.origin.orientation.x = 0.0;
		res->map.info.origin.orientation.y = 0.0;
		res->map.info.origin.orientation.z = 0.0;
		res->map.info.origin.orientation.w = 1.0;

		res->map.info.width = pixels.cols;
		res->map.info.height = pixels.rows;
		res->map.info.origin.position.x = xMin;
		res->map.info.origin.position.y = yMin;
		res->map.data.resize(res->map.info.width * res->map.info.height);

		memcpy(res->map.data.data(), pixels.data, res->map.info.width * res->map.info.height);

		res->map.header.frame_id = mapFrameId_;
		res->map.header.stamp = now();
	}
	else
	{
		RCLCPP_WARN(get_logger(), "rtabmap: The map is empty!");
	}
}

void CoreWrapper::getProbMapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
		std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
	// Make sure grid map cache is up to date (in case there is no subscriber on map topics)
	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), true, false);

	// create the grid map
	float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
	cv::Mat pixels = mapsManager_.getGridProbMap(xMin, yMin, gridCellSize);

	if(!pixels.empty())
	{
		//init
		res->map.info.resolution = gridCellSize;
		res->map.info.origin.position.x = 0.0;
		res->map.info.origin.position.y = 0.0;
		res->map.info.origin.position.z = 0.0;
		res->map.info.origin.orientation.x = 0.0;
		res->map.info.origin.orientation.y = 0.0;
		res->map.info.origin.orientation.z = 0.0;
		res->map.info.origin.orientation.w = 1.0;

		res->map.info.width = pixels.cols;
		res->map.info.height = pixels.rows;
		res->map.info.origin.position.x = xMin;
		res->map.info.origin.position.y = yMin;
		res->map.data.resize(res->map.info.width * res->map.info.height);

		memcpy(res->map.data.data(), pixels.data, res->map.info.width * res->map.info.height);

		res->map.header.frame_id = mapFrameId_;
		res->map.header.stamp = now();
	}
	else
	{
		RCLCPP_WARN(get_logger(), "rtabmap: The map is empty!");
	}
}

void CoreWrapper::publishMapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::PublishMap::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::PublishMap::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Publishing map...");

	rclcpp::Time stampNow = now();

	if(mapDataPub_->get_subscription_count() ||
	   (!req->graph_only && mapsManager_.hasSubscribers()) ||
	   (req->graph_only && (labelsPub_->get_subscription_count() || mapGraphPub_->get_subscription_count() || mapPathPub_->get_subscription_count())))
	{
		std::map<int, Transform> poses;
		std::multimap<int, rtabmap::Link> constraints;
		std::map<int, Signature > signatures;

		rtabmap_.getGraph(
				poses,
				constraints,
				req->optimized,
				req->global_map,
				&signatures,
				!req->graph_only,
				!req->graph_only,
				!req->graph_only,
				!req->graph_only);

		mapToOdomMutex_.lock();
		Transform mapToOdomSafe = mapToOdom_.clone();
		mapToOdomMutex_.unlock();

		if(mapDataPub_->get_subscription_count())
		{
			rtabmap_msgs::msg::MapData::UniquePtr msg(new rtabmap_msgs::msg::MapData);
			msg->header.stamp = stampNow;
			msg->header.frame_id = mapFrameId_;

			rtabmap_conversions::mapDataToROS(poses,
				constraints,
				signatures,
				mapToOdomSafe,
				*msg);

			mapDataPub_->publish(std::move(msg));
		}

		if(mapGraphPub_->get_subscription_count())
		{
			rtabmap_msgs::msg::MapGraph::UniquePtr msg(new rtabmap_msgs::msg::MapGraph);
			msg->header.stamp = stampNow;
			msg->header.frame_id = mapFrameId_;

			rtabmap_conversions::mapGraphToROS(poses,
				constraints,
				mapToOdomSafe,
				*msg);

			mapGraphPub_->publish(std::move(msg));
		}

		bool pubLabels = labelsPub_->get_subscription_count();
		visualization_msgs::msg::MarkerArray markers;
		if((landmarksPub_->get_subscription_count() || pubLabels) && !poses.empty() && poses.begin()->first < 0)
		{
			geometry_msgs::msg::PoseArray::UniquePtr msg(new geometry_msgs::msg::PoseArray);
			msg->header.stamp = stampNow;
			msg->header.frame_id = mapFrameId_;
			for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end() && iter->first<0; ++iter)
			{
				geometry_msgs::msg::Pose p;
				rtabmap_conversions::transformToPoseMsg(iter->second, p);
				msg->poses.push_back(p);

				if(pubLabels)
				{
					// Add landmark ids
					visualization_msgs::msg::Marker marker;
					marker.header.frame_id = mapFrameId_;
					marker.header.stamp = stampNow;
					marker.ns = "landmarks";
					marker.id = iter->first;
					marker.action = visualization_msgs::msg::Marker::ADD;
					marker.pose.position.x = iter->second.x();
					marker.pose.position.y = iter->second.y();
					marker.pose.position.z = iter->second.z();
					marker.pose.orientation.x = 0.0;
					marker.pose.orientation.y = 0.0;
					marker.pose.orientation.z = 0.0;
					marker.pose.orientation.w = 1.0;
					marker.scale.x = 1;
					marker.scale.y = 1;
					marker.scale.z = 0.35;
					marker.color.a = 0.5;
					marker.color.r = 1.0;
					marker.color.g = 1.0;
					marker.color.b = 0.0;
					marker.lifetime = rclcpp::Duration::from_seconds(2.0f/rate_);

					marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
					marker.text = uNumber2Str(iter->first);

					markers.markers.push_back(marker);
				}
			}

			landmarksPub_->publish(std::move(msg));
		}

		if(!req->graph_only)
		{
			if(mapsManager_.hasSubscribers())
			{
				std::map<int, Transform> filteredPoses(poses.lower_bound(1), poses.end());
				if((mappingMaxNodes_ > 0 || mappingAltitudeDelta_>0.0) && filteredPoses.size()>1)
				{
					std::map<int, Transform> nearestPoses = filterNodesToAssemble(filteredPoses, filteredPoses.rbegin()->second);
				}
				if(signatures.size())
				{
					filteredPoses = mapsManager_.updateMapCaches(
							filteredPoses,
							rtabmap_.getMemory(),
							false,
							false,
							signatures);
				}
				else
				{
					filteredPoses = mapsManager_.getFilteredPoses(filteredPoses);
				}
				mapsManager_.publishMaps(filteredPoses, stampNow, mapFrameId_);
			}
			else
			{
				// this will cleanup the cache if there are no subscribers
				mapsManager_.publishMaps(std::map<int, Transform>(), stampNow, mapFrameId_);
			}
		}

		bool pubPath = mapPathPub_->get_subscription_count();
		if(pubLabels || pubPath)
		{
			if(poses.size() && signatures.size())
			{
				nav_msgs::msg::Path path;
				if(pubPath)
				{
					path.poses.resize(poses.size());
				}
				int oi=0;
				for(std::map<int, Signature>::const_iterator iter=signatures.begin();
					iter!=signatures.end();
					++iter)
				{
					std::map<int, Transform>::const_iterator poseIter= poses.find(iter->first);
					if(poseIter!=poses.end())
					{
						if(pubLabels)
						{
							// Add labels
							if(!iter->second.getLabel().empty())
							{
								visualization_msgs::msg::Marker marker;
								marker.header.frame_id = mapFrameId_;
								marker.header.stamp = stampNow;
								marker.ns = "labels";
								marker.id = iter->first;
								marker.action = visualization_msgs::msg::Marker::ADD;
								marker.pose.position.x = poseIter->second.x();
								marker.pose.position.y = poseIter->second.y();
								marker.pose.position.z = poseIter->second.z();
								marker.pose.orientation.x = 0.0;
								marker.pose.orientation.y = 0.0;
								marker.pose.orientation.z = 0.0;
								marker.pose.orientation.w = 1.0;
								marker.scale.x = 1;
								marker.scale.y = 1;
								marker.scale.z = 0.5;
								marker.color.a = 0.7;
								marker.color.r = 1.0;
								marker.color.g = 0.0;
								marker.color.b = 0.0;

								marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
								marker.text = iter->second.getLabel();

								markers.markers.push_back(marker);
							}
							// Add node ids
							visualization_msgs::msg::Marker marker;
							marker.header.frame_id = mapFrameId_;
							marker.header.stamp = stampNow;
							marker.ns = "ids";
							marker.id = iter->first;
							marker.action = visualization_msgs::msg::Marker::ADD;
							marker.pose.position.x = poseIter->second.x();
							marker.pose.position.y = poseIter->second.y();
							marker.pose.position.z = poseIter->second.z();
							marker.pose.orientation.x = 0.0;
							marker.pose.orientation.y = 0.0;
							marker.pose.orientation.z = 0.0;
							marker.pose.orientation.w = 1.0;
							marker.scale.x = 1;
							marker.scale.y = 1;
							marker.scale.z = 0.2;
							marker.color.a = 0.5;
							marker.color.r = 1.0;
							marker.color.g = 1.0;
							marker.color.b = 1.0;
							marker.lifetime = rclcpp::Duration::from_seconds(2.0f/rate_);

							marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
							marker.text = uNumber2Str(iter->first);

							markers.markers.push_back(marker);
						}
						if(pubPath)
						{
							rtabmap_conversions::transformToPoseMsg(poseIter->second, path.poses.at(oi).pose);
							path.poses.at(oi).header.frame_id = mapFrameId_;
							path.poses.at(oi).header.stamp = rtabmap_conversions::timestampToROS(iter->second.getStamp());
							++oi;
						}
					}
				}

				if(pubLabels && markers.markers.size())
				{
					labelsPub_->publish(markers);
				}
				if(pubPath && oi)
				{
					path.header.frame_id = mapFrameId_;
					path.header.stamp = stampNow;
					path.poses.resize(oi);
					mapPathPub_->publish(path);
				}
			}
		}
	}
	else
	{
		UWARN("No subscribers, don't need to publish!");
		if(!req->graph_only)
		{
			// this will cleanup the cache if there are no subscribers
			mapsManager_.publishMaps(std::map<int, Transform>(), stampNow, mapFrameId_);
		}
	}
}

void CoreWrapper::getPlanCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
		std::shared_ptr<nav_msgs::srv::GetPlan::Response> res)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring get_plan: guarded relocalization mode forbids path mutation");
		return;
	}
	Transform pose = rtabmap_conversions::transformFromPoseMsg(req->goal.pose, true);
	UTimer timer;
	if(!pose.isNull())
	{
		// transform goal in /map frame
		Transform coordinateTransform = Transform::getIdentity();
		if(!req->goal.header.frame_id.empty() && mapFrameId_.compare(req->goal.header.frame_id) != 0)
		{
			coordinateTransform = rtabmap_conversions::getTransform(mapFrameId_, req->goal.header.frame_id, rclcpp::Time(req->goal.header.stamp.sec, req->goal.header.stamp.nanosec), *tfBuffer_, waitForTransform_);
			if(coordinateTransform.isNull())
			{
				RCLCPP_ERROR(this->get_logger(), "Cannot transform goal pose from \"%s\" frame to \"%s\" frame!",
						req->goal.header.frame_id.c_str(), mapFrameId_.c_str());
				return;
			}
			pose = coordinateTransform * pose;
		}
		//else assume map frame if not set

		// To convert back the poses in goal frame
		coordinateTransform = coordinateTransform.inverse();

		if(rtabmap_.computePath(pose, req->tolerance))
		{
			RCLCPP_INFO(this->get_logger(), "Planning: Time computing path = %f s", timer.ticks());
			const std::vector<std::pair<int, Transform> > & poses = rtabmap_.getPath();
			res->plan.header.frame_id = req->goal.header.frame_id;
			res->plan.header.stamp = req->goal.header.stamp;
			if(poses.size() == 0)
			{
				RCLCPP_WARN(this->get_logger(), "Planning: Goal already reached (RGBD/GoalReachedRadius=%fm).",
						rtabmap_.getGoalReachedRadius());
				// just set the goal directly
				res->plan.poses.resize(1);
				rtabmap_conversions::transformToPoseMsg(coordinateTransform*pose, res->plan.poses[0].pose);
			}
			else
			{
				res->plan.poses.resize(poses.size());
				int oi = 0;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					res->plan.poses[oi].header = res->plan.header;
					rtabmap_conversions::transformToPoseMsg(coordinateTransform*iter->second, res->plan.poses[oi].pose);
					++oi;
				}
				if(!rtabmap_.getPathTransformToGoal().isIdentity())
				{
					res->plan.poses.resize(res->plan.poses.size()+1);
					res->plan.poses[res->plan.poses.size()-1].header = res->plan.header;
					Transform p = rtabmap_.getPath().back().second*rtabmap_.getPathTransformToGoal();
					rtabmap_conversions::transformToPoseMsg(coordinateTransform*p, res->plan.poses[res->plan.poses.size()-1].pose);
				}

				// Just output the path on screen
				std::stringstream stream;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					if(iter != poses.begin())
					{
						stream << " ";
					}
					stream << iter->first;
				}
				RCLCPP_INFO(this->get_logger(), "Planned path: [%s]", stream.str().c_str());
			}
		}
		rtabmap_.clearPath(0);
	}
}

void CoreWrapper::getPlanNodesCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::GetPlan::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::GetPlan::Response> res)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring get_plan_nodes: guarded relocalization mode forbids path mutation");
		return;
	}
	Transform pose;
	if(req->goal_node <= 0)
	{
		pose = rtabmap_conversions::transformFromPoseMsg(req->goal.pose, true);
	}
	UTimer timer;
	if(req->goal_node > 0 || !pose.isNull())
	{
		Transform coordinateTransform = Transform::getIdentity();
		// transform goal in /map frame
		if(!pose.isNull() && !req->goal.header.frame_id.empty() && mapFrameId_.compare(req->goal.header.frame_id) != 0)
		{
			coordinateTransform = rtabmap_conversions::getTransform(mapFrameId_, req->goal.header.frame_id, req->goal.header.stamp, *tfBuffer_, waitForTransform_);
			if(coordinateTransform.isNull())
			{
				RCLCPP_ERROR(this->get_logger(), "Cannot transform goal pose from \"%s\" frame to \"%s\" frame!",
						req->goal.header.frame_id.c_str(), mapFrameId_.c_str());
				return;
			}
			if(!pose.isNull())
			{
				pose = coordinateTransform * pose;
			}
		}
		//else assume map frame if not set

		// To convert back the poses in goal frame
		coordinateTransform = coordinateTransform.inverse();

		if((req->goal_node > 0 && rtabmap_.computePath(req->goal_node, req->tolerance)) ||
		   (req->goal_node <= 0 && rtabmap_.computePath(pose, req->tolerance)))
		{
			RCLCPP_INFO(this->get_logger(), "Planning: Time computing path = %f s", timer.ticks());
			const std::vector<std::pair<int, Transform> > & poses = rtabmap_.getPath();
			res->plan.header.frame_id = mapFrameId_;
			if(req->goal_node > 0)
			{
				res->plan.header.stamp = now();
			}
			else
			{
				res->plan.header.stamp = req->goal.header.stamp;
			}
			if(poses.size() == 0)
			{
				RCLCPP_WARN(this->get_logger(), "Planning: Goal already reached (RGBD/GoalReachedRadius=%fm).",
						rtabmap_.getGoalReachedRadius());
				if(!pose.isNull())
				{
					// just set the goal directly
					res->plan.poses.resize(1);
					res->plan.node_ids.resize(1);
					rtabmap_conversions::transformToPoseMsg(coordinateTransform*pose, res->plan.poses[0]);
					res->plan.node_ids[0] = 0;
				}
			}
			else
			{
				res->plan.poses.resize(poses.size());
				res->plan.node_ids.resize(poses.size());
				int oi = 0;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					rtabmap_conversions::transformToPoseMsg(coordinateTransform*iter->second, res->plan.poses[oi]);
					res->plan.node_ids[oi] = iter->first;
					++oi;
				}
				if(!rtabmap_.getPathTransformToGoal().isIdentity())
				{
					res->plan.poses.resize(res->plan.poses.size()+1);
					res->plan.node_ids.resize(res->plan.node_ids.size()+1);
					Transform p = rtabmap_.getPath().back().second*rtabmap_.getPathTransformToGoal();
					rtabmap_conversions::transformToPoseMsg(coordinateTransform*p, res->plan.poses[res->plan.poses.size()-1]);
					res->plan.node_ids[res->plan.node_ids.size()-1] = 0;
				}

				// Just output the path on screen
				std::stringstream stream;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					if(iter != poses.begin())
					{
						stream << " ";
					}
					stream << iter->first;
				}
				RCLCPP_INFO(this->get_logger(), "Planned path: [%s]", stream.str().c_str());
			}
		}
		rtabmap_.clearPath(0);
	}
}

void CoreWrapper::setGoalCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::SetGoal::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::SetGoal::Response> res)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring set_goal: guarded relocalization mode forbids path mutation");
		return;
	}
	double planningTime = 0.0;
	goalCommonCallback(req->node_id, req->node_label, req->frame_id, Transform(), now(), &planningTime);
	const std::vector<std::pair<int, Transform> > & path = rtabmap_.getPath();
	res->path_ids.resize(path.size());
	res->path_poses.resize(path.size());
	res->planning_time = planningTime;
	for(unsigned int i=0; i<path.size(); ++i)
	{
		res->path_ids[i] = path[i].first;
		rtabmap_conversions::transformToPoseMsg(path[i].second, res->path_poses[i]);
	}
}

void CoreWrapper::cancelGoalCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(), "Ignoring cancel_goal: guarded relocalization mode forbids path mutation");
		return;
	}
	if(rtabmap_.getPath().size())
	{
		RCLCPP_WARN(this->get_logger(), "Goal cancelled!");
		rtabmap_.clearPath(0);
		currentMetricGoal_.setNull();
		lastPublishedMetricGoal_.setNull();
		goalFrameId_.clear();
		latestNodeWasReached_ = false;
		if(goalReachedPub_->get_subscription_count())
		{
			std_msgs::msg::Bool result;
			result.data = false;
			goalReachedPub_->publish(result);
		}
	}
#ifdef WITH_NAV2_MSGS
	if(nav2Client_.get() != NULL && nav2Client_->action_server_is_ready())
	{
		nav2Client_->async_cancel_all_goals();
	}
#endif
}

void CoreWrapper::setLabelCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::SetLabel::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::SetLabel::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(),
				"Ignoring /set_label: safe relocalization owns persistent map mutations");
		return;
	}
	if(rtabmap_.labelLocation(req->node_id, req->node_label))
	{
		if(req->node_id > 0)
		{
			RCLCPP_INFO(this->get_logger(), "Set label \"%s\" to node %d", req->node_label.c_str(), req->node_id);
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Set label \"%s\" to last node", req->node_label.c_str());
		}
	}
	else
	{
		if(req->node_id > 0)
		{
			RCLCPP_ERROR(this->get_logger(), "Could not set label \"%s\" to node %d", req->node_label.c_str(), req->node_id);
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Could not set label \"%s\" to last node", req->node_label.c_str());
		}
	}
}

void CoreWrapper::listLabelsCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::ListLabels::Request>,
		std::shared_ptr<rtabmap_msgs::srv::ListLabels::Response> res)
{
	if(rtabmap_.getMemory())
	{
		std::map<int, std::string> labels = rtabmap_.getMemory()->getAllLabels();
		res->ids = uKeys(labels);
		res->labels = uValues(labels);
		RCLCPP_INFO(this->get_logger(), "List labels service: %d labels found.", (int)res->labels.size());
	}
}

void CoreWrapper::removeLabelCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::RemoveLabel::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::RemoveLabel::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(this->get_logger(),
				"Ignoring /remove_label: safe relocalization owns persistent map mutations");
		return;
	}
	if(rtabmap_.getMemory())
	{
		int id = rtabmap_.getMemory()->getSignatureIdByLabel(req->label, true);
		if(id == 0)
		{
			RCLCPP_WARN(this->get_logger(), "Label \"%s\" not found in the map, cannot remove it!", req->label.c_str());
		}
		else if(!rtabmap_.labelLocation(id, ""))
		{
			RCLCPP_ERROR(this->get_logger(), "Failed removing label \"%s\".", req->label.c_str());
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Removed label \"%s\".", req->label.c_str());
		}
	}
}

void CoreWrapper::addLinkCallback(const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::AddLink::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::AddLink::Response>)
{
	if(safeRelocalizationEnabled_)
	{
		RCLCPP_ERROR(get_logger(), "Ignoring /add_link: guarded transactions are the only external graph-edge path");
		return;
	}
	if(rtabmap_.getMemory())
	{
		RCLCPP_INFO(get_logger(), "Adding external link %d -> %d", req->link.from_id, req->link.to_id);
		if(rtabmap_.addLink(rtabmap_conversions::linkFromROS(req->link)))
		{
			mapsManager_.clear();
			std::map<int, rtabmap::Transform> cachePoses = mapsManager_.updateMapCaches(
					rtabmap_.getLocalOptimizedPoses(), rtabmap_.getMemory(), false, false);
			mapsManager_.publishMaps(cachePoses, now(), mapFrameId_);
			++safeRelocalizationGraphRevision_;
			publishResetEpoch();
		}
	}
}

rtabmap::ImageInputIdentity CoreWrapper::captureAuthenticatedImageIdentity(
        const cv_bridge::CvImageConstPtr & image,
        const sensor_msgs::msg::CameraInfo & info)
{
    rtabmap::ImageInputIdentity input;
    if(!resetQuarantineEnabled_ || !safeRelocalizationEnabled_ || !image ||
       rclcpp::Time(image->header.stamp).nanoseconds() != rclcpp::Time(info.header.stamp).nanoseconds() ||
       image->header.frame_id != resetQuarantineExpectedSensorFrame_ ||
       info.header.frame_id != resetQuarantineExpectedSensorFrame_) return input;
    cv::Mat canonical;
    if(image->encoding == "bgr8" && image->image.type() == CV_8UC3) canonical = image->image;
    else if(image->encoding == "rgb8" && image->image.type() == CV_8UC3)
        cv::cvtColor(image->image, canonical, cv::COLOR_RGB2BGR);
    else if(image->encoding == "mono8" && image->image.type() == CV_8UC1) canonical = image->image;
    else return input;
    std::lock_guard<std::mutex> lock(resetQuarantineRgbdAuthMutex_);
    if(!resetQuarantineRgbPublisherBound_ || !resetQuarantineDepthPublisherBound_ ||
       !resetQuarantineCameraInfoPublisherBound_ || resetQuarantineRgbPublisherCompromised_ ||
       resetQuarantineDepthPublisherCompromised_ || resetQuarantineCameraInfoPublisherCompromised_) return input;
    auto hex = [](const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid) {
        std::string value;
        for(uint8_t byte : gid) value += uFormat("%02x", static_cast<unsigned int>(byte));
        return value;
    };
    input.headerStampNs = rclcpp::Time(info.header.stamp).nanoseconds();
    input.width = info.width;
    input.height = info.height;
    input.encoding = canonical.type() == CV_8UC1 ? "mono8" : "bgr8";
    input.pixelSha256 = rtabmap::canonicalImageSha256(canonical);
    std::copy(info.k.begin(), info.k.end(), input.K.begin());
    input.D = info.d;
    input.sensorFrame = info.header.frame_id;
    input.rgbPublisherGidHex = hex(resetQuarantineRgbPublisherGid_);
    input.infoPublisherGidHex = hex(resetQuarantineCameraInfoPublisherGid_);
    input.sessionTokenSha256 = safeSha256(safeRelocalizationSessionToken_);
    input.databaseSha256 = safeRelocalizationDatabaseSha256_;
    input.provenanceScope = "managed_ros_rgbd";
    return input;
}

bool CoreWrapper::queuedImageIdentityStillValid(const rtabmap::ImageInputIdentity & input)
{
    if(!resetQuarantineEnabled_ || !safeRelocalizationEnabled_ ||
       input.provenanceScope != "managed_ros_rgbd" || input.headerStampNs <= 0 ||
       input.sensorFrame != resetQuarantineExpectedSensorFrame_ ||
       input.sessionTokenSha256 != safeSha256(safeRelocalizationSessionToken_) ||
       input.databaseSha256 != safeRelocalizationDatabaseSha256_) return false;
    std::lock_guard<std::mutex> lock(resetQuarantineRgbdAuthMutex_);
    if(!resetQuarantineRgbPublisherBound_ || !resetQuarantineDepthPublisherBound_ ||
       !resetQuarantineCameraInfoPublisherBound_ || resetQuarantineRgbPublisherCompromised_ ||
       resetQuarantineDepthPublisherCompromised_ || resetQuarantineCameraInfoPublisherCompromised_) return false;
    auto hex = [](const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid) {
        std::string value;
        for(uint8_t byte : gid) value += uFormat("%02x", static_cast<unsigned int>(byte));
        return value;
    };
    return input.rgbPublisherGidHex == hex(resetQuarantineRgbPublisherGid_) &&
           input.infoPublisherGidHex == hex(resetQuarantineCameraInfoPublisherGid_);
}

void CoreWrapper::publishAcceptedImageReceipt(const rtabmap::AcceptedImageReceipt & receipt)
{
    if(!acceptedImageReceiptPub_ || receipt.signatureId <= 0 ||
       !queuedImageIdentityStillValid(receipt.input)) return;
    cv::FileStorage json("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    json << "schema_version" << 1 << "producer" << "wbrov_core_accepted_image_input";
    json << "node_id" << receipt.signatureId << "map_id" << receipt.mapId;
    // Decimal strings preserve integers beyond JSON/OpenCV's double range.
    json << "header_stamp_ns" << std::to_string(receipt.input.headerStampNs);
    json << "reset_epoch" << std::to_string(receipt.resetEpoch);
    json << "image_width" << receipt.input.width << "image_height" << receipt.input.height;
    json << "encoding" << receipt.input.encoding << "rgb_sha256" << receipt.input.pixelSha256;
    json << "camera_frame" << receipt.input.sensorFrame;
    json << "K" << "["; for(double value : receipt.input.K) json << value; json << "]";
    json << "D" << "["; for(double value : receipt.input.D) json << value; json << "]";
    json << "body_from_camera" << "["; for(double value : receipt.bodyFromCamera) json << value; json << "]";
    json << "rgb_publisher_gid" << receipt.input.rgbPublisherGidHex;
    json << "camera_info_publisher_gid" << receipt.input.infoPublisherGidHex;
    json << "session_token_sha256" << receipt.input.sessionTokenSha256;
    json << "database_sha256" << receipt.input.databaseSha256;
    json << "source_scope" << receipt.input.provenanceScope;
    json << "pixel_role" << "authenticated_pre_memory_input";
    json << "saved_jpeg_bytes_are_original_pixels" << 0;
    std_msgs::msg::String message;
    message.data = json.releaseAndGetString();
    acceptedImageReceiptPub_->publish(message);
}


void CoreWrapper::publishResetEpoch()
{
	if(safeRelocalizationEnabled_)
	{
		verifySafeRelocalizationDatabasePathBinding();
	}
	if(resetEpochPub_)
	{
		std_msgs::msg::UInt64 msg;
		msg.data = rtabmap_.getResetEpoch();
		resetEpochPub_->publish(msg);
	}
	if(safeRelocalizationStatePub_)
	{
		std::ostringstream graphPoses;
		graphPoses << std::setprecision(17);
		const std::map<int, rtabmap::Transform> & poses = rtabmap_.getLocalOptimizedPoses();
		bool graphSnapshotValid = poses.size() <= 20000;
		bool firstPose = true;
		for(std::map<int, rtabmap::Transform>::const_iterator iter=poses.lower_bound(1);
				graphSnapshotValid &&
				iter!=poses.end(); ++iter)
		{
			const rtabmap::Transform & pose = iter->second;
			if(pose.isNull() || !std::isfinite(pose.x()) || !std::isfinite(pose.y()) ||
					!std::isfinite(pose.z()))
			{
				graphSnapshotValid = false;
				break;
			}
			const Eigen::Quaterniond quaternion = pose.getQuaterniond();
			if(!std::isfinite(quaternion.x()) || !std::isfinite(quaternion.y()) ||
					!std::isfinite(quaternion.z()) || !std::isfinite(quaternion.w()))
			{
				graphSnapshotValid = false;
				break;
			}
			graphPoses << (firstPose?"":",")
					<< "{\"id\":" << iter->first
					<< ",\"translation\":[" << pose.x() << "," << pose.y() << "," << pose.z() << "]"
					<< ",\"quaternion_xyzw\":[" << quaternion.x() << "," << quaternion.y() << ","
					<< quaternion.z() << "," << quaternion.w() << "]}";
			firstPose = false;
		}
		std_msgs::msg::String state;
		state.data = uFormat(
				"{\"schema_version\":3,\"producer\":\"wbrov_rtabmap_safe_relocalization_core\",\"reset_epoch\":%llu,\"epoch_start_stamp_ns\":%lld,\"graph_revision\":%llu,\"database_sha256\":\"%s\",\"database_binding_valid\":%s,\"session_token_sha256\":\"%s\",\"graph_snapshot_valid\":%s,\"graph_poses\":[%s]}",
				rtabmap_.getResetEpoch(),
				resetQuarantineEpochStartStampNs_.load(std::memory_order_acquire),
				safeRelocalizationGraphRevision_,
				safeRelocalizationDatabaseSha256_.c_str(),
				safeRelocalizationDatabaseBindingValid_?"true":"false",
				safeRelocalizationSessionTokenSha256_.c_str(),
				graphSnapshotValid?"true":"false",
				graphSnapshotValid?graphPoses.str().c_str():"");
		safeRelocalizationStatePub_->publish(state);
	}
}

std::string CoreWrapper::publishSafeRelocalizationStatus(
		const std::string & result,
		const std::string & reason,
		const std::string & transactionToken,
		int tagId,
		unsigned long long sourceEpoch,
		unsigned long long observerGraphRevision,
		const std::vector<rtabmap::Link> & evidence,
		bool publish)
{
	std::string pairs;
	for(unsigned int i=0; i<evidence.size(); ++i)
	{
		pairs += uFormat("%s[%d,%d]", i?",":"", evidence[i].from(), evidence[i].to());
	}
	std_msgs::msg::String msg;
	msg.data = uFormat(
			"{\"schema_version\":1,\"producer\":\"wbrov_rtabmap_safe_relocalization_core\",\"result\":\"%s\",\"reason\":\"%s\",\"transaction_token\":\"%s\",\"tag_id\":%d,\"source_reset_epoch\":%llu,\"observer_graph_revision\":%llu,\"core_reset_epoch\":%llu,\"core_graph_revision\":%llu,\"database_sha256\":\"%s\",\"session_token_sha256\":\"%s\",\"evidence_pairs\":[%s]}",
			jsonSafe(result).c_str(), jsonSafe(reason).c_str(), jsonSafe(transactionToken).c_str(), tagId,
			sourceEpoch, observerGraphRevision, rtabmap_.getResetEpoch(),
			safeRelocalizationGraphRevision_, safeRelocalizationDatabaseSha256_.c_str(),
			safeRelocalizationSessionTokenSha256_.c_str(), pairs.c_str());
	if(publish && safeRelocalizationStatusPub_)
	{
		safeRelocalizationStatusPub_->publish(msg);
	}
	return msg.data;
}

void CoreWrapper::clearSafeRelocalizationTokens()
{
	// Reset only the per-epoch admission budget. Terminal token meanings are
	// session-wide and immutable: a delayed duplicate from an earlier epoch
	// must replay its original outcome, never acquire a second meaning after a
	// reset boundary.
	safeRelocalizationSeenTokens_.clear();
}

void CoreWrapper::clearSafeRelocalizationProtocolState(bool firstUpdateHandled, bool clearTokens)
{
	safeRelocalizationFirstUpdateHandled_ = firstUpdateHandled;
	if(clearTokens)
	{
		clearSafeRelocalizationTokens();
	}
	triggerNewMapBeforeNextUpdate_ = false;
	resetQuarantineLostStreak_.store(0, std::memory_order_release);
	resetQuarantineRecoveryPending_.store(false, std::memory_order_release);
	resetQuarantineSuspectedReset_.store(false, std::memory_order_release);
	resetQuarantineOdomInfoSeen_.store(false, std::memory_order_release);
	resetQuarantineLastOdomInfoSteadyNs_.store(0, std::memory_order_release);
	resetQuarantineLastOdomInfoStampNs_.store(0, std::memory_order_release);
	resetQuarantineLostStartStampNs_.store(0, std::memory_order_release);
	resetQuarantineRecoveryStampNs_.store(0, std::memory_order_release);
	resetQuarantineEpochStartStampNs_.store(0, std::memory_order_release);
	{
		UScopeMutex sourceLock(resetQuarantineStateMutex_);
		resetQuarantineAuthenticatedOdomInfoStamps_.clear();
	}
	{
		std::lock_guard<std::mutex> authLock(resetQuarantineOdomAuthMutex_);
		resetQuarantineAuthenticatedOdomSamples_.clear();
		resetQuarantineOdomContinuity_.clear();
		resetQuarantineContinuityStampNs_ = 0;
		resetQuarantineContinuityPose_ = Transform();
		resetQuarantineContinuityChain_ = 0;
	}
	resetQuarantineOdomAuthCondition_.notify_all();
	{
		std::lock_guard<std::mutex> authLock(resetQuarantineOdomInfoAuthMutex_);
		resetQuarantineAuthenticatedOdomInfoSamples_.clear();
	}
	resetQuarantineOdomInfoAuthCondition_.notify_all();
	{
		std::lock_guard<std::mutex> authLock(resetQuarantineRgbdAuthMutex_);
		resetQuarantineAuthenticatedRgbSamples_.clear();
		resetQuarantineAuthenticatedDepthSamples_.clear();
		resetQuarantineAuthenticatedCameraInfoSamples_.clear();
	}
	resetQuarantineRgbdAuthCondition_.notify_all();
	resetQuarantineEpoch_.store(rtabmap_.getResetEpoch(), std::memory_order_release);
	++safeRelocalizationGraphRevision_;
	publishResetEpoch();
}

void CoreWrapper::safeRelocalizationProposalCallback(
		const std_msgs::msg::String::SharedPtr msg,
		const rclcpp::MessageInfo & messageInfo)
{
	if(!msg || !safeRelocalizationEnabled_)
	{
		return;
	}
	const rmw_gid_t & incomingGid = messageInfo.get_rmw_message_info().publisher_gid;
	{
		std::lock_guard<std::mutex> authLock(safeRelocalizationProposalAuthMutex_);
		if(!validateResetQuarantinePublisher(
				safeRelocalizationProposalSub_->get_topic_name(),
				"safe-relocalization proposal",
				safeRelocalizationExpectedProposalNode_,
				safeRelocalizationExpectedProposalNamespace_,
				safeRelocalizationProposalPublisherBound_,
				safeRelocalizationProposalPublisherCompromised_,
				safeRelocalizationProposalPublisherGid_,
				&incomingGid))
		{
			if(safeRelocalizationProposalPublisherCompromised_)
			{
				safeRelocalizationFatal_.store(true, std::memory_order_release);
				rclcpp::shutdown();
			}
			return;
		}
	}
	std::string sessionToken;
	std::string transactionToken;
	std::string producer;
	std::string anchorDatabaseSha256;
	int tagId = 0;
	unsigned long long sourceEpoch = 0;
	unsigned long long observerGraphRevision = 0;
	double createdMonotonic = 0.0;
	std::vector<rtabmap::Link> evidence;
    std::vector<rtabmap::AcceptedImageReceipt> acceptedImages;
	std::vector<rtabmap::AcceptedTagCornerObservation> acceptedCorners;
	std::string transactionReason;
	std::string terminalStatus;
	std::pair<std::string, std::string> * terminalReservation = 0;
	bool trustedEnvelope = false;
	auto reject = [&](const std::string & why)
	{
		// Before the full session/database/epoch/revision binding is trusted, a
		// diagnostic rejection must not carry an attacker-supplied token which
		// the observer could mistake for the terminal result of its pending
		// transaction. After binding, every outcome is first cached in the
		// process-wide reservation and only then published.
		terminalStatus = publishSafeRelocalizationStatus(
				"rejected", why, terminalReservation?transactionToken:std::string(),
				tagId, sourceEpoch, observerGraphRevision, evidence, false);
		if(terminalReservation)
		{
			if(!terminalReservation->second.empty())
			{
				RCLCPP_FATAL(this->get_logger(),
						"Safe relocalization attempted to materialize two terminal outcomes for one reserved token");
				safeRelocalizationFatal_.store(true, std::memory_order_release);
				rclcpp::shutdown();
				return;
			}
			terminalReservation->second = terminalStatus;
		}
		if(safeRelocalizationStatusPub_)
		{
			std_msgs::msg::String terminalMessage;
			terminalMessage.data = terminalStatus;
			safeRelocalizationStatusPub_->publish(terminalMessage);
		}
	};

	try
	{
		if(!exactlyOneJsonObject(msg->data))
		{
			reject("proposal must contain exactly one bounded JSON object");
			return;
		}
		cv::FileStorage storage(msg->data,
				cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
		if(!storage.isOpened())
		{
			reject("proposal must be parseable JSON");
			return;
		}
		const cv::FileNode root = storage.root();
		// Once a syntactically valid token can be extracted, its terminal meaning
		// is immutable for the whole Core process.  This lookup intentionally
		// precedes every other schema/value/evidence rejection: otherwise a late
		// retry that changes (for example) tag_id, producer or SHA could publish a
		// second contradictory terminal result for an already-committed token.
		if(root["transaction_token"].empty() || !root["transaction_token"].isString())
		{
			reject("transaction token is missing or has an invalid JSON type");
			return;
		}
		root["transaction_token"] >> transactionToken;
		if(!safeToken(transactionToken))
		{
			reject("transaction token failed syntax validation");
			return;
		}
		std::map<std::string, std::pair<std::string, std::string> >::const_iterator terminal =
				safeRelocalizationTerminalTokens_.find(transactionToken);
		if(terminal != safeRelocalizationTerminalTokens_.end())
		{
			if(terminal->second.first != msg->data)
			{
				// A token has one immutable terminal meaning.  Never publish a
				// second, contradictory rejection for a token whose committed ACK
				// may merely have been lost.  Replay the cached terminal first so
				// the observer converges, then fail-stop on the collision.
				if(!terminal->second.second.empty() && safeRelocalizationStatusPub_)
				{
					std_msgs::msg::String replay;
					replay.data = terminal->second.second;
					safeRelocalizationStatusPub_->publish(replay);
				}
				RCLCPP_FATAL(this->get_logger(),
						"Safe relocalization transaction token collided with different raw proposal bytes; replayed the original terminal outcome and stopping");
				safeRelocalizationFatal_.store(true, std::memory_order_release);
				rclcpp::shutdown();
			}
			else if(terminal->second.second.empty())
			{
				RCLCPP_FATAL(this->get_logger(),
						"Safe relocalization token is reserved without a terminal outcome; stopping instead of reporting an ambiguous rejection");
				safeRelocalizationFatal_.store(true, std::memory_order_release);
				rclcpp::shutdown();
			}
			else if(safeRelocalizationStatusPub_)
			{
				std_msgs::msg::String replay;
				replay.data = terminal->second.second;
				safeRelocalizationStatusPub_->publish(replay);
			}
			return;
		}
		const std::set<std::string> rootKeys = {
				"schema_version", "producer", "session_token", "transaction_token", "tag_id",
				"source_reset_epoch", "observer_graph_revision", "anchor_database_sha256",
				"created_monotonic_s", "evidence", "accepted_images", "corner_observations"};
		if(!hasExactMapKeys(root, rootKeys))
		{
			reject("proposal must be one strict twelve-field JSON object with accepted images and Tag corners");
			return;
		}
		int schemaVersion = 0;
		if(!root["schema_version"].isInt() || !root["producer"].isString() ||
				!root["session_token"].isString() || !root["transaction_token"].isString() ||
				!root["tag_id"].isInt() || !root["anchor_database_sha256"].isString())
		{
			reject("schema, producer, token, tag and database SHA have invalid JSON types");
			return;
		}
		root["schema_version"] >> schemaVersion;
		root["producer"] >> producer;
		root["session_token"] >> sessionToken;
		root["tag_id"] >> tagId;
		root["anchor_database_sha256"] >> anchorDatabaseSha256;
		if((!root["created_monotonic_s"].isInt() && !root["created_monotonic_s"].isReal()) ||
				!parseUnsignedJson(root["source_reset_epoch"], sourceEpoch) ||
				!parseUnsignedJson(root["observer_graph_revision"], observerGraphRevision))
		{
			reject("epoch, revision or monotonic timestamp has an invalid type");
			return;
		}
		root["created_monotonic_s"] >> createdMonotonic;
		if(schemaVersion != 3 || producer != "wbrov_apriltag_safe_relocalizer" ||
				tagId < 0 || !std::isfinite(createdMonotonic) ||
				sessionToken.size() != 64 ||
				sessionToken.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos ||
				anchorDatabaseSha256.size() != 64 ||
				anchorDatabaseSha256.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos)
		{
			reject("proposal envelope failed schema, producer, token, tag, time or SHA validation");
			return;
		}
		std::string normalizedProposalSha = anchorDatabaseSha256;
		std::string normalizedSessionToken = sessionToken;
		std::transform(normalizedProposalSha.begin(), normalizedProposalSha.end(), normalizedProposalSha.begin(),
				[](unsigned char c){return (char)std::tolower(c);});
		std::transform(normalizedSessionToken.begin(), normalizedSessionToken.end(), normalizedSessionToken.begin(),
				[](unsigned char c){return (char)std::tolower(c);});
		const double nowMonotonic = std::chrono::duration<double>(
				std::chrono::steady_clock::now().time_since_epoch()).count();
		if(!safeRelocalizationDatabaseBindingValid_ ||
				normalizedSessionToken != safeRelocalizationSessionToken_ ||
				normalizedProposalSha != safeRelocalizationDatabaseSha256_)
		{
			reject("proposal does not match the bound session or database");
			return;
		}
		// Once the unguessable session secret and frozen database binding are
		// authenticated, reserve the token before checking volatile epoch,
		// revision or age.  A normal RGB-D update racing this callback therefore
		// produces one token-bound rejection which the observer can consume and
		// retry, instead of an empty-token diagnostic that times out the session.
		if(safeRelocalizationTerminalTokens_.size() >= 4096)
		{
			RCLCPP_FATAL(this->get_logger(),
					"Safe relocalization session terminal-token capacity reached; restart instead of evicting immutable outcomes");
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
			return;
		}
		std::pair<std::map<std::string, std::pair<std::string, std::string> >::iterator, bool> reservation =
				safeRelocalizationTerminalTokens_.insert(std::make_pair(
						transactionToken, std::make_pair(msg->data, std::string())));
		if(!reservation.second)
		{
			RCLCPP_FATAL(this->get_logger(), "Safe relocalization terminal reservation invariant failed");
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
			return;
		}
		terminalReservation = &reservation.first->second;
		trustedEnvelope = true;
		if(sourceEpoch != rtabmap_.getResetEpoch() ||
				observerGraphRevision != safeRelocalizationGraphRevision_ ||
				createdMonotonic > nowMonotonic+0.25 ||
				nowMonotonic-createdMonotonic > safeRelocalizationTimeoutSec_)
		{
			reject("proposal is stale or does not match current epoch and graph revision");
			return;
		}
		if(safeRelocalizationSeenTokens_.size() >= 256)
		{
			reject("transaction token capacity reached for this reset epoch");
			return;
		}
		if(!safeRelocalizationSeenTokens_.insert(transactionToken).second)
		{
			RCLCPP_FATAL(this->get_logger(),
					"Safe relocalization per-epoch token ledger conflicts with a new terminal reservation");
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
			return;
		}

        const cv::FileNode imagesNode = root["accepted_images"];
        if(!imagesNode.isSeq() || imagesNode.size() != 6)
        { reject("accepted_images must contain exactly six complete Core receipts"); return; }
        for(const auto & node : imagesNode)
        {
            rtabmap::AcceptedImageReceipt receipt;
            if(!readAcceptedImageReceipt(node, receipt))
            { reject("invalid accepted image receipt schema or exact identity"); return; }
            acceptedImages.push_back(receipt);
        }
		const cv::FileNode cornersNode = root["corner_observations"];
		if(!cornersNode.isSeq() || cornersNode.size()!=6)
		{
			reject("corner_observations must contain exactly six accepted-image bindings");
			return;
		}
		std::set<int> cornerSignatureIds;
		for(cv::FileNodeIterator iter=cornersNode.begin(); iter!=cornersNode.end(); ++iter)
		{
			const std::set<std::string> cornerKeys = {
					"signature_id", "tag_id", "corner_order", "pixel_record_sha256",
					"image_sha256", "corners_xy"};
			if(!hasExactMapKeys(*iter,cornerKeys) || !(*iter)["signature_id"].isInt() ||
					!(*iter)["tag_id"].isInt() || !(*iter)["corner_order"].isString() ||
					!(*iter)["pixel_record_sha256"].isString() || !(*iter)["image_sha256"].isString())
			{
				reject("each Tag corner observation must be a strict six-field object");
				return;
			}
			rtabmap::AcceptedTagCornerObservation observation;
			(*iter)["signature_id"] >> observation.signatureId;
			(*iter)["tag_id"] >> observation.tagId;
			(*iter)["corner_order"] >> observation.cornerOrder;
			(*iter)["pixel_record_sha256"] >> observation.pixelRecordSha256;
			(*iter)["image_sha256"] >> observation.imageSha256;
			std::vector<double> values;
			if(observation.signatureId<=0 || observation.tagId!=tagId ||
					observation.cornerOrder!="opencv_aruco_canonical_0_1_2_3" ||
					!safeToken(observation.pixelRecordSha256) || !safeToken(observation.imageSha256) ||
					!readFiniteSequence((*iter)["corners_xy"],8,values) ||
					!cornerSignatureIds.insert(observation.signatureId).second)
			{
				reject("Tag corner identity, order, digest or coordinates are invalid");
				return;
			}
			for(int i=0; i<8; ++i)
			{
				observation.cornersXY[i]=values[i];
			}
			acceptedCorners.push_back(observation);
		}
		const cv::FileNode evidenceNode = root["evidence"];
		if(!evidenceNode.isSeq() || evidenceNode.size() != 3)
		{
			reject("evidence must contain exactly three bridge objects");
			return;
		}
		for(cv::FileNodeIterator iter=evidenceNode.begin(); iter!=evidenceNode.end(); ++iter)
		{
			const std::set<std::string> bridgeKeys = {
					"from_id", "to_id", "translation", "quaternion_xyzw", "information_row_major"};
			if(!hasExactMapKeys(*iter, bridgeKeys) ||
					!(*iter)["from_id"].isInt() || !(*iter)["to_id"].isInt())
			{
				reject("each bridge must be a strict five-field object with integer IDs");
				return;
			}
			int fromId = 0;
			int toId = 0;
			(*iter)["from_id"] >> fromId;
			(*iter)["to_id"] >> toId;
			std::vector<double> translation;
			std::vector<double> quaternion;
			std::vector<double> informationValues;
			if(fromId <= 0 || toId <= 0 || fromId == toId ||
					!readFiniteSequence((*iter)["translation"], 3, translation) ||
					!readFiniteSequence((*iter)["quaternion_xyzw"], 4, quaternion) ||
					!readFiniteSequence((*iter)["information_row_major"], 36, informationValues))
			{
				reject("bridge IDs, transform or 6x6 row-major information are invalid");
				return;
			}
			const double quaternionNorm = std::sqrt(
					quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] +
					quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3]);
			if(std::fabs(quaternionNorm-1.0) > 1e-3)
			{
				reject("bridge quaternion is not unit length");
				return;
			}
			for(std::vector<double>::const_iterator value=translation.begin();
					value!=translation.end(); ++value)
			{
				if(std::fabs(*value) > std::numeric_limits<float>::max())
				{
					reject("bridge translation cannot be represented by Transform");
					return;
				}
			}
			cv::Mat information;
			if(!makeSafeInformationMatrix(
					informationValues,
					safeRelocalizationMaxInformationCondition_,
					information,
					transactionReason))
			{
				reject(transactionReason);
				return;
			}
			evidence.push_back(rtabmap::Link(
					fromId, toId, rtabmap::Link::kUserClosure,
					rtabmap::Transform(
							(float)translation[0], (float)translation[1], (float)translation[2],
							(float)quaternion[0], (float)quaternion[1], (float)quaternion[2], (float)quaternion[3]),
					information));
		}
	}
	catch(const cv::Exception & exception)
	{
		if(trustedEnvelope && !terminalReservation)
		{
			RCLCPP_FATAL(this->get_logger(),
					"Safe relocalization failed after trusting an envelope but before reserving its terminal token: %s",
					exception.what());
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
			return;
		}
		reject(std::string("JSON parse failed: ")+exception.what());
		return;
	}
	catch(const std::exception & exception)
	{
		if(trustedEnvelope && !terminalReservation)
		{
			RCLCPP_FATAL(this->get_logger(),
					"Safe relocalization failed after trusting an envelope but before reserving its terminal token: %s",
					exception.what());
			safeRelocalizationFatal_.store(true, std::memory_order_release);
			rclcpp::shutdown();
			return;
		}
		reject(std::string("proposal parse failed: ")+exception.what());
		return;
	}

	rtabmap::Rtabmap::SafeRelocalizationLimits limits;
	limits.maxHoldoutTranslation = (float)safeRelocalizationMaxTranslation_;
	limits.maxHoldoutRotation = (float)safeRelocalizationMaxRotation_;
	limits.minBridgeTranslation = (float)safeRelocalizationMinBridgeTranslation_;
	limits.minBridgeRotation = (float)safeRelocalizationMinBridgeRotation_;
	limits.maxAnchorZ = (float)safeRelocalizationMaxAnchorZ_;
	limits.maxAnchorTilt = (float)safeRelocalizationMaxAnchorTilt_;
	limits.maxOldPoseTranslation = (float)safeRelocalizationMaxOldPoseTranslation_;
	limits.maxOldPoseRotation = (float)safeRelocalizationMaxOldPoseRotation_;
	limits.maxNewRigidTranslation = (float)safeRelocalizationMaxNewRigidTranslation_;
	limits.maxNewRigidRotation = (float)safeRelocalizationMaxNewRigidRotation_;
	limits.maxNewRigidZError = (float)safeRelocalizationMaxNewRigidZError_;
	limits.maxNewRigidTiltError = (float)safeRelocalizationMaxNewRigidTiltError_;
	limits.maxGravityResidualIncrease = (float)safeRelocalizationMaxGravityResidualIncrease_;
	limits.maxInformationCondition = safeRelocalizationMaxInformationCondition_;
	bool committed = false;
	try
	{
        {
            // Authentication callbacks use auth mutexes before the lost/recovery
            // state mutex. The processing callback group excludes other graph
            // mutations. Hold both kinds of state until Core finishes preflight
            // and commit; never call a helper which reacquires these mutexes.
            std::scoped_lock authLocks(safeRelocalizationProposalAuthMutex_, resetQuarantineRgbdAuthMutex_,
                                       resetQuarantineOdomAuthMutex_, resetQuarantineOdomInfoAuthMutex_);
            UScopeMutex stateLock(resetQuarantineStateMutex_);
            auto endpointCurrent = [&](const std::string & topic, const std::string & expectedNode,
                    const std::string & expectedNamespace, bool bound, bool compromised,
                    const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid) {
                if(!bound || compromised) return false;
                const auto publishers = this->get_publishers_info_by_topic(topic);
                return publishers.size() == 1 && publishers.front().node_name() == expectedNode &&
                    publishers.front().node_namespace() == expectedNamespace &&
                    publishers.front().endpoint_gid().size() == gid.size() &&
                    std::equal(gid.begin(), gid.end(), publishers.front().endpoint_gid().begin());
            };
            const bool sourcesCurrent =
                endpointCurrent(safeRelocalizationProposalSub_->get_topic_name(), safeRelocalizationExpectedProposalNode_,
                    safeRelocalizationExpectedProposalNamespace_, safeRelocalizationProposalPublisherBound_,
                    safeRelocalizationProposalPublisherCompromised_, safeRelocalizationProposalPublisherGid_) &&
                endpointCurrent(resetQuarantineRgbSub_->get_topic_name(), resetQuarantineExpectedSensorNode_,
                    resetQuarantineExpectedSensorNamespace_, resetQuarantineRgbPublisherBound_,
                    resetQuarantineRgbPublisherCompromised_, resetQuarantineRgbPublisherGid_) &&
                endpointCurrent(resetQuarantineDepthSub_->get_topic_name(), resetQuarantineExpectedSensorNode_,
                    resetQuarantineExpectedSensorNamespace_, resetQuarantineDepthPublisherBound_,
                    resetQuarantineDepthPublisherCompromised_, resetQuarantineDepthPublisherGid_) &&
                endpointCurrent(resetQuarantineCameraInfoSub_->get_topic_name(), resetQuarantineExpectedSensorNode_,
                    resetQuarantineExpectedSensorNamespace_, resetQuarantineCameraInfoPublisherBound_,
                    resetQuarantineCameraInfoPublisherCompromised_, resetQuarantineCameraInfoPublisherGid_) &&
                endpointCurrent(resetQuarantineOdomSub_->get_topic_name(), resetQuarantineExpectedOdomInfoNode_,
                    resetQuarantineExpectedOdomNamespace_, resetQuarantineOdomPublisherBound_,
                    resetQuarantineOdomPublisherCompromised_, resetQuarantineOdomPublisherGid_) &&
                endpointCurrent(resetQuarantineOdomInfoSub_->get_topic_name(), resetQuarantineExpectedOdomInfoNode_,
                    resetQuarantineExpectedOdomNamespace_, resetQuarantineOdomInfoPublisherBound_,
                    resetQuarantineOdomInfoPublisherCompromised_, resetQuarantineOdomInfoPublisherGid_);
            const double currentMonotonic = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            const double odomAge = currentMonotonic - double(resetQuarantineLastOdomInfoSteadyNs_.load(std::memory_order_acquire))/1e9;
            if(!sourcesCurrent)
                transactionReason = "managed source is absent replaced or duplicated at image receipt commit";
            else if(!resetQuarantineEnabled_ || safeRelocalizationFatal_.load(std::memory_order_acquire) ||
                    !safeRelocalizationDatabaseBindingValid_ || !resetQuarantineOdomInfoSeen_.load(std::memory_order_acquire) ||
                    resetQuarantineLostStreak_.load(std::memory_order_acquire) != 0 ||
                    resetQuarantineRecoveryPending_.load(std::memory_order_acquire) ||
                    resetQuarantineSuspectedReset_.load(std::memory_order_acquire) ||
                    odomAge < 0 || odomAge > resetQuarantineOdomInfoMaxAgeSec_)
                transactionReason = "quarantine is not ready for image receipt commit";
            else if(sourceEpoch != rtabmap_.getResetEpoch() || observerGraphRevision != safeRelocalizationGraphRevision_ ||
                    currentMonotonic-createdMonotonic > safeRelocalizationTimeoutSec_ || createdMonotonic > currentMonotonic+.25)
                transactionReason = "image receipt proposal expired or changed epoch while waiting for commit";
            else
            {
                auto hex = [](const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid) {
                    std::string text;for(uint8_t byte : gid) text += uFormat("%02x", static_cast<unsigned int>(byte));return text;
                };
                rtabmap::AcceptedImageSourceContext context;
                context.sessionTokenSha256 = safeSha256(safeRelocalizationSessionToken_);
                context.databaseSha256 = safeRelocalizationDatabaseSha256_;
                context.rgbPublisherGidHex = hex(resetQuarantineRgbPublisherGid_);
                context.infoPublisherGidHex = hex(resetQuarantineCameraInfoPublisherGid_);
                context.provenanceScope = "managed_ros_rgbd";
                const bool expectedCamera = std::all_of(acceptedImages.begin(), acceptedImages.end(), [&](const rtabmap::AcceptedImageReceipt & receipt) {
                    return receipt.input.sensorFrame == resetQuarantineExpectedSensorFrame_;
                });
                if(!expectedCamera) transactionReason = "image receipt camera frame differs from the managed sensor";
                else committed = rtabmap_.addDepthCornerLinksWithAcceptedImagesAtomically(
                    evidence, sourceEpoch, limits, acceptedImages, acceptedCorners, context, transactionReason);
            }
        }
		if(committed)
		{
		// Revision belongs to the graph commit itself, not to later display-cache
		// work.  Reserve and materialize the exact terminal response before any
		// operation which may throw, so an exact retransmit can never be answered
		// as an ambiguous "already consumed" rejection while this Core is alive.
			++safeRelocalizationGraphRevision_;
		}
		terminalStatus = publishSafeRelocalizationStatus(
				committed?"committed":"rejected", transactionReason,
				transactionToken, tagId, sourceEpoch, observerGraphRevision, evidence, false);
		terminalReservation->second = terminalStatus;
		if(committed)
		{
			mapToOdomMutex_.lock();
			mapToOdom_ = rtabmap_.getMapCorrection();
			mapToOdomMutex_.unlock();
			publishResetEpoch();
			mapsManager_.clear();
			std::map<int, rtabmap::Transform> cachePoses = mapsManager_.updateMapCaches(
					rtabmap_.getLocalOptimizedPoses(), rtabmap_.getMemory(), false, false);
			mapsManager_.publishMaps(cachePoses, now(), mapFrameId_);
		}
	}
	catch(const std::exception & exception)
	{
		if(!terminalStatus.empty() && safeRelocalizationStatusPub_)
		{
			std_msgs::msg::String cached;
			cached.data = terminalStatus;
			safeRelocalizationStatusPub_->publish(cached);
		}
		if(terminalStatus.empty())
		{
			RCLCPP_FATAL(this->get_logger(),
					"Safe relocalization transaction threw before a terminal outcome could be cached: %s; stopping fail-closed",
					exception.what());
		}
		else
		{
			RCLCPP_FATAL(this->get_logger(),
					"Safe relocalization graph outcome is cached but post-commit state/map publication failed: %s; stopping fail-closed",
					exception.what());
		}
		safeRelocalizationFatal_.store(true, std::memory_order_release);
		rclcpp::shutdown();
		return;
	}
	if(safeRelocalizationStatusPub_)
	{
		std_msgs::msg::String terminalMessage;
		terminalMessage.data = terminalStatus;
		safeRelocalizationStatusPub_->publish(terminalMessage);
	}
}

void CoreWrapper::getNodesInRadiusCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::GetNodesInRadius::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::GetNodesInRadius::Response> res)
{
	RCLCPP_INFO(get_logger(), "Get nodes in radius (%f): node_id=%d pose=(%f,%f,%f)", req->radius, req->node_id, req->x, req->y, req->z);
	std::map<int, Transform> poses;
	std::map<int, float> dists;
	if(req->node_id != 0 || (req->x == 0.0f && req->y == 0.0f && req->z == 0.0f))
	{
		poses = rtabmap_.getNodesInRadius(req->node_id, req->radius, req->k, &dists);
	}
	else
	{
		poses = rtabmap_.getNodesInRadius(Transform(req->x, req->y, req->z, 0,0,0), req->radius, req->k, &dists);
	}

	//Optimized graph
	res->ids.resize(poses.size());
	res->poses.resize(poses.size());
	res->dists_sqr.resize(poses.size());
	int index = 0;
	for(std::map<int, rtabmap::Transform>::const_iterator iter = poses.begin();
		iter != poses.end();
		++iter)
	{
		res->ids[index] = iter->first;
		rtabmap_conversions::transformToPoseMsg(iter->second, res->poses[index]);
		UASSERT(dists.find(iter->first) != dists.end());
		res->dists_sqr[index] = dists.at(iter->first);
		++index;
	}
}

void CoreWrapper::publishStats(const rclcpp::Time & stamp)
{
	UDEBUG("Publishing stats...");
	const rtabmap::Statistics & stats = rtabmap_.getStatistics();

	if(infoPub_->get_subscription_count())
	{
		//RCLCPP_INFO(this->get_logger(), "Sending RtabmapInfo msg (last_id=%d)...", stat.refImageId());
		rtabmap_msgs::msg::Info::UniquePtr msg(new rtabmap_msgs::msg::Info);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		rtabmap_conversions::infoToROS(stats, *msg);
		// Loop/Map_id/ describes a loop target and is -1 before any closure.
		// Publish the reference frame's own map explicitly for the managed bridge.
		msg->stats_keys.push_back("WBROV/Reference_map_id/");
		msg->stats_values.push_back(static_cast<float>(stats.refImageMapId()));
		infoPub_->publish(std::move(msg));
	}

	if(mapDataPub_->get_subscription_count())
	{
		rtabmap_msgs::msg::MapData::UniquePtr msg(new rtabmap_msgs::msg::MapData);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		rtabmap_conversions::mapDataToROS(
			stats.poses(),
			stats.constraints(),
			stats.getSignaturesData(),
			stats.mapCorrection(),
			*msg);

		mapDataPub_->publish(std::move(msg));
	}

	if(mapGraphPub_->get_subscription_count())
	{
		if(mapsManager_.isMapUpdated())
		{
			graphLatched_ = false;
		}
		if(!(mapsManager_.isLatching() && graphLatched_))
		{
			rtabmap_msgs::msg::MapGraph::UniquePtr msg(new rtabmap_msgs::msg::MapGraph);
			msg->header.stamp = stamp;
			msg->header.frame_id = mapFrameId_;

			rtabmap_conversions::mapGraphToROS(
				stats.poses(),
				stats.constraints(),
				stats.mapCorrection(),
				*msg);

			mapGraphPub_->publish(std::move(msg));
			graphLatched_ = mapsManager_.isLatching();
		}
		// else we already published the latched graph
	}
	else
	{
		graphLatched_ = false;
	}

	if(odomCachePub_->get_subscription_count())
	{
		rtabmap_msgs::msg::MapGraph::UniquePtr msg(new rtabmap_msgs::msg::MapGraph);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		// For visualization of the constraints (MapGraph rviz plugin), we should include target nodes from the map
		std::map<int, Transform> poses = stats.odomCachePoses();
		// transform in map frame
		for(std::map<int, Transform>::iterator iter=poses.begin();
			iter!=poses.end();
			++iter)
		{
			iter->second = stats.mapCorrection() * iter->second;
		}
		for(std::multimap<int, rtabmap::Link>::const_iterator iter=stats.odomCacheConstraints().begin();
			iter!=stats.odomCacheConstraints().end();
			++iter)
		{
			std::map<int, Transform>::const_iterator pter = stats.poses().find(iter->second.to());
			if(pter != stats.poses().end())
			{
				poses.insert(*pter);
			}
		}
		rtabmap_conversions::mapGraphToROS(
			poses,
			stats.odomCacheConstraints(),
			stats.mapCorrection(),
			*msg);

		odomCachePub_->publish(std::move(msg));
	}

	if(localGridObstacle_->get_subscription_count() && !stats.getLastSignatureData().sensorData().gridObstacleCellsRaw().empty())
	{
		pcl::PCLPointCloud2::Ptr cloud = rtabmap::util3d::laserScanToPointCloud2(LaserScan::backwardCompatibility(stats.getLastSignatureData().sensorData().gridObstacleCellsRaw()));
		sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2);
		pcl_conversions::moveFromPCL(*cloud, *msg);
		msg->header.stamp = stamp;
		msg->header.frame_id = frameId_;
		localGridObstacle_->publish(std::move(msg));
	}
	if(localGridEmpty_->get_subscription_count() && !stats.getLastSignatureData().sensorData().gridEmptyCellsRaw().empty())
	{
		pcl::PCLPointCloud2::Ptr cloud = rtabmap::util3d::laserScanToPointCloud2(LaserScan::backwardCompatibility(stats.getLastSignatureData().sensorData().gridEmptyCellsRaw()));
		sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2);
		pcl_conversions::moveFromPCL(*cloud, *msg);
		msg->header.stamp = stamp;
		msg->header.frame_id = frameId_;
		localGridEmpty_->publish(std::move(msg));
	}
	if(localGridGround_->get_subscription_count() && !stats.getLastSignatureData().sensorData().gridGroundCellsRaw().empty())
	{
		pcl::PCLPointCloud2::Ptr cloud = rtabmap::util3d::laserScanToPointCloud2(LaserScan::backwardCompatibility(stats.getLastSignatureData().sensorData().gridGroundCellsRaw()));
		sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2);
		pcl_conversions::moveFromPCL(*cloud, *msg);
		msg->header.stamp = stamp;
		msg->header.frame_id = frameId_;
		localGridGround_->publish(std::move(msg));
	}

	bool pubLabels = labelsPub_->get_subscription_count();
	visualization_msgs::msg::MarkerArray markers;
	if((landmarksPub_->get_subscription_count() || pubLabels) && !stats.poses().empty() && stats.poses().begin()->first < 0)
	{
		geometry_msgs::msg::PoseArray::UniquePtr msg(new geometry_msgs::msg::PoseArray);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;
		for(std::map<int, Transform>::const_iterator iter=stats.poses().begin(); iter!=stats.poses().end() && iter->first<0; ++iter)
		{
			geometry_msgs::msg::Pose p;
			rtabmap_conversions::transformToPoseMsg(iter->second, p);
			msg->poses.push_back(p);

			if(pubLabels)
			{
				// Add landmark ids
				visualization_msgs::msg::Marker marker;
				marker.header.frame_id = mapFrameId_;
				marker.header.stamp = stamp;
				marker.ns = "landmarks";
				marker.id = iter->first;
				marker.action = visualization_msgs::msg::Marker::ADD;
				marker.pose.position.x = iter->second.x();
				marker.pose.position.y = iter->second.y();
				marker.pose.position.z = iter->second.z();
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 1;
				marker.scale.y = 1;
				marker.scale.z = 0.35;
				marker.color.a = 0.7;
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				marker.lifetime = rclcpp::Duration::from_seconds(2.0f/rate_);

				marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
				marker.text = uNumber2Str(iter->first);

				markers.markers.push_back(marker);
			}
		}

		landmarksPub_->publish(std::move(msg));
	}

	bool pubPath = mapPathPub_->get_subscription_count();
	if(pubLabels || pubPath)
	{
		if(stats.poses().size())
		{
			nav_msgs::msg::Path path;
			if(pubPath)
			{
				// Ignore pose of current location in Localization mode
				path.poses.resize(stats.poses().size()-(rtabmap_.getMemory()->isIncremental()?0:1));
			}
			int oi = 0;
			for(std::map<int, Transform>::const_iterator poseIter=stats.poses().begin();
				poseIter!=stats.poses().end();
				++poseIter)
			{
				if(pubLabels && rtabmap_.getMemory())
				{
					// Add labels
					std::map<int, std::string>::const_iterator lter = rtabmap_.getMemory()->getAllLabels().find(poseIter->first);
					if(lter != rtabmap_.getMemory()->getAllLabels().end() && !lter->second.empty())
					{
						visualization_msgs::msg::Marker marker;
						marker.header.frame_id = mapFrameId_;
						marker.header.stamp = stamp;
						marker.ns = "labels";
						marker.id = -poseIter->first;
						marker.action = visualization_msgs::msg::Marker::ADD;
						marker.pose.position.x = poseIter->second.x();
						marker.pose.position.y = poseIter->second.y();
						marker.pose.position.z = poseIter->second.z();
						marker.pose.orientation.x = 0.0;
						marker.pose.orientation.y = 0.0;
						marker.pose.orientation.z = 0.0;
						marker.pose.orientation.w = 1.0;
						marker.scale.x = 1;
						marker.scale.y = 1;
						marker.scale.z = 0.5;
						marker.color.a = 0.7;
						marker.color.r = 1.0;
						marker.color.g = 0.0;
						marker.color.b = 0.0;

						marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
						marker.text = lter->second;

						markers.markers.push_back(marker);
					}

					// Add node ids
					visualization_msgs::msg::Marker marker;
					marker.header.frame_id = mapFrameId_;
					marker.header.stamp = stamp;
					marker.ns = "ids";
					marker.id = poseIter->first;
					marker.action = visualization_msgs::msg::Marker::ADD;
					marker.pose.position.x = poseIter->second.x();
					marker.pose.position.y = poseIter->second.y();
					marker.pose.position.z = poseIter->second.z();
					marker.pose.orientation.x = 0.0;
					marker.pose.orientation.y = 0.0;
					marker.pose.orientation.z = 0.0;
					marker.pose.orientation.w = 1.0;
					marker.scale.x = 1;
					marker.scale.y = 1;
					marker.scale.z = 0.2;
					marker.color.a = 0.5;
					marker.color.r = 1.0;
					marker.color.g = 1.0;
					marker.color.b = 1.0;
					marker.lifetime = rclcpp::Duration::from_seconds(2.0f/rate_);

					marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
					marker.text = uNumber2Str(poseIter->first);

					markers.markers.push_back(marker);
				}
				if(pubPath && (rtabmap_.getMemory()->isIncremental() || poseIter->first != stats.poses().rbegin()->first))
				{
					rtabmap_conversions::transformToPoseMsg(poseIter->second, path.poses.at(oi).pose);
					path.poses.at(oi).header.frame_id = mapFrameId_;
					path.poses.at(oi).header.stamp = stamp;
					++oi;
				}
			}

			if(pubLabels && markers.markers.size())
			{
				labelsPub_->publish(markers);
			}
			if(pubPath && oi)
			{
				path.header.frame_id = mapFrameId_;
				path.header.stamp = stamp;
				path.poses.resize(oi);
				mapPathPub_->publish(path);
			}
		}
	}
}

void CoreWrapper::publishCurrentGoal(const rclcpp::Time & stamp)
{
	if(!currentMetricGoal_.isNull() && currentMetricGoal_ != lastPublishedMetricGoal_)
	{
		RCLCPP_INFO(this->get_logger(), "Publishing next goal: %d -> %s",
				rtabmap_.getPathCurrentGoalId(), currentMetricGoal_.prettyPrint().c_str());

		geometry_msgs::msg::PoseStamped poseMsg;
		poseMsg.header.frame_id = mapFrameId_;
		poseMsg.header.stamp = stamp;
		rtabmap_conversions::transformToPoseMsg(currentMetricGoal_, poseMsg.pose);
#ifdef WITH_NAV2_MSGS
		if(useActionForGoal_)
		{
			if(nav2Client_.get() == NULL || !nav2Client_->action_server_is_ready())
			{
				RCLCPP_INFO(this->get_logger(), "Connecting to navigate_to_pose action server...");
				if(nav2Client_.get() == NULL)
				{
					nav2Client_ = rclcpp_action::create_client<NavigateToPose>(
					      this,
					      "navigate_to_pose");
				}
				if (!nav2Client_->wait_for_action_server(std::chrono::duration<double>(5.0))) {
				  RCLCPP_ERROR(this->get_logger(), " navigate_to_pose action server not available after waiting 5 seconds");
				}
			}
			if(nav2Client_.get() != NULL && nav2Client_->action_server_is_ready())
			{
				NavigateToPose::Goal goal_msg;
				goal_msg.pose = poseMsg;

				auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
				send_goal_options.goal_response_callback = std::bind(&CoreWrapper::goalResponseCallback, this, std::placeholders::_1);
				send_goal_options.result_callback = std::bind(&CoreWrapper::resultCallback, this, std::placeholders::_1);
				nav2Client_->async_send_goal(goal_msg, send_goal_options);
				lastPublishedMetricGoal_ = currentMetricGoal_;
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Cannot connect to navigate_to_pose action server!");
			}
		}
		else
#endif
		if(nextMetricGoalPub_->get_subscription_count())
		{
			nextMetricGoalPub_->publish(poseMsg);
			lastPublishedMetricGoal_ = currentMetricGoal_;
		}
	}
}
#ifdef WITH_NAV2_MSGS
void CoreWrapper::goalResponseCallback(
#ifdef NAV_MSGS_FOXY
		std::shared_future<GoalHandleNav2::SharedPtr> future)
{
        auto goal_handle = future.get();
#else
        const GoalHandleNav2::SharedPtr & goal_handle)
{
#endif
	if (!goal_handle) {
		RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
		rtabmap_.clearPath(1);
		currentMetricGoal_.setNull();
		lastPublishedMetricGoal_.setNull();
		goalFrameId_.clear();
		latestNodeWasReached_ = false;
	} else {
		RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
		lastGoalSent_ = goal_handle->get_goal_id();
	}
}

void CoreWrapper::resultCallback(
		const GoalHandleNav2::WrappedResult & result)
{
	bool ignore = false;
	if(!currentMetricGoal_.isNull())
	{
		if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
		{
			if(rtabmap_.getPath().size() &&
				rtabmap_.getPathCurrentGoalId() != rtabmap_.getPath().back().first &&
				(!uContains(rtabmap_.getLocalOptimizedPoses(), rtabmap_.getPath().back().first) || !latestNodeWasReached_))
			{
				RCLCPP_WARN(this->get_logger(), "Planning: nav2 reached current goal but it is not "
						 "the last one planned by rtabmap. A new goal should be sent when "
						 "rtabmap will be able to retrieve next locations on the path.");
				ignore = true;
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "Planning: nav2 success!");
			}
		}
		else if(result.code==rclcpp_action::ResultCode::ABORTED && result.goal_id != lastGoalSent_)
		{
			// Just ignored, it is from an old goal
			ignore = true;
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Planning: nav2 failed for some reason: %s. Aborting the plan...",
					result.code==rclcpp_action::ResultCode::ABORTED?"Aborted":
					result.code==rclcpp_action::ResultCode::CANCELED?"Canceled":"Unkown");
		}

		if(!ignore && goalReachedPub_->get_subscription_count())
		{
			std_msgs::msg::Bool resultMsg;
			resultMsg.data = result.code == rclcpp_action::ResultCode::SUCCEEDED;
			goalReachedPub_->publish(resultMsg);
		}
	}

	if(!ignore)
	{
		rtabmap_.clearPath(1);
		currentMetricGoal_.setNull();
		lastPublishedMetricGoal_.setNull();
		goalFrameId_.clear();
		latestNodeWasReached_ = false;
	}
}
#endif

void CoreWrapper::publishLocalPath(const rclcpp::Time & stamp)
{
	if(rtabmap_.getPath().size())
	{
		std::vector<std::pair<int, Transform> > poses = rtabmap_.getPathNextPoses();
		if(poses.size())
		{
			if(localPathPub_->get_subscription_count() || localPathNodesPub_->get_subscription_count())
			{
				nav_msgs::msg::Path path;
				rtabmap_msgs::msg::Path pathNodes;
				path.header.frame_id = pathNodes.header.frame_id = mapFrameId_;
				path.header.stamp = pathNodes.header.stamp = stamp;
				path.poses.resize(poses.size());
				pathNodes.node_ids.resize(poses.size());
				pathNodes.poses.resize(poses.size());
				int oi = 0;
				for(std::vector<std::pair<int, Transform> >::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					path.poses[oi].header = path.header;
					rtabmap_conversions::transformToPoseMsg(iter->second, path.poses[oi].pose);
					pathNodes.poses[oi] = path.poses[oi].pose;
					pathNodes.node_ids[oi] = iter->first;
					++oi;
				}
				if(localPathPub_->get_subscription_count())
				{
					localPathPub_->publish(path);
				}
				if(localPathNodesPub_->get_subscription_count())
				{
					localPathNodesPub_->publish(pathNodes);
				}
			}
		}
	}
}

void CoreWrapper::publishGlobalPath(const rclcpp::Time & stamp)
{
	if((globalPathPub_->get_subscription_count() || globalPathNodesPub_->get_subscription_count()) && rtabmap_.getPath().size())
	{
		Transform pose = uValue(rtabmap_.getLocalOptimizedPoses(), rtabmap_.getPathCurrentGoalId(), Transform());
		if(!pose.isNull() && rtabmap_.getPathCurrentGoalIndex() < rtabmap_.getPath().size())
		{
			// transform the global path in the goal referential
			Transform t = pose * rtabmap_.getPath().at(rtabmap_.getPathCurrentGoalIndex()).second.inverse();

			nav_msgs::msg::Path path;
			rtabmap_msgs::msg::Path pathNodes;
			path.header.frame_id = pathNodes.header.frame_id = mapFrameId_;
			path.header.stamp = pathNodes.header.stamp = stamp;
			path.poses.resize(rtabmap_.getPath().size());
			pathNodes.node_ids.resize(rtabmap_.getPath().size());
			pathNodes.poses.resize(rtabmap_.getPath().size());
			int oi = 0;
			for(std::vector<std::pair<int, Transform> >::const_iterator iter=rtabmap_.getPath().begin(); iter!=rtabmap_.getPath().end(); ++iter)
			{
				path.poses[oi].header = path.header;
				rtabmap_conversions::transformToPoseMsg(t*iter->second, path.poses[oi].pose);
				pathNodes.poses[oi] = path.poses[oi].pose;
				pathNodes.node_ids[oi] = iter->first;
				++oi;
			}
			Transform goalLocalTransform = Transform::getIdentity();
			if(!goalFrameId_.empty() && goalFrameId_.compare(frameId_) != 0)
			{
				Transform localT = rtabmap_conversions::getTransform(frameId_, goalFrameId_, stamp, *tfBuffer_, waitForTransform_);
				if(!localT.isNull())
				{
					goalLocalTransform = localT.inverse().to3DoF();
				}
			}

			if(!rtabmap_.getPathTransformToGoal().isIdentity() || !goalLocalTransform.isIdentity())
			{
				path.poses.resize(path.poses.size()+1);
				path.poses[path.poses.size()-1].header = path.header;
				pathNodes.node_ids.resize(pathNodes.node_ids.size()+1);
				pathNodes.poses.resize(pathNodes.poses.size()+1);
				Transform p = t * rtabmap_.getPath().back().second*rtabmap_.getPathTransformToGoal() * goalLocalTransform;
				rtabmap_conversions::transformToPoseMsg(p, path.poses[path.poses.size()-1].pose);
				pathNodes.poses[pathNodes.poses.size()-1] = path.poses[path.poses.size()-1].pose;
				pathNodes.node_ids[pathNodes.node_ids.size()-1] = 0;
			}
			if(globalPathPub_->get_subscription_count())
			{
				globalPathPub_->publish(path);
			}
			if(globalPathNodesPub_->get_subscription_count())
			{
				globalPathNodesPub_->publish(pathNodes);
			}
		}
	}
}

CoreWrapper::LocalizationStatusTask::LocalizationStatusTask() :
		diagnostic_updater::DiagnosticTask("Localization status"),
		localizationThreshold_(0.0),
		localizationError_(9999)
{}

void CoreWrapper::LocalizationStatusTask::setLocalizationThreshold(double value)
{
	localizationThreshold_ = value;
}

void CoreWrapper::LocalizationStatusTask::updateStatus(const cv::Mat & cov, bool twoDMapping)
{
	if(localizationThreshold_ > 0.0 && !cov.empty())
	{
		if(cov.at<double>(0,0) >= 9999.0)
		{
			localizationError_ = 9999.0;
		}
		else
		{
			localizationError_ = sqrt(uMax3(cov.at<double>(0,0), cov.at<double>(1,1), twoDMapping?0.0:cov.at<double>(2,2)));
		}
	}
}

void CoreWrapper::LocalizationStatusTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if(localizationError_>=9999)
	{
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not localized!");
	}
	else if(localizationError_ > localizationThreshold_)
	{
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Localization error is high!");
	}
	else
	{
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Localized.");
	}
	stat.add("Localization error (m)", localizationError_);
	stat.add("loc_thr (m)", localizationThreshold_);
}

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
void CoreWrapper::octomapBinaryCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>,
		std::shared_ptr<octomap_msgs::srv::GetOctomap::Response> res)
{
	RCLCPP_INFO(this->get_logger(), "Sending binary map data on service request");
	res->map.header.frame_id = mapFrameId_;
	res->map.header.stamp = now();

	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	if((mappingMaxNodes_ > 0 || mappingAltitudeDelta_>0.0) && poses.size()>1)
	{
		poses = filterNodesToAssemble(poses, poses.rbegin()->second);
	}

	mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), false, true);

	const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
	if(octomap->octree()->size()>0)
	{
		octomap_msgs::binaryMapToMsg(*octomap->octree(), res->map);
	}
}

void CoreWrapper::octomapFullCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>,
		std::shared_ptr<octomap_msgs::srv::GetOctomap::Response> res)
{
	RCLCPP_INFO(this->get_logger(), "Sending full map data on service request");
	res->map.header.frame_id = mapFrameId_;
	res->map.header.stamp = now();

	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	if((mappingMaxNodes_ > 0 || mappingAltitudeDelta_>0.0) && poses.size()>1)
	{
		poses = filterNodesToAssemble(poses, poses.rbegin()->second);
	}

	mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), false, true);

	const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
	if(octomap->octree()->size()>0)
	{
		octomap_msgs::fullMapToMsg(*octomap->octree(), res->map);
	}
}
#endif
#endif


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_slam::CoreWrapper)
