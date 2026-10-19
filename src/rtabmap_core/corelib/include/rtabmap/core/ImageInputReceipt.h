#pragma once
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/CameraModel.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
namespace rtabmap {
namespace image_receipt_detail {
inline uint32_t safeRotateRight(uint32_t value, unsigned int count) { return (value >> count) | (value << (32U-count)); }
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

} // namespace image_receipt_detail
// This is an accepted-input receipt, not a hash of JPEG-compressed node data.
// Only the CoreWrapper friend can submit managed input to Rtabmap's private API.
struct ImageInputIdentity
{
    std::int64_t headerStampNs = 0;
    int width = 0;
    int height = 0;
    std::string encoding;
    std::string pixelSha256;
    std::array<double, 9> K{};
    std::vector<double> D;
    std::string sensorFrame;
    std::string rgbPublisherGidHex;
    std::string infoPublisherGidHex;
    std::string sessionTokenSha256;
    std::string databaseSha256;
    std::string provenanceScope;
};

struct AcceptedImageReceipt
{
    int signatureId = 0;
    int mapId = -1;
    unsigned long long resetEpoch = 0;
    ImageInputIdentity input;
    std::array<double, 12> bodyFromCamera{};
};

// Ordered detector corners bound by the managed proposal publisher to one
// accepted image receipt. Core reopens its own Signature depth and camera model
// to obtain metric points; nominal Tag size and the detector's PnP pose are not
// used by the graph transaction.
struct AcceptedTagCornerObservation
{
    int signatureId = 0;
    int tagId = -1;
    std::string cornerOrder;
    std::string pixelRecordSha256;
    std::string imageSha256;
    std::array<double, 8> cornersXY{};
};

// Filled from the wrapper's managed source state, never from an untrusted
// proposal's self-reported source fields. Historical databases without receipts
// need a separate saved-node provenance path; they cannot mint runtime receipts.
struct AcceptedImageSourceContext
{
    std::string sessionTokenSha256;
    std::string databaseSha256;
    std::string rgbPublisherGidHex;
    std::string infoPublisherGidHex;
    std::string provenanceScope;
};

inline bool sameAcceptedImageReceipt(const AcceptedImageReceipt & a, const AcceptedImageReceipt & b)
{
    const auto & x = a.input; const auto & y = b.input;
    return a.signatureId == b.signatureId && a.mapId == b.mapId && a.resetEpoch == b.resetEpoch &&
        a.bodyFromCamera == b.bodyFromCamera && x.headerStampNs == y.headerStampNs &&
        x.width == y.width && x.height == y.height && x.encoding == y.encoding &&
        x.pixelSha256 == y.pixelSha256 && x.K == y.K && x.D == y.D && x.sensorFrame == y.sensorFrame &&
        x.rgbPublisherGidHex == y.rgbPublisherGidHex && x.infoPublisherGidHex == y.infoPublisherGidHex &&
        x.sessionTokenSha256 == y.sessionTokenSha256 && x.databaseSha256 == y.databaseSha256 &&
        x.provenanceScope == y.provenanceScope;
}

inline bool receiptHex(const std::string & value, size_t size)
{
    if(value.size() != size) return false;
    for(char c : value) if(!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    return true;
}

// Hash the same logical pixel rows, excluding stride padding. The compiled
// implementation uses the platform SHA-256 provider; no receipt is cached and
// every caller still validates its current bytes. Empty means invalid input or
// a digest-provider failure and cannot satisfy a valid 64-character receipt.
RTABMAP_CORE_EXPORT std::string canonicalImageSha256(const cv::Mat & image);

inline bool receiptCameraMatches(const CameraModel & camera, const ImageInputIdentity & input)
{
    if(!camera.isValidForProjection() || camera.imageWidth() != input.width || camera.imageHeight() != input.height) return false;
    const cv::Mat K = camera.K();
    const cv::Mat D = camera.D();
    if(K.rows != 3 || K.cols != 3 || K.type() != CV_64FC1) return false;
    for(int i = 0; i < 9; ++i)
        if(!std::isfinite(input.K[i]) || K.at<double>(i/3, i%3) != input.K[i]) return false;
    // The current managed stream is rectified. Other distortion models require
    // an explicit conversion and cannot be silently marked pixel-equivalent.
    for(double value : input.D) if(!std::isfinite(value) || value != 0.0) return false;
    if(!D.empty())
    {
        if(D.type() != CV_64FC1) return false;
        for(int r = 0; r < D.rows; ++r) for(int c = 0; c < D.cols; ++c)
            if(!std::isfinite(D.at<double>(r,c)) || D.at<double>(r,c) != 0.0) return false;
    }
    return true;
}

inline bool receiptIdentityMatches(const SensorData & data, const ImageInputIdentity & input, std::string & reason)
{
    const cv::Mat & image = data.imageRaw();
    const size_t gidSize = input.rgbPublisherGidHex.size();
    if(input.headerStampNs <= 0 || !std::isfinite(data.stamp()) ||
       data.stamp() != static_cast<double>(input.headerStampNs) / 1e9)
        { reason = "integer_header_and_sensor_stamp_mismatch"; return false; }
    if(input.provenanceScope != "managed_ros_rgbd" && input.provenanceScope != "offline_native_fixture" &&
       input.provenanceScope != "verified_recording_rgbd")
        { reason = "unsupported_provenance_scope"; return false; }
    if(!receiptHex(input.pixelSha256, 64) || !receiptHex(input.sessionTokenSha256, 64) ||
       input.sessionTokenSha256 == std::string(64, '0') || !receiptHex(input.databaseSha256, 64) ||
       input.sensorFrame.empty() || input.sensorFrame.size() > 256 || gidSize < 32 || gidSize > 64 || gidSize % 2 ||
       !receiptHex(input.rgbPublisherGidHex, gidSize) || !receiptHex(input.infoPublisherGidHex, gidSize) ||
       input.rgbPublisherGidHex == std::string(gidSize, '0') || input.infoPublisherGidHex == std::string(gidSize, '0'))
        { reason = "missing_source_session_or_database_identity"; return false; }
    if(image.empty() || image.cols != input.width || image.rows != input.height ||
       !((input.encoding == "bgr8" && image.type() == CV_8UC3) || (input.encoding == "mono8" && image.type() == CV_8UC1)) ||
       canonicalImageSha256(image) != input.pixelSha256)
        { reason = "canonical_input_pixel_mismatch"; return false; }
    if(data.cameraModels().size() != 1 || !data.stereoCameraModels().empty() ||
       !receiptCameraMatches(data.cameraModels().front(), input))
        { reason = "input_camera_model_mismatch"; return false; }
    reason.clear();
    return true;
}

} // namespace rtabmap
