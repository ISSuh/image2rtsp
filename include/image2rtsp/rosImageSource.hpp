#ifndef IMAGE2RTSP_ROSIMAGESOURCE
#define IMAGE2RTSP_ROSIMAGESOURCE

#include <live555/ByteStreamMemoryBufferSource.hh>

namespace i2r{
namespace net{

class RosImageSource{
public:
    RosImageSource(){}
    ~RosImageSource(){}
    
private:
    ByteStreamMemoryBufferSource* m_byteBurrerSource;
};

} // net
} // i2r

#endif