
#ifndef IMAGE2RTSP_ENCODER
#define IMAGE2RTSP_ENCODER

#include <ros/ros.h>

#include <queue>
#include <memory>

extern "C"{
#include <x264.h>
#include <libswscale/swscale.h>
#include <libavcodec/avcodec.h>
}

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
#define av_frame_alloc  avcodec_alloc_frame
#endif

#include <image2rtsp/buffer.hpp>

namespace i2r{
namespace enc{
    
class Encoder{
public:
    Encoder(i2r::util::Buffer<x264_nal_t>& buffer): m_encoder(nullptr), m_numNals(0), m_pts(0), m_buffer(buffer) {
        memset((char *)&m_picRaw, 0, sizeof(m_picRaw));
    }

    ~Encoder(){
        if (*m_encoder)
            x264_encoder_close(*m_encoder);

        if (*m_sws)
            sws_freeContext(*m_sws);
        
        memset((char *)&m_picRaw, 0, sizeof(m_picRaw));
    }

    bool open(const int& srcWidth, const int& srcHeight, const int& dstWidth, const int& dstHeight, const int& fps){
        m_dstWidth = dstWidth;
        m_dstHeight = dstHeight;

        {
            x264_param_default_preset(&m_x264Params, "veryfast", "zerolatency");
            m_x264Params.i_log_level = X264_LOG_ERROR;

            m_x264Params.i_threads = 2;
            m_x264Params.i_width = srcWidth;
            m_x264Params.i_height = srcHeight;
            m_x264Params.i_fps_num = fps;
            m_x264Params.i_fps_den = 1;
            m_x264Params.i_csp = X264_CSP_I420;
            
            m_x264Params.i_keyint_max = fps;
            m_x264Params.b_intra_refresh = 1;
            
            m_x264Params.rc.i_rc_method = X264_RC_CRF;
            m_x264Params.rc.i_vbv_buffer_size = 100;
            m_x264Params.rc.i_vbv_max_bitrate = 1000;
            m_x264Params.rc.f_rf_constant = 25;
            m_x264Params.rc.f_rf_constant_max = 35;
            m_x264Params.i_sps_id = 7;
            
            m_x264Params.b_repeat_headers = 1;
            m_x264Params.b_annexb = 1;

            x264_param_apply_profile(&m_x264Params, "baseline");

            m_in.i_type = X264_TYPE_AUTO;
            m_in.img.i_csp = X264_CSP_I420;
        }

        if(m_encoder){
            ROS_ERROR("Already opened. first call close()");
            return false;
        }

        m_encoder = std::make_shared<x264_t*>(x264_encoder_open(&m_x264Params));
        if (!m_encoder){
            ROS_ERROR("Cannot open the encoder");
            return false;
        }

        if(x264_picture_alloc(&m_in, m_x264Params.i_csp, m_dstWidth, m_dstHeight)){
            ROS_ERROR("Cannot allocate x264 picure");
            return false;
        }

        m_sws = std::make_shared<SwsContext*>(sws_getContext(m_x264Params.i_width, m_x264Params.i_height, AV_PIX_FMT_RGB24, m_dstWidth, m_dstHeight, AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL));
        if (!*m_sws){
            ROS_ERROR("Cannot create SWS context");
            return false;
        }

        return true;
    }

    bool encoding(const uint8_t* src){
        int srcStride = m_x264Params.i_width * 3;
        int dstStride[4] = {m_dstWidth, m_dstWidth >> 1, m_dstWidth >> 1, 0};
        
        {
            if (!*m_sws){
                ROS_ERROR("Not initialized, so cannot encode");
                return false;
            }
        
            int h = sws_scale(*m_sws, &src, &srcStride, 0, m_x264Params.i_height, m_in.img.plane, dstStride);
            if (h != m_dstHeight){
                ROS_ERROR("scale failed: %d", h);
                return false;
            }
        
            m_in.i_pts = m_pts;
        }
        
        {                
            int frame_size = x264_encoder_encode(*m_encoder, &m_nals, &m_numNals, &m_in, &m_out);
            if(frame_size){        
                
                static bool alreadydone = false;
                if(!alreadydone){
                    x264_encoder_headers(*m_encoder, &m_nals, &m_numNals);
                    alreadydone = true;
                }

                for(int i = 0 ; i < m_numNals ; i++)
                    m_buffer.Push(m_nals[i]);
            }
            else
                return false;
            
            m_pts++;
        }

        return true;
    }

private:
    // x264
    x264_param_t m_x264Params;
    std::shared_ptr<x264_t*> m_encoder;
    x264_nal_t* m_nals;

    x264_picture_t m_in;
    x264_picture_t m_out;

    std::shared_ptr<SwsContext*> m_sws;
    AVPicture m_picRaw;

    int m_numNals;
    int m_pts;

    int m_dstWidth;
    int m_dstHeight;

    // buffer
    i2r::util::Buffer<x264_nal_t>& m_buffer;
};

} // i2r
} // enc

#endif