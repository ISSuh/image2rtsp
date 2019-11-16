
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

#include <image2rtsp/buffer.hpp>

namespace i2r{
namespace enc{
    
class Encoder{
public:
    Encoder(i2r::util::Buffer<x264_nal_t>& buffer): m_encoder(nullptr), m_inPixelFormat(AV_PIX_FMT_NONE), m_outPixelFormat(AV_PIX_FMT_NONE), m_numNals(0), m_pts(0), m_buffer(buffer){
        memset((char *)&m_picRaw, 0, sizeof(m_picRaw));
    }

    ~Encoder(){
        if (*m_encoder)
            x264_encoder_close(*m_encoder);

        if (*m_sws)
            sws_freeContext(*m_sws);

        memset((char *)&m_picRaw, 0, sizeof(m_picRaw));
    }

    bool open(const int& width, const int& height,const int& fps){
        if(!validParam(width, height, fps))
            return false;

        {
            x264_param_default_preset(&m_x264Params, "ultrafast", "zerolatency");
            m_x264Params.i_threads = 1;
            m_x264Params.i_width = width;
            m_x264Params.i_height = height;
            m_x264Params.i_fps_num = fps;
            m_x264Params.i_fps_den = 1;
            m_x264Params.i_csp = X264_CSP_I420;
            
            // Intra refres:
            m_x264Params.i_keyint_max = 60;
            m_x264Params.b_intra_refresh = 1;
            
            //Rate control:
            m_x264Params.rc.i_rc_method = X264_RC_CRF;
            m_x264Params.rc.f_rf_constant = 25;
            m_x264Params.rc.f_rf_constant_max = 35;
            
            //For streaming:
            m_x264Params.b_repeat_headers = 1;
            m_x264Params.b_annexb = 1;            
            x264_param_apply_profile(&m_x264Params, "baseline");

            m_in.i_type = X264_TYPE_AUTO;
            m_in.img.i_csp = X264_CSP_I420;
            m_in.img.i_plane = 3;

            m_inPixelFormat = AV_PIX_FMT_RGB24;
            m_outPixelFormat = AV_PIX_FMT_YUV420P;
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

        if(x264_picture_alloc(&m_in, m_x264Params.i_csp, m_x264Params.i_width, m_x264Params.i_height)){
            ROS_ERROR("Cannot allocate x264 picure");
            return false;
        }

        m_sws = std::make_shared<SwsContext*>(sws_getContext(m_x264Params.i_width, m_x264Params.i_height, m_inPixelFormat,
                        m_x264Params.i_width, m_x264Params.i_height, m_outPixelFormat,
                        SWS_FAST_BILINEAR, NULL, NULL, NULL));

        if (!*m_sws){
            ROS_ERROR("Cannot create SWS context");
            return false;
        }

        return true;
    }

    bool encoding(const uint8_t* src){
        int srcStride = m_x264Params.i_width * 3;

        if (!*m_sws){
            ROS_ERROR("Not initialized, so cannot encode");
            return false;
        }

        if (!avpicture_fill(&m_picRaw, src, m_inPixelFormat, m_x264Params.i_width, m_x264Params.i_height)){
            ROS_ERROR("Cannot fill the raw input buffer");
            return false;
        }
        
        int h = sws_scale(*m_sws, m_picRaw.data, &srcStride, 0, m_x264Params.i_height, m_in.img.plane, m_in.img.i_stride);
        if (h != m_x264Params.i_height){
            ROS_ERROR("scale failed: %d", h);
            return false;
        }

        m_in.i_pts = m_pts;
                
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

        return true;
    }

private:
    void setParam(const int& width, const int& height,const int& fps){
        x264_param_default_preset(&m_x264Params, "ultrafast", "zerolatency");
        
        m_x264Params.i_threads = 1;
        m_x264Params.i_width = width;
        m_x264Params.i_height = height;
        m_x264Params.i_fps_num = fps;
        m_x264Params.i_fps_den = 1;
        m_x264Params.i_csp = X264_CSP_I420;
        
        // Intra refres:
        m_x264Params.i_keyint_max = fps;
        m_x264Params.b_intra_refresh = 1;
        
        //Rate control:
        m_x264Params.rc.i_rc_method = X264_RC_CRF;
        m_x264Params.rc.f_rf_constant = 25;
        m_x264Params.rc.f_rf_constant_max = 35;
        
        //For streaming:
        m_x264Params.b_repeat_headers = 1;
        m_x264Params.b_annexb = 1;

        m_x264Params.i_log_level = X264_LOG_ERROR;
        
        x264_param_apply_profile(&m_x264Params, "baseline");

        m_in.i_type = X264_TYPE_AUTO;
        m_in.img.i_csp = X264_CSP_I420;

        m_inPixelFormat = AV_PIX_FMT_RGB24;
        m_outPixelFormat = AV_PIX_FMT_YUV420P;
    }

    bool validParam(const int& width, const int& height,const int& fps){
        if (width <= 0){
            ROS_ERROR("No width set");
            return false;
        }
        
        if (height <= 0){
            ROS_ERROR("No height set");
            return false;
        }

        if (fps <= 0){
            ROS_ERROR("No fps set");
            return false;
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

    AVPixelFormat m_inPixelFormat;
    AVPixelFormat m_outPixelFormat;
    AVPicture m_picRaw;

    std::shared_ptr<SwsContext*> m_sws;

    int m_numNals;
    int m_pts;

    // buffer
    i2r::util::Buffer<x264_nal_t>& m_buffer;
};

} // i2r
} // enc

#endif