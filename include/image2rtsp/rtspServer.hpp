#ifndef IMAGE2RTSP_RTSP
#define IMAGE2RTSP_RTSP

#include <memory>
#include <vector>

#include <ros/ros.h>

#include <live555/BasicUsageEnvironment.hh>
#include <live555/GroupsockHelper.hh>
#include <live555/liveMedia.hh>

#include <image2rtsp/rosTaskScheduler.hpp>
#include <image2rtsp/rosImageSource.hpp>

extern "C"{
#include <x264.h>
}

#include <image2rtsp/buffer.hpp>

namespace i2r{
namespace net{

extern EventTriggerId RosImageSource::eventTriggerId = 0;

class RosRtspServer{
public:
    RosRtspServer(const int port, const int sessionNum, i2r::util::Buffer<x264_nal_t>* buffer) :  
            m_rtpPortNum(18888), m_rtcpPortNum(m_rtpPortNum+1), m_ttl(255),
            m_rtpPayloadFormat(96), m_estimatedSessionBandwidth(500),
            m_rtspPort(port), m_sessionNum(sessionNum), m_buffer(buffer){
        
        m_scheduler = std::unique_ptr<TaskScheduler>(BasicTaskScheduler::createNew());
        m_env = BasicUsageEnvironment::createNew(*m_scheduler);
        m_authDB = std::unique_ptr<UserAuthenticationDatabase>(nullptr);

        m_rtcp = std::vector<RTCPInstance*>(m_sessionNum, nullptr);
        m_videoSink = std::vector<RTPSink*>(m_sessionNum, nullptr);
        m_videoES = std::vector<FramedSource*>(m_sessionNum, nullptr);
        m_videoSource = std::vector<H264VideoStreamFramer*>(m_sessionNum, nullptr);
    }

    ~RosRtspServer() {}

    bool Init(){
        m_destinationAddress.s_addr = chooseRandomIPv4SSMAddress(*m_env);

        OutPacketBuffer::maxSize = 100000;
        
        // get hostname
        int cNameLen = 100;
        m_cName.resize(cNameLen + 1, 0);
        gethostname((char*)&(m_cName[0]), cNameLen);
        
        // rtp, rtcp handle create
        for(auto i = 0 ; i < m_sessionNum ; ++i){            
            const Port rtpPort(m_rtpPortNum + (i * 2));
            const Port rtcpPort(m_rtcpPortNum + (i * 2));

            auto rtpGroupSock = new Groupsock(*m_env, m_destinationAddress, rtpPort, m_ttl);
            m_rtpGroupSock->multicastSendOnly();
            
            auto rtcpGroupSock =new Groupsock(*m_env, m_destinationAddress, rtcpPort, m_ttl);
            m_rtpGroupSock->multicastSendOnly();
        
            m_videoSink[i] = H264VideoRTPSink::createNew(*m_env, rtpGroupSock, m_rtpPayloadFormat);

            m_rtcp[i] =  RTCPInstance::createNew(*m_env, rtcpGroupSock, m_estimatedSessionBandwidth, &(m_cName[0]), m_videoSink[i], NULL, True);
        }

        // server handle create
        m_rtspServer = RTSPServer::createNew(*m_env, m_rtspPort, m_authDB.get());
        if(m_rtspServer == nullptr){
            ROS_ERROR("Failed to create RTSP server: %s", m_env->getResultMsg());
            return false;
        }

        return true;
    }

    void AddSession(const std::string& streamName, const int index){
        m_sms = ServerMediaSession::createNew(*m_env, streamName.c_str(), "ROS_IMAGE", "Session streamed ROS IMAGE", True );
        m_sms->addSubsession(PassiveServerMediaSubsession::createNew(*m_videoSink[index], m_rtcp[index]));
        m_rtspServer->addServerMediaSession(m_sms);

        ROS_INFO("Play this stream using the URL %s",  m_rtspServer->rtspURL(m_sms));
    }

    void Play(const int index){
        auto rosImageSource = i2r::net::RosImageSource::createNew(*m_env, &m_buffer[index], 0, 0);
        m_videoES[index] = rosImageSource;

        m_videoSource[index] = H264VideoStreamFramer::createNew(*m_env, m_videoES[index]);
        
        m_videoSink[index]->startPlaying(*m_videoSource[index], NULL, &m_videoSink[index]);
    }

    void DoEvent(){
        m_env->taskScheduler().doEventLoop();
    }

private:
    void FinishPlay(void* ){
        for(auto i = 0 ; i < m_sessionNum ; ++i){
            m_videoSink[i]->stopPlaying();
            Medium::close(m_videoSource[i]);
        }
    }

private:
    // live555
    std::unique_ptr<TaskScheduler> m_scheduler;
    UsageEnvironment* m_env;
    std::unique_ptr<UserAuthenticationDatabase> m_authDB;

    RTSPServer* m_rtspServer;

    struct in_addr m_destinationAddress;
    
    const unsigned int m_rtpPortNum;
    const unsigned int m_rtcpPortNum;
    const unsigned char m_ttl;
    
    std::unique_ptr<Groupsock> m_rtpGroupSock;
    std::unique_ptr<Groupsock> m_rtcpGroupSock;
    
    std::vector<unsigned char> m_cName;
    const unsigned m_estimatedSessionBandwidth;
    std::vector<RTCPInstance*> m_rtcp;

    std::vector<RTPSink*> m_videoSink;
    ServerMediaSession* m_sms;
    std::vector<FramedSource*> m_videoES;

    std::vector<H264VideoStreamFramer*> m_videoSource;

    // param
    const int m_rtspPort;
    const unsigned char m_rtpPayloadFormat;
    const int m_sessionNum;

    //buffer
    i2r::util::Buffer<x264_nal_t>* m_buffer;
};

} // net
} // i2r

#endif
