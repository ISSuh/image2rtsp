#ifndef IMAGE2RTSP_RTSP
#define IMAGE2RTSP_RTSP

#include <memory>
#include <thread>

#include <ros/ros.h>

#include <live555/BasicUsageEnvironment.hh>
#include <live555/GroupsockHelper.hh>
#include <live555/liveMedia.hh>

#include <image2rtsp/rosTaskScheduler.hpp>
#include <image2rtsp/rosImageSource.hpp>

namespace i2r{
namespace net{

extern EventTriggerId RosImageSource::eventTriggerId = 0;

class Rtsp{
public:
    Rtsp(const int port, const std::string& streamName, const std::string& subTopic, i2r::util::Buffer<x264_nal_t>& buffer) :  
                                    m_rtpPortNum(18888), m_rtcpPortNum(m_rtpPortNum+1), m_ttl(255),
                                    m_rtpPort(m_rtpPortNum), m_rtcpPort(m_rtcpPortNum),
                                    m_rtpPayloadFormat(96), m_estimatedSessionBandwidth(500),
                                    m_rtspPort(port), m_streamName(streamName), m_subTopic(subTopic),
                                    m_buffer(buffer){
        m_scheduler = std::unique_ptr<TaskScheduler>(BasicTaskScheduler::createNew());
        m_env = BasicUsageEnvironment::createNew(*m_scheduler);
        m_authDB = std::unique_ptr<UserAuthenticationDatabase>(nullptr);
    }

    ~Rtsp() {}

    bool Init(){
        m_destinationAddress.s_addr = chooseRandomIPv4SSMAddress(*m_env);

        m_rtpGroupSock = std::unique_ptr<Groupsock>(new Groupsock(*m_env, m_destinationAddress, m_rtpPort, m_ttl));
        m_rtpGroupSock->multicastSendOnly();
        m_rtcpGroupSock = std::unique_ptr<Groupsock>(new Groupsock(*m_env, m_destinationAddress, m_rtpPort, m_ttl));
        m_rtpGroupSock->multicastSendOnly();
        
        OutPacketBuffer::maxSize = 100000;
        
        m_videoSink = H264VideoRTPSink::createNew(*m_env, m_rtpGroupSock.get(), 103);

        int cNameLen = 100;
        m_cName.resize(cNameLen + 1, 0);
        gethostname((char*)&(m_cName[0]), cNameLen);

        m_rtcp =  RTCPInstance::createNew(*m_env, m_rtcpGroupSock.get(),
			    m_estimatedSessionBandwidth, &(m_cName[0]), m_videoSink, NULL, True);

        m_rtspServer = RTSPServer::createNew(*m_env, m_rtspPort, m_authDB.get());
        if(m_rtspServer == nullptr){
            ROS_ERROR("Failed to create RTSP server: %s", m_env->getResultMsg());
            return false;
        }

        return true;
    }

    void AddSession(){
        m_sms = ServerMediaSession::createNew(*m_env, m_streamName.c_str(), 
                "ROS_IMAGE", "Session streamed ROS IMAGE", True );
        m_sms->addSubsession(PassiveServerMediaSubsession::createNew(*m_videoSink, m_rtcp));
        m_rtspServer->addServerMediaSession(m_sms);

        char* url = m_rtspServer->rtspURL(m_sms);
        *m_env << "Play this stream using the URL \"" << url << "\"\n";
        delete[] url;
    }

    void Play(){

        m_rosImageSource = i2r::net::RosImageSource::createNew(*m_env, m_buffer, 0, 0);
        m_videoES = m_rosImageSource;

        m_videoSource = H264VideoStreamFramer::createNew(*m_env, m_videoES);
        
        m_videoSink->startPlaying(*m_videoSource, NULL, m_videoSink);
    }


    void DoEvent(){
          m_env->taskScheduler().doEventLoop();
    }

private:
    void FinishPlay(void* ){
        m_videoSink->stopPlaying();
        Medium::close(m_videoSource);
    }

private:
    // live555
    std::unique_ptr<TaskScheduler> m_scheduler;
    UsageEnvironment* m_env;
    std::unique_ptr<UserAuthenticationDatabase> m_authDB;

    RTSPServer* m_rtspServer;

    struct in_addr m_destinationAddress;
    
    const unsigned short m_rtpPortNum;
    const unsigned short m_rtcpPortNum;
    const unsigned char m_ttl;
    const Port m_rtpPort;
    const Port m_rtcpPort;
    
    std::unique_ptr<Groupsock> m_rtpGroupSock;
    std::unique_ptr<Groupsock> m_rtcpGroupSock;
    
    std::vector<unsigned char> m_cName;
    const unsigned m_estimatedSessionBandwidth;
    RTCPInstance* m_rtcp;

    RTPSink* m_videoSink;
    ServerMediaSession* m_sms;
    FramedSource* m_videoES;

    i2r::net::RosImageSource* m_rosImageSource; 

    H264VideoStreamFramer* m_videoSource;

    // param
    int m_rtspPort;
    std::string m_streamName;
    std::string m_subTopic;
    unsigned char m_rtpPayloadFormat;

    //buffer
    i2r::util::Buffer<x264_nal_t>& m_buffer;

    std::unique_ptr<std::thread> m_evnetThread;
};

} // net
} // i2r

#endif
