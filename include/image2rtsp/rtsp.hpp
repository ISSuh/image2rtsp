#ifndef IMAGE2RTSP_RTSP
#define IMAGE2RTSP_RTSP

#include <memory>

#include <ros/ros.h>

#include <live555/BasicUsageEnvironment/BasicUsageEnvironment.hh>
#include <live555/groupsock/Groupsock.hh>
#include <live555/groupsock/GroupsockHelper.hh>
#include <live555/liveMedia/liveMedia.hh>
#include <live555/UsageEnvironment/UsageEnvironment.hh>

namespace i2r{

namespace net{

// TODO : implement
class Rtsp{
public:
    Rtsp(const int port, const std::string& streamName) : m_rtspPort(port), m_streamName(streamName), 
                                    m_rtpPortNum(18888), m_rtcpPortNum(m_rtpPortNum+1), m_ttl(255),
                                    m_rtpPort(m_rtpPortNum), m_rtcpPort(m_rtcpPortNum),
                                    m_rtpPayloadFormat(96){
        m_scheduler = std::unique_ptr<TaskScheduler>(BasicTaskScheduler::createNew());
        m_env = std::unique_ptr<UsageEnvironment>(BasicUsageEnvironment::createNew(*m_scheduler));
        m_authDB = std::unique_ptr<UserAuthenticationDatabase>(nullptr);
    }

    bool Init(){
        m_destinationAddress.s_addr = chooseRandomIPv4SSMAddress(*m_env);

        m_rtpGroupSock = std::unique_ptr<Groupsock>(new Groupsock(*m_env, m_destinationAddress, m_rtpPort, m_ttl));
        m_rtpGroupSock->multicastSendOnly();
        m_rtcpGroupSock = std::unique_ptr<Groupsock>(new Groupsock(*m_env, m_destinationAddress, m_rtpPort, m_ttl));
        m_rtpGroupSock->multicastSendOnly();
        
        OutPacketBuffer::maxSize = 100000;


        m_rtspServer = std::unique_ptr<RTSPServer>(RTSPServer::createNew(*m_env, m_rtspPort, m_authDB.get()));
        if(m_rtspServer == nullptr){
            ROS_ERROR("Failed to create RTSP server: %s", m_env->getResultMsg());
            return false;
        }
    }

    ~Rtsp();

private:

private:
    // live555
    std::unique_ptr<TaskScheduler> m_scheduler;
    std::unique_ptr<UsageEnvironment> m_env;
    std::unique_ptr<UserAuthenticationDatabase> m_authDB;

    std::unique_ptr<RTSPServer> m_rtspServer;

    struct in_addr m_destinationAddress;
    
    const unsigned short m_rtpPortNum;
    const unsigned short m_rtcpPortNum;
    const unsigned char m_ttl;
    const Port m_rtpPort;
    const Port m_rtcpPort;
    
    std::unique_ptr<Groupsock> m_rtpGroupSock;
    std::unique_ptr<Groupsock> m_rtcpGroupSock;

    // param
    int m_rtspPort;
    std::string m_streamName;
    unsigned char m_rtpPayloadFormat;
};

} // net

} // i2r

#endif
