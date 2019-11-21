#include <iostream>
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <image2rtsp/encode.hpp>
#include <image2rtsp/rtspServer.hpp>
#include <image2rtsp/buffer.hpp>

class Image2RTSP_sample{
public:
	Image2RTSP_sample() {
		m_nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));

		m_nh->getParam("port", m_serverPort);
		m_nh->getParam("sessionNumber", m_sessionNumber);

		m_srcWidth = std::vector<int>(m_sessionNumber, 0);
		m_srcHeight = std::vector<int>(m_sessionNumber, 0);
		m_dstWidth = std::vector<int>(m_sessionNumber, 0);
		m_dstHeight = std::vector<int>(m_sessionNumber, 0);
		m_fps = std::vector<int>(m_sessionNumber, 0);
		m_streamUrl = std::vector<std::string>(m_sessionNumber);
		m_subTopic = std::vector<std::string>(m_sessionNumber);
		
		m_buffer = std::vector<i2r::util::Buffer<x264_nal_t>>(m_sessionNumber);

		for(auto i = 0 ; i < m_sessionNumber ; ++i){
			std::string index = std::to_string(i);
			
			m_nh->getParam("subTopic_" 	+ index, m_subTopic[i]);
			m_nh->getParam("srcWidth_" 	+ index, m_srcWidth[i]);
			m_nh->getParam("srcHeight_" + index, m_srcHeight[i]);
			m_nh->getParam("dstWidth_" 	+ index, m_dstWidth[i]);
			m_nh->getParam("dstHeight_" + index, m_dstHeight[i]);
        	m_nh->getParam("fps_" 		+ index, m_fps[i]);
			m_nh->getParam("streamUrl_" + index, m_streamUrl[i]);
		}

		m_dstWidth = m_srcWidth;
		m_dstHeight = m_srcHeight;
	}

	bool Init(){
		// m_encoder = std::unique_ptr<i2r::enc::Encoder>(new i2r::enc::Encoder(m_buffer));
        // if(!m_encoder->open(m_width, m_height, m_fps))
		// 	return false;

		m_rtsp = std::unique_ptr<i2r::net::RosRtspServer>(new i2r::net::RosRtspServer(m_serverPort));
		if(!m_rtsp->Init())
			return false;

		for(auto& streamUrl : m_streamUrl)
			m_rtsp->AddSession(streamUrl);

		m_sub.push_back(m_nh->subscribe(m_subTopic[0], 10, &Image2RTSP_sample::Session_0_callback, this));
		m_sub.push_back(m_nh->subscribe(m_subTopic[1], 10, &Image2RTSP_sample::Session_1_callback, this));
		
		m_resize_sub = m_nh->subscribe("/image2rtsp/resize/commend", 10, &Image2RTSP_sample::resizeCmd, this);

		m_thread = std::unique_ptr<std::thread>(new std::thread(&Image2RTSP_sample::ServerRun, this));

		return true;
	}

	void ServerRun(){
		for(auto i = 0 ; i < m_sessionNumber ; ++i)
			m_rtsp->Play(i, &(m_buffer[i]));

		m_rtsp->DoEvent();
	}

private:
	void Session_0_callback(const sensor_msgs::Image::ConstPtr &msg){
		i2r::enc::Encoder enc(m_buffer[0]);
		if(!enc.open(m_srcWidth[0], m_srcHeight[0], m_dstWidth[0], m_dstHeight[0], m_fps[0]))
			return ;

		enc.encoding(&(msg->data[0]));
	}

	void Session_1_callback(const sensor_msgs::Image::ConstPtr &msg){
		i2r::enc::Encoder enc(m_buffer[1]);
		if(!enc.open(m_srcWidth[1], m_srcHeight[1], m_dstWidth[1], m_dstHeight[1], m_fps[1]))
			return ;

		enc.encoding(&(msg->data[0]));
	}

	void resizeCmd(const std_msgs::String::ConstPtr &msg){
		if(msg->data == "1"){
			m_dstWidth[0] = 640;
			m_dstHeight[0] = 480;
		}
		else if(msg->data == "2"){
			m_dstWidth[0]= 480;
			m_dstHeight[0] = 240;
		}
	}

private:
    // ros
	std::unique_ptr<ros::NodeHandle> m_nh;
	std::vector<ros::Subscriber> m_sub;
	ros::Subscriber m_resize_sub;
    
    // encoder
    // std::unique_ptr<i2r::enc::Encoder> m_encoder;
	std::unique_ptr<i2r::net::RosRtspServer> m_rtsp;
    std::vector<i2r::util::Buffer<x264_nal_t>> m_buffer;

	std::unique_ptr<std::thread> m_thread;

    // param
	int m_serverPort;
	int m_sessionNumber;

	std::vector<int> m_srcWidth;
	std::vector<int> m_srcHeight;
	std::vector<int> m_dstWidth;
	std::vector<int> m_dstHeight;
    std::vector<int> m_fps;
	std::vector<std::string> m_streamUrl;
	std::vector<std::string> m_subTopic;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "image2rtsp");

	Image2RTSP_sample sample;
	
	if(!sample.Init())
		ros::shutdown();

	ros::spin();
}