#include <signal.h>
#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <image2rtsp/encode.hpp>
#include <image2rtsp/rtsp.hpp>
#include <image2rtsp/buffer.hpp>

class Image2RTSP_sample{
public:
	Image2RTSP_sample() : m_width(0), m_height(0), m_fps(0) {
		m_nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));

		m_nh->getParam("subTopic", m_subTopic);
		m_nh->getParam("width", m_width);
		m_nh->getParam("height", m_height);
        m_nh->getParam("fps", m_fps);
		m_nh->getParam("streamUrl", m_streamUrl);
		m_nh->getParam("port", m_serverPort);
	}

	bool Init(){
		m_encoder = std::unique_ptr<i2r::enc::Encoder>(new i2r::enc::Encoder(m_buffer));
        if(!m_encoder->open(m_width, m_height, m_fps))
			return false;

		m_rtsp = std::unique_ptr<i2r::net::Rtsp>(new i2r::net::Rtsp(m_serverPort, m_streamUrl, m_subTopic, m_buffer));
		if(!m_rtsp->Init())
			return false;

		m_rtsp->AddSession();

		m_sub = m_nh->subscribe(m_subTopic, 10, &Image2RTSP_sample::callback, this);

		m_thread = std::unique_ptr<std::thread>(new std::thread(&Image2RTSP_sample::ServerRun, this));

		return true;
	}

	void ServerRun(){
		m_rtsp->Play();

		m_rtsp->DoEvent();
	}

private:
	void callback(const sensor_msgs::Image::ConstPtr &msg){
        std::vector<x264_nal_t> h264Image;

		// ros image to h264 stream
		try{
        	m_encoder->encoding(&(msg->data[0]));
		}
		catch(int e){
			std::cout << "encode error - " << e << std::endl;
		}
	}

private:
    // ros
	std::unique_ptr<ros::NodeHandle> m_nh;
	ros::Subscriber m_sub;
	std::string m_subTopic;
    
    // encoder
    std::unique_ptr<i2r::enc::Encoder> m_encoder;
	std::unique_ptr<i2r::net::Rtsp> m_rtsp;
	i2r::util::Buffer<x264_nal_t> m_buffer;

	std::unique_ptr<std::thread> m_thread;

    // param
	int m_width;
	int m_height;
    int m_fps;
	std::string m_streamUrl;
	int m_serverPort;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "image2rtsp");

	Image2RTSP_sample sample;
	
	if(!sample.Init())
		ros::shutdown();

	ros::spin();
}