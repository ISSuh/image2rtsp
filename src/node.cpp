#include <signal.h>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <image2rtsp/encode.hpp>

class Image2RTSP_sample{
public:
	Image2RTSP_sample() : m_width(0), m_height(0), m_fps(0) {
		m_nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));

		m_nh->getParam("subTopic", m_subTopic);
		m_nh->getParam("width", m_width);
		m_nh->getParam("height", m_height);
        m_nh->getParam("fps", m_fps);

		m_sub = m_nh->subscribe(m_subTopic, 10, &Image2RTSP_sample::callback, this);

        m_encoder.open(m_width, m_height, m_fps);
	}

	void callback(const sensor_msgs::Image::ConstPtr &msg){
        std::vector<uint8_t> h264Image;
        m_encoder.encoding(&(msg->data[0]), h264Image);
	}

private:
    // ros
	std::unique_ptr<ros::NodeHandle> m_nh;
	ros::Subscriber m_sub;
	std::string m_subTopic;
    
    // encoder
    Encoder m_encoder;

    // param
	int m_width;
	int m_height;
    int m_fps;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "image2rtsp");

	Image2RTSP_sample sample;

	ros::spin();
}