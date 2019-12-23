# Image2RTSP

## RTSP Server for ROS(ROS Image to RTSP stream)

----------------------------

### ***Overview***

This 'image2rtsp package' is basically ROS based project.

This 'image2rtsp package' is subscribed some of image topic(***sensor_msgs/Image***) from image publish node. After received image data, the node encodes the image to h264 and push buffer. Concurrently, Live555 event is emit when image push the buffer. Finally, ***RosImageSource*** can pop image data from buffer and stream image to session.

### ***Development Enviroments***

- Ubuntu 18.04 LTS
- ROS Melodic
- Live555
- x264
- swscale
- avformat
- avcodec
- avutil

### ***Install dependency library***

#### Install library related image encoding(encode to H264)

```bash
sudo apt-get install libx264-dev libswscale-dev libavformat-dev libavcodec-dev libavutil-dev
```

After install library, you must modify "image2rtsp/CMakeLists.txt" to your development.  For example try check your path of installed library or path of include file.

### ***Supported color channel***

- ROS Image\
-- Only Supported RGB8, BGR8, RGBA8

### ***How to Use?***

Befor compile it, add your ROS image callback as many as session num at 'image2rtsp/src/node.cpp ' line 61, 82

```C++

...

    // regist ros subscribe callback
    // regist your callback as many as your sessions
    {
        m_sub.push_back(m_nh->subscribe(m_subTopic[0], 10,
                        &Image2RTSP_sample::Session_0_callback, this));
        m_sub.push_back(m_nh->subscribe(m_subTopic[1], 10,
                        &Image2RTSP_sample::Session_1_callback, this));
    }

...

private:
    // write your callback as many as your sessions
    void Session_0_callback(const sensor_msgs::Image::ConstPtr &msg){
        /**
        * void StreamImage(const uint8_t* src, const int index)
        * 
        * @params src ROS image data.
        * @params index Session index numbers.
        **/
        m_rtsp->StreamImage(&(msg->data[0]), 0);
    }

    void Session_1_callback(const sensor_msgs::Image::ConstPtr &msg){
        m_rtsp->StreamImage(&(msg->data[0]), 1);
    }

    void ServerRun(){
        m_rtsp->DoEvent();
    }

...

```

After modify code on fit your enviroment, try fixing roslaunch file.

If there is an error related to linker(ld), you compile relate live555 and copy to 'image2rtsp/lib/your_arch'.
When you compile the live555, refer this script.

``` bash
# Download Live555 source code and conpile
wget http://www.live555.com/liveMedia/public/live555-latest.tar.gz
tar xvf live555-latest.tar.gz
cd live
./genMakefiles linux
makes

# Copy compiled ibrary
scp live/UsageEnvironment/libUsageEnvironment.a /path/to/image2rtsp/lib/your_arch
scp live/BasicUsageEnvironment/libBasicUsageEnvironment.a /path/to/image2rtsp/lib/your_arch
scp live/groupsock/libgroupsock.a /path/to/image2rtsp/lib/your_arch
scp live/liveMedia/libliveMedia.a.a /path/to/image2rtsp/lib/your_arch
```

### History

- Add arm64 compiled live555 library
- Add proxy server for external access
- Fix some of bug
- Modify class struct
- Set multi session
- Initial version implement
