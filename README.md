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
sudo apt-get install x264 libswscale-dev libavformat-dev libavcodec-dev libavutil-dev
```

After install library, should modify "image2rtsp/CMakeLists.txt" to your development envaroment.

### ***Supported color channel***

- ROS Image\
-- Only Supported RGB8, BGR8, RGBA8

### ***Caution!!***

I tested Only my development enviroments. \
This project has some of bug. So I try fix it continuously.

### History

- fix some of bug

- modify class struct

- Set multi session

- Initial version implement
