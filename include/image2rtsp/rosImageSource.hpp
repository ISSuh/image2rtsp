#ifndef IMAGE2RTSP_ROSIMAGESOURCE
#define IMAGE2RTSP_ROSIMAGESOURCE

#include <live555/FramedSource.hh>

extern "C"{
#include <x264.h>
}

#include <image2rtsp/buffer.hpp>

namespace i2r{
namespace net{

class RosImageSource : public FramedSource{
public:
    static EventTriggerId eventTriggerId;
    
    static RosImageSource* createNew(UsageEnvironment& env, i2r::util::Buffer<x264_nal_t>& buffer, unsigned preferredFrameSize, unsigned playTimePerFrame){
        return new RosImageSource(env, buffer, preferredFrameSize, playTimePerFrame);
    }

protected:
    RosImageSource(UsageEnvironment& env, i2r::util::Buffer<x264_nal_t>& buffer, unsigned preferredFrameSize, unsigned playTimePerFrame) 
        :   FramedSource(env),
            fPreferredFrameSize(fMaxSize),
            fPlayTimePerFrame(playTimePerFrame),
            fLastPlayTime(0),
            m_buffer(buffer){
        if(m_referenceCount == 0)
        {

        }
        
        ++m_referenceCount;
        
        if(eventTriggerId == 0){
            eventTriggerId = envir().taskScheduler().createEventTrigger(deliverFrame0);
        }
    }
    
    virtual ~RosImageSource(void){
        --m_referenceCount;
        envir().taskScheduler().deleteEventTrigger(eventTriggerId);
        eventTriggerId = 0;
    }

private:
    virtual void doGetNextFrame(){
        deliverFrame();
    }
    
    static void deliverFrame0(void* clientData){
        ((RosImageSource*)clientData)->deliverFrame();
    }
    
    void deliverFrame(){
        if(!isCurrentlyAwaitingData()) return;
        
        if (fPlayTimePerFrame > 0 && fPreferredFrameSize > 0) {
            if (fPresentationTime.tv_sec == 0 && fPresentationTime.tv_usec == 0) {
                // This is the first frame, so use the current time:
                gettimeofday(&fPresentationTime, NULL);
            } 
            else {
                // Increment by the play time of the previous data:
                unsigned uSeconds	= fPresentationTime.tv_usec + fLastPlayTime;
                fPresentationTime.tv_sec += uSeconds/1000000;
                fPresentationTime.tv_usec = uSeconds%1000000;
            }

            fLastPlayTime = (fPlayTimePerFrame*fFrameSize)/fPreferredFrameSize;
            fDurationInMicroseconds = fLastPlayTime;
        } 
        else {
            gettimeofday(&fPresentationTime, NULL);
        }

        x264_nal_t nalToDeliver = m_buffer.WaitPop();

        unsigned newFrameSize = nalToDeliver.i_payload;

        if (newFrameSize > fMaxSize) {
            fFrameSize = fMaxSize;
            fNumTruncatedBytes = newFrameSize - fMaxSize;
        }
        else {
            fFrameSize = newFrameSize;
        }
        
        memcpy(fTo, nalToDeliver.p_payload, nalToDeliver.i_payload);
        FramedSource::afterGetting(this);
    }

private:
    unsigned fPreferredFrameSize;
    unsigned fPlayTimePerFrame;
    unsigned fNumSources;
    unsigned fCurrentlyReadSourceNumber;
    unsigned fLastPlayTime;

    unsigned m_referenceCount;
    i2r::util::Buffer<x264_nal_t>& m_buffer;
    timeval m_currentTime;
};

} // net
} // i2r

#endif