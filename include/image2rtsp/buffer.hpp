#ifndef IMAGE2RTSP_BUFFER
#define IMAGE2RTSP_BUFFER

#include <queue>
#include <mutex>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

namespace i2r{
namespace util{

template<typename Data>
class Buffer{
public:
    void Push(Data const& data){
        boost::mutex::scoped_lock lock(m_mutex);
        m_queue.push(data);
        lock.unlock();
        m_cv.notify_one();
    }

    bool Empty() const{
        boost::mutex::scoped_lock lock(m_mutex);
        return m_queue.empty();
    }

    bool Pop(Data& value){
        boost::mutex::scoped_lock lock(m_mutex);
        
        if(m_queue.empty())
            return false;
                
        value=m_queue.front();
        m_queue.pop();

        return true;
    }

    void WaitPop(Data& value){
        boost::mutex::scoped_lock lock(m_mutex);
        
        while(m_queue.empty())
            m_cv.wait(lock);
        
        value=m_queue.front();
        m_queue.pop();
    }

private:
    std::queue<Data> m_queue;
    mutable boost::mutex m_mutex;
    boost::condition_variable m_cv;
};

} // util
} // i2r

#endif