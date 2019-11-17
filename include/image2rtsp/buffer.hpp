#ifndef IMAGE2RTSP_BUFFER
#define IMAGE2RTSP_BUFFER

#include <queue>
#include <mutex>
#include <condition_variable>

namespace i2r{
namespace util{

template<typename Data>
class Buffer{
public:
    void Push(Data const& data){
        std::unique_lock<std::mutex> lock(m_mutex);
        m_queue.push(data);

        m_cv.notify_one();
    }

    bool Empty(){
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    const Data WaitPop(){
        std::unique_lock<std::mutex> lock(m_mutex);
        
        while(m_queue.empty())
            m_cv.wait(lock);
        
        auto value = m_queue.front();
        m_queue.pop();

        return value;
    }

    const size_t Size(){
        std:: unique_lock<std::mutex> lock(m_mutex);

        return m_queue.size();
    }
    

private:
    std::queue<Data> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cv;
};

} // util
} // i2r

#endif