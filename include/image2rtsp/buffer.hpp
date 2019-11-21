#ifndef IMAGE2RTSP_BUFFER
#define IMAGE2RTSP_BUFFER

#include <queue>
#include <mutex>
#include <condition_variable>

namespace i2r{
namespace util{

// todo : fix memory leak 
template<typename Data>
class Buffer{
public:
    void SetBuffSize(const int buffSize) { m_buffSize = buffSize; }

    void Push(Data const& data){
        std::unique_lock<std::mutex> lock(m_mutex);

        if(m_queue.size() >= m_buffSize)
            m_queue.pop();

        m_queue.push(data);

        m_cv.notify_one();
    }

    bool Empty(){
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    const Data Pop(){
        std::unique_lock<std::mutex> lock(m_mutex);
        
        while(m_queue.empty())
            m_cv.wait(lock);
        
        std::cout << m_queue.empty() << std::endl;

        m_value = m_queue.front();
        m_queue.pop();

        return m_value;
    }

    const size_t Size(){
        std:: unique_lock<std::mutex> lock(m_mutex);
        return m_queue.size();
    }
    

private:
    Data m_value;
    std::queue<Data> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    int m_buffSize = 100;
};

} // util
} // i2r

#endif