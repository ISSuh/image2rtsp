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
    Buffer() : m_mutex(), m_cv() {};
    ~Buffer() {};


    void SetBuffSize(const int buffSize) { m_buffSize = buffSize; }

    void Push(const Data& data){
        std::unique_lock<std::mutex> lock(m_mutex);

        if(m_queue.size() >= m_buffSize)
            m_queue.pop();

        m_queue.push(data);

        std::cout << "push : " << this << " / " << m_queue.size() << std::endl;

        m_cv.notify_one();
    }

    bool Empty(){
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    void Pop(Data& data){
        std::unique_lock<std::mutex> lock(m_mutex);
        
        while(m_queue.empty()){
            m_cv.wait(lock);
        }
                
        data = m_queue.front();
        m_queue.pop();

        std::cout << "pop : " << this << " / " << m_queue.size() << std::endl;
    }

    const size_t Size(){
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.size();
    }
    
private:
    std::queue<Data> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    int m_buffSize;
};

} // util
} // i2r

#endif