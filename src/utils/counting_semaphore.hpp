#pragma once
#include <condition_variable>
#include <mutex>

namespace utils {

class CountingSemaphore {
public:
    explicit CountingSemaphore(int count = 0)
        : m_count(count)
    {
    }

    void acquire()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [this] { return m_count > 0; });
        m_count--;
    }

    void release()
    {
        std::unique_lock<std::mutex> const lock(m_mutex);
        m_count++;
        m_cv.notify_one();
    }

    bool try_acquire()
    {
        std::unique_lock<std::mutex> const lock(m_mutex);
        if (m_count > 0) {
            m_count--;
            return true;
        }
        return false;
    }

private:
    std::mutex m_mutex;
    std::condition_variable m_cv;
    int m_count;
};

} // namespace utils
