#include "thread_pool.hpp"

ThreadPool::ThreadPool(size_t num_threads)
    : m_stop(false)
    , m_active_tasks(0)
{
    for (size_t i = 0; i < num_threads; ++i) {
        m_workers.emplace_back([this] {
            while (true) {
                std::function<void()> task;

                {
                    std::unique_lock<std::mutex> lock(m_queue_mutex);

                    m_condition.wait(
                        lock, [this] { return m_stop || !m_tasks.empty(); });

                    if (m_stop && m_tasks.empty()) {
                        return;
                    }

                    task = std::move(m_tasks.front());
                    m_tasks.pop();
                }

                m_active_tasks++;

                task();

                m_active_tasks--;

                // Notify wait_all if no more active tasks
                if (m_tasks.empty() && m_active_tasks == 0) {
                    std::lock_guard<std::mutex> const wait_lock(m_wait_mutex);
                    m_wait_condition.notify_all();
                }
            }
        });
    }
}

ThreadPool::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> const lock(m_queue_mutex);
        m_stop = true;
    }

    m_condition.notify_all();

    for (std::thread& worker : m_workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void ThreadPool::wait_all()
{
    std::unique_lock<std::mutex> lock(m_wait_mutex);
    m_wait_condition.wait(
        lock, [this] { return m_tasks.empty() && m_active_tasks == 0; });
}
