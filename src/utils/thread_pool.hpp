#pragma once
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {
public:
    ThreadPool(size_t num_threads);
    ~ThreadPool();

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args)
        -> std::future<typename std::invoke_result<F, Args...>::type>;

    void wait_all();

private:
    std::vector<std::thread> m_workers;

    std::queue<std::function<void()>> m_tasks;

    std::mutex m_queue_mutex;
    std::condition_variable m_condition;

    bool m_stop;

    std::atomic<int> m_active_tasks;
    std::condition_variable m_wait_condition;
    std::mutex m_wait_mutex;
};

template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args)
    -> std::future<typename std::invoke_result<F, Args...>::type>
{
    using return_type = typename std::invoke_result<F, Args...>::type;

    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    std::future<return_type> result = task->get_future();

    {
        std::unique_lock<std::mutex> const lock(m_queue_mutex);

        if (m_stop) {
            throw std::runtime_error("enqueue on stopped ThreadPool");
        }

        m_tasks.emplace([task]() { (*task)(); });
    }

    m_condition.notify_one();

    return result;
}
