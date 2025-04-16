#pragma once
#include "config.hpp"
#include "utils/status.hpp"
#include "utils/thread_pool.hpp"
#include "zed/zed_camera.hpp"
#include <memory>
#include <vector>

namespace camera {

enum class StatusCode : uint8_t {
    Success,
    InvalidType,
    ParseError,
    InitializationFailed,
    ProcessingFailed,
};

struct Status : StatusBase<StatusCode, StatusCode::Success> {
    using StatusBase::StatusBase;
};

using FrameCallback = std::function<void(
    zed::ZEDCamera&, std::vector<zed::ChannelImage> const&, sl::Timestamp)>;

class CameraManager {
public:
    CameraManager(size_t thread_count = 1)
        : m_thread_pool(thread_count)
        , m_is_running(false)
        , m_frames_processed(0)
        , m_frames_limit(0)
    {
    }
    Status init(config::Config const& config);
    void process_frame(FrameCallback const& callback);
    void start_processing(FrameCallback const& callback);

    void stop_processing();

    void wait()
    {
        if (m_processing_thread.joinable()) {
            m_processing_thread.join();
        }
    }

    size_t frames_processed() const { return m_frames_processed; }

    std::vector<std::unique_ptr<zed::ZEDCamera>> const& cameras() const
    {
        return m_cameras;
    }

    void close_all()
    {
        for (auto& camera : m_cameras) {
            camera->close();
        }
    }

private:
    std::vector<std::unique_ptr<zed::ZEDCamera>> m_cameras;
    ThreadPool m_thread_pool;
    std::thread m_processing_thread;
    std::atomic_bool m_is_running;
    std::atomic<size_t> m_frames_processed;
    size_t m_frames_limit;
};
}
