#pragma once
#include "config.hpp"
#include "utils/counting_semaphore.hpp"
#include "utils/status.hpp"
#include "zed/zed_camera.hpp"
#include <memory>
#include <queue>
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

struct FrameData {
    std::string camera_name;
    sl::Timestamp timestamp;
    std::vector<zed::ChannelImage> channel_images;
};

using WriterCallback = std::function<Status(
    std::string const&, zed::ChannelImage const&, sl::Timestamp const&)>;

class CameraManager {
public:
    CameraManager()
        : m_running(false)
        , m_frames_processed(0)
        , m_frames_limit(0)
    {
    }
    Status init(config::Config const& config);
    Status process_frames(std::function<Status(
            std::string const&, zed::ChannelImage const&, sl::Timestamp const&)>
            writer_callback);

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
    void producer_thread(zed::ZEDCamera* camera,
        std::queue<FrameData>& frame_queue, std::mutex& queue_mutex,
        utils::CountingSemaphore& queue_slots,
        utils::CountingSemaphore& items_available,
        std::atomic<size_t>& active_producers);

    void consumer_thread(std::queue<FrameData>& frame_queue,
        std::mutex& queue_mutex, utils::CountingSemaphore& queue_slots,
        utils::CountingSemaphore& items_available,
        std::atomic<size_t>& active_producers,
        WriterCallback const& writer_callback);

    std::vector<std::unique_ptr<zed::ZEDCamera>> m_cameras;
    std::atomic_bool m_running;
    std::atomic<size_t> m_frames_processed;
    size_t m_frames_limit;
};
}
