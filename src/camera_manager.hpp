#pragma once
#include "config.hpp"
#include "utils/status.hpp"
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

struct FrameData {
    std::string camera_name;
    sl::Timestamp timestamp;
};

using WriterCallback = std::function<Status(zed::ZEDCamera& camera,
    zed::ChannelImage const& image, sl::Timestamp const& timestamp)>;

class CameraManager {
public:
    CameraManager()
        : m_frames_processed(0)
    {
    }
    Status init(config::Config const& config);
    Status process_frames(WriterCallback const& writer_callback);

    [[nodiscard]] size_t frames_processed() const { return m_frames_processed; }

    [[nodiscard]] std::vector<std::unique_ptr<zed::ZEDCamera>> const&
    cameras() const
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
    [[nodiscard]] bool all_cameras_finished() const;
    std::vector<std::unique_ptr<zed::ZEDCamera>> m_cameras;
    std::atomic<size_t> m_frames_processed;
};
}
