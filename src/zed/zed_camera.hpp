#pragma once
#include "../config.hpp"
#include "../utils/status.hpp"
#include "../utils/zed_utils.hpp"
#include <cassert>
#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>
#include <sl/Camera.hpp>

namespace zed {

struct Status : StatusBase<sl::ERROR_CODE, sl::ERROR_CODE::SUCCESS> {
    using StatusBase::StatusBase;
};

struct ChannelImage {
    std::string name;
    sl::Mat mat;
    std::variant<sl::VIEW, sl::MEASURE> type;
    std::string frame_id { "fix_me" };
};

class ZEDCamera {
public:
    Status init(config::Camera const& camera_cfg);

    [[nodiscard]] std::vector<ChannelImage> const& channel_images() const
    {
        return m_channel_images;
    }

    sl::Timestamp timestamp(sl::TIME_REFERENCE time_ref)
    {
        return m_zed.getTimestamp(time_ref);
    }

    sl::CameraInformation camera_information()
    {
        return m_zed.getCameraInformation();
    }

    int get_svo_number_of_frames() { return m_zed.getSVONumberOfFrames(); }
    int get_svo_position() { return m_zed.getSVOPosition(); }

    Status grab(sl::RuntimeParameters rt_parameters = sl::RuntimeParameters());

    [[nodiscard]] bool done() const { return m_done; }

    [[nodiscard]] std::string const& name() const { return m_name; }

    sl::Camera& zed() { return m_zed; }

    void close() { m_zed.close(); }

private:
    Status initialize_channel_images(config::Camera const& camera_cfg);

    std::string m_name;
    sl::Camera m_zed;
    std::vector<ChannelImage> m_channel_images;
    bool m_done = false;
};
}
