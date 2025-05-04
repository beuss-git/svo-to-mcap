#pragma once
#include "mcap/mcap_writer.hpp"
#include "zed/zed_camera.hpp"
#include <mcap/writer.hpp>

extern std::atomic_bool stop_requested; // NOLINT

class Converter {

public:
    Converter() = default;
    bool init(std::filesystem::path const& config_path);
    int run();

private:
    [[nodiscard]] bool all_cameras_finished() const;
    mcap_writer::Status write_camera_info(zed::ZEDCamera& camera,
        zed::ChannelImage const& channel_image, sl::Timestamp timestamp);

    mcap_writer::Status handle_image(zed::ZEDCamera& camera,
        zed::ChannelImage const& channel_image, sl::Timestamp timestamp);

    mcap_writer::Status handle_point_cloud(zed::ZEDCamera& camera,
        zed::ChannelImage const& channel_image, sl::Timestamp timestamp);

    config::Config m_config {};
    mcap_writer::McapWriter m_mcap_writer;
    std::vector<std::unique_ptr<zed::ZEDCamera>> m_cameras;
    size_t m_frames_processed {};
};
