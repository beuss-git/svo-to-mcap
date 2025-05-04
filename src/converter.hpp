#pragma once
#include "camera_manager.hpp"
#include "config.hpp"
#include "mcap/mcap_writer.hpp"
#include "zed/zed_camera.hpp"
#include <filesystem>
#include <mcap/writer.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

class Converter {

public:
    Converter() = default;
    bool init(std::filesystem::path const& config_path);
    int run();

private:
    mcap_writer::Status write_camera_info(zed::ZEDCamera& camera,
        zed::ChannelImage const& channel_image, sl::Timestamp timestamp);

    mcap_writer::Status handle_image(zed::ZEDCamera& camera,
        zed::ChannelImage const& channel_image, sl::Timestamp timestamp);

    mcap_writer::Status handle_point_cloud(zed::ZEDCamera const& camera,
        zed::ChannelImage const& channel_image, sl::Timestamp timestamp);

    config::Config m_config {};
    camera::CameraManager m_camera_manager;

    mcap_writer::McapWriter m_mcap_writer;
};
