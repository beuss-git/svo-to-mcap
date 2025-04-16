#pragma once
#include "camera_manager.hpp"
#include "config.hpp"
#include "mcap/mcap_writer.hpp"
#include <filesystem>
#include <mcap/writer.hpp>

class Converter {

public:
    Converter() = default;
    bool init(std::filesystem::path const& config_path)
    {
        auto config_status = config::parse(m_config, config_path);
        if (!config_status.ok()) {
            std::cerr << "Failed to parse config: " << config_status.message
                      << "\n";
            return false;
        }

        auto camera_status = m_camera_manager.init(m_config);
        if (!camera_status.ok()) {
            std::cerr << "Failed to initialize camera manager: "
                      << camera_status.message << "\n";
            return false;
        }

        auto mcap_status = m_mcap_writer.init(m_config);
        if (!mcap_status.ok()) {
            std::cerr << "Failed to initialize MCAP writer: "
                      << mcap_status.message << "\n";
            return false;
        }

        for (auto const& camera : m_camera_manager.cameras()) {
            for (auto const& channel_image : camera->channel_images()) {
                auto status = m_mcap_writer.register_channel(
                    camera->name(), channel_image.name, "foxglove.RawImage");
                if (!status.ok()) {
                    std::cerr
                        << "Failed to register channel: " << status.message
                        << "\n";
                    return false;
                }
            }
        }

        return true;
    }
    int run()
    {
        std::cout << "Starting SVO to MCAP conversion...\n";
        auto start_time = std::chrono::high_resolution_clock::now();

        // First write camera calibration data.
        // for (auto const& camera : m_camera_manager.cameras()) {
        //     sl::Timestamp timestamp;
        //     timestamp.setNanoseconds(
        //         std::chrono::duration_cast<std::chrono::nanoseconds>(
        //             std::chrono::system_clock::now().time_since_epoch())
        //             .count());
        //
        // }
        //
        // std::string const&, zed::ChannelImage const&, sl::Timestamp const&

        auto frame_callback = [this](std::string const& camera_name,
                                  zed::ChannelImage const& channel_image,
                                  sl::Timestamp const& timestamp) {
            auto status = m_mcap_writer.write_image(
                camera_name, channel_image, timestamp);
            if (!status.ok()) {
                std::cerr << "Failed to write image: " << status.message
                          << "\n";
            }
            return camera::Status();
        };

        m_camera_manager.process_frames(frame_callback);
        m_camera_manager.close_all();

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);

        std::cout << "Conversion complete!\n";
        std::cout << "Processed " << m_camera_manager.frames_processed()
                  << " frames in " << duration.count() / 1000.0 << " seconds\n";
        std::cout << "Average FPS: "
                  << static_cast<double>(m_camera_manager.frames_processed())
                / (duration.count() / 1000.0)
                  << "\n";
        std::cout << "Output file: " << m_config.output.file.string() << "\n";

        return 0;
    }

private:
    config::Config m_config {};
    camera::CameraManager m_camera_manager {};

    mcap_writer::McapWriter m_mcap_writer {};
};
