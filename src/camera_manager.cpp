#include "camera_manager.hpp"

namespace camera {

Status CameraManager::init(config::Config const& config)
{
    for (auto const& camera_cfg : config.cameras) {
        auto camera = std::make_unique<zed::ZEDCamera>();

        auto status = camera->init(camera_cfg);
        if (!status.ok()) {
            return { StatusCode::InitializationFailed,
                fmt::format("Failed to initialize camera '{}': {}",
                    camera_cfg.name, status.message) };
        }

        m_cameras.push_back(std::move(camera));
    }

    return {};
}

Status CameraManager::process_frames(WriterCallback const& writer_callback)
{
    m_frames_processed = 0;

    size_t total_frames = 0;
    for (auto const& camera : m_cameras) {
        total_frames += static_cast<size_t>(camera->get_svo_number_of_frames());
    }

    while (!all_cameras_finished()) {
        for (auto const& camera : m_cameras) {
            if (camera->done()) {
                continue;
            }

            auto status = camera->grab();
            if (!status.ok()) {
                continue;
            }

            auto timestamp = camera->timestamp(sl::TIME_REFERENCE::IMAGE);

            for (auto const& channel : camera->channel_images()) {
                zed::ChannelImage safe;
                safe.name = channel.name;
                safe.type = channel.type;
                safe.frame_id = channel.frame_id;
                if (std::holds_alternative<sl::VIEW>(channel.type)) {
                    camera->zed().retrieveImage(
                        safe.mat, std::get<sl::VIEW>(channel.type));
                } else if (std::holds_alternative<sl::MEASURE>(channel.type)) {
                    camera->zed().retrieveMeasure(
                        safe.mat, std::get<sl::MEASURE>(channel.type));
                }

                auto res = writer_callback(*camera, safe, timestamp);
                if (!res.ok()) {
                    std::cerr << "Failed to write image: " << res.message
                              << "\n";
                }
            }
            m_frames_processed++;
            std::cout << m_frames_processed << "/" << total_frames
                      << " frames processed\n";
        }
    }

    return {};
}

bool CameraManager::all_cameras_finished() const
{
    for (auto const& camera : m_cameras) {
        if (!camera->done()) {
            return false;
        }
    }
    return true;
}
}
