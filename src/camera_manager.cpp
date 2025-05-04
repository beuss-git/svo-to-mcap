#include "camera_manager.hpp"

namespace camera {

Status CameraManager::init(config::Config const& config)
{
    m_frames_limit = config.processing.frames_limit;

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
    m_running = true;
    m_frames_processed = 0;

    std::cout << "Camera length: " << m_cameras.size() << "\n";
    auto& camera = m_cameras[0];

    int const total_frames = camera->get_svo_number_of_frames();
    while (m_running && !camera->done()
        && (m_frames_limit == 0
            || static_cast<size_t>(camera->get_svo_position()) < m_frames_limit)
        && camera->get_svo_position() < total_frames) {

        if (!m_running) {
            break;
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
                    safe.image, std::get<sl::VIEW>(channel.type));
            } else if (std::holds_alternative<sl::MEASURE>(channel.type)) {
                camera->zed().retrieveMeasure(
                    safe.image, std::get<sl::MEASURE>(channel.type));
            }

            auto res = writer_callback(*camera, safe, timestamp);
            if (!res.ok()) {
                std::cerr << "Failed to write image: " << res.message << "\n";
            }
        }
        m_frames_processed++;
        std::cout << m_frames_processed << " frames processed\n";

        // } else if (status.code == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
        //     std::cout << "End of SVO reached for camera '" << camera->name()
        //               << "'\n";
        //     break;
        // } else {
        //     std::cerr << "Failed to grab images from camera '" <<
        //     camera->name()
        //               << "': " << status.message << "\n";
        //     break;
        // }
    }

    return {};
}
}
