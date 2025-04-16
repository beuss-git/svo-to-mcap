#include "camera_manager.hpp"

namespace camera {

Status CameraManager::init(config::Config const& config)
{
    m_frames_limit = config.processing.frames_limit;

    for (auto const& camera_cfg : config.cameras) {
        auto camera = std::make_unique<zed::ZEDCamera>();

        auto status = camera->init(camera_cfg);
        if (!status.ok()) {
            return Status(StatusCode::InitializationFailed,
                fmt::format("Failed to initialize camera '{}': {}",
                    camera_cfg.name, status.message));
        }

        m_cameras.push_back(std::move(camera));
    }

    return Status();
}

void CameraManager::process_frame(FrameCallback const& callback)
{
    std::vector<std::future<void>> futures;

    // Process each camera in parallel
    for (auto& camera : m_cameras) {
        futures.push_back(m_thread_pool.enqueue([&camera, callback, this]() {
            auto status = camera->grab_images();
            if (status.ok()) {
                auto timestamp = camera->timestamp(sl::TIME_REFERENCE::IMAGE);

                callback(*camera, camera->channel_images(), timestamp);
            } else {
                std::cerr << "Failed to grab images from camera '"
                          << camera->name() << "': " << status.message
                          << std::endl;
            }
        }));
    }

    for (auto& future : futures) {
        future.wait();
    }

    // Increment frame counter
    m_frames_processed++;
}
void CameraManager::start_processing(FrameCallback const& callback)
{
    if (m_is_running) {
        return;
    }

    m_is_running = true;
    m_frames_processed = 0;

    m_processing_thread = std::thread([this, callback]() {
        while (m_is_running
            && (m_frames_limit == 0 || m_frames_processed < m_frames_limit)) {
            bool all_done = true;

            // Process each camera
            for (auto& camera : m_cameras) {
                int svo_position = camera->get_svo_position();
                int total_frames = camera->get_svo_number_of_frames();

                if (svo_position >= total_frames) {
                    continue;
                }

                all_done = false;

                auto status = camera->grab_images();
                if (status.ok()) {
                    auto timestamp
                        = camera->timestamp(sl::TIME_REFERENCE::IMAGE);

                    m_thread_pool.enqueue([&camera, callback, timestamp,
                                              this]() {
                        callback(*camera, camera->channel_images(), timestamp);
                    });
                } else if (status.code
                    == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
                    std::cout << "End of SVO reached for camera '"
                              << camera->name() << "'\n";
                } else {
                    std::cerr << "Failed to grab images from camera '"
                              << camera->name() << "': " << status.message
                              << "\n";
                }
            }

            if (all_done) {
                std::cout << "All SVOs have been processed. Stopping."
                          << std::endl;
                m_is_running = false;
                break;
            }

            // Wait for all tasks to complete
            m_thread_pool.wait_all();

            m_frames_processed++;

            if (m_frames_processed % 10 == 0) {
                std::cout << "Processed " << m_frames_processed << " frames.\n";
            }
        }
    });
}
void CameraManager::stop_processing()
{
    if (m_is_running) {
        m_is_running = false;

        if (m_processing_thread.joinable()) {
            m_processing_thread.join();
        }
    }
}
}
