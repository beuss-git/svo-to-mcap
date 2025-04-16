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

    return {};
}

Status CameraManager::process_frames(std::function<Status(
        std::string const&, zed::ChannelImage const&, sl::Timestamp const&)>
        writer_callback)
{
    m_running = true;
    m_frames_processed = 0;

    size_t constexpr max_queue_size = 30;

    utils::CountingSemaphore queue_slots(max_queue_size);
    utils::CountingSemaphore items_available(0);

    std::queue<FrameData> frame_queue;
    std::mutex queue_mutex;

    std::atomic<size_t> active_producers(m_cameras.size());

    std::vector<std::thread> producers;
    producers.reserve(m_cameras.size());
    for (auto& camera : m_cameras) {
        producers.emplace_back([&, camera = camera.get()]() {
            producer_thread(camera, frame_queue, queue_mutex, queue_slots,
                items_available, active_producers);
        });
    }

    std::thread consumer([&]() {
        consumer_thread(frame_queue, queue_mutex, queue_slots, items_available,
            active_producers, writer_callback);
    });

    for (auto& thread : producers) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    if (consumer.joinable()) {
        consumer.join();
    }

    return Status();
}

void CameraManager::producer_thread(zed::ZEDCamera* camera,
    std::queue<FrameData>& frame_queue, std::mutex& queue_mutex,
    utils::CountingSemaphore& queue_slots,
    utils::CountingSemaphore& items_available,
    std::atomic<size_t>& active_producers)
{
    int const total_frames = camera->get_svo_number_of_frames();

    while (m_running
        && (m_frames_limit == 0 || camera->get_svo_position() < m_frames_limit)
        && camera->get_svo_position() < total_frames) {

        auto status = camera->grab_images();
        if (status.ok()) {
            queue_slots.acquire();

            if (!m_running) {
                queue_slots.release();
                break;
            }

            auto timestamp = camera->timestamp(sl::TIME_REFERENCE::IMAGE);

            FrameData frame;
            frame.camera_name = camera->name();
            frame.timestamp = timestamp;
            frame.channel_images = camera->channel_images();

            {
                std::lock_guard<std::mutex> const lock(queue_mutex);
                frame_queue.push(std::move(frame));
            }

            items_available.release();
        } else if (status.code == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
            std::cout << "End of SVO reached for camera '" << camera->name()
                      << "'\n";
            break;
        } else {
            std::cerr << "Failed to grab images from camera '" << camera->name()
                      << "': " << status.message << "\n";
            break;
        }
    }

    if (--active_producers == 0) {
        items_available.release();
    }
}

void CameraManager::consumer_thread(std::queue<FrameData>& frame_queue,
    std::mutex& queue_mutex, utils::CountingSemaphore& queue_slots,
    utils::CountingSemaphore& items_available,
    std::atomic<size_t>& active_producers,
    WriterCallback const& writer_callback)
{
    while (m_running) {
        items_available.acquire();

        if (!m_running) {
            break;
        }

        FrameData frame;
        bool has_frame = false;
        bool producers_finished = false;

        {
            std::lock_guard<std::mutex> const lock(queue_mutex);
            if (!frame_queue.empty()) {
                frame = std::move(frame_queue.front());
                frame_queue.pop();
                has_frame = true;
            }
            producers_finished = (active_producers == 0);
        }

        if (has_frame) {
            queue_slots.release();

            for (auto const& image : frame.channel_images) {
                auto status = writer_callback(
                    frame.camera_name, image, frame.timestamp);
                if (!status.ok()) {
                    std::cerr << "Failed to write image: " << status.message
                              << "\n";
                }
            }

            m_frames_processed++;

            if (m_frames_processed % 10 == 0) {
                std::cout << "Processed " << m_frames_processed << " frames\n";
            }
        } else if (producers_finished) {
            break;
        }
    }
}

}
