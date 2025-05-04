#include "converter.hpp"
#include "utils/zed_utils.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

std::atomic_bool stop_requested { false }; // NOLINT

// NOTE: this is a bit inefficient due to copying
template<typename Message>
static std::vector<std::byte> serialize_ros2_message(Message const& msg)
{
    rclcpp::Serialization<Message> const serializer;

    rclcpp::SerializedMessage serialized_msg;

    // Serialize the message into the buffer
    serializer.serialize_message(&msg, &serialized_msg);

    return { reinterpret_cast<std::byte*>(
                 serialized_msg.get_rcl_serialized_message().buffer),
        reinterpret_cast<std::byte*>(
            serialized_msg.get_rcl_serialized_message().buffer)
            + serialized_msg.size() };
}

bool Converter::init(std::filesystem::path const& config_path)
{
    auto config_status = config::parse(m_config, config_path);
    if (!config_status.ok()) {
        std::cerr << "Failed to parse config: " << config_status.message
                  << "\n";
        return false;
    }

    auto mcap_status = m_mcap_writer.init(m_config);
    if (!mcap_status.ok()) {
        std::cerr << "Failed to initialize MCAP writer: " << mcap_status.message
                  << "\n";
        return false;
    }

    // Initialize cameras
    for (auto const& camera_cfg : m_config.cameras) {
        auto camera = std::make_unique<zed::ZEDCamera>();

        auto status = camera->init(camera_cfg);
        if (!status.ok()) {
            std::cout << fmt::format("Failed to initialize camera '{}': {}\n",
                camera_cfg.name, status.message);
            return false;
        }

        m_cameras.push_back(std::move(camera));
    }

    // Register message channels
    for (auto const& camera : m_cameras) {
        for (auto const& channel_image : camera->channel_images()) {
            bool const is_point_cloud
                = zed_utils::is_point_cloud(channel_image.type);

            std::string const channel_name
                = fmt::format("{}/{}", camera->name(), channel_image.name);

            mcap_writer::Status status;
            if (is_point_cloud) {
                status = m_mcap_writer.register_channel(channel_name,
                    std::string(ros2schemas::sensor_msgs_msg_PointCloud2.id));
            } else {
                status = m_mcap_writer.register_channel(channel_name,
                    std::string(ros2schemas::sensor_msgs_msg_Image.id));

                if (status.ok()) {
                    std::string const camera_info_channel_name
                        = fmt::format("{}/{}/camera_info", camera->name(),
                            channel_image.name);

                    status = m_mcap_writer.register_channel(
                        camera_info_channel_name,
                        std::string(
                            ros2schemas::sensor_msgs_msg_CameraInfo.id));
                }
            }

            if (!status.ok()) {
                std::cerr << "Failed to register channel: " << status.message
                          << "\n";
                return false;
            }
        }
    }

    return true;
}

static void update_progress_bar(size_t progress, size_t total, float fps)
{
    int const bar_width = 50;
    float const ratio
        = static_cast<float>(progress) / static_cast<float>(total);
    int const pos = static_cast<int>(bar_width * ratio);

    std::cout << "[";
    for (int i = 0; i < bar_width; ++i) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << static_cast<int>(ratio * 100.f) << "% | " << fps
              << " FPS\r";

    std::cout.flush();
}

int Converter::run()
{
    std::cout << "Starting SVO to MCAP conversion...\n";
    auto start_time = std::chrono::high_resolution_clock::now();

    size_t total_frames = 0;
    for (auto const& camera : m_cameras) {
        total_frames += static_cast<size_t>(camera->get_svo_number_of_frames());
    }

    auto last_update_time = std::chrono::steady_clock::now();
    size_t frames_since_last_update = 0;

    while (!all_cameras_finished() && !stop_requested) {
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
                zed::ChannelImage channel_image;
                channel_image.name = channel.name;
                channel_image.type = channel.type;
                channel_image.frame_id = channel.frame_id;
                if (std::holds_alternative<sl::VIEW>(channel.type)) {
                    camera->zed().retrieveImage(
                        channel_image.mat, std::get<sl::VIEW>(channel.type));
                } else if (std::holds_alternative<sl::MEASURE>(channel.type)) {
                    camera->zed().retrieveMeasure(
                        channel_image.mat, std::get<sl::MEASURE>(channel.type));
                }

                if (zed_utils::is_point_cloud(channel_image.type)) {
                    auto write_status
                        = handle_point_cloud(*camera, channel_image, timestamp);
                    if (!write_status.ok()) {
                        std::cerr << "Failed to write point cloud: "
                                  << write_status.message << "\n";
                    }
                } else {
                    auto write_status
                        = handle_image(*camera, channel_image, timestamp);
                    if (!write_status.ok()) {
                        std::cerr
                            << "Failed to write image: " << write_status.message
                            << "\n";
                    }
                }
            }
            m_frames_processed++;
            frames_since_last_update++;
        }
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> const elapsed = now - last_update_time;
        if (elapsed.count() >= 1.0) {
            float const fps = static_cast<float>(frames_since_last_update)
                / static_cast<float>(elapsed.count());
            update_progress_bar(m_frames_processed, total_frames, fps);
            last_update_time = now;
            frames_since_last_update = 0;
        }
    }

    if (stop_requested) {
        std::cout << "\nStopping conversion...\n";
    }

    // Close cameras
    for (auto& camera : m_cameras) {
        camera->close();
    }

    m_mcap_writer.shutdown(stop_requested);

    if (stop_requested) {
        m_mcap_writer.flush();
        return 0;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);

    std::cout << "Processed " << m_frames_processed << " frames in "
              << static_cast<double>(duration.count()) / 1000.0 << " seconds\n";
    std::cout << "Average FPS: "
              << static_cast<double>(m_frames_processed)
            / (static_cast<double>(duration.count()) / 1000.0)
              << "\n";
    std::cout << "Flushing the remaining data to disk..." << std::flush;
    m_mcap_writer.flush();
    std::cout << "Done!\n";

    return 0;
}

mcap_writer::Status Converter::write_camera_info(zed::ZEDCamera& camera,
    zed::ChannelImage const& channel_image, sl::Timestamp timestamp)
{
    bool const is_left_camera = zed_utils::is_left_camera(channel_image.type);
    bool const is_raw_image = zed_utils::is_raw_image(channel_image.type);
    auto const frame_id
        = zed_utils::get_frame_id(camera.name(), channel_image.type);

    std::string const channel_name
        = fmt::format("{}/{}/camera_info", camera.name(), channel_image.name);

    // TODO: This is a bit hacky, we should split the fill_cam_info
    // function into left and right versions.
    sensor_msgs::msg::CameraInfo cam_info_left {};
    sensor_msgs::msg::CameraInfo cam_info_right {};

    auto const resolution
        = camera.camera_information().camera_configuration.resolution;

    zed_utils::fill_cam_info(camera.zed(), cam_info_left, cam_info_right,
        frame_id, frame_id, resolution, is_raw_image);

    if (is_left_camera) {
        auto const payload_left = serialize_ros2_message(cam_info_left);
        m_mcap_writer.queue_message(payload_left, channel_name, timestamp);
    } else {
        auto const payload_right = serialize_ros2_message(cam_info_right);
        m_mcap_writer.queue_message(payload_right, channel_name, timestamp);
    }
    return {};
}

mcap_writer::Status Converter::handle_image(zed::ZEDCamera& camera,
    zed::ChannelImage const& channel_image, sl::Timestamp timestamp)
{
    std::string const channel_name
        = fmt::format("{}/{}", camera.name(), channel_image.name);

    sensor_msgs::msg::Image ros_image;
    zed_utils::image_to_ros_msg(
        ros_image, channel_image.mat, channel_image.frame_id, timestamp);

    auto const payload = serialize_ros2_message(ros_image);

    m_mcap_writer.queue_message(payload, channel_name, timestamp);

    write_camera_info(camera, channel_image, timestamp);
    return {};
}

mcap_writer::Status Converter::handle_point_cloud(zed::ZEDCamera& camera,
    zed::ChannelImage const& channel_image, sl::Timestamp timestamp)
{
    std::string const channel_name
        = fmt::format("{}/{}", camera.name(), channel_image.name);

    auto const resolution
        = camera.camera_information().camera_configuration.resolution;

    sensor_msgs::msg::PointCloud2 ros_pointcloud;
    zed_utils::pointcloud_to_ros_msg(ros_pointcloud, channel_image.mat,
        resolution, channel_image.frame_id, timestamp);

    auto const payload = serialize_ros2_message(ros_pointcloud);

    m_mcap_writer.queue_message(payload, channel_name, timestamp);
    return {};
}

bool Converter::all_cameras_finished() const
{
    for (auto const& camera : m_cameras) {
        if (!camera->done()) {
            return false;
        }
    }
    return true;
}
