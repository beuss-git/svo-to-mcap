#include "converter.hpp"
#include "utils/zed_to_ros.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// NOTE: this is a bit inefficient due to copying
template<typename Message>
static std::vector<std::byte> serialize_ros2_message(Message const& msg)
{
    rclcpp::Serialization<Message> const serializer;

    rclcpp::SerializedMessage serialized_msg;

    // Serialize the message into the buffer
    serializer.serialize_message(&msg, &serialized_msg);

    std::cout << "Serialied message size: " << serialized_msg.size() << '\n';

    return { (std::byte*)serialized_msg.get_rcl_serialized_message().buffer,
        (std::byte*)serialized_msg.get_rcl_serialized_message().buffer
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

    auto camera_status = m_camera_manager.init(m_config);
    if (!camera_status.ok()) {
        std::cerr << "Failed to initialize camera manager: "
                  << camera_status.message << "\n";
        return false;
    }

    auto mcap_status = m_mcap_writer.init(m_config);
    if (!mcap_status.ok()) {
        std::cerr << "Failed to initialize MCAP writer: " << mcap_status.message
                  << "\n";
        return false;
    }

    for (auto const& camera : m_camera_manager.cameras()) {
        for (auto const& channel_image : camera->channel_images()) {
            bool const is_point_cloud = zed::is_point_cloud(channel_image.type);

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

int Converter::run()
{
    std::cout << "Starting SVO to MCAP conversion...\n";
    auto start_time = std::chrono::high_resolution_clock::now();

    size_t message_count = 0;
    auto frame_callback = [this, &message_count](zed::ZEDCamera& camera,
                              zed::ChannelImage const& channel_image,
                              sl::Timestamp const& timestamp) {
        if (zed::is_point_cloud(channel_image.type)) {
            auto status = handle_point_cloud(camera, channel_image, timestamp);
            if (!status.ok()) {
                std::cerr << "Failed to write point cloud: " << status.message
                          << "\n";
            }
        } else {
            auto status = handle_image(camera, channel_image, timestamp);
            if (!status.ok()) {
                std::cerr << "Failed to write image: " << status.message
                          << "\n";
            }
        }
        std::cout << "Messages written: " << ++message_count << "\n";
        return camera::Status();
    };

    m_camera_manager.process_frames(frame_callback);
    m_camera_manager.close_all();

    m_mcap_writer.shutdown();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);

    std::cout << "Conversion complete!\n";
    std::cout << "Processed " << m_camera_manager.frames_processed()
              << " frames in " << static_cast<double>(duration.count()) / 1000.0
              << " seconds\n";
    std::cout << "Average FPS: "
              << static_cast<double>(m_camera_manager.frames_processed())
            / (static_cast<double>(duration.count()) / 1000.0)
              << "\n";
    std::cout << "Output file: " << m_config.output.file.string() << "\n";

    return 0;
}

mcap_writer::Status Converter::write_camera_info(zed::ZEDCamera& camera,
    zed::ChannelImage const& channel_image, sl::Timestamp timestamp)
{
    bool const is_left_camera = zed::is_left_camera(channel_image.type);
    bool const is_raw_image = zed::is_raw_image(channel_image.type);
    auto const frame_id = zed::get_frame_id(camera.name(), channel_image.type);

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
        ros_image, channel_image.image, channel_image.frame_id, timestamp);

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
    zed_utils::pointcloud_to_ros_msg(ros_pointcloud, channel_image.image,
        resolution, channel_image.frame_id, timestamp);

    auto const payload = serialize_ros2_message(ros_pointcloud);

    m_mcap_writer.queue_message(payload, channel_name, timestamp);
    return {};
}
