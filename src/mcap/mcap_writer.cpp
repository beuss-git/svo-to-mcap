#include "mcap_writer.hpp"

namespace mcap_writer {
static constexpr size_t FIELDS_PER_POINT = 4;

static void zed_point_cloud_to_foxglove_msg(sl::Mat const& img,
    foxglove::PointCloud& point_cloud, std::string const& frame_id,
    sl::Timestamp svo_timestamp)
{
    auto* timestamp = point_cloud.mutable_timestamp();
    timestamp->set_seconds(
        static_cast<int64_t>(svo_timestamp.getNanoseconds() / 1000000000));
    timestamp->set_nanos(
        static_cast<int32_t>(svo_timestamp.getNanoseconds() % 1000000000));

    timestamp->set_seconds(static_cast<int64_t>(0));
    timestamp->set_nanos(static_cast<int32_t>(0));

    point_cloud.set_frame_id(frame_id);

    point_cloud.set_point_stride(sizeof(float) * FIELDS_PER_POINT);

    // Position the pointclouds in the center of their coordinate frame.
    auto* pose = point_cloud.mutable_pose();
    auto* position = pose->mutable_position();
    position->set_x(0);
    position->set_y(0);
    position->set_z(0);
    auto* orientation = pose->mutable_orientation();
    orientation->set_x(0);
    orientation->set_y(0);
    orientation->set_z(0);
    orientation->set_w(1);

    std::array<char const*, FIELDS_PER_POINT> const field_names
        = { "x", "y", "z", "rgb" };
    size_t field_offset = 0;
    for (auto const& name : field_names) {
        auto* field = point_cloud.add_fields();
        field->set_name(name);
        field->set_offset(static_cast<uint32_t>(field_offset));
        field->set_type(foxglove::PackedElementField_NumericType_FLOAT32);
        field_offset += sizeof(float);
    }

    auto* mut_data = point_cloud.mutable_data();
    auto const points_count = img.getWidth() * img.getHeight();
    auto const data_size = FIELDS_PER_POINT * points_count * sizeof(float);
    mut_data->resize(data_size);

    memcpy(mut_data->data(), reinterpret_cast<float*>(img.getPtr<sl::float4>()),
        data_size);
}

static void zed_image_to_foxglove_msg(sl::Mat const& img,
    foxglove::RawImage& imgMsg, std::string const& frameId,
    sl::Timestamp svo_timestamp)
{
    auto* timestamp = imgMsg.mutable_timestamp();
    timestamp->set_seconds(
        static_cast<int64_t>(svo_timestamp.getNanoseconds() / 1000000000));
    timestamp->set_nanos(
        static_cast<int32_t>(svo_timestamp.getNanoseconds() % 1000000000));

    imgMsg.set_frame_id(frameId);

    imgMsg.set_width(static_cast<uint32_t>(img.getWidth()));
    imgMsg.set_height(static_cast<uint32_t>(img.getHeight()));

    // int num = 1; // for endianness detection
    // imgMsg.is_bigendian = !(*(char*)&num == 1);

    imgMsg.set_step(static_cast<uint32_t>(img.getStepBytes()));

    size_t const size = static_cast<size_t>(imgMsg.step()) * imgMsg.height();
    auto* mut_data = imgMsg.mutable_data();
    mut_data->append(size, '\0');
    // mut_data->resize(size);

    sl::MAT_TYPE const dataType = img.getDataType();

    switch (dataType) {
    case sl::MAT_TYPE::F32_C1:        /**< float 1 channel.*/
        imgMsg.set_encoding("32FC1"); // little endian
        memcpy(imgMsg.mutable_data()->data(), img.getPtr<sl::float1>(), size);
        break;

    // case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
    //     imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
    //     memcpy((char*)(&imgMsg.data()), img.getPtr<sl::float2>(), size);
    //     break;
    //
    // case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
    //     imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
    //     memcpy((char*)(&imgMsg.data()), img.getPtr<sl::float3>(), size);
    //     break;
    //
    // case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
    //     imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC4;
    //     memcpy((char*)(&imgMsg.data()), img.getPtr<sl::float4>(), size);
    //     break;
    //
    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
        imgMsg.set_encoding("mono8");
        memcpy(imgMsg.mutable_data()->data(), img.getPtr<sl::uchar1>(), size);
        break;
    //
    // case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
    //     imgMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC2;
    //     memcpy((char*)(&imgMsg.data()), img.getPtr<sl::uchar2>(), size);
    //     break;
    //
    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
        imgMsg.set_encoding("bgr8");
        memcpy(imgMsg.mutable_data()->data(),
            reinterpret_cast<char*>(img.getPtr<sl::uchar3>()), size);
        break;
    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
        imgMsg.set_encoding("bgra8");
        memcpy(imgMsg.mutable_data()->data(),
            reinterpret_cast<char*>(img.getPtr<sl::uchar4>()), size);
        break;
    case sl::MAT_TYPE::U16_C1: /**< unsigned short 1 channel.*/
        imgMsg.set_encoding("16UC1");
        memcpy(mut_data->data(), img.getPtr<sl::ushort1>(), size);
        break;
    default:
        std::cerr << "Unsupported image type: " << dataType << "\n";
        std::exit(1);
    }
}

Status McapWriter::write_image(std::string const& camera_name,
    zed::ChannelImage const& channel_image, sl::Timestamp const& timestamp)
{
    std::cout << "Channel image name: " << channel_image.name << "\n";
    std::string channel_key
        = fmt::format("{}/{}", camera_name, channel_image.name);

    if (!m_channels.contains(channel_key)
        || !m_channels[channel_key].registered) {
        return { StatusCode::MessageWriteFailed,
            fmt::format("Channel '{}' not registered", channel_key) };
    }

    foxglove::RawImage image_msg;

    zed_image_to_foxglove_msg(
        channel_image.image, image_msg, channel_image.frame_id, timestamp);
    auto const payload = image_msg.SerializeAsString();

    queue_message(
        MessageData { .channel_id = m_channels[channel_key].channel.id,
            .timestamp = timestamp.getNanoseconds(),
            .payload = payload });

    return {};
}

Status McapWriter::write_point_cloud(std::string const& camera_name,
    zed::ChannelImage const& channel_image, sl::Timestamp const& timestamp)
{
    std::string channel_key
        = fmt::format("{}/{}", camera_name, channel_image.name);

    if (!m_channels.contains(channel_key)
        || !m_channels[channel_key].registered) {
        return { StatusCode::MessageWriteFailed,
            fmt::format("Channel '{}' not registered", channel_key) };
    }

    foxglove::PointCloud point_cloud_msg;

    zed_point_cloud_to_foxglove_msg(channel_image.image, point_cloud_msg,
        channel_image.frame_id, timestamp);
    auto const payload = point_cloud_msg.SerializeAsString();

    queue_message(
        MessageData { .channel_id = m_channels[channel_key].channel.id,
            .timestamp = timestamp.getNanoseconds(),
            .payload = payload });
    return {};
}

Status McapWriter::queue_message(MessageData const& message)
{
    std::unique_lock lock(m_mutex);
    if (m_messages.size() >= m_max_queue_size) {
        std::cerr << "Queue full, waiting for space...\n";
    }
    m_cv.wait(lock,
        [this]() { return m_messages.size() < m_max_queue_size || m_done; });

    if (m_done) {
        return { StatusCode::WriterShutdown,
            "McapWriter is done, cannot queue more messages." };
    }

    m_messages.push(message);
    m_cv.notify_one(); // wake up the consumer

    return {};
}

void McapWriter::worker_thread()
{
    size_t messages_written = 0;
    while (true) {
        std::unique_lock lock(m_mutex);
        m_cv.wait(lock, [this]() { return !m_messages.empty() || m_done; });
        if (m_done && m_messages.empty()) {
            break;
        }
        assert(!m_messages.empty());

        MessageData const job = std::move(m_messages.front());
        m_messages.pop();

        m_cv.notify_one();
        lock.unlock();

        mcap::Message msg;
        msg.channelId = job.channel_id;
        msg.sequence = 0;
        msg.logTime = job.timestamp;
        msg.publishTime = msg.logTime;
        msg.data = reinterpret_cast<std::byte const*>(job.payload.data());
        msg.dataSize = job.payload.size();

        auto const res = m_writer->write(msg);

        if (!res.ok()) {
            std::cerr << "Failed to write message: " << res.message << "\n";
            m_writer->terminate();
            break;
        }
        std::cout << "Wrote message: " << ++messages_written << "\n";
    }
    std::cout << "Worker thread done.\n";
}
}
