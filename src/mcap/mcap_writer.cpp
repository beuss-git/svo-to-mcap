#include "mcap_writer.hpp"

namespace mcap_writer {
static constexpr size_t FIELDS_PER_POINT = 4;

static void zed_point_cloud_to_foxglove_msg(sl::Mat const& img,
    foxglove::PointCloud& point_cloud, std::string const& frame_id,
    sl::Timestamp const svo_timestamp)
{
    auto* timestamp = point_cloud.mutable_timestamp();
    timestamp->set_seconds(
        (int64_t)(svo_timestamp.getNanoseconds() / 1000000000));
    timestamp->set_nanos(
        (int32_t)(svo_timestamp.getNanoseconds() % 1000000000));

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
    int field_offset = 0;
    for (auto const& name : field_names) {
        auto* field = point_cloud.add_fields();
        field->set_name(name);
        field->set_offset(field_offset);
        field->set_type(foxglove::PackedElementField_NumericType_FLOAT32);
        field_offset += sizeof(float);
    }

    auto* mut_data = point_cloud.mutable_data();
    auto const points_count = img.getWidth() * img.getHeight();
    auto const data_size = FIELDS_PER_POINT * points_count * sizeof(float);
    mut_data->resize(data_size);

    memcpy(mut_data->data(), (float*)img.getPtr<sl::float4>(), data_size);
}

static void zed_image_to_foxglove_msg(sl::Mat const& img,
    foxglove::RawImage& imgMsg, std::string const& frameId,
    sl::Timestamp const svo_timestamp)
{
    auto* timestamp = imgMsg.mutable_timestamp();
    timestamp->set_seconds(
        (int64_t)(svo_timestamp.getNanoseconds() / 1000000000));
    timestamp->set_nanos(
        (int32_t)(svo_timestamp.getNanoseconds() % 1000000000));

    imgMsg.set_frame_id(frameId);

    imgMsg.set_width(img.getWidth());
    imgMsg.set_height(img.getHeight());

    // int num = 1; // for endianness detection
    // imgMsg.is_bigendian = !(*(char*)&num == 1);

    imgMsg.set_step(img.getStepBytes());

    size_t const size = imgMsg.step() * imgMsg.height();
    auto* mut_data = imgMsg.mutable_data();
    mut_data->resize(size);

    sl::MAT_TYPE const dataType = img.getDataType();

    switch (dataType) {
    case sl::MAT_TYPE::F32_C1:        /**< float 1 channel.*/
        imgMsg.set_encoding("32FC1"); // little endian
        memcpy((char*)(imgMsg.mutable_data()->data()), img.getPtr<sl::float1>(),
            size);
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
        memcpy((char*)(&imgMsg.data()), img.getPtr<sl::uchar1>(), size);
        break;
    //
    // case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
    //     imgMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC2;
    //     memcpy((char*)(&imgMsg.data()), img.getPtr<sl::uchar2>(), size);
    //     break;
    //
    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
        imgMsg.set_encoding("bgr8");
        memcpy((char*)(imgMsg.mutable_data()->data()), img.getPtr<sl::uchar3>(),
            size);
        break;
    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
        imgMsg.set_encoding("bgra8");
        memcpy((char*)(imgMsg.mutable_data()->data()), img.getPtr<sl::uchar4>(),
            size);
        break;
    case sl::MAT_TYPE::U16_C1: /**< unsigned short 1 channel.*/
        imgMsg.set_encoding("16UC1");
        memcpy((uint16_t*)(imgMsg.mutable_data()->data()),
            img.getPtr<sl::ushort1>(), size);
        break;
    default:
        std::cerr << "Unsupported image type: " << dataType << "\n";
        std::exit(1);
    }
}

Status McapWriter::write_image(std::string const& camera_name,
    zed::ChannelImage const& channel_image, sl::Timestamp const& timestamp)
{
    std::unique_lock<std::mutex> const lock(m_mutex);
    std::string channel_key
        = fmt::format("{}/{}", camera_name, channel_image.name);

    if (m_channels.count(channel_key) == 0
        || !m_channels[channel_key].registered) {
        return Status(StatusCode::MessageWriteFailed,
            fmt::format("Channel '{}' not registered", channel_key));
    }

    foxglove::RawImage image_msg {};

    zed_image_to_foxglove_msg(
        channel_image.image, image_msg, channel_image.frame_id, timestamp);
    auto const payload = image_msg.SerializeAsString();

    mcap::Message msg;
    msg.channelId = m_channels[channel_key].channel.id;
    msg.sequence = 0;
    msg.publishTime = timestamp.getNanoseconds();
    msg.logTime = timestamp.getNanoseconds();
    msg.data = reinterpret_cast<std::byte const*>(payload.data());
    msg.dataSize = payload.size();

    auto res = m_writer->write(msg);
    if (!res.ok()) {
        // FIXME: do somewhere else
        m_writer->terminate();
        return Status(StatusCode::MessageWriteFailed,
            fmt::format("Failed to write message: {}", res.message));
    }
    return {};
}

Status McapWriter::write_point_cloud(std::string const& camera_name,
    zed::ChannelImage const& channel_image, sl::Timestamp const& timestamp)
{
    std::unique_lock<std::mutex> const lock(m_mutex);
    std::string channel_key
        = fmt::format("{}/{}", camera_name, channel_image.name);

    if (m_channels.count(channel_key) == 0
        || !m_channels[channel_key].registered) {
        return Status(StatusCode::MessageWriteFailed,
            fmt::format("Channel '{}' not registered", channel_key));
    }

    foxglove::PointCloud point_cloud_msg {};

    zed_point_cloud_to_foxglove_msg(channel_image.image, point_cloud_msg,
        channel_image.frame_id, timestamp);
    auto const payload = point_cloud_msg.SerializeAsString();

    mcap::Message msg;
    msg.channelId = m_channels[channel_key].channel.id;
    msg.sequence = 0;
    msg.publishTime = timestamp.getNanoseconds();
    msg.logTime = timestamp.getNanoseconds();
    msg.data = reinterpret_cast<std::byte const*>(payload.data());
    msg.dataSize = payload.size();

    auto res = m_writer->write(msg);
    if (!res.ok()) {
        // FIXME: do somewhere else
        m_writer->terminate();
        return Status(StatusCode::MessageWriteFailed,
            fmt::format("Failed to write message: {}", res.message));
    }
    return {};
}
}
