#include "Ros2ImageWriter.hpp"
#include <foxglove/RawImage.pb.h>

void zed_image_to_foxglove_msg(sl::Mat img, foxglove::RawImage& imgMsg,
    std::string frameId, sl::Timestamp const svo_timestamp)
{
    auto timestamp = imgMsg.mutable_timestamp();
    timestamp->set_seconds(svo_timestamp.getNanoseconds() / 1000000000);
    timestamp->set_nanos(svo_timestamp.getNanoseconds() % 1000000000);

    imgMsg.set_frame_id(frameId);

    imgMsg.set_width(img.getWidth());
    imgMsg.set_height(img.getHeight());

    // int num = 1; // for endianness detection
    // imgMsg.is_bigendian = !(*(char*)&num == 1);

    imgMsg.set_step(img.getStepBytes());

    size_t const size = imgMsg.step() * imgMsg.height();
    std::cout << "Image size: " << size << "\n";
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
        memcpy((char*)(&imgMsg.data()), img.getPtr<sl::uchar3>(), size);
        break;
    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
        imgMsg.set_encoding("bgra8");
        memcpy((char*)(imgMsg.mutable_data()->data()), img.getPtr<sl::uchar4>(),
            size);
        break;
    //
    // case sl::MAT_TYPE::U16_C1: /**< unsigned short 1 channel.*/
    //     imgMsg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    //     memcpy((uint16_t*)(&imgMsg.data()), img.getPtr<sl::ushort1>(), size);
    //     break;
    default:
        std::cerr << "Unsupported image type: " << dataType << "\n";
        std::exit(1);
    }
}

bool Ros2ImageWriter::write_image(sl::Mat& img,
    sl::Timestamp const svo_timestamp, std::string const& frame_id)
{
    std::cout << "Serializing image\n";

    foxglove::RawImage foxglove_image {};

    zed_image_to_foxglove_msg(img, foxglove_image, frame_id, svo_timestamp);
    auto const payload = foxglove_image.SerializeAsString();
    std::cout << "Done serializing image\n";
    std::cout << "Payload size: " << payload.size() << "\n";

    mcap::Message msg;
    msg.channelId = m_cameras["prt"]->left_image_rect_color().id;
    msg.sequence = 0; // TODO: ? Research what benefits this gives us.
    msg.publishTime = svo_timestamp.getNanoseconds();
    msg.logTime = msg.publishTime;
    msg.data = reinterpret_cast<std::byte const*>(payload.data());
    msg.dataSize = payload.size();

    std::cout << "Writing image\n";
    auto res = m_writer.write(msg);
    if (!res.ok()) {
        std::cerr << "Failed to write message: " << res.message << "\n";
        m_writer.terminate();
        return false;
    }
    std::cout << "Done writing image\n";
    return true;
}
