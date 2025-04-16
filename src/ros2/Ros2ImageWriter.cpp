#include "Ros2ImageWriter.hpp"

#include "rclcpp/time.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

void imageToROSmsg(sensor_msgs::msg::Image& imgMsg, sl::Mat img,
    std::string frameId, rclcpp::Time t)
{
    imgMsg.header.stamp = t;
    imgMsg.header.frame_id = frameId;
    imgMsg.height = img.getHeight();
    imgMsg.width = img.getWidth();

    int num = 1; // for endianness detection
    imgMsg.is_bigendian = !(*(char*)&num == 1);

    imgMsg.step = img.getStepBytes();

    size_t size = imgMsg.step * imgMsg.height;
    std::cout << "Image size: " << size << std::endl;
    imgMsg.data.resize(size);

    sl::MAT_TYPE dataType = img.getDataType();

    switch (dataType) {
    case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        memcpy((char*)(&imgMsg.data[0]), img.getPtr<sl::float1>(), size);
        break;

    case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
        memcpy((char*)(&imgMsg.data[0]), img.getPtr<sl::float2>(), size);
        break;

    case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
        memcpy((char*)(&imgMsg.data[0]), img.getPtr<sl::float3>(), size);
        break;

    case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC4;
        memcpy((char*)(&imgMsg.data[0]), img.getPtr<sl::float4>(), size);
        break;

    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
        imgMsg.encoding = sensor_msgs::image_encodings::MONO8;
        memcpy((char*)(&imgMsg.data[0]), img.getPtr<sl::uchar1>(), size);
        break;

    case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC2;
        memcpy((char*)(&imgMsg.data[0]), img.getPtr<sl::uchar2>(), size);
        break;

    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::BGR8;
        memcpy((char*)(&imgMsg.data[0]), img.getPtr<sl::uchar3>(), size);
        break;

    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::BGRA8;
        memcpy((char*)(&imgMsg.data[0]), img.getPtr<sl::uchar4>(), size);
        break;

    case sl::MAT_TYPE::U16_C1: /**< unsigned short 1 channel.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        memcpy((uint16_t*)(&imgMsg.data[0]), img.getPtr<sl::ushort1>(), size);
        break;
    }
}
std::vector<std::byte> serializeImageMsgWithFastCDR(
    sensor_msgs::msg::Image const& msg)
{

    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;

    rclcpp::SerializedMessage serialized_msg;

    // Serialize the message into the buffer
    serializer.serialize_message(&msg, &serialized_msg);

    std::cout << "Serialied message size: " << serialized_msg.size()
              << std::endl;

    // Convert to vector
    return std::vector<std::byte>(
        (std::byte*)serialized_msg.get_rcl_serialized_message().buffer,
        (std::byte*)serialized_msg.get_rcl_serialized_message().buffer
            + serialized_msg.size());
}

rclcpp::Time slTime2Ros(sl::Timestamp t)
{
    uint32_t sec = static_cast<uint32_t>(t.getNanoseconds() / 1000000000);
    uint32_t nsec = static_cast<uint32_t>(t.getNanoseconds() % 1000000000);
    return rclcpp::Time(sec, nsec);
}

bool Ros2ImageWriter::write_image(sl::Mat& img,
    sl::Timestamp const svo_timestamp, std::string const& frame_id)
{
    std::cout << "Serializing image\n";

    sensor_msgs::msg::Image ros_image;

    imageToROSmsg(ros_image, img, frame_id, slTime2Ros(svo_timestamp));
    auto const payload = serializeImageMsgWithFastCDR(ros_image);
    std::cout << "Done serializing image\n";
    static int seq = 0;

    std::cout << "Payload size: " << payload.size() << std::endl;

    mcap::Message msg;
    msg.channelId = m_cameras["prt"]->left_image_rect_color().id;
    msg.sequence = 0; // TODO: ? Research what benefits this gives us.
    msg.publishTime = svo_timestamp.getNanoseconds();
    msg.logTime = msg.publishTime;
    msg.data = payload.data();
    msg.dataSize = payload.size();

    std::cout << "Writing image\n";
    auto res = m_writer.write(msg);
    if (!res.ok()) {
        std::cerr << "Failed to write message: " << res.message << std::endl;
        m_writer.terminate();
        return false;
    }
    std::cout << "Done writing image\n";
    return true;
}
