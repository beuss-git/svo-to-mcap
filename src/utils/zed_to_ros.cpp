#include "zed_to_ros.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace zed_utils {

rclcpp::Time sl_time_to_ros2_time(sl::Timestamp t)
{
    auto const sec = static_cast<uint32_t>(t.getNanoseconds() / 1000000000);
    auto const nsec = static_cast<uint32_t>(t.getNanoseconds() % 1000000000);
    return { static_cast<int32_t>(sec), nsec };
}

static auto const mPcResol = sl::Resolution(1920, 1080);

void pointcloud_to_ros_msg(sensor_msgs::msg::PointCloud2& pcMsg,
    sl::Mat const& mMatCloud, std::string const& frame_id,
    sl::Timestamp timestamp)
{

    int const width = mPcResol.width;
    int const height = mPcResol.height;
    int const ptsCount = width * height;

    pcMsg.header.stamp = sl_time_to_ros2_time(timestamp);

    if (pcMsg.width != width || pcMsg.height != height) {
        pcMsg.header.frame_id
            = frame_id; // Set the header values of the ROS message

        pcMsg.is_bigendian = false;
        pcMsg.is_dense = false;

        pcMsg.width = width;
        pcMsg.height = height;

        sensor_msgs::PointCloud2Modifier modifier(pcMsg);
        modifier.setPointCloud2Fields(4, "x", 1,
            sensor_msgs::msg::PointField::FLOAT32, "y", 1,
            sensor_msgs::msg::PointField::FLOAT32, "z", 1,
            sensor_msgs::msg::PointField::FLOAT32, "rgb", 1,
            sensor_msgs::msg::PointField::FLOAT32);
    }

    auto* cpu_cloud = mMatCloud.getPtr<sl::float4>();

    // Data copy
    auto* ptCloudPtr = reinterpret_cast<float*>(&pcMsg.data[0]);
    memcpy(ptCloudPtr, reinterpret_cast<float*>(cpu_cloud),
        ptsCount * 4 * sizeof(float));
}

void image_to_ros_msg(sensor_msgs::msg::Image& imgMsg, sl::Mat img,
    std::string const& frameId, sl::Timestamp timestamp)
{
    imgMsg.header.stamp = sl_time_to_ros2_time(timestamp);
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

auto const mCamRealModel = sl::MODEL::ZED_X;
void fill_cam_info(sl::Camera& camera,
    sensor_msgs::msg::CameraInfo& leftCamInfoMsg,
    sensor_msgs::msg::CameraInfo& rightCamInfoMsg,
    std::string const& leftFrameId, std::string const& rightFrameId,
    sl::Resolution resolution, bool rawParam)
{

    sl::CalibrationParameters const zedParam
        = [&camera, &rawParam, &resolution]() {
              if (rawParam) {
                  return camera.getCameraInformation(resolution)
                      .camera_configuration.calibration_parameters_raw;
              }
              return camera.getCameraInformation(resolution)
                  .camera_configuration.calibration_parameters;
          }();

    float baseline = zedParam.getCameraBaseline();

    // ----> Distortion models
    // ZED SDK params order: [ k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
    // Radial (k1, k2, k3, k4, k5, k6), Tangential (p1,p2) and Prism (s1, s2,
    // s3, s4) distortion. Prism not currently used.

    // ROS2 order (OpenCV) -> k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
    switch (mCamRealModel) {
    case sl::MODEL::ZED: // PLUMB_BOB
        leftCamInfoMsg.distortion_model
            = sensor_msgs::distortion_models::PLUMB_BOB;
        rightCamInfoMsg.distortion_model
            = sensor_msgs::distortion_models::PLUMB_BOB;
        leftCamInfoMsg.d.resize(5);
        rightCamInfoMsg.d.resize(5);
        leftCamInfoMsg.d[0] = zedParam.left_cam.disto[0];   // k1
        leftCamInfoMsg.d[1] = zedParam.left_cam.disto[1];   // k2
        leftCamInfoMsg.d[2] = zedParam.left_cam.disto[2];   // p1
        leftCamInfoMsg.d[3] = zedParam.left_cam.disto[3];   // p2
        leftCamInfoMsg.d[4] = zedParam.left_cam.disto[4];   // k3
        rightCamInfoMsg.d[0] = zedParam.right_cam.disto[0]; // k1
        rightCamInfoMsg.d[1] = zedParam.right_cam.disto[1]; // k2
        rightCamInfoMsg.d[2] = zedParam.right_cam.disto[2]; // p1
        rightCamInfoMsg.d[3] = zedParam.right_cam.disto[3]; // p2
        rightCamInfoMsg.d[4] = zedParam.right_cam.disto[4]; // k3
        break;

    case sl::MODEL::ZED2:          // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED2i:         // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED_X:         // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED_XM:        // RATIONAL_POLYNOMIAL
    case sl::MODEL::VIRTUAL_ZED_X: // RATIONAL_POLYNOMIAL
        leftCamInfoMsg.distortion_model
            = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
        rightCamInfoMsg.distortion_model
            = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
        leftCamInfoMsg.d.resize(8);
        rightCamInfoMsg.d.resize(8);
        leftCamInfoMsg.d[0] = zedParam.left_cam.disto[0];   // k1
        leftCamInfoMsg.d[1] = zedParam.left_cam.disto[1];   // k2
        leftCamInfoMsg.d[2] = zedParam.left_cam.disto[2];   // p1
        leftCamInfoMsg.d[3] = zedParam.left_cam.disto[3];   // p2
        leftCamInfoMsg.d[4] = zedParam.left_cam.disto[4];   // k3
        leftCamInfoMsg.d[5] = zedParam.left_cam.disto[5];   // k4
        leftCamInfoMsg.d[6] = zedParam.left_cam.disto[6];   // k5
        leftCamInfoMsg.d[7] = zedParam.left_cam.disto[7];   // k6
        rightCamInfoMsg.d[0] = zedParam.right_cam.disto[0]; // k1
        rightCamInfoMsg.d[1] = zedParam.right_cam.disto[1]; // k2
        rightCamInfoMsg.d[2] = zedParam.right_cam.disto[2]; // p1
        rightCamInfoMsg.d[3] = zedParam.right_cam.disto[3]; // p2
        rightCamInfoMsg.d[4] = zedParam.right_cam.disto[4]; // k3
        rightCamInfoMsg.d[5] = zedParam.right_cam.disto[5]; // k4
        rightCamInfoMsg.d[6] = zedParam.right_cam.disto[6]; // k5
        rightCamInfoMsg.d[7] = zedParam.right_cam.disto[7]; // k6
        break;

    case sl::MODEL::ZED_M:
        if (zedParam.left_cam.disto[5] != 0 &&  // k4!=0
            zedParam.right_cam.disto[2] == 0 && // p1==0
            zedParam.right_cam.disto[3] == 0)   // p2==0
        {
            leftCamInfoMsg.distortion_model
                = sensor_msgs::distortion_models::EQUIDISTANT;
            rightCamInfoMsg.distortion_model
                = sensor_msgs::distortion_models::EQUIDISTANT;

            leftCamInfoMsg.d.resize(4);
            rightCamInfoMsg.d.resize(4);
            leftCamInfoMsg.d[0] = zedParam.left_cam.disto[0];   // k1
            leftCamInfoMsg.d[1] = zedParam.left_cam.disto[1];   // k2
            leftCamInfoMsg.d[2] = zedParam.left_cam.disto[4];   // k3
            leftCamInfoMsg.d[3] = zedParam.left_cam.disto[5];   // k4
            rightCamInfoMsg.d[0] = zedParam.right_cam.disto[0]; // k1
            rightCamInfoMsg.d[1] = zedParam.right_cam.disto[1]; // k2
            rightCamInfoMsg.d[2] = zedParam.right_cam.disto[4]; // k3
            rightCamInfoMsg.d[3] = zedParam.right_cam.disto[5]; // k4
        } else {
            leftCamInfoMsg.distortion_model
                = sensor_msgs::distortion_models::PLUMB_BOB;
            rightCamInfoMsg.distortion_model
                = sensor_msgs::distortion_models::PLUMB_BOB;
            leftCamInfoMsg.d.resize(5);
            rightCamInfoMsg.d.resize(5);
            leftCamInfoMsg.d[0] = zedParam.left_cam.disto[0];   // k1
            leftCamInfoMsg.d[1] = zedParam.left_cam.disto[1];   // k2
            leftCamInfoMsg.d[2] = zedParam.left_cam.disto[2];   // p1
            leftCamInfoMsg.d[3] = zedParam.left_cam.disto[3];   // p2
            leftCamInfoMsg.d[4] = zedParam.left_cam.disto[4];   // k3
            rightCamInfoMsg.d[0] = zedParam.right_cam.disto[0]; // k1
            rightCamInfoMsg.d[1] = zedParam.right_cam.disto[1]; // k2
            rightCamInfoMsg.d[2] = zedParam.right_cam.disto[2]; // p1
            rightCamInfoMsg.d[3] = zedParam.right_cam.disto[3]; // p2
            rightCamInfoMsg.d[4] = zedParam.right_cam.disto[4]; // k3
        }
    }

    leftCamInfoMsg.k.fill(0.0);
    rightCamInfoMsg.k.fill(0.0);
    leftCamInfoMsg.k[0] = static_cast<double>(zedParam.left_cam.fx);
    leftCamInfoMsg.k[2] = static_cast<double>(zedParam.left_cam.cx);
    leftCamInfoMsg.k[4] = static_cast<double>(zedParam.left_cam.fy);
    leftCamInfoMsg.k[5] = static_cast<double>(zedParam.left_cam.cy);
    leftCamInfoMsg.k[8] = 1.0;
    rightCamInfoMsg.k[0] = static_cast<double>(zedParam.right_cam.fx);
    rightCamInfoMsg.k[2] = static_cast<double>(zedParam.right_cam.cx);
    rightCamInfoMsg.k[4] = static_cast<double>(zedParam.right_cam.fy);
    rightCamInfoMsg.k[5] = static_cast<double>(zedParam.right_cam.cy);
    rightCamInfoMsg.k[8] = 1.0;
    leftCamInfoMsg.r.fill(0.0);
    rightCamInfoMsg.r.fill(0.0);

    for (size_t i = 0; i < 3; i++) {
        // identity
        rightCamInfoMsg.r[i + i * 3] = 1;
        leftCamInfoMsg.r[i + i * 3] = 1;
    }

    if (rawParam) {
        // ROS frame (X forward, Z up, Y left)
        for (int i = 0; i < 9; i++) {
            rightCamInfoMsg.r[i]
                = zedParam.stereo_transform.getRotationMatrix().r[i];
        }
    }

    leftCamInfoMsg.p.fill(0.0);
    rightCamInfoMsg.p.fill(0.0);
    leftCamInfoMsg.p[0] = static_cast<double>(zedParam.left_cam.fx);
    leftCamInfoMsg.p[2] = static_cast<double>(zedParam.left_cam.cx);
    leftCamInfoMsg.p[5] = static_cast<double>(zedParam.left_cam.fy);
    leftCamInfoMsg.p[6] = static_cast<double>(zedParam.left_cam.cy);
    leftCamInfoMsg.p[10] = 1.0;
    // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    rightCamInfoMsg.p[3]
        = static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
    rightCamInfoMsg.p[0] = static_cast<double>(zedParam.right_cam.fx);
    rightCamInfoMsg.p[2] = static_cast<double>(zedParam.right_cam.cx);
    rightCamInfoMsg.p[5] = static_cast<double>(zedParam.right_cam.fy);
    rightCamInfoMsg.p[6] = static_cast<double>(zedParam.right_cam.cy);
    rightCamInfoMsg.p[10] = 1.0;
    leftCamInfoMsg.width = rightCamInfoMsg.width
        = static_cast<uint32_t>(resolution.width);
    leftCamInfoMsg.height = rightCamInfoMsg.height
        = static_cast<uint32_t>(resolution.height);
    leftCamInfoMsg.header.frame_id = leftFrameId;
    rightCamInfoMsg.header.frame_id = rightFrameId;
}
}
