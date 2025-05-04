#include "zed_to_ros.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <bit>
#include <cstddef>
#include <gsl/gsl>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sl/Camera.hpp>

// A lot of this is taken from the zed ros2 wrapper:
// https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_components/src/zed_camera/src/zed_camera_component.cpp
namespace zed_utils {
static rclcpp::Time sl_time_to_ros2_time(sl::Timestamp t)
{
    auto const sec = static_cast<uint32_t>(t.getNanoseconds() / 1000000000);
    auto const nsec = static_cast<uint32_t>(t.getNanoseconds() % 1000000000);
    return { static_cast<int32_t>(sec), nsec };
}

static constexpr bool is_big_endian()
{
    if constexpr (std::endian::native == std::endian::big) {
        return true;
    } else if constexpr (std::endian::native == std::endian::little) {
        return false;
    }
}

void pointcloud_to_ros_msg(sensor_msgs::msg::PointCloud2& pcMsg,
    sl::Mat const& mat_cloud, sl::Resolution const& resolution,
    std::string const& frame_id, sl::Timestamp timestamp)
{
    auto const width = static_cast<uint32_t>(resolution.width);
    auto const height = static_cast<uint32_t>(resolution.height);
    auto const ptsCount = width * height;

    pcMsg.header.stamp = sl_time_to_ros2_time(timestamp);

    if (pcMsg.width != width || pcMsg.height != height) {
        pcMsg.header.frame_id = frame_id;

        pcMsg.is_bigendian = is_big_endian();
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

    memcpy(pcMsg.data.data(), mat_cloud.getPtr<sl::uchar1>(),
        static_cast<size_t>(ptsCount * 4) * sizeof(float));
}

void image_to_ros_msg(sensor_msgs::msg::Image& imgMsg, sl::Mat const& img,
    std::string const& frame_id, sl::Timestamp timestamp)
{
    imgMsg.header.stamp = sl_time_to_ros2_time(timestamp);
    imgMsg.header.frame_id = frame_id;
    imgMsg.height = static_cast<uint32_t>(img.getHeight());
    imgMsg.width = static_cast<uint32_t>(img.getWidth());

    imgMsg.is_bigendian = static_cast<uint8_t>(is_big_endian());

    imgMsg.step = static_cast<uint32_t>(img.getStepBytes());

    size_t const size = static_cast<size_t const>(imgMsg.step) * imgMsg.height;
    imgMsg.data.resize(size);

    sl::MAT_TYPE const dataType = img.getDataType();

    switch (dataType) {
    case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // NOLINT
        break;
    case sl::MAT_TYPE::F32_C2: /**< float 1 channel.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC2; // NOLINT
        break;
    case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC3; // NOLINT
        break;
    case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC4; // NOLINT
        break;
    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
        imgMsg.encoding = sensor_msgs::image_encodings::MONO8; // NOLINT
        break;
    case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC2; // NOLINT
        break;
    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::BGR8; // NOLINT
        break;
    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
        imgMsg.encoding = sensor_msgs::image_encodings::BGRA8; // NOLINT
        break;
    case sl::MAT_TYPE::U16_C1: /**< unsigned short 1 channel.*/
        imgMsg.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // NOLINT
        break;
    default:
        assert(false && "Unknown image encoding");
        break;
    }

    memcpy(imgMsg.data.data(), img.getPtr<sl::uchar1>(), size);
}

auto const mCamRealModel = sl::MODEL::ZED_X;
void fill_cam_info(sl::Camera& camera,
    sensor_msgs::msg::CameraInfo& left_cam_info_msg,
    sensor_msgs::msg::CameraInfo& right_cam_info_msg,
    std::string const& left_frame_id, std::string const& right_frame_id,
    sl::Resolution resolution, bool is_raw_image)
{

    sl::CalibrationParameters const zedParam
        = [&camera, &is_raw_image, &resolution]() {
              if (is_raw_image) {
                  return camera.getCameraInformation(resolution)
                      .camera_configuration.calibration_parameters_raw;
              }
              return camera.getCameraInformation(resolution)
                  .camera_configuration.calibration_parameters;
          }();

    float const baseline = zedParam.getCameraBaseline();

    // ----> Distortion models
    // ZED SDK params order: [ k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
    // Radial (k1, k2, k3, k4, k5, k6), Tangential (p1,p2) and Prism (s1, s2,
    // s3, s4) distortion. Prism not currently used.

    static auto constexpr plumb_bob
        = gsl::span(sensor_msgs::distortion_models::PLUMB_BOB);
    static auto constexpr rational_polynomial
        = gsl::span(sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL);
    static auto constexpr equidistant
        = gsl::span(sensor_msgs::distortion_models::EQUIDISTANT);

    // ROS2 order (OpenCV) -> k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
    switch (mCamRealModel) {
    case sl::MODEL::ZED: // PLUMB_BOB
        left_cam_info_msg.distortion_model = plumb_bob.data();
        right_cam_info_msg.distortion_model = plumb_bob.data();
        left_cam_info_msg.d.resize(5);
        right_cam_info_msg.d.resize(5);
        left_cam_info_msg.d[0] = zedParam.left_cam.disto[0];   // k1
        left_cam_info_msg.d[1] = zedParam.left_cam.disto[1];   // k2
        left_cam_info_msg.d[2] = zedParam.left_cam.disto[2];   // p1
        left_cam_info_msg.d[3] = zedParam.left_cam.disto[3];   // p2
        left_cam_info_msg.d[4] = zedParam.left_cam.disto[4];   // k3
        right_cam_info_msg.d[0] = zedParam.right_cam.disto[0]; // k1
        right_cam_info_msg.d[1] = zedParam.right_cam.disto[1]; // k2
        right_cam_info_msg.d[2] = zedParam.right_cam.disto[2]; // p1
        right_cam_info_msg.d[3] = zedParam.right_cam.disto[3]; // p2
        right_cam_info_msg.d[4] = zedParam.right_cam.disto[4]; // k3
        break;

    case sl::MODEL::ZED2:          // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED2i:         // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED_X:         // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED_XM:        // RATIONAL_POLYNOMIAL
    case sl::MODEL::VIRTUAL_ZED_X: // RATIONAL_POLYNOMIAL
        left_cam_info_msg.distortion_model = rational_polynomial.data();
        right_cam_info_msg.distortion_model = rational_polynomial.data();
        left_cam_info_msg.d.resize(8);
        right_cam_info_msg.d.resize(8);
        left_cam_info_msg.d[0] = zedParam.left_cam.disto[0];   // k1
        left_cam_info_msg.d[1] = zedParam.left_cam.disto[1];   // k2
        left_cam_info_msg.d[2] = zedParam.left_cam.disto[2];   // p1
        left_cam_info_msg.d[3] = zedParam.left_cam.disto[3];   // p2
        left_cam_info_msg.d[4] = zedParam.left_cam.disto[4];   // k3
        left_cam_info_msg.d[5] = zedParam.left_cam.disto[5];   // k4
        left_cam_info_msg.d[6] = zedParam.left_cam.disto[6];   // k5
        left_cam_info_msg.d[7] = zedParam.left_cam.disto[7];   // k6
        right_cam_info_msg.d[0] = zedParam.right_cam.disto[0]; // k1
        right_cam_info_msg.d[1] = zedParam.right_cam.disto[1]; // k2
        right_cam_info_msg.d[2] = zedParam.right_cam.disto[2]; // p1
        right_cam_info_msg.d[3] = zedParam.right_cam.disto[3]; // p2
        right_cam_info_msg.d[4] = zedParam.right_cam.disto[4]; // k3
        right_cam_info_msg.d[5] = zedParam.right_cam.disto[5]; // k4
        right_cam_info_msg.d[6] = zedParam.right_cam.disto[6]; // k5
        right_cam_info_msg.d[7] = zedParam.right_cam.disto[7]; // k6
        break;

    case sl::MODEL::ZED_M:
        if (zedParam.left_cam.disto[5] != 0 &&  // k4!=0
            zedParam.right_cam.disto[2] == 0 && // p1==0
            zedParam.right_cam.disto[3] == 0)   // p2==0
        {
            left_cam_info_msg.distortion_model = equidistant.data();
            right_cam_info_msg.distortion_model = equidistant.data();

            left_cam_info_msg.d.resize(4);
            right_cam_info_msg.d.resize(4);
            left_cam_info_msg.d[0] = zedParam.left_cam.disto[0];   // k1
            left_cam_info_msg.d[1] = zedParam.left_cam.disto[1];   // k2
            left_cam_info_msg.d[2] = zedParam.left_cam.disto[4];   // k3
            left_cam_info_msg.d[3] = zedParam.left_cam.disto[5];   // k4
            right_cam_info_msg.d[0] = zedParam.right_cam.disto[0]; // k1
            right_cam_info_msg.d[1] = zedParam.right_cam.disto[1]; // k2
            right_cam_info_msg.d[2] = zedParam.right_cam.disto[4]; // k3
            right_cam_info_msg.d[3] = zedParam.right_cam.disto[5]; // k4
        } else {
            left_cam_info_msg.distortion_model = plumb_bob.data();
            right_cam_info_msg.distortion_model = plumb_bob.data();
            left_cam_info_msg.d.resize(5);
            right_cam_info_msg.d.resize(5);
            left_cam_info_msg.d[0] = zedParam.left_cam.disto[0];   // k1
            left_cam_info_msg.d[1] = zedParam.left_cam.disto[1];   // k2
            left_cam_info_msg.d[2] = zedParam.left_cam.disto[2];   // p1
            left_cam_info_msg.d[3] = zedParam.left_cam.disto[3];   // p2
            left_cam_info_msg.d[4] = zedParam.left_cam.disto[4];   // k3
            right_cam_info_msg.d[0] = zedParam.right_cam.disto[0]; // k1
            right_cam_info_msg.d[1] = zedParam.right_cam.disto[1]; // k2
            right_cam_info_msg.d[2] = zedParam.right_cam.disto[2]; // p1
            right_cam_info_msg.d[3] = zedParam.right_cam.disto[3]; // p2
            right_cam_info_msg.d[4] = zedParam.right_cam.disto[4]; // k3
        }
        break;
    case sl::MODEL::ZED_X_HDR:
    case sl::MODEL::ZED_X_HDR_MINI:
    case sl::MODEL::ZED_X_HDR_MAX:
    case sl::MODEL::ZED_XONE_GS:
    case sl::MODEL::ZED_XONE_UHD:
    case sl::MODEL::ZED_XONE_HDR:
    case sl::MODEL::LAST:
        break;
    }

    left_cam_info_msg.k.fill(0.0);
    right_cam_info_msg.k.fill(0.0);
    left_cam_info_msg.k[0] = static_cast<double>(zedParam.left_cam.fx);
    left_cam_info_msg.k[2] = static_cast<double>(zedParam.left_cam.cx);
    left_cam_info_msg.k[4] = static_cast<double>(zedParam.left_cam.fy);
    left_cam_info_msg.k[5] = static_cast<double>(zedParam.left_cam.cy);
    left_cam_info_msg.k[8] = 1.0;
    right_cam_info_msg.k[0] = static_cast<double>(zedParam.right_cam.fx);
    right_cam_info_msg.k[2] = static_cast<double>(zedParam.right_cam.cx);
    right_cam_info_msg.k[4] = static_cast<double>(zedParam.right_cam.fy);
    right_cam_info_msg.k[5] = static_cast<double>(zedParam.right_cam.cy);
    right_cam_info_msg.k[8] = 1.0;
    left_cam_info_msg.r.fill(0.0);
    right_cam_info_msg.r.fill(0.0);

    for (size_t i = 0; i < 3; i++) {
        // identity
        right_cam_info_msg.r.at(i + (i * 3)) = 1;
        left_cam_info_msg.r.at(i + (i * 3)) = 1;
    }

    if (is_raw_image) {
        // ROS frame (X forward, Z up, Y left)
        for (size_t i = 0; i < 9; i++) {
            right_cam_info_msg.r.at(i)
                = zedParam.stereo_transform.getRotationMatrix().r[i]; // NOLINT
        }
    }

    left_cam_info_msg.p.fill(0.0);
    right_cam_info_msg.p.fill(0.0);
    left_cam_info_msg.p[0] = static_cast<double>(zedParam.left_cam.fx);
    left_cam_info_msg.p[2] = static_cast<double>(zedParam.left_cam.cx);
    left_cam_info_msg.p[5] = static_cast<double>(zedParam.left_cam.fy);
    left_cam_info_msg.p[6] = static_cast<double>(zedParam.left_cam.cy);
    left_cam_info_msg.p[10] = 1.0;
    // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    right_cam_info_msg.p[3]
        = static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
    right_cam_info_msg.p[0] = static_cast<double>(zedParam.right_cam.fx);
    right_cam_info_msg.p[2] = static_cast<double>(zedParam.right_cam.cx);
    right_cam_info_msg.p[5] = static_cast<double>(zedParam.right_cam.fy);
    right_cam_info_msg.p[6] = static_cast<double>(zedParam.right_cam.cy);
    right_cam_info_msg.p[10] = 1.0;
    left_cam_info_msg.width = right_cam_info_msg.width
        = static_cast<uint32_t>(resolution.width);
    left_cam_info_msg.height = right_cam_info_msg.height
        = static_cast<uint32_t>(resolution.height);
    left_cam_info_msg.header.frame_id = left_frame_id;
    right_cam_info_msg.header.frame_id = right_frame_id;
}
}
