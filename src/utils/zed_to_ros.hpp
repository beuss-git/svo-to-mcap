#pragma once
#include "sensor_msgs/msg/camera_info.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sl/Camera.hpp>

namespace zed_utils {

void pointcloud_to_ros_msg(sensor_msgs::msg::PointCloud2& pcMsg,
    sl::Mat const& mMatCloud, std::string const& frame_id,
    sl::Timestamp timestamp);

void image_to_ros_msg(sensor_msgs::msg::Image& imgMsg, sl::Mat img,
    std::string const& frameId, sl::Timestamp timestamp);

void fill_cam_info(sl::Camera& camera,
    sensor_msgs::msg::CameraInfo& leftCamInfoMsg,
    sensor_msgs::msg::CameraInfo& rightCamInfoMsg,
    std::string const& leftFrameId, std::string const& rightFrameId,
    sl::Resolution resolution, bool rawParam);
}
