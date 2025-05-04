#pragma once
#include "sensor_msgs/msg/camera_info.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sl/Camera.hpp>

namespace zed_utils {

void pointcloud_to_ros_msg(sensor_msgs::msg::PointCloud2& pcMsg,
    sl::Mat const& mat_cloud, sl::Resolution const& resolution,
    std::string const& frame_id, sl::Timestamp timestamp);

void image_to_ros_msg(sensor_msgs::msg::Image& imgMsg, sl::Mat const& img,
    std::string const& frame_id, sl::Timestamp timestamp);

void fill_cam_info(sl::Camera& camera,
    sensor_msgs::msg::CameraInfo& left_cam_info_msg,
    sensor_msgs::msg::CameraInfo& right_cam_info_msg,
    std::string const& left_frame_id, std::string const& right_frame_id,
    sl::Resolution resolution, bool is_raw_image);
}
