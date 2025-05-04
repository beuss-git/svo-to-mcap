#pragma once
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sl/Camera.hpp>
#include <variant>

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

sl::MAT_TYPE get_mat_type(sl::VIEW view);
sl::MAT_TYPE get_mat_type(sl::MEASURE measure);
std::string get_frame_id(std::string const& camera_name,
    std::variant<sl::VIEW, sl::MEASURE> const& type);
bool is_left_camera(std::variant<sl::VIEW, sl::MEASURE> const& type);
bool is_raw_image(std::variant<sl::VIEW, sl::MEASURE> const& type);
bool is_point_cloud(std::variant<sl::VIEW, sl::MEASURE> type);
}
