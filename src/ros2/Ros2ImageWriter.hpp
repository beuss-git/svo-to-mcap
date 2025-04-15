#pragma once
#include "../config.hpp"
#include "schemas.hpp"
#include <mcap/writer.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

// https://mcap.dev/docs/python/ros2_noenv_example

class ZEDCamera {
public:
  ZEDCamera() = delete;
  ZEDCamera(std::string_view name) : m_name(name) {}

  void register_topics(const mcap::Schema &image_schema,
                       mcap::McapWriter &writer) {
    m_left_image_rect_color =
        mcap::Channel(std::string(m_name) + "zed_node/left/image_rect_color",
                      "cdr", image_schema.id);
    m_right_image_rect_color =
        mcap::Channel(std::string(m_name) + "zed_node/right/image_rect_color",
                      "cdr", image_schema.id);

    writer.addChannel(m_left_image_rect_color);
    writer.addChannel(m_right_image_rect_color);
  }

  mcap::Channel left_image_rect_color() const {
    return m_left_image_rect_color;
  }

private:
  std::string_view m_name{};

  mcap::Channel m_left_image_rect_color;
  mcap::Channel m_right_image_rect_color;
};

class Ros2ImageWriter {
public:
  Ros2ImageWriter() = delete;
  Ros2ImageWriter(mcap::McapWriter &writer, const Config &config)
      : m_writer(writer), m_config(config) {}

  bool init() {
    register_schemas();

    register_cameras();
    return true;
  }

  bool write_image(sl::Mat &img, const sl::Timestamp svo_timestamp,
                   const std::string &frame_id);

private:
  void register_cameras() {
    for (const auto &camera : m_config.cameras) {
      m_cameras[camera.name] =
          std::make_unique<ZEDCamera>(std::string_view(camera.name));
      m_cameras[camera.name]->register_topics(m_image_schema, m_writer);
    }
  }

  void register_schemas() {
    // https://mcap.dev/spec/registry
    m_image_schema =
        mcap::Schema("sensor_msgs/msg/Image", "ros2msg", ros2::schema);

    m_writer.addSchema(m_image_schema);
  }
  std::vector<std::byte> serialize_image(const cv::Mat &image,
                                         const std::string &frame_id);

  mcap::McapWriter &m_writer;

  mcap::Schema m_image_schema;

  std::map<std::string, std::unique_ptr<ZEDCamera>> m_cameras{};
  Config m_config;
};
