#pragma once
#include "schemas.hpp"
#include <mcap/writer.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

// https://mcap.dev/docs/python/ros2_noenv_example
class Ros2ImageWriter {
public:
  Ros2ImageWriter(mcap::McapWriter &writer) : m_writer(writer) {}

  bool init() {
    register_schemas();
    return true;
  }

  struct Topic {
    std::string name;
  };
  void register_topics() {
    m_prt_image_channel =
        mcap::Channel("/zed/prt/Image", "cdr", m_image_schema.id);
    m_writer.addChannel(m_prt_image_channel);
  }

  bool write_image(sl::Mat &img, const sl::Timestamp svo_timestamp);

private:
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

  mcap::Channel m_prt_image_channel;
};
