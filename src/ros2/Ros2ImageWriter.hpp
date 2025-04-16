#pragma once
#include "../config.hpp"
#include <foxglove/RawImage.pb.h>
#include <mcap/writer.hpp>
#include <sl/Camera.hpp>
#include "../foxglove/BuildFileDescriptorSet.hpp"

// https://mcap.dev/docs/python/ros2_noenv_example

class ZEDCamera {
public:
    ZEDCamera() = delete;
    ZEDCamera(std::string_view name)
        : m_name(name)
    {
    }

    void register_topics(
        mcap::Schema const& image_schema, mcap::McapWriter& writer)
    {
        m_left_image_rect_color = mcap::Channel(
            std::string(m_name) + "zed_node/left/image_rect_color", "protobuf",
            image_schema.id);
        m_right_image_rect_color = mcap::Channel(
            std::string(m_name) + "zed_node/right/image_rect_color", "protobuf",
            image_schema.id);

        writer.addChannel(m_left_image_rect_color);
        writer.addChannel(m_right_image_rect_color);
    }

    mcap::Channel left_image_rect_color() const
    {
        return m_left_image_rect_color;
    }

private:
    std::string_view m_name {};

    mcap::Channel m_left_image_rect_color;
    mcap::Channel m_right_image_rect_color;
};

class Ros2ImageWriter {
public:
    Ros2ImageWriter() = delete;
    Ros2ImageWriter(mcap::McapWriter& writer, Config const& config)
        : m_writer(writer)
        , m_config(config)
    {
    }

    bool init()
    {
        register_schemas();

        register_cameras();
        return true;
    }

    bool write_image(sl::Mat& img, sl::Timestamp const svo_timestamp,
        std::string const& frame_id);

private:
    void register_cameras()
    {
        for (auto const& camera : m_config.cameras) {
            m_cameras[camera.name]
                = std::make_unique<ZEDCamera>(std::string_view(camera.name));
            m_cameras[camera.name]->register_topics(
                m_raw_image_schema, m_writer);
        }
    }

    void register_schemas()
    {
        // https://mcap.dev/spec/registry
        m_raw_image_schema = mcap::Schema("foxglove.RawImage", "protobuf",
            foxglove::BuildFileDescriptorSet(foxglove::RawImage::descriptor())
                .SerializeAsString());
        m_writer.addSchema(m_raw_image_schema);
    }

    mcap::McapWriter& m_writer;

    mcap::Schema m_raw_image_schema;

    std::map<std::string, std::unique_ptr<ZEDCamera>> m_cameras {};
    Config m_config;
};
