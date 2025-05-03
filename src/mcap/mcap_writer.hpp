#pragma once
#include "../config.hpp"
#include "../foxglove/BuildFileDescriptorSet.hpp"
#include "../zed/zed_camera.hpp"
#include <fmt/format.h>
#include <foxglove/CameraCalibration.pb.h>
#include <foxglove/PointCloud.pb.h>
#include <foxglove/RawImage.pb.h>
#include <map>
#include <mcap/writer.hpp>

// https://mcap.dev/docs/python/ros2_noenv_example

namespace mcap_writer {

enum class StatusCode : uint8_t {
    Success,
    WriterOpenFailed,
    ChannelRegistrationFailed,
    MessageWriteFailed,
};

struct Status : StatusBase<StatusCode, StatusCode::Success> {
    using StatusBase::StatusBase;
};

struct SchemaInfo {
    mcap::Schema schema;
    bool registered = false;
};

struct ChannelInfo {
    mcap::Channel channel;
    bool registered = false;
    uint64_t sequence = 0;
};

class McapWriter {
public:
    McapWriter()
        : m_writer(std::make_unique<mcap::McapWriter>())
    {
    }

    Status init(config::Config const& config)
    {
        mcap::McapWriterOptions options("protobuf");
        if (config.output.compression == "lz4") {
            options.compression = mcap::Compression::Lz4;
        } else if (config.output.compression == "zstd") {
            options.compression = mcap::Compression::Zstd;
        } else {
            options.compression = mcap::Compression::None;
        }
        options.compressionLevel = mcap::CompressionLevel::Fastest;
        // options.noChunking = true;
        // options.noSummaryCRC = true;

        auto result = m_writer->open(config.output.file.string(), options);
        if (!result.ok()) {
            return { StatusCode::WriterOpenFailed,
                fmt::format("Failed to open file {}: {}",
                    config.output.file.string(), result.message) };
        }

        register_schemas();

        return {};
    }

    Status write_image(std::string const& camera_name,
        zed::ChannelImage const& channel_image, sl::Timestamp const& timestamp);

    Status write_point_cloud(std::string const& camera_name,
        zed::ChannelImage const& channel_image, sl::Timestamp const& timestamp);

    Status register_channel(std::string const& camera_name,
        std::string const& channel_name, std::string const& schema_name)
    {
        std::unique_lock<std::mutex> const lock(m_mutex);

        std::string const channel_key
            = fmt::format("{}/{}", camera_name, channel_name);

        if (m_channels.contains(channel_key)
            && m_channels[channel_key].registered) {
            return {};
        }

        if (!m_schemas.contains(schema_name)
            || !m_schemas[schema_name].registered) {
            return { StatusCode::ChannelRegistrationFailed,
                fmt::format("Schema '{}' not registered", schema_name) };
        }

        mcap::Channel channel(
            channel_key, "protobuf", m_schemas[schema_name].schema.id);
        m_writer->addChannel(channel);
        std::cout << fmt::format("Registered channel '{}'", channel_key)
                  << '\n';

        m_channels[channel_key]
            = { .channel = channel, .registered = true, .sequence = 0 };

        return {};
    }

private:
    // https://mcap.dev/spec/registry
    void register_schemas()
    {
        // Register RawImage schema
        {
            mcap::Schema schema("foxglove.RawImage", "protobuf",
                foxglove::BuildFileDescriptorSet(
                    foxglove::RawImage::descriptor())
                    .SerializeAsString());

            m_writer->addSchema(schema);

            m_schemas["foxglove.RawImage"]
                = { .schema = schema, .registered = true };
        }

        // Register CameraCalibration schema
        {
            mcap::Schema schema("foxglove.CameraCalibration", "protobuf",
                foxglove::BuildFileDescriptorSet(
                    foxglove::CameraCalibration::descriptor())
                    .SerializeAsString());

            m_writer->addSchema(schema);

            m_schemas["foxglove.CameraCalibration"]
                = { .schema = schema, .registered = true };
        }

        // Register PointCloud schema
        {
            mcap::Schema schema("foxglove.PointCloud", "protobuf",
                foxglove::BuildFileDescriptorSet(
                    foxglove::PointCloud::descriptor())
                    .SerializeAsString());

            m_writer->addSchema(schema);

            m_schemas["foxglove.PointCloud"]
                = { .schema = schema, .registered = true };
        }
    }

    std::unique_ptr<mcap::McapWriter> m_writer;
    std::map<std::string, SchemaInfo> m_schemas;
    std::map<std::string, ChannelInfo> m_channels;
    std::mutex m_mutex;
};

}
