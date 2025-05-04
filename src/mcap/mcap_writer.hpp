#pragma once
#include "../config.hpp"
#include "ros2_schemas.hpp"
#include <condition_variable>
#include <fmt/format.h>
#include <map>
#include <mcap/writer.hpp>
#include <queue>

// https://mcap.dev/docs/python/ros2_noenv_example

namespace mcap_writer {

enum class StatusCode : uint8_t {
    Success,
    WriterOpenFailed,
    ChannelRegistrationFailed,
    MessageWriteFailed,
    WriterShutdown,
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
    ~McapWriter() { }

    Status init(config::Config const& config)
    {
        mcap::McapWriterOptions options("ros2");
        if (config.output.compression == "lz4") {
            options.compression = mcap::Compression::Lz4;
        } else if (config.output.compression == "zstd") {
            options.compression = mcap::Compression::Zstd;
        } else {
            options.compression = mcap::Compression::None;
        }
        options.compressionLevel = mcap::CompressionLevel::Fastest;
        options.profile = "fastwrite";
        // options.noChunking = true;
        // options.noSummaryCRC = true;

        auto result = m_writer->open(config.output.file.string(), options);
        if (!result.ok()) {
            return { StatusCode::WriterOpenFailed,
                fmt::format("Failed to open file {}: {}",
                    config.output.file.string(), result.message) };
        }

        register_schemas();

        m_worker_thread = std::thread([this]() { worker_thread(); });

        return {};
    }

    void shutdown()
    {
        m_done = true;
        m_cv.notify_all();
        m_worker_thread.join();
    }

    struct MessageData {
        mcap::ChannelId channel_id;
        mcap::Timestamp timestamp;
        std::vector<std::byte> payload;
    };

    Status queue_message(std::vector<std::byte> const& payload,
        std::string const& channel_name, sl::Timestamp timestamp);

    Status register_channel(
        std::string const& channel_name, std::string const& schema_name)
    {
        if (m_channels.contains(channel_name)
            && m_channels[channel_name].registered) {
            return {};
        }

        if (!m_schemas.contains(schema_name)
            || !m_schemas[schema_name].registered) {
            return { StatusCode::ChannelRegistrationFailed,
                fmt::format("Schema '{}' not registered", schema_name) };
        }

        mcap::Channel channel(
            channel_name, "cdr", m_schemas[schema_name].schema.id);
        m_writer->addChannel(channel);
        std::cout << fmt::format("Registered channel '{}'", channel_name)
                  << '\n';

        m_channels[channel_name]
            = { .channel = channel, .registered = true, .sequence = 0 };

        return {};
    }

private:
    // https://mcap.dev/spec/registry
    void register_schemas()
    {
        for (auto const [schema_name, schema_data] :
            { ros2schemas::sensor_msgs_msg_Image,
                ros2schemas::sensor_msgs_msg_CameraInfo,
                ros2schemas::sensor_msgs_msg_PointCloud2 }) {
            mcap::Schema schema(schema_name, "ros2msg", schema_data);

            m_writer->addSchema(schema);

            m_schemas[std::string(schema_name)]
                = { .schema = schema, .registered = true };
        }
    }

    void worker_thread();

    std::unique_ptr<mcap::McapWriter> m_writer;
    std::map<std::string, SchemaInfo> m_schemas;
    std::map<std::string, ChannelInfo> m_channels;

    std::mutex m_queue_mutex;
    std::queue<MessageData> m_messages;
    std::thread m_worker_thread;
    std::atomic_bool m_done = false;
    std::condition_variable m_cv;
    size_t m_max_queue_size = 400;
};

}
