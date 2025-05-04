#include "mcap_writer.hpp"
#include <cassert>

namespace mcap_writer {
Status McapWriter::queue_message(std::vector<std::byte> const& payload,
    std::string const& channel_name, sl::Timestamp timestamp)
{
    if (!m_channels.contains(channel_name)
        || !m_channels[channel_name].registered) {
        return { StatusCode::MessageWriteFailed,
            fmt::format("Channel '{}' not registered", channel_name) };
    }

    std::unique_lock lock(m_queue_mutex);
    // if (m_messages.size() >= m_max_queue_size) {
    //     std::cerr << "Queue full, waiting for space...\n";
    // }
    m_cv.wait(lock,
        [this]() { return m_messages.size() < m_max_queue_size || m_done; });

    if (m_done) {
        return { StatusCode::WriterShutdown,
            "McapWriter is done, cannot queue more messages." };
    }

    m_messages.push(
        MessageData { .channel_id = m_channels[channel_name].channel.id,
            .timestamp = timestamp.getNanoseconds(),
            .payload = payload });
    m_cv.notify_one(); // wake up the consumer

    return {};
}

void McapWriter::worker_thread()
{
    while (true) {
        std::unique_lock lock(m_queue_mutex);
        m_cv.wait(lock, [this]() {
            return !m_messages.empty() || m_done || m_force_shutdown;
        });
        if ((m_done && m_messages.empty()) || m_force_shutdown) {
            break;
        }
        assert(!m_messages.empty());

        MessageData const job = std::move(m_messages.front());
        m_messages.pop();

        m_cv.notify_one();
        lock.unlock();

        mcap::Message msg;
        msg.channelId = job.channel_id;
        msg.sequence = 0;
        msg.logTime = job.timestamp;
        msg.publishTime = msg.logTime;
        msg.data = job.payload.data();
        msg.dataSize = job.payload.size();

        auto const res = m_writer->write(msg);

        if (!res.ok()) {
            std::cerr << "Failed to write message: " << res.message << "\n";
            break;
        }
    }
}
}
