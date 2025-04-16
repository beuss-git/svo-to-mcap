#include "config.hpp"
#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>

namespace config {

Status parse_channels(Camera& camera, const YAML::Node& ychannels)
{
    for (auto ychannel : ychannels) {
        Channel channel {};
        try {
            channel.name = ychannel["name"].as<std::string>();
        } catch (const YAML::Exception& e) {
            return Status(StatusCode::ParseError,
                fmt::format("Invalid channel name for camera {}: {}",
                    camera.name, e.what()));
        }

        std::string const type_str = ychannel["type"].as<std::string>();
        if (type_str.find("VIEW") == 0) {
            std::string_view const view_type
                = std::string_view(type_str).substr(6);
            auto maybe_view = magic_enum::enum_cast<sl::VIEW>(view_type);
            if (maybe_view.has_value()) {
                channel.type = maybe_view.value();
            } else {
                return Status(StatusCode::InvalidType,
                    fmt::format("View type {} not found.", view_type));
            }
        } else if (type_str.find("MEASURE") == 0) {
            std::string_view const measure_type
                = std::string_view(type_str).substr(9);
            auto maybe_measure
                = magic_enum::enum_cast<sl::MEASURE>(measure_type);
            if (maybe_measure.has_value()) {
                channel.type = maybe_measure.value();
            } else {
                return Status(StatusCode::InvalidType,
                    fmt::format("Measure type {} not found.", measure_type));
            }
        } else {
            return Status(StatusCode::InvalidType,
                fmt::format("Invalid type {}.", type_str));
        }

        camera.channels.push_back(channel);
    }
    return {};
}

Status parse(Config& config, std::filesystem::path const& config_path)
{
    YAML::Node yconfig = YAML::LoadFile(config_path.string());
    if (!yconfig["cameras"]) {
        return Status(StatusCode::ParseError,
            fmt::format(
                "No cameras found in config file: {}", config_path.string()));
    }

    auto const cameras = yconfig["cameras"];
    for (auto ycam : cameras) {
        Camera camera {};
        try {
            camera.name = ycam["name"].as<std::string>();
        } catch (const YAML::Exception& e) {
            return Status(StatusCode::ParseError,
                fmt::format(
                    "Invalid name for camera {}: {}", camera.name, e.what()));
        }
        try {
            camera.svo_path = ycam["svo_file"].as<std::string>();
        } catch (const YAML::Exception& e) {
            return Status(StatusCode::ParseError,
                fmt::format("Invalid SVO path for camera {}: {}", camera.name,
                    e.what()));
        }

        auto const status = parse_channels(camera, ycam["channels"]);
        if (!status.ok()) {
            return status;
        }

        config.cameras.push_back(camera);
    }

    return {};
}
} // namespace config
