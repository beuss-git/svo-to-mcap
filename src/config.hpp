#pragma once
#include "utils/status.hpp"
#include <filesystem>
#include <sl/Camera.hpp>
#include <variant>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace config {
struct Channel {
    std::string name;
    std::variant<sl::VIEW, sl::MEASURE> type;
};

struct Camera {
    std::string name;
    std::filesystem::path svo_path;
    std::vector<Channel> channels;
    sl::DEPTH_MODE depth_mode = sl::DEPTH_MODE::NONE;
    sl::UNIT coordinate_units = sl::UNIT::METER;
};

struct OutputConfig {
    std::filesystem::path file = "output_new_current.mcap";
    std::string compression = "zstd";
};

struct Config {
    std::vector<Camera> cameras;
    OutputConfig output;
};

enum class StatusCode : uint8_t {
    Success,
    InvalidType,
    ParseError,
};

struct Status : StatusBase<StatusCode, StatusCode::Success> {
    using StatusBase::StatusBase;
};

Status parse(Config& config, std::filesystem::path const& config_path);
} // namespace config
