#pragma once
#include <filesystem>
#include <optional>
#include <vector>

#include <yaml-cpp/yaml.h>

struct Camera {
    std::string name {};
};

struct Config {
    std::vector<Camera> cameras {};
};
namespace config {

std::optional<Config> parse(std::filesystem::path const& config_path);
} // namespace config
