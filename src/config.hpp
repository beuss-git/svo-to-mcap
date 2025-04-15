#pragma once
#include <filesystem>
#include <optional>
#include <vector>

#include <yaml-cpp/yaml.h>

struct Camera {
  std::string name{};
};

struct Config {
  std::vector<Camera> cameras{};
};
namespace config {

std::optional<Config> parse(const std::filesystem::path &config_path);
} // namespace config
