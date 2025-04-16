#include "config.hpp"

namespace config {

std::optional<Config> parse(std::filesystem::path const& config_path)
{
    Config config {};
    YAML::Node yconfig = YAML::LoadFile(config_path.string());
    if (yconfig["cameras"]) {
        auto const cameras = yconfig["cameras"];
        for (auto camera : cameras) {
            config.cameras.push_back(
                Camera { camera["name"].as<std::string>() });
        }
    }
    return config;
}
} // namespace config
