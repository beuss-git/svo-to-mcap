#include "converter.hpp"
#include <iostream>
#include <span>

int main(int argc, char const* argv[])
{
    std::string config_path = "config.yaml";
    std::span<char const*> const args(argv, static_cast<size_t>(argc));
    if (args.size() > 1) {
        config_path = args[1];
    }

    Converter converter;
    if (!converter.init(config_path)) {
        std::cerr << "Failed to initialize converter\n";
        return EXIT_FAILURE;
    }

    return converter.run();
}
