#include "converter.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
    std::string config_path = "config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    Converter converter;
    if (!converter.init(config_path)) {
        std::cerr << "Failed to initialize converter\n";
        return EXIT_FAILURE;
    }

    return converter.run();

    return EXIT_SUCCESS;
}
