#include "converter.hpp"
#include <csignal>
#include <gsl/gsl>
#include <iostream>

static void handle_signal(int signal)
{
    if (signal == SIGINT) {
        stop_requested = true;
    }
}

int main(int argc, char const* argv[])
{
    std::string config_path = "config.yaml";
    auto const args = gsl::span(argv, static_cast<size_t>(argc));
    if (args.size() > 1) {
        config_path = args[1];
    }

    (void)std::signal(SIGINT, handle_signal);

    Converter converter;
    if (!converter.init(config_path)) {
        std::cerr << "Failed to initialize converter\n";
        return EXIT_FAILURE;
    }

    return converter.run();
}
