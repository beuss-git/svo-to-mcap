#pragma once
#include <string>
#include <utility>

template<typename StatusCode, StatusCode success> struct StatusBase {
    StatusCode code;
    std::string message;

    StatusBase()
        : code(success)
    {
    }

    StatusBase(StatusCode code_, std::string message_)
        : code(code_)
        , message(std::move(message_))
    {
    }

    [[nodiscard]] bool ok() const { return code == success; }
};
