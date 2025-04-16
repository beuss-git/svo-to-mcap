#pragma once
#include <string>

template<typename StatusCode, StatusCode success> struct StatusBase {
    StatusCode code;
    std::string message;

    StatusBase()
        : code(success)
    {
    }

    StatusBase(StatusCode code, std::string const& message)
        : code(code)
        , message(message)
    {
    }

    bool ok() const { return code == success; }
};
