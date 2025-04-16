#pragma once
#include <string>

template<typename StatusCode, StatusCode success> struct StatusBase {
    StatusCode code;
    std::string message;

    StatusBase()
        : code(success)
    {
    }

    StatusBase(StatusCode code_, std::string const& message_)
        : code(code_)
        , message(message_)
    {
    }

    bool ok() const { return code == success; }
};
