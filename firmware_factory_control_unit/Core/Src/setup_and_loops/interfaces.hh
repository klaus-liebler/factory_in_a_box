#pragma once
#include <cstdint>

class ISetupAndLoop {
    public:
        virtual ~ISetupAndLoop() = default;
        virtual void Setup() = 0;
        virtual void Loop(uint32_t now) = 0;
};
