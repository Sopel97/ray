#pragma once

#include <cstdint>

namespace ray
{
    struct ColorRGBi8
    {
        ColorRGBi8() = default;

        ColorRGBi8(std::uint8_t r, std::uint8_t g, std::uint8_t b) :
            r(r),
            g(g),
            b(b)
        {

        }

        std::uint8_t r, g, b;
    };

    struct ColorRGBf32
    {
        ColorRGBf32() = default;

        ColorRGBf32(float r, float g, float b) :
            r(r),
            g(g),
            b(b)
        {

        }

        float r, g, b;
    };
}
