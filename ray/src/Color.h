#pragma once

#include <cstdint>

namespace ray
{
    struct ColorRGBf
    {
        constexpr ColorRGBf() :
            r{},
            g{},
            b{}
        {
        }

        constexpr ColorRGBf(float r, float g, float b) :
            r(r),
            g(g),
            b(b)
        {

        }

        constexpr ColorRGBf(const ColorRGBf&) = default;
        constexpr ColorRGBf(ColorRGBf&&) = default;
        constexpr ColorRGBf& operator=(const ColorRGBf&) = default;
        constexpr ColorRGBf& operator=(ColorRGBf&&) = default;

        float r, g, b;
    };

    struct ColorRGBi
    {
        constexpr ColorRGBi() :
            r{},
            g{},
            b{}
        {
        }

        constexpr ColorRGBi(std::uint8_t r, std::uint8_t g, std::uint8_t b) :
            r(r),
            g(g),
            b(b)
        {

        }

        constexpr explicit ColorRGBi(const ColorRGBf& other) :
            r(static_cast<std::uint8_t>(other.r * 255.0f + 0.5f)),
            g(static_cast<std::uint8_t>(other.g * 255.0f + 0.5f)),
            b(static_cast<std::uint8_t>(other.b * 255.0f + 0.5f))
        {

        }

        constexpr ColorRGBi(const ColorRGBi&) = default;
        constexpr ColorRGBi(ColorRGBi&&) = default;
        constexpr ColorRGBi& operator=(const ColorRGBi&) = default;
        constexpr ColorRGBi& operator=(ColorRGBi&&) = default;

        std::uint8_t r, g, b;
    };
}
