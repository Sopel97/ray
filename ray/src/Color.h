#pragma once

#include <cstdint>

namespace ray
{
    struct ColorRGBf
    {
        constexpr ColorRGBf() :
            r(0.0f),
            g(0.0f),
            b(0.0f)
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
            r(0),
            g(0),
            b(0)
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
            g(static_cast<std::uint8_t>(other.r * 255.0f + 0.5f)),
            b(static_cast<std::uint8_t>(other.r * 255.0f + 0.5f))
        {

        }

        constexpr ColorRGBi(const ColorRGBi&) = default;
        constexpr ColorRGBi(ColorRGBi&&) = default;
        constexpr ColorRGBi& operator=(const ColorRGBi&) = default;
        constexpr ColorRGBi& operator=(ColorRGBi&&) = default;

        std::uint8_t r, g, b;
    };
}
