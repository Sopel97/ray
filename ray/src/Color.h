#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

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

        ColorRGBf& operator+=(const ColorRGBf& rhs)
        {
            r += rhs.r;
            g += rhs.g;
            b += rhs.b;
            return *this;
        }

        constexpr float total() const
        {
            return r + g + b;
        }

        float r, g, b;
    };

    constexpr ColorRGBf operator*(const ColorRGBf& lhs, float rhs)
    {
        return ColorRGBf(lhs.r * rhs, lhs.g * rhs, lhs.b * rhs);
    }

    constexpr ColorRGBf operator*(float lhs, const ColorRGBf& rhs)
    {
        return ColorRGBf(lhs * rhs.r, lhs * rhs.g, lhs * rhs.b);
    }

    constexpr ColorRGBf operator+(const ColorRGBf& lhs, const ColorRGBf& rhs)
    {
        return ColorRGBf(lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b);
    }

    constexpr ColorRGBf operator*(const ColorRGBf& lhs, const ColorRGBf& rhs)
    {
        return ColorRGBf(lhs.r * rhs.r, lhs.g * rhs.g, lhs.b * rhs.b);
    }

    inline ColorRGBf operator^(const ColorRGBf& lhs, float gamma)
    {
        return ColorRGBf(
            std::pow(lhs.r, gamma),
            std::pow(lhs.g, gamma),
            std::pow(lhs.b, gamma)
        );
    }

    constexpr ColorRGBf operator-(const ColorRGBf& lhs)
    {
        return ColorRGBf(
            -lhs.r,
            -lhs.g,
            -lhs.b
        );
    }

    inline ColorRGBf exp(const ColorRGBf& lhs)
    {
        return ColorRGBf(
            std::exp(lhs.r),
            std::exp(lhs.g),
            std::exp(lhs.b)
        );
    }

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

        // gamma correction temporarily here
        explicit ColorRGBi(const ColorRGBf& other) :
            r(static_cast<std::uint8_t>(std::clamp(other.r, 0.0f, 1.0f) * 255.0f + 0.5f)),
            g(static_cast<std::uint8_t>(std::clamp(other.g, 0.0f, 1.0f) * 255.0f + 0.5f)),
            b(static_cast<std::uint8_t>(std::clamp(other.b, 0.0f, 1.0f) * 255.0f + 0.5f))
        {

        }

        constexpr ColorRGBi(const ColorRGBi&) = default;
        constexpr ColorRGBi(ColorRGBi&&) = default;
        constexpr ColorRGBi& operator=(const ColorRGBi&) = default;
        constexpr ColorRGBi& operator=(ColorRGBi&&) = default;

        std::uint8_t r, g, b;
    };
}
