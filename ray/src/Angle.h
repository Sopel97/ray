#pragma once

#include <cmath>

#include "MathConstants.h"

namespace ray
{
    struct Angle
    {
        constexpr Angle() :
            m_radians{}
        {

        }

        constexpr Angle(const Angle&) = default;
        constexpr Angle(Angle&&) = default;
        constexpr Angle& operator=(const Angle&) = default;
        constexpr Angle& operator=(Angle&&) = default;

        constexpr static Angle radians(float r)
        {
            return Angle(r);
        }

        constexpr static Angle degrees(float d)
        {
            return Angle(d * (pi / 180.0f));
        }

        float sin() const
        {
            return std::sin(m_radians);
        }

        float cos() const
        {
            return std::cos(m_radians);
        }

        float tan() const
        {
            return std::tan(m_radians);
        }

    private:
        float m_radians;

        constexpr explicit Angle(float r) :
            m_radians(r)
        {

        }
    };
}
