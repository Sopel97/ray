#pragma once

#include "MathConstants.h"

#include <cmath>
#include <utility>

namespace ray
{
    template <typename T>
    struct Angle2
    {
        constexpr Angle2() :
            m_radians{}
        {

        }

        constexpr Angle2(const Angle2<T>&) = default;
        constexpr Angle2(Angle2<T>&&) = default;
        constexpr Angle2& operator=(const Angle2<T>&) = default;
        constexpr Angle2& operator=(Angle2<T>&&) = default;

        constexpr static Angle2<T> radians(float r)
        {
            return Angle2<T>(r);
        }

        constexpr static Angle2<T> degrees(float d)
        {
            return Angle2<T>(d * (pi / static_cast<T>(180)));
        }

        T sin() const
        {
            using std::sin;
            return sin(m_radians);
        }

        T cos() const
        {
            using std::cos;
            return cos(m_radians);
        }

        std::pair<T, T> sincos() const
        {
            using std::sin;
            using std::cos;
            return std::make_pair(sin(m_radians), cos(m_radians));
        }

        T tan() const
        {
            using std::tan;
            return tan(m_radians);
        }

        Angle2<T>& operator*=(float s)
        {
            m_radians *= s;
            return *this;
        }

        Angle2<T>& operator/=(float s)
        {
            m_radians /= s;
            return *this;
        }

        Angle2<T>& operator+=(const Angle2<T>& s)
        {
            m_radians += s.m_radians;
            return *this;
        }

        Angle2<T>& operator-=(const Angle2<T>& s)
        {
            m_radians -= s.m_radians;
            return *this;
        }

        friend Angle2<T> operator*(const Angle2<T>& a, float s)
        {
            return Angle2<T>(a.m_radians * s);
        }

    private:
        T m_radians;

        constexpr explicit Angle2(const T& r) :
            m_radians(r)
        {

        }
    };

    using Angle2f = Angle2<float>;
}
