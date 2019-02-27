#pragma once

#include "Angle2.h"
#include "Vec3.h"

namespace ray
{
    template <typename T>
    struct AxisAngle3
    {
    public:
        constexpr AxisAngle3() noexcept = default;
        constexpr AxisAngle3(const Normal3<T>& axis, const Angle2<T>& angle) :
            m_axis(axis),
            m_angle(angle)
        {

        }
        constexpr AxisAngle3(const AxisAngle3<T>&) = default;
        constexpr AxisAngle3(AxisAngle3<T>&&) noexcept = default;
        constexpr AxisAngle3<T>& operator=(const AxisAngle3<T>&) = default;
        constexpr AxisAngle3<T>& operator=(AxisAngle3<T>&&) noexcept = default;

        constexpr const Normal3<T>& axis() const
        {
            return m_axis;
        }

        constexpr const Angle2<T>& angle() const
        {
            return m_angle;
        }

    private:
        Normal3<T> m_axis;
        Angle2<T> m_angle;
    };

    using AxisAngle3d = AxisAngle3<double>;
    using AxisAngle3f = AxisAngle3<float>;
}
