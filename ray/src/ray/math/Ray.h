#pragma once

#include <ray/math/Vec3.h>

#include <array>
#include <cmath>

namespace ray
{
    struct Ray
    {
        [[nodiscard]] static Ray between(const Point3f& from, const Point3f& to) noexcept
        {
            return Ray(from, (to - from).normalized());
        }

        Ray(const Point3f& origin, const UnitVec3f& direction) noexcept :
            m_origin(origin),
            m_direction(direction),
            m_invDirection(safeInv(m_direction)),
            m_signs(m_direction < 0.0f)
        {

        }

        [[nodiscard]] constexpr const Point3f& origin() const
        {
            return m_origin;
        }

        [[nodiscard]] constexpr const UnitVec3f& direction() const
        {
            return m_direction;
        }

        [[nodiscard]] constexpr const Vec3f& invDirection() const
        {
            return m_invDirection;
        }

        [[nodiscard]] Vec3Mask<float> signs() const
        {
            return m_signs;
        }

        void setOrigin(const Point3f& newOrigin)
        {
            m_origin = newOrigin;
        }

        void setDirection(const UnitVec3f& newDirection)
        {
            m_direction = newDirection;
            m_invDirection = safeInv(m_direction);
            m_signs = m_direction < 0.0f;
        }

        void translate(const Vec3f& v)
        {
            m_origin += v;
        }

        [[nodiscard]] Ray translated(const Vec3f& v) const
        {
            return Ray(m_origin + v, m_direction);
        }

    private:
        Point3f m_origin;
        UnitVec3f m_direction;
        Vec3f m_invDirection;
        Vec3Mask<float> m_signs;

        [[nodiscard]] Vec3f safeInv(const UnitVec3f& n) const
        {
            return rcp(Vec3f::blend(n, Vec3f::broadcast(0.00001f), abs(n) < 0.00001f));
        }
    };
}
