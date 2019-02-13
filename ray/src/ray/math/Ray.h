#pragma once

#include <ray/math/Vec3.h>

#include <array>

namespace ray
{
    struct Ray
    {
        static Ray between(const Point3f& from, const Point3f& to)
        {
            return Ray(from, (to - from).normalized());
        }

        Ray(const Point3f& origin, const Normal3f& direction) :
            m_origin(origin),
            m_direction(direction),
            m_invDirection(m_direction.reciprocal()),
            m_signs(m_direction < 0.0f)
        {

        }

        constexpr const Point3f& origin() const
        {
            return m_origin;
        }

        constexpr const Normal3f& direction() const
        {
            return m_direction;
        }

        constexpr const Vec3f& invDirection() const
        {
            return m_invDirection;
        }

        constexpr Vec3Mask<float> signs() const
        {
            return m_signs;
        }

        void setOrigin(const Point3f& newOrigin)
        {
            m_origin = newOrigin;
        }

        void setDirection(const Normal3f& newDirection)
        {
            m_direction = newDirection;
            m_invDirection = m_direction.reciprocal();
            m_signs = m_direction < 0.0f;
        }

        void translate(const Vec3f& v)
        {
            m_origin += v;
        }

        Ray translated(const Vec3f& v) const
        {
            return Ray(m_origin + v, m_direction);
        }

    private:
        Point3f m_origin;
        Normal3f m_direction;
        Vec3f m_invDirection;
        Vec3Mask<float> m_signs;
    };
}
