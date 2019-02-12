#pragma once

#include <ray/math/Vec3.h>

#include <array>

namespace ray
{
    struct Ray
    {
        static constexpr Ray between(const Point3f& from, const Point3f& to)
        {
            return Ray(from, (to - from).normalized());
        }

        constexpr Ray(const Point3f& origin, const Normal3f& direction) :
            m_origin(origin),
            m_direction(direction),
            m_invDirection(m_direction.reciprocal()),
            m_signs{
                m_direction.x < 0.0f,
                m_direction.y < 0.0f,
                m_direction.z < 0.0f
            }
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

        constexpr bool sign(int i) const
        {
            return m_signs[i];
        }

        constexpr std::array<bool, 3> signs() const
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
            m_signs = {
                m_direction.x < 0.0f,
                m_direction.y < 0.0f,
                m_direction.z < 0.0f
            };
        }

        void translate(const Vec3f& v)
        {
            m_origin += v;
        }

        constexpr Ray translated(const Vec3f& v) const
        {
            return Ray(m_origin + v, m_direction);
        }

    private:
        Point3f m_origin;
        Normal3f m_direction;
        Vec3f m_invDirection;
        std::array<bool, 3> m_signs;
    };
}
