#pragma once

#include "Vec3.h"

namespace ray
{
    struct Ray
    {
        constexpr Ray(const Vec3f& origin, const Vec3f& direction) :
            m_origin(origin),
            m_direction(direction.normalized())
        {

        }

        constexpr const Vec3f& origin() const
        {
            return m_origin;
        }

        constexpr const Vec3f& direction() const
        {
            return m_direction;
        }

        void setOrigin(const Vec3f& newOrigin)
        {
            m_origin = newOrigin;
        }

        void setDirection(const Vec3f& newDirection)
        {
            m_direction = newDirection.normalized();
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
        Vec3f m_origin;
        Vec3f m_direction;
    };
}
