#pragma once

#include "Vec3.h"

namespace ray
{
    struct Ray
    {
        constexpr Ray(const Point3f& origin, const Normal3f& direction) :
            m_origin(origin),
            m_direction(direction)
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

        void setOrigin(const Point3f& newOrigin)
        {
            m_origin = newOrigin;
        }

        void setDirection(const Normal3f& newDirection)
        {
            m_direction = newDirection;
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
    };
}
