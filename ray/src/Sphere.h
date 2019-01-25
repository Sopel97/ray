#pragma once

#include "Vec3.h"

namespace ray
{
    struct Sphere
    {
        constexpr Sphere() :
            m_center{},
            m_radius{}
        {

        }

        constexpr Sphere(const Point3f& center, float radius) :
            m_center(center),
            m_radius(radius)
        {

        }

        constexpr const Point3f& center() const
        {
            return m_center;
        }

        constexpr float radius() const
        {
            return m_radius;
        }

        void setCenter(const Point3f& newCenter)
        {
            m_center = newCenter;
        }

        void setRadius(float newRadius)
        {
            m_radius = newRadius;
        }

        void translate(const Vec3f& v)
        {
            m_center += v;
        }

        Sphere translated(const Vec3f& v) const
        {
            return Sphere(m_center + v, m_radius);
        }

    private:
        Point3f m_center;
        float m_radius;
    };
}
