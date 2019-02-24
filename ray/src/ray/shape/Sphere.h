#pragma once

#include "Box3.h"

#include <ray/math/Vec3.h>

namespace ray
{
    struct Sphere
    {
        Sphere() :
            m_center{},
            m_radius{}
        {

        }

        Sphere(const Point3f& center, float radius) :
            m_center(center),
            m_radius(radius)
        {

        }

        const Point3f& center() const
        {
            return m_center;
        }

        float radius() const
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
