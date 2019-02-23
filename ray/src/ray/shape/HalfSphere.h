#pragma once

#include "Box3.h"

#include <ray/math/Vec3.h>

namespace ray
{
    // All points P on sphere for which
    // dot(m_normal, P - m_center) > 0
    struct HalfSphere
    {
        HalfSphere() :
            m_center{},
            m_normal{},
            m_radius{}
        {

        }

        HalfSphere(const Point3f& center, const Normal3f& normal, float radius) :
            m_center(center),
            m_normal(normal),
            m_radius(radius)
        {

        }

        Box3 aabb() const
        {
            const Vec3f halfExtent(m_radius, m_radius, m_radius);
            return Box3(m_center - halfExtent, m_center + halfExtent);
        }

        const Point3f& center() const
        {
            return m_center;
        }

        const Normal3f& normal() const
        {
            return m_normal;
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

        HalfSphere translated(const Vec3f& v) const
        {
            return HalfSphere(m_center + v, m_normal, m_radius);
        }

    private:
        Point3f m_center;
        Normal3f m_normal;
        float m_radius;
    };
}
