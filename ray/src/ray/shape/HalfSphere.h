#pragma once

#include "Box3.h"

#include <ray/math/Vec3.h>

namespace ray
{
    // All points P on sphere for which
    // dot3(m_normal, P - m_center) > 0
    // Sphere is solid, HalfSphere is hollow - just surface
    struct HalfSphere
    {
        HalfSphere() noexcept :
            m_center{},
            m_normal{},
            m_radius{}
        {

        }

        HalfSphere(const Point3f& center, const Normal3f& normal, float radius) noexcept :
            m_center(center),
            m_normal(normal),
            m_radius(radius)
        {

        }

        [[nodiscard]] const Point3f& center() const
        {
            return m_center;
        }

        [[nodiscard]] const Normal3f& normal() const
        {
            return m_normal;
        }

        [[nodiscard]] float radius() const
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

        [[nodiscard]] HalfSphere translated(const Vec3f& v) const
        {
            return HalfSphere(m_center + v, m_normal, m_radius);
        }

    private:
        Point3f m_center;
        Normal3f m_normal;
        float m_radius;
    };
}
