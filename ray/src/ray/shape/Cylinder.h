#pragma once

#include "Box3.h"
#include "Disc3.h"

#include <ray/math/Vec3.h>

namespace ray
{
    struct Cylinder
    {
        Point3f begin;
        UnitVec3f axis;
        float length;
        float radius;

        Cylinder() noexcept = default;

        Cylinder(const Point3f& begin, const Point3f& end, float radius) noexcept :
            begin(begin),
            axis((end - begin).normalized()),
            length((end - begin).length()),
            radius(radius)
        {
        }

        [[nodiscard]] Point3f center() const
        {
            return begin + axis * (0.5f * length);
        }

        [[nodiscard]] Vec3f extent() const
        {
            return axis * length;
        }
    };
}
