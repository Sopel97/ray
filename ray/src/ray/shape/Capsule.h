#pragma once

#include "Box3.h"

#include <ray/math/Vec3.h>

namespace ray
{
    struct Capsule
    {
        Point3f begin;
        Normal3f axis;
        float length;
        float radius;

        Capsule() noexcept = default;

        Capsule(const Point3f& begin, const Point3f& end, float radius) noexcept :
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
