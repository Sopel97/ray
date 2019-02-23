#pragma once

#include "Box3.h"
#include "Disc3.h"

#include <ray/math/Vec3.h>

namespace ray
{
    struct Cylinder
    {
        Point3f begin;
        Normal3f axis;
        float length;
        float radius;

        Cylinder() = default;

        Cylinder(const Point3f& begin, const Point3f& end, float radius) :
            begin(begin),
            axis((end - begin).normalized()),
            length((end - begin).length()),
            radius(radius)
        {
        }

        Point3f center() const
        {
            return begin + axis * (0.5f * length);
        }

        Box3 aabb() const
        {
            Disc3 dBegin(begin, axis, radius);
            Disc3 dEnd(begin + axis * length, axis, radius);
            Box3 bb = dBegin.aabb();
            bb.extend(dEnd.aabb());
            return bb;
        }

        Vec3f extent() const
        {
            return axis * length;
        }
    };
}
