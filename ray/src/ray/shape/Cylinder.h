#pragma once

#include "Box3.h"
#include "Disc3.h"

#include <ray/math/Vec3.h>

namespace ray
{
    struct Cylinder
    {
        Point3f begin;
        Point3f end;
        float radius;

        Cylinder() = default;

        Cylinder(const Point3f& begin, const Point3f& end, float radius) :
            begin(begin),
            end(end),
            radius(radius)
        {
        }

        Point3f center() const
        {
            return begin + (end - begin) * 0.5f;
        }

        Box3 aabb() const
        {
            const Normal3f n = (end - begin).normalized();
            Disc3 dBegin(begin, n, radius);
            Disc3 dEnd(end, n, radius);
            Box3 bb = dBegin.aabb();
            bb.extend(dEnd.aabb());
            return bb;
        }

        Vec3f axis() const
        {
            return end - begin;
        }
    };
}
