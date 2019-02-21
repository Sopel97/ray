#pragma once

#include "Box3.h"

#include <ray/math/Vec3.h>

#include <array>

namespace ray
{
    struct OrientedBox3
    {
        Point3f origin;
        Vec3f halfSize;
        Normal3f axes[3];

        const Point3f& center() const
        {
            return origin;
        }

        Point3f min() const
        {
            return origin - halfSize.x * axes[0] - halfSize.y * axes[1] - halfSize.z * axes[2];
        }

        Vec3f extent() const
        {
            const Point3f min = origin - halfSize.x * axes[0] - halfSize.y * axes[1] - halfSize.z * axes[2];
            const Point3f max = origin + halfSize.x * axes[0] + halfSize.y * axes[1] + halfSize.z * axes[2];
            return max - min;
        }

        std::array<Point3f, 8> vertices() const
        {
            std::array<Point3f, 8> v;

            v[0] = origin + halfSize.x * axes[0] + halfSize.y * axes[1] + halfSize.z * axes[2];
            v[1] = origin + halfSize.x * axes[0] + halfSize.y * axes[1] - halfSize.z * axes[2];
            v[2] = origin + halfSize.x * axes[0] - halfSize.y * axes[1] + halfSize.z * axes[2];
            v[3] = origin + halfSize.x * axes[0] - halfSize.y * axes[1] - halfSize.z * axes[2];
            v[4] = origin - halfSize.x * axes[0] + halfSize.y * axes[1] + halfSize.z * axes[2];
            v[5] = origin - halfSize.x * axes[0] + halfSize.y * axes[1] - halfSize.z * axes[2];
            v[6] = origin - halfSize.x * axes[0] - halfSize.y * axes[1] + halfSize.z * axes[2];
            v[7] = origin - halfSize.x * axes[0] - halfSize.y * axes[1] - halfSize.z * axes[2];

            return v;
        }

        Box3 aabb() const
        {
            Box3 bb(origin, origin);
            for (const auto& p : vertices())
            {
                bb.extend(p);
            }
            return bb;
        }
    };
}
