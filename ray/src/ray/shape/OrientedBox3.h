#pragma once

#include "Box3.h"

#include <ray/math/Transform3.h>
#include <ray/math/Vec3.h>

#include <array>

namespace ray
{
    struct OrientedBox3
    {
        Point3f origin;
        Vec3f halfSize;
        Rotation4f worldToLocalRot;

        OrientedBox3(const Point3f& origin, const Vec3f& halfSize, const Rotation4f& worldToLocalRot) noexcept :
            origin(origin),
            halfSize(halfSize),
            worldToLocalRot(worldToLocalRot)
        {
        }

        [[nodiscard]] const Point3f& center() const
        {
            return origin;
        }

        [[nodiscard]] Point3f min() const
        {
            return origin - (worldToLocalRot.inverse() * halfSize);
        }

        [[nodiscard]] Point3f max() const
        {
            return origin + (worldToLocalRot.inverse() * halfSize);
        }

        [[nodiscard]] Vec3f extent() const
        {
            return 2.0f * (worldToLocalRot.inverse() * halfSize);
        }

        [[nodiscard]] std::array<Point3f, 8> vertices() const
        {
            std::array<Point3f, 8> v;
            const auto localToWorld = worldToLocalRot.inverse();

            v[0] = origin + localToWorld * Vec3f(halfSize.x, halfSize.y, halfSize.z);
            v[1] = origin + localToWorld * Vec3f(halfSize.x, halfSize.y, -halfSize.z);
            v[2] = origin + localToWorld * Vec3f(halfSize.x, -halfSize.y, halfSize.z);
            v[3] = origin + localToWorld * Vec3f(halfSize.x, -halfSize.y, -halfSize.z);
            v[4] = origin + localToWorld * Vec3f(-halfSize.x, halfSize.y, halfSize.z);
            v[5] = origin + localToWorld * Vec3f(-halfSize.x, halfSize.y, -halfSize.z);
            v[6] = origin + localToWorld * Vec3f(-halfSize.x, -halfSize.y, halfSize.z);
            v[7] = origin + localToWorld * Vec3f(-halfSize.x, -halfSize.y, -halfSize.z);

            return v;
        }
    };
}
