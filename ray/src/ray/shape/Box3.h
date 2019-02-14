#pragma once

#include <ray/math/Vec3.h>

namespace ray
{
    struct Box3
    {
        Point3f min, max;

        Box3() :
            min{},
            max{}
        {
        }

        Box3(const Point3f& min, const Point3f& max) :
            min(min),
            max(max)
        {
        }

        Point3f center() const
        {
            return (min + (max - min) * 0.5f);
        }

        Vec3f extent() const
        {
            return max - min;
        }

        const Box3& aabb() const
        {
            return *this;
        }

        void extend(const Point3f& point)
        {
            if (point.x < min.x) min.x = point.x;
            if (point.y < min.y) min.y = point.y;
            if (point.z < min.z) min.z = point.z;

            if (point.x > max.x) max.x = point.x;
            if (point.y > max.y) max.y = point.y;
            if (point.z > max.z) max.z = point.z;
        }

        void extend(const Box3& box)
        {
            if (box.min.x < min.x) min.x = box.min.x;
            if (box.min.y < min.y) min.y = box.min.y;
            if (box.min.z < min.z) min.z = box.min.z;

            if (box.max.x > max.x) max.x = box.max.x;
            if (box.max.y > max.y) max.y = box.max.y;
            if (box.max.z > max.z) max.z = box.max.z;
        }
    };
}
