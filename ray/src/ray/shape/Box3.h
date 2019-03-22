#pragma once

#include <ray/math/Vec3.h>

#include <array>

namespace ray
{
    struct Box3
    {
        Point3f min, max;

        Box3() noexcept :
            min{},
            max{}
        {
        }

        Box3(const Point3f& min, const Point3f& max) noexcept :
            min(min),
            max(max)
        {
        }

        [[nodiscard]] Point3f center() const
        {
            return (min + (max - min) * 0.5f);
        }

        [[nodiscard]] Vec3f extent() const
        {
            return max - min;
        }

        void extend(const Point3f& point)
        {
            min = Point3f::blend(min, point, point < min);
            max = Point3f::blend(max, point, point > max);
        }

        void extend(const Box3& box)
        {
            min = Point3f::blend(min, box.min, box.min < min);

            if (box.max.x > max.x) max.x = box.max.x;
            if (box.max.y > max.y) max.y = box.max.y;
            if (box.max.z > max.z) max.z = box.max.z;
        }

        [[nodiscard]] std::array<Point3f, 8> vertices() const
        {
            std::array<Point3f, 8> v;
            Point3f origin = center();
            Vec3f halfSize = extent() / 2.0f;

            v[0] = origin + Vec3f(halfSize.x, halfSize.y, halfSize.z);
            v[1] = origin + Vec3f(halfSize.x, halfSize.y, -halfSize.z);
            v[2] = origin + Vec3f(halfSize.x, -halfSize.y, halfSize.z);
            v[3] = origin + Vec3f(halfSize.x, -halfSize.y, -halfSize.z);
            v[4] = origin + Vec3f(-halfSize.x, halfSize.y, halfSize.z);
            v[5] = origin + Vec3f(-halfSize.x, halfSize.y, -halfSize.z);
            v[6] = origin + Vec3f(-halfSize.x, -halfSize.y, halfSize.z);
            v[7] = origin + Vec3f(-halfSize.x, -halfSize.y, -halfSize.z);

            return v;
        }
    };
}
