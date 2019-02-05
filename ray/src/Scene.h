#pragma once

#include <optional>
#include <vector>

#include "Vec3.h"

namespace ray
{
    struct Ray;
    struct ResolvableRaycastHit;

    struct Scene
    {
        virtual std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray) const = 0;
        virtual std::vector<ResolvableRaycastHit> queryVisibleLights(const Point3f& point) const = 0;
        virtual const ColorRGBf& backgroundColor() const = 0;
    };
}
