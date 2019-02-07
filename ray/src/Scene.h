#pragma once

#include <optional>
#include <vector>

#include "Vec3.h"

namespace ray
{
    struct Ray;
    struct ResolvableRaycastHit;
    struct LightHandle;
    struct ColorRGBf;
    struct RaycastQueryStats;

    struct Scene
    {
        virtual std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray, RaycastQueryStats* stats = nullptr) const = 0;
        virtual const std::vector<LightHandle>& lights() const = 0;
        virtual const ColorRGBf& backgroundColor() const = 0;
    };
}
