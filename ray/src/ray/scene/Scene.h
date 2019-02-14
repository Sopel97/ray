#pragma once

#include <optional>
#include <vector>

namespace ray
{
    struct Ray;
    struct ResolvableRaycastHit;
    struct LightHandle;
    struct ColorRGBf;

    struct Scene
    {
        virtual std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray, float& tNearest) const = 0;
        virtual const std::vector<LightHandle>& lights() const = 0;
        virtual const ColorRGBf& backgroundColor() const = 0;
    };
}
