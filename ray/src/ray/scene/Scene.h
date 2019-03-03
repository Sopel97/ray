#pragma once

#include <optional>
#include <vector>

namespace ray
{
    struct MediumMaterial;
    struct Ray;
    struct ResolvableRaycastHit;
    struct LightHandle;
    struct ColorRGBf;

    struct Scene
    {
        [[nodiscard]] virtual bool queryNearest(const Ray& ray, ResolvableRaycastHit& hit) const = 0;
        [[nodiscard]] virtual const std::vector<LightHandle>& lights() const = 0;
        [[nodiscard]] virtual const ColorRGBf& backgroundColor() const = 0;
        [[nodiscard]] virtual const MediumMaterial* mediumMaterial() const = 0;
        [[nodiscard]] virtual float backgroundDistance() const = 0;
    };
}
