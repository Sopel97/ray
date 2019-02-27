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
        virtual bool queryNearest(const Ray& ray, ResolvableRaycastHit& hit) const = 0;
        virtual const std::vector<LightHandle>& lights() const = 0;
        virtual const ColorRGBf& backgroundColor() const = 0;
        virtual const MediumMaterial* mediumMaterial() const = 0;
        virtual float backgroundDistance() const = 0;
    };
}
