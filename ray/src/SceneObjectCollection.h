#pragma once

#include <optional>

namespace ray
{
    struct ResolvedRaycastHit;
    struct ResolvableRaycastHit;
    struct Ray;

    struct SceneObjectCollection
    {
        virtual std::optional<ResolvableRaycastHit> queryLocal(const Ray& ray, int shapeNo) const = 0;
        virtual ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const = 0;
    };
}
