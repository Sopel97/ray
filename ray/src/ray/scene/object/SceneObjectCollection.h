#pragma once

#include "SceneObjectId.h"

#include <optional>

namespace ray
{
    struct Ray;
    struct ResolvedRaycastHit;
    struct ResolvableRaycastHit;

    struct HomogeneousSceneObjectCollection
    {
        [[nodiscard]] virtual bool queryLocal(const Ray& ray, int shapeNo, ResolvableRaycastHit& hit) const = 0;
        [[nodiscard]] virtual ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const = 0;
        [[nodiscard]] virtual SceneObjectId id(int shapeNo) const = 0;
    };

    // Only construction
    struct StaticHeterogeneousSceneObjectCollection
    {

    };

    // Allows updates
    struct DynamicHeterogeneousSceneObjectCollection : StaticHeterogeneousSceneObjectCollection
    {

    };
}
