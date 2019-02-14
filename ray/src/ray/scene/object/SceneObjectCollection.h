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
        virtual bool queryLocal(const Ray& ray, int shapeNo, ResolvableRaycastHit& hit) const = 0;
        virtual ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const = 0;
        virtual SceneObjectId id(int shapeNo) const = 0;
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
