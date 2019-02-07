#pragma once

#include "SceneObject.h"

#include <optional>

namespace ray
{
    struct Ray;
    struct ResolvedRaycastHit;
    struct ResolvableRaycastHit;
    struct RaycastQueryStats;

    struct SceneObjectCollection
    {
        virtual std::optional<ResolvableRaycastHit> queryLocal(const Ray& ray, int shapeNo, RaycastQueryStats* stats = nullptr) const = 0;
        virtual ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const = 0;
        virtual SceneObjectId id(int shapeNo) const = 0;
    };
}
