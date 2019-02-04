#pragma once

#include <optional>

namespace ray
{
    struct ResolvedRaycastHit;

    struct SceneObjectCollection
    {
        virtual std::optional<ResolvedRaycastHit> queryLocal(const Ray& ray, int shapeNo) const = 0;
    };
}
