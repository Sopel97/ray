#include "SceneRaycastHit.h"

#include <ray/scene/object/SceneObjectId.h>
#include <ray/scene/object/SceneObjectCollection.h>

namespace ray
{
    ResolvedRaycastHit ResolvableRaycastHit::resolve() const
    {
        return owner->resolveHit(*this);
    }
    SceneObjectId ResolvableRaycastHit::objectId() const
    {
        return owner->id(shapeNo);
    }

    std::optional<ResolvableRaycastHit> ResolvedRaycastHit::next(const Ray& ray) const
    {
        if (!owner || !isLocallyContinuable) return std::nullopt;

        return owner->queryLocal(ray, shapeNo);
    }

    SceneObjectId ResolvedRaycastHit::objectId() const
    {
        return owner->id(shapeNo);
    }
}