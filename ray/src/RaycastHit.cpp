#include "RaycastHit.h"

#include "SceneObject.h"
#include "SceneObjectCollection.h"

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

    std::optional<ResolvableRaycastHit> ResolvedRaycastHit::next(const Ray& ray, RaycastQueryStats* stats) const
    {
        if (!owner || !isLocallyContinuable) return std::nullopt;

        return owner->queryLocal(ray, shapeNo, stats);
    }

    SceneObjectId ResolvedRaycastHit::objectId() const
    {
        return owner->id(shapeNo);
    }
}
