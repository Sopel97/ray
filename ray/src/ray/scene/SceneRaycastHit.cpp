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

    bool ResolvedRaycastHit::next(const Ray& ray, ResolvableRaycastHit& hit) const
    {
        if (!owner || !isLocallyContinuable) return false;

        return owner->queryLocal(ray, shapeNo, hit);
    }

    SceneObjectId ResolvedRaycastHit::objectId() const
    {
        return owner->id(shapeNo);
    }
}
