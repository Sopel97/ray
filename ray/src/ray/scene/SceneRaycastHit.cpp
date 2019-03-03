#include "SceneRaycastHit.h"

#include <ray/scene/object/SceneObjectId.h>
#include <ray/scene/object/SceneObjectCollection.h>

namespace ray
{
    [[nodiscard]] ResolvedRaycastHit ResolvableRaycastHit::resolve() const
    {
        return owner->resolveHit(*this);
    }
    [[nodiscard]] SceneObjectId ResolvableRaycastHit::objectId() const
    {
        return owner->id(shapeNo);
    }

    [[nodiscard]] bool ResolvedRaycastHit::next(const Ray& ray, ResolvableRaycastHit& hit) const
    {
        if (!owner || !isLocallyContinuable) return false;

        return owner->queryLocal(ray, shapeNo, hit);
    }

    [[nodiscard]] SceneObjectId ResolvedRaycastHit::objectId() const
    {
        return owner->id(shapeNo);
    }
}
