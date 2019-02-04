#include "RaycastHit.h"

namespace ray
{
    ResolvedRaycastHit ResolvableRaycastHit::resolve() const
    {
        return owner->resolveHit(*this);
    }
}