#pragma once

#include "Material.h"
#include "Vec3.h"

namespace ray
{
    struct RaycastHit
    {
        Vec3f point;
        Vec3f normal;
        int shapeNo;
        int materialNo;
    };
}
