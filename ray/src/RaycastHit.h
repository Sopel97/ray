#pragma once

#include "Material.h"
#include "Vec3.h"

namespace ray
{
    struct RaycastHit
    {
        Point3f point;
        Normal3f normal;
        int shapeNo;
        int materialNo;
    };
}
