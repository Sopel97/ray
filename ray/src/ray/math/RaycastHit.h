#pragma once

#include <ray/material/Material.h>

namespace ray
{
    // for bounding volumes we don't need that much information
    struct RaycastBvHit
    {
        float dist;
    };

    struct RaycastHit
    {
        float dist;
        Point3f point;
        Normal3f normal;
        int shapeInPackNo;
        MaterialIndex materialIndex;
        bool isInside;
    };
}
