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
        Point3f point;
        Normal3f normal;
        float dist;
        int shapeInPackNo;
        MaterialIndex materialIndex;
        bool isInside;
        const void* additionalData;
    };
}
