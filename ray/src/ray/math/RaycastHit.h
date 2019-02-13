#pragma once

namespace ray
{
    // for bounding volumes we don't need that much information
    struct RaycastBvHit
    {
        float distSqr;
    };

    struct RaycastHit
    {
        Point3f point;
        Normal3f normal;
        int shapeInPackNo;
        int materialNo;
        bool isInside;
    };
}
