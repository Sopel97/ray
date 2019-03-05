#pragma once

#include "Color.h"
#include <ray/math/Vec3.h>

namespace ray
{
    struct SurfaceShaderOutput
    {
        Point3f point;
        Normal3f normal;
        float dist;
        ColorRGBf surfaceColor;
        ColorRGBf emissionColor;
        float transparency;
        float reflectivity;
        float diffuse;
    };
}