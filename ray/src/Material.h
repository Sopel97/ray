#pragma once

#include "Color.h"

namespace ray
{
    struct Material
    {
        ColorRGBf surfaceColor;
        ColorRGBf emissionColor;
        float transparency;
        float refractiveIndex;
        float reflectivity;
        float diffuse;
    };
}
