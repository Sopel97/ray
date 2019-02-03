#pragma once

#include "Color.h"
#include "TexCoords.h"

namespace ray
{
    struct Texture
    {
        virtual ColorRGBf sample(const TexCoords& coords) const = 0;
    };
}
