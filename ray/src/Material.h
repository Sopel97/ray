#pragma once

#include "Color.h"
#include "Texture.h"

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
        ColorRGBf absorbtion;
        const Texture* texture;

        ColorRGBf sampleTexture(const TexCoords& coords) const
        {
            if (!texture) return ColorRGBf(1.0f, 1.0f, 1.0f);

            return texture->sample(coords);
        }
    };
}
