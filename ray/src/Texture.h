#pragma once

namespace ray
{
    struct TexCoords;
    struct ColorRGBf;

    struct Texture
    {
        virtual ColorRGBf sample(const TexCoords& coords) const = 0;
    };
}
