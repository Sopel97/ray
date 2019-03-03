#pragma once

namespace ray
{
    struct TexCoords;
    struct ColorRGBf;

    struct Texture
    {
        [[nodiscard]] virtual ColorRGBf sample(const TexCoords& coords) const = 0;
    };
}
