#pragma once

#include "Texture.h"

#include <cmath>

namespace ray
{
    struct SquarePattern : Texture
    {
        SquarePattern(const ColorRGBf& primaryColor, const ColorRGBf& secondaryColor, float scale) :
            m_colors{primaryColor, secondaryColor},
            m_scale(scale)
        {

        }

        ColorRGBf sample(const TexCoords& coords) const override
        {
            const int idx = (std::fmod(coords.u * m_scale, 1.0f) > 0.5f) != (std::fmod(coords.v * m_scale, 1.0f) > 0.5f);
            return m_colors[idx];
        }

    private:
        ColorRGBf m_colors[2];
        float m_scale;
    };
}
