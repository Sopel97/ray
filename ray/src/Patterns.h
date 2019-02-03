#pragma once

#include "Texture.h"

#include <cmath>

namespace ray
{
    struct SquarePattern : Texture
    {
        SquarePattern(const ColorRGBf& primaryColor, const ColorRGBf& secondaryColor, float scale) :
            m_primaryColor(primaryColor),
            m_secondaryColor(secondaryColor),
            m_scale(scale)
        {

        }

        ColorRGBf sample(const TexCoords& coords) const override
        {
            return (std::fmod(coords.u * m_scale, 1.0f) > 0.5f) ^ (std::fmod(coords.v * m_scale, 1.0f) > 0.5f) ? m_primaryColor : m_secondaryColor;
        }

    private:
        ColorRGBf m_primaryColor;
        ColorRGBf m_secondaryColor;
        float m_scale;
    };
}
