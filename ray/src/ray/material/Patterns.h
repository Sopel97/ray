#pragma once

#include "Texture.h"

#include <cmath>

namespace ray
{
    struct SquarePattern : Texture
    {
        SquarePattern(const ColorRGBf& primaryColor, const ColorRGBf& secondaryColor, float scale) noexcept :
            m_colors{primaryColor, secondaryColor},
            m_scale(scale)
        {

        }

        [[nodiscard]] ColorRGBf sample(const TexCoords& coords) const override
        {
            const int idx = (frac(coords.u * m_scale) > 0.5f) != (frac(coords.v * m_scale) > 0.5f);
            return m_colors[idx];
        }

    private:
        ColorRGBf m_colors[2];
        float m_scale;

        [[nodiscard]] float frac(float a) const
        {
            return a - std::floor(a);
        }
    };
}
