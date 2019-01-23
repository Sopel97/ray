#pragma once

#include "Array2.h"
#include "Color.h"

#include <cstdint>
#include <vector>

#include <SFML/Graphics/Image.hpp>

namespace ray
{
    struct Image
    {
        Image(int width, int height) :
            m_width(width),
            m_height(height),
            m_pixelColors(width, height, ColorRGBi8(255, 255, 0))
        {

        }

        sf::Image toSfImage() const
        {
            auto pixels = rawRGBAi8();

            sf::Image img;
            img.create(m_width, m_height, pixels.data());

            return img;
        }

        std::vector<std::uint8_t> rawRGBAi8() const
        {
            std::vector<std::uint8_t> data;
            data.reserve(m_width * m_height * 4);

            for (int y = 0; y < m_height; ++y)
            {
                for (int x = 0; x < m_width; ++x)
                {
                    ColorRGBi8 pixel = m_pixelColors(x, y);
                    data.emplace_back(pixel.r);
                    data.emplace_back(pixel.g);
                    data.emplace_back(pixel.b);
                    data.emplace_back(static_cast<std::uint8_t>(255));
                }
            }

            return data;
        }

    private:
        int m_width;
        int m_height;
        Array2<ColorRGBi8> m_pixelColors;
    };
}
