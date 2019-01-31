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
        using ColorType = ColorRGBi;

        Image(int width, int height) :
            m_width(width),
            m_height(height),
            m_pixelColors(width, height, ColorRGBi(0, 0, 0))
        {

        }

        const ColorType& operator()(int x, int y) const
        {
            return m_pixelColors(x, y);
        }

        ColorType& operator()(int x, int y)
        {
            return m_pixelColors(x, y);
        }

        sf::Image toSfImage() const
        {
            auto pixels = rawRGBAi();

            sf::Image img;
            img.create(m_width, m_height, pixels.data());

            return img;
        }

        std::vector<std::uint8_t> rawRGBAi() const
        {
            std::vector<std::uint8_t> data;
            data.reserve(m_width * m_height * 4);

            for (int y = 0; y < m_height; ++y)
            {
                for (int x = 0; x < m_width; ++x)
                {
                    ColorType pixel = m_pixelColors(x, y);
                    data.emplace_back(pixel.r);
                    data.emplace_back(pixel.g);
                    data.emplace_back(pixel.b);
                    data.emplace_back(static_cast<std::uint8_t>(255));
                }
            }

            return data;
        }

        int width() const
        {
            return m_width;
        }

        int height() const
        {
            return m_height;
        }

    private:
        int m_width;
        int m_height;
        Array2<ColorType> m_pixelColors;
    };
}
