#pragma once

#include "Array2.h"
#include "Color.h"

#include <SFML/Graphics/Image.hpp>

#include <cstdint>
#include <vector>

namespace ray
{
    struct Image
    {
        using ColorType = ColorRGBi;

        Image(int width, int height) :
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
            img.create(width(), height(), pixels.data());

            return img;
        }

        std::vector<std::uint8_t> rawRGBAi() const
        {
            std::vector<std::uint8_t> data;
            const int w = width();
            const int h = height();
            data.reserve(w * h * 4);

            for (int y = 0; y < h; ++y)
            {
                for (int x = 0; x < w; ++x)
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
            return m_pixelColors.width();
        }

        int height() const
        {
            return m_pixelColors.height();
        }

    private:
        Array2<ColorType> m_pixelColors;
    };
}
