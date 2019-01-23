#pragma once

#include <algorithm>
#include <memory>

namespace ray
{
    template <typename T>
    struct Array2
    {
        Array2(int width, int height) :
            m_data(std::make_unique<T[]>(width * height)),
            m_width(width),
            m_height(height)
        {

        }

        Array2(int width, int height, const T& v) :
            m_data(std::make_unique<T[]>(width * height)),
            m_width(width),
            m_height(height)
        {
            std::fill(m_data.get(), m_data.get() + width * height, v);
        }

        const T& operator()(int x, int y) const
        {
            return m_data[x * m_height + y];
        }

        T& operator()(int x, int y)
        {
            return m_data[x * m_height + y];
        }

    private:
        std::unique_ptr<T[]> m_data;
        int m_width;
        int m_height;
    };
}
