#pragma once

#include <xmmintrin.h>
#include <smmintrin.h>

namespace ray
{
    namespace m128
    {
        namespace lane
        {
            static constexpr int x = 0;
            static constexpr int y = 1;
            static constexpr int z = 2;
            static constexpr int w = 3;

            template <bool X, bool Y, bool Z, bool W>
            [[nodiscard]] inline int mask()
            {
                return static_cast<int>(X) + (Y << 1) + (Z << 2) + (W << 3);
            }
        }
    }
}