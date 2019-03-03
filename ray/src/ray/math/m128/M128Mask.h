#pragma once

#include <xmmintrin.h>
#include <smmintrin.h>

#include <cstdint>

namespace ray
{
    namespace m128
    {
        template <unsigned L0, unsigned L1 = L0, unsigned L2 = L1, unsigned L3 = L2>
        [[nodiscard]] inline __m128 mask()
        {
            static_assert(L0 < 4 && L1 < 4 && L2 < 4 && L3 < 4);
            constexpr unsigned mask = (1 << L0) | (1 << L1) | (1 << L2) | (1 << L3);
            return _mm_castsi128_ps(_mm_set_epi32(
                mask & 0b1000 ? 0xFFFFFFFF : 0,
                mask & 0b0100 ? 0xFFFFFFFF : 0,
                mask & 0b0010 ? 0xFFFFFFFF : 0,
                mask & 0b0001 ? 0xFFFFFFFF : 0
            ));
        }

        [[nodiscard]] inline __m128 mask_xyzw() { return mask<0, 1, 2, 3>(); }
        [[nodiscard]] inline __m128 mask_xyz() { return mask<0, 1, 2>(); }
        [[nodiscard]] inline __m128 mask_xyw() { return mask<0, 1, 3>(); }
        [[nodiscard]] inline __m128 mask_xzw() { return mask<0, 2, 3>(); }
        [[nodiscard]] inline __m128 mask_yzw() { return mask<1, 2, 3>(); }
        [[nodiscard]] inline __m128 mask_xy() { return mask<0, 1>(); }
        [[nodiscard]] inline __m128 mask_xz() { return mask<0, 2>(); }
        [[nodiscard]] inline __m128 mask_xw() { return mask<0, 3>(); }
        [[nodiscard]] inline __m128 mask_yz() { return mask<1, 2>(); }
        [[nodiscard]] inline __m128 mask_yw() { return mask<1, 3>(); }
        [[nodiscard]] inline __m128 mask_zw() { return mask<2, 3>(); }
        [[nodiscard]] inline __m128 mask_x() { return mask<0>(); }
        [[nodiscard]] inline __m128 mask_y() { return mask<1>(); }
        [[nodiscard]] inline __m128 mask_z() { return mask<2>(); }
        [[nodiscard]] inline __m128 mask_w() { return mask<3>(); }

        [[nodiscard]] inline __m128 makeMask(bool x, bool y, bool z, bool w)
        {
            return _mm_castsi128_ps(_mm_set_epi32(w ? 0xFFFFFFFF : 0, z ? 0xFFFFFFFF : 0, y ? 0xFFFFFFFF : 0, x ? 0xFFFFFFFF : 0));
        }

        inline const __m128 s_masks[16] = {
            mask<0, 0, 0, 0>(),
            mask<1, 0, 0, 0>(),
            mask<0, 1, 0, 0>(),
            mask<1, 1, 0, 0>(),
            mask<0, 0, 1, 0>(),
            mask<1, 0, 1, 0>(),
            mask<0, 1, 1, 0>(),
            mask<1, 1, 1, 0>(),
            mask<0, 0, 0, 1>(),
            mask<1, 0, 0, 1>(),
            mask<0, 1, 0, 1>(),
            mask<1, 1, 0, 1>(),
            mask<0, 0, 1, 1>(),
            mask<1, 0, 1, 1>(),
            mask<0, 1, 1, 1>(),
            mask<1, 1, 1, 1>()
        };

        [[nodiscard]] inline __m128 mask(bool x, bool y, bool z, bool w)
        {
            return s_masks[static_cast<std::size_t>(x) | (y << 1) | (z << 2) | (w << 3)];
        }

        [[nodiscard]] inline __m128 mask(int i)
        {
            return s_masks[static_cast<std::size_t>(1) << i];
        }
    }
}
