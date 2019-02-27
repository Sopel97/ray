#pragma once

#include "M128Swizzle.h"

#include <xmmintrin.h>
#include <smmintrin.h>

namespace ray
{
    namespace detail
    {
        inline void spill3(__m128 vec, __m128& xxxx, __m128& yyyy, __m128& zzzz)
        {
            xxxx = _mm_shuffle_ps(vec, vec, _MM_SHUFFLE(0, 0, 0, 0));
            yyyy = _mm_shuffle_ps(vec, vec, _MM_SHUFFLE(1, 1, 1, 1));
            zzzz = _mm_shuffle_ps(vec, vec, _MM_SHUFFLE(2, 2, 2, 2));
        }

        inline void spill(__m128 vec, __m128& xxxx, __m128& yyyy, __m128& zzzz, __m128& wwww)
        {
            xxxx = _mm_shuffle_ps(vec, vec, _MM_SHUFFLE(0, 0, 0, 0));
            yyyy = _mm_shuffle_ps(vec, vec, _MM_SHUFFLE(1, 1, 1, 1));
            zzzz = _mm_shuffle_ps(vec, vec, _MM_SHUFFLE(2, 2, 2, 2));
            wwww = _mm_shuffle_ps(vec, vec, _MM_SHUFFLE(3, 3, 3, 3));
        }

        template <unsigned L0, unsigned L1 = L0, unsigned L2 = L1, unsigned L3 = L2>
        inline __m128 mask128()
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

        inline __m128 mask_xyzw() { return mask128<0, 1, 2, 3>(); }
        inline __m128 mask_xyz() { return mask128<0, 1, 2>(); }
        inline __m128 mask_xyw() { return mask128<0, 1, 3>(); }
        inline __m128 mask_xzw() { return mask128<0, 2, 3>(); }
        inline __m128 mask_yzw() { return mask128<1, 2, 3>(); }
        inline __m128 mask_xy() { return mask128<0, 1>(); }
        inline __m128 mask_xz() { return mask128<0, 2>(); }
        inline __m128 mask_xw() { return mask128<0, 3>(); }
        inline __m128 mask_yz() { return mask128<1, 2>(); }
        inline __m128 mask_yw() { return mask128<1, 3>(); }
        inline __m128 mask_zw() { return mask128<2, 3>(); }
        inline __m128 mask_x() { return mask128<0>(); }
        inline __m128 mask_y() { return mask128<1>(); }
        inline __m128 mask_z() { return mask128<2>(); }
        inline __m128 mask_w() { return mask128<3>(); }

        inline __m128 mask128(bool x, bool y, bool z, bool w)
        {
            return _mm_castsi128_ps(_mm_set_epi32(w ? 0xFFFFFFFF : 0, z ? 0xFFFFFFFF : 0, y ? 0xFFFFFFFF : 0, x ? 0xFFFFFFFF : 0));
        }

        // for xyzw vector make it xyz0
        inline __m128 truncate3(__m128 v)
        {
            return _mm_and_ps(v, mask_xyz());
        }

        inline __m128 undefined_ps()
        {
            // msvc lacks _mm_undefined_ps
            // maybe there is a better way?
            return _mm_setzero_ps();
        }

        inline float hadd(__m128 a)
        {
            __m128 shuf = _mm_movehdup_ps(a); // broadcast elements 3,1 to 2,0
            __m128 sums = _mm_add_ps(a, shuf);
            shuf = _mm_movehl_ps(shuf, sums); // high half -> low half
            sums = _mm_add_ss(sums, shuf);
            return _mm_cvtss_f32(sums);
        }

        // ignores the last component
        inline float hadd3(__m128 a)
        {
            // a = x y z _
            __m128 shuf = _mm_movehdup_ps(a); // broadcast elements 3,1 to 2,0
            // shuf = y y _ _
            __m128 sums = _mm_add_ps(a, shuf);
            // sums = x+y, 2y, z+_, 2_
            a = _mm_movehl_ps(a, a); // high half -> low half
            // a = z, _, z, _
            sums = _mm_add_ss(sums, a);
            // sums = x+y+z, ...
            return _mm_cvtss_f32(sums);
        }

        inline __m128 abs(__m128 a)
        {
            return _mm_and_ps(a, _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF)));
        }

        inline __m128 sub(__m128 a, __m128 b)
        {
            return _mm_sub_ps(a, b);
        }

        inline __m128 add(__m128 a, __m128 b)
        {
            return _mm_add_ps(a, b);
        }

        inline __m128 add(__m128 a, __m128 b, __m128 c)
        {
            return _mm_add_ps(_mm_add_ps(a, b), c);
        }

        inline __m128 add(__m128 a, __m128 b, __m128 c, __m128 d)
        {
            return _mm_add_ps(_mm_add_ps(a, b), _mm_add_ps(c, d));
        }

        inline __m128 div(__m128 a, __m128 b)
        {
            return _mm_div_ps(a, b);
        }

        inline __m128 mul(__m128 a, __m128 b)
        {
            return _mm_mul_ps(a, b);
        }

        inline __m128 div(float a, __m128 b)
        {
            return _mm_div_ps(_mm_set1_ps(a), b);
        }

        inline __m128 mul(float a, __m128 b)
        {
            return _mm_mul_ps(_mm_set1_ps(a), b);
        }

        inline __m128 div(__m128 a, float b)
        {
            return _mm_div_ps(a, _mm_set1_ps(b));
        }

        inline __m128 mul(__m128 a, float b)
        {
            return _mm_mul_ps(a, _mm_set1_ps(b));
        }

        inline __m128 cmplt(__m128 a, __m128 b)
        {
            return _mm_cmplt_ps(a, b);
        }

        inline __m128 cmplt(float a, __m128 b)
        {
            return _mm_cmplt_ps(_mm_set1_ps(a), b);
        }

        inline __m128 cmplt(__m128 a, float b)
        {
            return _mm_cmplt_ps(a, _mm_set1_ps(b));
        }

        inline __m128 cmple(__m128 a, __m128 b)
        {
            return _mm_cmple_ps(a, b);
        }

        inline __m128 cmple(float a, __m128 b)
        {
            return _mm_cmple_ps(_mm_set1_ps(a), b);
        }

        inline __m128 cmple(__m128 a, float b)
        {
            return _mm_cmple_ps(a, _mm_set1_ps(b));
        }

        inline __m128 cmpeq(__m128 a, __m128 b)
        {
            return _mm_cmpeq_ps(a, b);
        }

        inline __m128 cmpeq(float a, __m128 b)
        {
            return _mm_cmpeq_ps(_mm_set1_ps(a), b);
        }

        inline __m128 cmpeq(__m128 a, float b)
        {
            return _mm_cmpeq_ps(a, _mm_set1_ps(b));
        }

        inline float dot3(__m128 a, __m128 b)
        {
            // mul first 3 components of xmm, sum it, and store in the first component, return first component
            return _mm_cvtss_f32(_mm_dp_ps(a, b, 0b0111'0001));

            // should be faster, but on nehalem it actually isn't?
            // return hadd3(mul(a, b));
        }

        inline float dot(__m128 a, __m128 b)
        {
            return hadd(mul(a, b));
        }

        inline __m128 cross(__m128 a, __m128 b)
        {
            // http://threadlocalmutex.com/?p=8
            __m128 a_yzx = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
            __m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
            __m128 c = _mm_sub_ps(_mm_mul_ps(a, b_yzx), _mm_mul_ps(a_yzx, b));
            return _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 0, 2, 1));
        }

        inline void transpose3(__m128& r0, __m128& r1, __m128& r2)
        {
            __m128 t0 = _mm_shuffle_ps(r0, r1, _MM_SHUFFLE(1, 0, 1, 0));
            __m128 t1 = _mm_shuffle_ps(r0, r1, _MM_SHUFFLE(2, 2, 2, 2));
            r0 = _mm_shuffle_ps(t0, r2, _MM_SHUFFLE(0, 0, 2, 0));
            r1 = _mm_shuffle_ps(t0, r2, _MM_SHUFFLE(0, 1, 3, 1));
            r2 = _mm_shuffle_ps(t1, r2, _MM_SHUFFLE(0, 2, 2, 0));
        }

        inline void transpose(__m128& r0, __m128& r1, __m128& r2, __m128& r3)
        {
            _MM_TRANSPOSE4_PS(r0, r1, r2, r3);
        }

        inline void transpose3(__m128 r[3])
        {
            __m128 t0 = _mm_shuffle_ps(r[0], r[1], _MM_SHUFFLE(1, 0, 1, 0));
            __m128 t1 = _mm_shuffle_ps(r[0], r[1], _MM_SHUFFLE(2, 2, 2, 2));
            r[0] = _mm_shuffle_ps(t0, r[2], _MM_SHUFFLE(0, 0, 2, 0));
            r[1] = _mm_shuffle_ps(t0, r[2], _MM_SHUFFLE(0, 1, 3, 1));
            r[2] = _mm_shuffle_ps(t1, r[2], _MM_SHUFFLE(0, 2, 2, 0));
        }

        inline void transpose(__m128 r[4])
        {
            _MM_TRANSPOSE4_PS(r[0], r[1], r[2], r[3]);
        }

        inline __m128 min(__m128 a, __m128 b)
        {
            return _mm_min_ps(a, b);
        }

        inline __m128 max(__m128 a, __m128 b)
        {
            return _mm_max_ps(a, b);
        }

        inline __m128 sqrt(__m128 a)
        {
            return _mm_sqrt_ps(a);
        }

        inline __m128 clamp(__m128 a, float minv, float maxv)
        {
            return max(min(a, _mm_set1_ps(maxv)), _mm_set1_ps(minv));
        }

        inline __m128 neg(__m128 a)
        {
            return _mm_xor_ps(a, _mm_set1_ps(-0.0f));
        }

        inline __m128 neg(__m128 a, __m128 mask)
        {
            return _mm_xor_ps(_mm_and_ps(mask, _mm_castsi128_ps(_mm_set1_epi32(0x80000000))), a);
        }

        inline __m128 blend(float a, float b, __m128 mask)
        {
            return _mm_blendv_ps(_mm_set1_ps(a), _mm_set1_ps(b), mask);
        }

        inline __m128 blend(__m128 a, float b, __m128 mask)
        {
            return _mm_blendv_ps(a, _mm_set1_ps(b), mask);
        }

        inline __m128 blend(float a, __m128 b, __m128 mask)
        {
            return _mm_blendv_ps(_mm_set1_ps(a), b, mask);
        }

        inline __m128 blend(__m128 a, __m128 b, __m128 mask)
        {
            return _mm_blendv_ps(a, b, mask);
        }
    }
}