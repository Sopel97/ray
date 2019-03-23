#pragma once

#include "M128Mask.h"
#include "M128Shuffle.h"

#include <xmmintrin.h>
#include <smmintrin.h>

#include <ray/utility/UtilityMacroDef.h>

namespace ray
{
    namespace m128
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

        template <int I>
        [[nodiscard]] inline float extract(__m128 xmm)
        {
            static_assert(I >= 0 && I <= 3);

            if constexpr (I == 0)
            {
                return _mm_cvtss_f32(xmm);
            }
            else
            {
                return _mm_cvtss_f32(permute<I, I, I, I>(xmm));
            }
        }

        [[nodiscard]] inline float extract(__m128 xmm, int i)
        {
            switch (i)
            {
            case 0:
                return extract<0>(xmm);
            case 1:
                return extract<1>(xmm);
            case 2:
                return extract<2>(xmm);
            case 3:
                return extract<3>(xmm);
            }

            RAY_UNREACHABLE();
        }

        template <int I>
        [[nodiscard]] inline __m128 insert(__m128 xmm, float s)
        {
            static_assert(I >= 0 && I <= 3);

            __m128 s4 = _mm_set1_ps(s);
            return _mm_blend_ps(xmm, s4, 1 << I);
        }

        [[nodiscard]] inline __m128 insert(__m128 xmm, float s, int i)
        {
            switch (i)
            {
            case 0:
                return insert<0>(xmm, s);
            case 1:
                return insert<1>(xmm, s);
            case 2:
                return insert<2>(xmm, s);
            case 3:
                return insert<3>(xmm, s);
            }

            RAY_UNREACHABLE();
        }

        template <bool X, bool Y, bool Z, bool W>
        [[nodiscard]] inline __m128 blend(__m128 xmm0, __m128 xmm1)
        {
            return _mm_blend_ps(xmm0, xmm1, lane::mask<X, Y, Z, W>());
        }

        // for xyzw vector make it xyz0
        [[nodiscard]] inline __m128 truncate3(__m128 v)
        {
            return _mm_and_ps(v, mask_xyz());
        }

        [[nodiscard]] inline __m128 undefined_ps()
        {
            // msvc lacks _mm_undefined_ps
            // maybe there is a better way?
            return _mm_setzero_ps();
        }

        [[nodiscard]] inline float hadd(__m128 a)
        {
            __m128 shuf = _mm_movehdup_ps(a); // broadcast elements 3,1 to 2,0
            __m128 sums = _mm_add_ps(a, shuf);
            shuf = _mm_movehl_ps(shuf, sums); // high half -> low half
            sums = _mm_add_ss(sums, shuf);
            return _mm_cvtss_f32(sums);
        }

        // ignores the last component
        [[nodiscard]] inline float hadd3(__m128 a)
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

        [[nodiscard]] inline __m128 abs(__m128 a)
        {
            return _mm_and_ps(a, _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF)));
        }

        [[nodiscard]] inline __m128 sub(__m128 a, __m128 b)
        {
            return _mm_sub_ps(a, b);
        }

        [[nodiscard]] inline __m128 add(__m128 a, __m128 b)
        {
            return _mm_add_ps(a, b);
        }

        [[nodiscard]] inline __m128 add(__m128 a, __m128 b, __m128 c)
        {
            return _mm_add_ps(_mm_add_ps(a, b), c);
        }

        [[nodiscard]] inline __m128 add(__m128 a, __m128 b, __m128 c, __m128 d)
        {
            return _mm_add_ps(_mm_add_ps(a, b), _mm_add_ps(c, d));
        }

        [[nodiscard]] inline __m128 div(__m128 a, __m128 b)
        {
            return _mm_div_ps(a, b);
        }

        [[nodiscard]] inline __m128 mul(__m128 a, __m128 b)
        {
            return _mm_mul_ps(a, b);
        }

        [[nodiscard]] inline __m128 div(float a, __m128 b)
        {
            return _mm_div_ps(_mm_set1_ps(a), b);
        }

        [[nodiscard]] inline __m128 mul(float a, __m128 b)
        {
            return _mm_mul_ps(_mm_set1_ps(a), b);
        }

        [[nodiscard]] inline __m128 div(__m128 a, float b)
        {
            return _mm_div_ps(a, _mm_set1_ps(b));
        }

        [[nodiscard]] inline __m128 mul(__m128 a, float b)
        {
            return _mm_mul_ps(a, _mm_set1_ps(b));
        }

        [[nodiscard]] inline __m128 cmplt(__m128 a, __m128 b)
        {
            return _mm_cmplt_ps(a, b);
        }

        [[nodiscard]] inline __m128 cmplt(float a, __m128 b)
        {
            return _mm_cmplt_ps(_mm_set1_ps(a), b);
        }

        [[nodiscard]] inline __m128 cmplt(__m128 a, float b)
        {
            return _mm_cmplt_ps(a, _mm_set1_ps(b));
        }

        [[nodiscard]] inline __m128 cmple(__m128 a, __m128 b)
        {
            return _mm_cmple_ps(a, b);
        }

        [[nodiscard]] inline __m128 cmple(float a, __m128 b)
        {
            return _mm_cmple_ps(_mm_set1_ps(a), b);
        }

        [[nodiscard]] inline __m128 cmple(__m128 a, float b)
        {
            return _mm_cmple_ps(a, _mm_set1_ps(b));
        }

        [[nodiscard]] inline __m128 cmpeq(__m128 a, __m128 b)
        {
            return _mm_cmpeq_ps(a, b);
        }

        [[nodiscard]] inline __m128 cmpeq(float a, __m128 b)
        {
            return _mm_cmpeq_ps(_mm_set1_ps(a), b);
        }

        [[nodiscard]] inline __m128 cmpeq(__m128 a, float b)
        {
            return _mm_cmpeq_ps(a, _mm_set1_ps(b));
        }

        [[nodiscard]] inline __m128 mod(__m128 lhs, __m128 rhs)
        {
            // http://dss.stephanierct.com/DevBlog/?p=8
            __m128 c = _mm_div_ps(lhs, rhs);
            __m128i i = _mm_cvttps_epi32(c);
            __m128 cTrunc = _mm_cvtepi32_ps(i);
            __m128 base = _mm_mul_ps(cTrunc, rhs);
            __m128 r = _mm_sub_ps(lhs, base);
            return r;
        }

        [[nodiscard]] inline float dot3(__m128 a, __m128 b)
        {
            // mul first 3 components of xmm, sum it, and store in the first component, return first component
            return _mm_cvtss_f32(_mm_dp_ps(a, b, 0b0111'0001));

            // should be faster, but on nehalem it actually isn't?
            // return hadd3(mul(a, b));
        }

        [[nodiscard]] inline float dot(__m128 a, __m128 b)
        {
            return hadd(mul(a, b));
        }

        // xyz contains cross(a.xyz, b.xyz)
        // w contains a.w*b.w - a.w*b.w ~= 0
        [[nodiscard]] inline __m128 cross3(__m128 a, __m128 b)
        {
            // http://threadlocalmutex.com/?p=8
            __m128 a_yzx = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
            __m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
            __m128 c = _mm_sub_ps(_mm_mul_ps(a, b_yzx), _mm_mul_ps(a_yzx, b));
            return _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 0, 2, 1));
        }

        // out r0.w, r1.w, r2.w are undefined
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

        // out r0.w, r1.w, r2.w are undefined
        inline void transpose3(__m128 r[3])
        {
            __m128 t0 = _mm_shuffle_ps(r[0], r[1], _MM_SHUFFLE(1, 0, 1, 0));
            __m128 t1 = _mm_shuffle_ps(r[0], r[1], _MM_SHUFFLE(2, 2, 2, 2));
            r[0] = _mm_shuffle_ps(t0, r[2], _MM_SHUFFLE(0, 0, 2, 0));
            r[1] = _mm_shuffle_ps(t0, r[2], _MM_SHUFFLE(0, 1, 3, 1));
            r[2] = _mm_shuffle_ps(t1, r[2], _MM_SHUFFLE(0, 2, 2, 0));
        }

        // out r0.w, r1.w, r2.w are equal to zero
        inline void transpose3zx(__m128& r0, __m128& r1, __m128& r2)
        {
            __m128 r3 = _mm_setzero_ps();
            transpose(r0, r1, r2, r3);
        }

        // out r0.w, r1.w, r2.w are equal to zero
        inline void transpose3zx(__m128 r[3])
        {
            __m128 r3 = _mm_setzero_ps();
            transpose(r[0], r[1], r[2], r3);
        }

        inline void transpose(__m128 r[4])
        {
            _MM_TRANSPOSE4_PS(r[0], r[1], r[2], r[3]);
        }

        [[nodiscard]] inline float hmin(__m128 x)
        {
            /*
            __m128 y = _mm_movehdup_ps(x);
            __m128 m1 = _mm_max_ps(x, y); // m1[0] = max(x[0], x[1]), m1[2] = max(x[2], x[3])
            __m128 m2 = _mm_movehl_ps(m1, m1); // m2[0] = max(x[2], x[3])
            return _mm_cvtss_f32(_mm_max_ps(m1, m2));
            */

            __m128 y = permute_zwxy(x); // hi <=> lo
            __m128 m1 = _mm_min_ps(x, y); // m1[0] = min(x[0], x[2]), m1[1] = min(x[1], x[3]), m1[2] = min(x[2], x[0]), m1[2] = min(x[3], x[1])
            __m128 m2 = permute_yxwz(m1); // m2[0] = m1[1], m2[1] = m1[0], m2[2] = m1[3], m2[3] = m1[2]
            return _mm_cvtss_f32(_mm_min_ps(m1, m2));
        }

        [[nodiscard]] inline float hmax(__m128 x)
        {

            __m128 y = permute_zwxy(x); // hi <=> lo
            __m128 m1 = _mm_max_ps(x, y); // m1[0] = max(x[0], x[2]), m1[1] = max(x[1], x[3]), m1[2] = max(x[2], x[0]), m1[2] = max(x[3], x[1])
            __m128 m2 = permute_yxwz(m1); // m2[0] = m1[1], m2[1] = m1[0], m2[2] = m1[3], m2[3] = m1[2]
            return _mm_cvtss_f32(_mm_max_ps(m1, m2));
        }

        [[nodiscard]] inline __m128 min(__m128 a, __m128 b)
        {
            return _mm_min_ps(a, b);
        }

        [[nodiscard]] inline __m128 max(__m128 a, __m128 b)
        {
            return _mm_max_ps(a, b);
        }

        [[nodiscard]] inline __m128 sqrt(__m128 a)
        {
            return _mm_sqrt_ps(a);
        }

        [[nodiscard]] inline __m128 clamp(__m128 a, float minv, float maxv)
        {
            return max(min(a, _mm_set1_ps(maxv)), _mm_set1_ps(minv));
        }

        [[nodiscard]] inline __m128 neg(__m128 a)
        {
            return _mm_xor_ps(a, _mm_set1_ps(-0.0f));
        }

        [[nodiscard]] inline __m128 neg(__m128 a, __m128 mask)
        {
            return _mm_xor_ps(_mm_and_ps(mask, _mm_castsi128_ps(_mm_set1_epi32(0x80000000))), a);
        }

        [[nodiscard]] inline __m128 blend(float a, float b, __m128 mask)
        {
            return _mm_blendv_ps(_mm_set1_ps(a), _mm_set1_ps(b), mask);
        }

        [[nodiscard]] inline __m128 blend(__m128 a, float b, __m128 mask)
        {
            return _mm_blendv_ps(a, _mm_set1_ps(b), mask);
        }

        [[nodiscard]] inline __m128 blend(float a, __m128 b, __m128 mask)
        {
            return _mm_blendv_ps(_mm_set1_ps(a), b, mask);
        }

        [[nodiscard]] inline __m128 blend(__m128 a, __m128 b, __m128 mask)
        {
            return _mm_blendv_ps(a, b, mask);
        }
    }
}

#include <ray/utility/UtilityMacroUndef.h>
