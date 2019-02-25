#pragma once

#include <xmmintrin.h>
#include <smmintrin.h>

namespace ray
{
    namespace detail
    {
        inline __m128 undefined_ps()
        {
            // msvc lacks _mm_undefined_ps
            // maybe there is a better way?
            return _mm_set1_ps(0.0f);
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

        inline float dot(__m128 a, __m128 b)
        {
            // mul first 3 components of xmm, sum it, and store in the first component, return first component
            return _mm_cvtss_f32(_mm_dp_ps(a, b, 0b0111'0001));

            // should be faster, but on nehalem it actually isn't?
            // return hadd3(mul(a, b));
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