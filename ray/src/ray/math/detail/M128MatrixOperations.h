#pragma once

#include "M128Math.h"

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

        // column-major
        inline void mulMat4Mat4(const __m128 lhs[4], const __m128 rhs[4], __m128 result[4])
        {
            __m128 c0 = lhs[0];
            __m128 c1 = lhs[1];
            __m128 c2 = lhs[2];
            __m128 c3 = lhs[3];

            // column 0
            {
                __m128 x, y, z, w;
                spill(rhs[0], x, y, z, w);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);
                w = _mm_mul_ps(w, c3);

                result[0] = _mm_add_ps(x, _mm_add_ps(y, _mm_add_ps(z, w)));
            }

            // column 1
            {
                __m128 x, y, z, w;
                spill(rhs[1], x, y, z, w);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);
                w = _mm_mul_ps(w, c3);

                result[1] = _mm_add_ps(x, _mm_add_ps(y, _mm_add_ps(z, w)));
            }

            // column 2
            {
                __m128 x, y, z, w;
                spill(rhs[2], x, y, z, w);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);
                w = _mm_mul_ps(w, c3);

                result[2] = _mm_add_ps(x, _mm_add_ps(y, _mm_add_ps(z, w)));
            }

            // column 3
            {
                __m128 x, y, z, w;
                spill(rhs[3], x, y, z, w);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);
                w = _mm_mul_ps(w, c3);

                result[3] = _mm_add_ps(x, _mm_add_ps(y, _mm_add_ps(z, w)));
            }
        }

        // column-major
        inline void mulMat3Mat3(const __m128 lhs[3], const __m128 rhs[3], __m128 result[3])
        {
            __m128 c0 = lhs[0];
            __m128 c1 = lhs[1];
            __m128 c2 = lhs[2];

            // column 0
            {
                __m128 x, y, z;
                spill3(rhs[0], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[0] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 1
            {
                __m128 x, y, z;
                spill3(rhs[1], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[1] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 2
            {
                __m128 x, y, z;
                spill3(rhs[2], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[2] = _mm_add_ps(x, _mm_add_ps(y, z));
            }
        }

        // column-major
        inline void mulMatAffineMatAffine(const __m128 lhs[4], const __m128 rhs[4], __m128 result[4])
        {
            __m128 c0 = lhs[0];
            __m128 c1 = lhs[1];
            __m128 c2 = lhs[2];

            // column 0
            {
                __m128 x, y, z;
                spill3(rhs[0], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[0] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 1
            {
                __m128 x, y, z;
                spill3(rhs[1], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[1] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 2
            {
                __m128 x, y, z;
                spill3(rhs[2], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[2] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 3
            {
                __m128 x, y, z;
                spill3(rhs[3], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);
                // lhs[3] = 1 * lhs[3]

                result[3] = _mm_add_ps(lhs[3], _mm_add_ps(x, _mm_add_ps(y, z)));
            }
        }

        // column-major
        inline void mulMat3MatAffine(const __m128 lhs[3], const __m128 rhs[4], __m128 result[4])
        {
            __m128 c0 = lhs[0];
            __m128 c1 = lhs[1];
            __m128 c2 = lhs[2];

            // column 0
            {
                __m128 x, y, z;
                spill3(rhs[0], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[0] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 1
            {
                __m128 x, y, z;
                spill3(rhs[1], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[1] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 2
            {
                __m128 x, y, z;
                spill3(rhs[2], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[2] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 3
            {
                __m128 x, y, z;
                spill3(rhs[3], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[3] = _mm_add_ps(x, _mm_add_ps(y, z));
            }
        }

        // column-major
        inline void mulMatAffineMat3(const __m128 lhs[4], const __m128 rhs[3], __m128 result[4])
        {
            __m128 c0 = lhs[0];
            __m128 c1 = lhs[1];
            __m128 c2 = lhs[2];

            // column 0
            {
                __m128 x, y, z;
                spill3(rhs[0], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[0] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 1
            {
                __m128 x, y, z;
                spill3(rhs[1], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[1] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 2
            {
                __m128 x, y, z;
                spill3(rhs[2], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[2] = _mm_add_ps(x, _mm_add_ps(y, z));
            }

            // column 3
            {
                // x = _mm_mul_ps(0, c0);
                // y = _mm_mul_ps(0, c1);
                // z = _mm_mul_ps(0, c2);
                // lhs[3] = 1 * lhs[3]

                result[3] = lhs[3];
            }
        }

        // column-major
        inline __m128 mulMat3Vec3(const __m128 lhs[3], __m128 rhs)
        {
            __m128 v0, v1, v2;
            spill3(rhs, v0, v1, v2);

            v0 = _mm_mul_ps(v0, lhs[0]);
            v1 = _mm_mul_ps(v1, lhs[1]);
            v2 = _mm_mul_ps(v2, lhs[2]);

            return _mm_add_ps(v0, _mm_add_ps(v1, v2));
        }

        inline __m128 mulMatAffineVec3(const __m128 lhs[4], __m128 rhs)
        {
            __m128 v0, v1, v2;
            spill3(rhs, v0, v1, v2);

            v0 = _mm_mul_ps(v0, lhs[0]);
            v1 = _mm_mul_ps(v1, lhs[1]);
            v2 = _mm_mul_ps(v2, lhs[2]);

            return _mm_add_ps(_mm_add_ps(v0, v1), _mm_add_ps(v2, lhs[3]));
        }

        inline __m128 mulMat4Vec3(const __m128 lhs[4], __m128 rhs)
        {
            __m128 v0, v1, v2;
            spill3(rhs, v0, v1, v2);

            v0 = _mm_mul_ps(v0, lhs[0]);
            v1 = _mm_mul_ps(v1, lhs[1]);
            v2 = _mm_mul_ps(v2, lhs[2]);

            // v3 should be 1, but we don't care about in the vector storage
            // we have to calculate the w as if v3 was 1
            __m128 res = _mm_add_ps(_mm_add_ps(v0, v1), _mm_add_ps(v2, lhs[3]));
            __m128 wwww = _mm_shuffle_ps(res, res, _MM_SHUFFLE(3, 3, 3, 3));
            return _mm_div_ps(res, wwww);
        }
    }
}
