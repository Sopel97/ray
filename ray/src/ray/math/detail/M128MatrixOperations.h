#pragma once

#include "M128Math.h"

#include <xmmintrin.h>
#include <smmintrin.h>

namespace ray
{
    namespace detail
    {
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

        inline void invertMatAffine(__m128 cols[4])
        {
            // column-major
            // X0 X1 X2 T0
            // Y0 Y1 Y2 T1
            // Z0 Z1 Z2 T2
            //
            // R = 3x3 rotation matrix
            // T = translation
            //
            // R | T
            // 0 | 1
            //
            // inverted:
            //
            // (1/(X^2))*X0 (1/(Y^2))*Y0 (1/(Z^2))*Z0 T0'
            // (1/(X^2))*X1 (1/(Y^2))*Y1 (1/(Z^2))*Z1 T1'
            // (1/(X^2))*X2 (1/(Y^2))*Y2 (1/(Z^2))*Z2 T2'
            // where T' = (R^-1)*(-T)
            //
            // R^-1 | (R^-1)*(-T)
            //   0  |    1
            //

            __m128 c0 = cols[0];
            __m128 c1 = cols[1];
            __m128 c2 = cols[2];

            // before inverting R compute inverse scales
            // scale should never be 0 - would make the matrix determinant 0 => not invertible
            __m128 scales =
                _mm_add_ps(
                    _mm_add_ps(
                        _mm_mul_ps(c0, c0),
                        _mm_mul_ps(c1, c1)
                    ),
                    _mm_mul_ps(c2, c2)
                );

            // truncate to be sure that w=0 so after multiplication the last row is 0, 0, 0, 1
            __m128 invScales = detail::truncate3(_mm_div_ps(_mm_set1_ps(1.0f), scales));

            // apply scaling
            c0 = _mm_mul_ps(c0, invScales);
            c1 = _mm_mul_ps(c1, invScales);
            c2 = _mm_mul_ps(c2, invScales);

            // R^-1 (with rescaling above)
            detail::transpose3(c0, c1, c2);

            cols[0] = c0;
            cols[1] = c1;
            cols[2] = c2;

            // multiply (R^-1) * T
            cols[3] = detail::neg(mulMat3Vec3(cols, cols[3]));
        }

        inline void invertMatAffineNoTrans(__m128 cols[4])
        {
            // column-major
            // X0 X1 X2 T0
            // Y0 Y1 Y2 T1
            // Z0 Z1 Z2 T2
            //
            // R = 3x3 rotation matrix
            // T = translation
            //
            // R | T
            // 0 | 1
            //
            // inverted:
            //
            // (1/(X^2))*X0 (1/(Y^2))*Y0 (1/(Z^2))*Z0 0
            // (1/(X^2))*X1 (1/(Y^2))*Y1 (1/(Z^2))*Z1 0
            // (1/(X^2))*X2 (1/(Y^2))*Y2 (1/(Z^2))*Z2 0
            //
            // R^-1 | 0
            //   0  | 1
            //

            __m128 c0 = cols[0];
            __m128 c1 = cols[1];
            __m128 c2 = cols[2];

            // before inverting R compute inverse scales
            // scale should never be 0 - would make the matrix determinant 0 => not invertible
            __m128 scales =
                _mm_add_ps(
                    _mm_add_ps(
                        _mm_mul_ps(c0, c0),
                        _mm_mul_ps(c1, c1)
                    ),
                    _mm_mul_ps(c2, c2)
                );

            // truncate to be sure that w=0 so after multiplication the last row is 0, 0, 0, 1
            __m128 invScales = detail::truncate3(_mm_div_ps(_mm_set1_ps(1.0f), scales));

            // apply scaling
            c0 = _mm_mul_ps(c0, invScales);
            c1 = _mm_mul_ps(c1, invScales);
            c2 = _mm_mul_ps(c2, invScales);

            // R^-1 (with rescaling above)
            detail::transpose3(c0, c1, c2);

            cols[0] = c0;
            cols[1] = c1;
            cols[2] = c2;
        }

        inline void invertMatAffineNoScale(__m128 cols[4])
        {
            // column-major
            // X0 X1 X2 T0
            // Y0 Y1 Y2 T1
            // Z0 Z1 Z2 T2
            //
            // R = 3x3 rotation matrix
            // T = translation
            //
            // R | T
            // 0 | 1
            //
            // inverted:
            //
            // X0 Y0 Z0 T0'
            // X1 Y1 Z1 T1'
            // X2 Y2 Z2 T2'
            // where T' = (R^-1)*(-T)
            //
            // R^-1 | (R^-1)*(-T)
            //   0  |    1
            //
            // but we don't care what the last row holds because it's always ignored in the computations

            __m128 c0 = cols[0];
            __m128 c1 = cols[1];
            __m128 c2 = cols[2];

            // R^-1
            detail::transpose3(c0, c1, c2);
            cols[0] = c0;
            cols[1] = c1;
            cols[2] = c2;

            // multiply (R^-1) * T
            cols[3] = detail::neg(mulMat3Vec3(cols, cols[3]));
        }
    }
}
