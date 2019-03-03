#pragma once

#include "M128Math.h"

#include <xmmintrin.h>
#include <smmintrin.h>

namespace ray
{
    namespace m128
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

                result[0] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
            }

            // column 1
            {
                __m128 x, y, z;
                spill3(rhs[1], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[1] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
            }

            // column 2
            {
                __m128 x, y, z;
                spill3(rhs[2], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[2] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
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

                result[0] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
            }

            // column 1
            {
                __m128 x, y, z;
                spill3(rhs[1], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[1] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
            }

            // column 2
            {
                __m128 x, y, z;
                spill3(rhs[2], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[2] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
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

                result[0] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
            }

            // column 1
            {
                __m128 x, y, z;
                spill3(rhs[1], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[1] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
            }

            // column 2
            {
                __m128 x, y, z;
                spill3(rhs[2], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[2] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
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

                result[0] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
            }

            // column 1
            {
                __m128 x, y, z;
                spill3(rhs[1], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[1] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
            }

            // column 2
            {
                __m128 x, y, z;
                spill3(rhs[2], x, y, z);

                x = _mm_mul_ps(x, c0);
                y = _mm_mul_ps(y, c1);
                z = _mm_mul_ps(z, c2);

                result[2] = truncate3(_mm_add_ps(x, _mm_add_ps(y, z)));
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
        [[nodiscard]] inline __m128 mulMat3Vec3(const __m128 lhs[3], __m128 rhs)
        {
            __m128 v0, v1, v2;
            spill3(rhs, v0, v1, v2);

            v0 = _mm_mul_ps(v0, lhs[0]);
            v1 = _mm_mul_ps(v1, lhs[1]);
            v2 = _mm_mul_ps(v2, lhs[2]);

            return _mm_add_ps(v0, _mm_add_ps(v1, v2));
        }

        // column-major, vector is in homogeneous coordinates
        [[nodiscard]] inline __m128 mulMat3Vec3homo(const __m128 lhs[3], __m128 rhs)
        {
            __m128 v0, v1, v2;
            spill3(rhs, v0, v1, v2);

            v0 = _mm_mul_ps(v0, lhs[0]);
            v1 = _mm_mul_ps(v1, lhs[1]);
            v2 = _mm_mul_ps(v2, lhs[2]);

            return _mm_blend_ps(
                _mm_add_ps(v0, _mm_add_ps(v1, v2)),
                rhs,
                0b1000
            );
        }

        [[nodiscard]] inline __m128 mulMatAffineVec3(const __m128 lhs[4], __m128 rhs)
        {
            __m128 v0, v1, v2;
            spill3(rhs, v0, v1, v2);

            v0 = _mm_mul_ps(v0, lhs[0]);
            v1 = _mm_mul_ps(v1, lhs[1]);
            v2 = _mm_mul_ps(v2, lhs[2]);

            return _mm_add_ps(_mm_add_ps(v0, v1), _mm_add_ps(v2, lhs[3]));
        }

        [[nodiscard]] inline __m128 mulMat4Vec3(const __m128 lhs[4], __m128 rhs)
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

        inline void invertMatAffinePerpAxes(__m128 cols[4])
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
            __m128 invScales = m128::truncate3(_mm_div_ps(_mm_set1_ps(1.0f), scales));

            // apply scaling
            c0 = _mm_mul_ps(c0, invScales);
            c1 = _mm_mul_ps(c1, invScales);
            c2 = _mm_mul_ps(c2, invScales);

            // R^-1 (with rescaling above)
            m128::transpose3zx(c0, c1, c2);

            cols[0] = c0;
            cols[1] = c1;
            cols[2] = c2;

            // multiply (R^-1) * T
            cols[3] = m128::neg(mulMat3Vec3homo(cols, cols[3]), mask_xyz());
        }

        inline void invertMatAffineNoTransPerpAxes(__m128 cols[4])
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
            __m128 invScales = m128::truncate3(_mm_div_ps(_mm_set1_ps(1.0f), scales));

            // apply scaling
            c0 = _mm_mul_ps(c0, invScales);
            c1 = _mm_mul_ps(c1, invScales);
            c2 = _mm_mul_ps(c2, invScales);

            // R^-1 (with rescaling above)
            m128::transpose3zx(c0, c1, c2);

            cols[0] = c0;
            cols[1] = c1;
            cols[2] = c2;
        }

        inline void invertMatAffineNoScalePerpAxes(__m128 cols[4])
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

            // R^-1
            m128::transpose3zx(cols[0], cols[1], cols[2]);

            // multiply (R^-1) * T
            cols[3] = m128::neg(mulMat3Vec3homo(cols, cols[3]), mask_xyz());
        }

        // column-major
        // __m128 mapped to
        // | A0  A2 |
        // | A1  A3 |
        [[nodiscard]] inline __m128 mulMat2Mat2(__m128 a, __m128 b)
        {
            return _mm_add_ps(
                _mm_mul_ps(a, permute_xxww(b)),
                _mm_mul_ps(permute_zwxy(a), permute_yyzz(b))
            );
        }

        // (A#)*B
        [[nodiscard]] inline __m128 mulMat2AdjMat2(__m128 a, __m128 b)
        {
            return _mm_sub_ps(
                _mm_mul_ps(permute_wxwx(a), b),
                _mm_mul_ps(permute_zyzy(a), permute_yxwz(b))
            );
        }

        // A*(B#)
        [[nodiscard]] inline __m128 mulMat2Mat2Adj(__m128 a, __m128 b)
        {
            return _mm_sub_ps(
                _mm_mul_ps(a, permute_wwxx(b)),
                _mm_mul_ps(permute_zwxy(a), permute_yyzz(b))
            );
        }

        // assumes that the matrix is in fact invertible
        inline void invertMat4(__m128 cols[4])
        {
            // by Eric Zhang
            // https://lxjk.github.io/2017/09/03/Fast-4x4-Matrix-Inverse-with-SSE-SIMD-Explained.html

            // uses blockwise inversion
            // https://en.wikipedia.org/wiki/Invertible_matrix#Blockwise_inversion
            // sub matrices
            __m128 A = shuffle_xyxy(cols[0], cols[1]);
            __m128 C = shuffle_zwzw(cols[0], cols[1]);
            __m128 B = shuffle_xyxy(cols[2], cols[3]);
            __m128 D = shuffle_zwzw(cols[2], cols[3]);

            // determinant as (|A| |C| |B| |D|)
            __m128 detSub = _mm_sub_ps(
                _mm_mul_ps(shuffle_xzxz(cols[0], cols[2]), shuffle_ywyw(cols[1], cols[3])),
                _mm_mul_ps(shuffle_ywyw(cols[0], cols[2]), shuffle_xzxz(cols[1], cols[3]))
            );

            __m128 detA, detB, detC, detD;
            spill(detSub, detA, detC, detB, detD);

            // let iM = 1/|M| * | X  Y |
            //                  | Z  W |

            // D#C
            __m128 D_C = mulMat2AdjMat2(D, C);
            // A#B
            __m128 A_B = mulMat2AdjMat2(A, B);
            // X# = |D|A - B(D#C)
            __m128 X_ = _mm_sub_ps(_mm_mul_ps(detD, A), mulMat2Mat2(B, D_C));
            // W# = |A|D - C(A#B)
            __m128 W_ = _mm_sub_ps(_mm_mul_ps(detA, D), mulMat2Mat2(C, A_B));

            // |M| = |A|*|D| + ... (continue later)
            __m128 detM = _mm_mul_ps(detA, detD);

            // Y# = |B|C - D(A#B)#
            __m128 Y_ = _mm_sub_ps(_mm_mul_ps(detB, C), mulMat2Mat2Adj(D, A_B));
            // Z# = |C|B - A(D#C)#
            __m128 Z_ = _mm_sub_ps(_mm_mul_ps(detC, B), mulMat2Mat2Adj(A, D_C));

            // |M| = |A|*|D| + |B|*|C| ... (continue later)
            detM = _mm_add_ps(detM, _mm_mul_ps(detB, detC));

            // tr((A#B)(D#C))
            __m128 tr = _mm_mul_ps(A_B, permute_xzyw(D_C));
            tr = _mm_hadd_ps(tr, tr);
            tr = _mm_hadd_ps(tr, tr);
            // |M| = |A|*|D| + |B|*|C| - tr((A#B)(D#C))
            detM = _mm_sub_ps(detM, tr);

            __m128 adjSignMask = _mm_set_ps(1.f, -1.f, -1.f, 1.f);
            // (1/|M|, -1/|M|, -1/|M|, 1/|M|)
            __m128 rDetM = _mm_div_ps(adjSignMask, detM);

            X_ = _mm_mul_ps(X_, rDetM);
            Y_ = _mm_mul_ps(Y_, rDetM);
            Z_ = _mm_mul_ps(Z_, rDetM);
            W_ = _mm_mul_ps(W_, rDetM);

            // apply adjugate and store, here we combine adjugate shuffle and store shuffle
            cols[0] = shuffle_wywy(X_, Z_);
            cols[1] = shuffle_zxzx(X_, Z_);
            cols[2] = shuffle_wywy(Y_, W_);
            cols[3] = shuffle_zxzx(Y_, W_);
        }

        inline void invertMat3(__m128 cols[3])
        {
            // https://en.wikipedia.org/wiki/Invertible_matrix#Inversion_of_3_%C3%97_3_matrices

            //                   [ (c1 x c2)^T ]
            // A^-1 = 1/det(A) * [ (c2 x c0)^T ]
            //                   [ (c0 x c1)^T ]

            // our m128 cross() preserves the last component (w), so we don't have to zero it explicitly

            __m128 c0 = cols[0];
            __m128 c1 = cols[1];
            __m128 c2 = cols[2];

            __m128 c0c1 = cross(c0, c1);
            const float invDet = 1.0f / dot3(c0c1, c2);

            cols[0] = mul(invDet, cross(c1, c2));
            cols[1] = mul(invDet, cross(c2, c0));
            cols[2] = mul(invDet, c0c1);

            transpose3zx(cols);
        }

        inline void invertTransposeMat3(__m128 cols[3])
        {
            // same as above but don't transpose back

            __m128 c0 = cols[0];
            __m128 c1 = cols[1];
            __m128 c2 = cols[2];

            __m128 c0c1 = cross(c0, c1);
            const float invDet = 1.0f / dot3(c0c1, c2);

            cols[0] = mul(invDet, cross(c1, c2));
            cols[1] = mul(invDet, cross(c2, c0));
            cols[2] = mul(invDet, c0c1);
        }

        inline void invertTransposeMat3(const __m128 cols[3], __m128 out[3])
        {
            // same as above but don't transpose back

            __m128 c0 = cols[0];
            __m128 c1 = cols[1];
            __m128 c2 = cols[2];

            __m128 c0c1 = cross(c0, c1);
            const float invDet = 1.0f / dot3(c0c1, c2);

            out[0] = mul(invDet, cross(c1, c2));
            out[1] = mul(invDet, cross(c2, c0));
            out[2] = mul(invDet, c0c1);
        }

        // inv(A) = [ inv(R)   -inv(R) * T ]
        //          [   0            1     ]
        inline void invertMatAffine(__m128 cols[4])
        {
            invertMat3(cols);
            cols[3] = m128::neg(mulMat3Vec3homo(cols, cols[3]), mask_xyz());
        }
    }
}
