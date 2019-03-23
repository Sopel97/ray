#pragma once

#include "M128Math.h"

namespace ray
{
    namespace m128
    {
        template <int I>
        struct ScalarAccessor
        {
            static_assert(I >= 0 && I <= 3);

            __m128 xmm;

            ScalarAccessor(const ScalarAccessor& other)
            {
                operator=(static_cast<float>(other));
            }
            ScalarAccessor(ScalarAccessor&& other)
            {
                operator=(static_cast<float>(other));
            }

            ScalarAccessor& operator=(const ScalarAccessor& other)
            {
                return operator=(static_cast<float>(other));
            }

            ScalarAccessor& operator=(ScalarAccessor&& other)
            {
                return operator=(static_cast<float>(other));
            }

            operator float() const
            {
                return extract<I>(xmm);
            }

            ScalarAccessor& operator=(float s)
            {
                xmm = insert<I>(xmm, s);
                return *this;
            }
        };

        template <int I>
        struct ReadonlyScalarAccessor : ScalarAccessor<I>
        {
            ReadonlyScalarAccessor(const ReadonlyScalarAccessor& other) = delete;
            ReadonlyScalarAccessor(ReadonlyScalarAccessor&& other) = delete;

            ReadonlyScalarAccessor& operator=(const ReadonlyScalarAccessor& other) = delete;

            ReadonlyScalarAccessor& operator=(ReadonlyScalarAccessor&& other) = delete;

            ReadonlyScalarAccessor& operator=(float s) = delete;
        };

        template <int Size, int I, int J>
        struct ScalarMatrixAccessor
        {
            static_assert(I >= 0 && I <= 3);
            static_assert(J >= 0 && J <= 3);
            static_assert(Size > 0);

            __m128 xmm[Size];

            ScalarMatrixAccessor(const ScalarMatrixAccessor& other)
            {
                operator=(static_cast<float>(other));
            }
            ScalarMatrixAccessor(ScalarMatrixAccessor&& other)
            {
                operator=(static_cast<float>(other));
            }

            ScalarMatrixAccessor& operator=(const ScalarMatrixAccessor& other)
            {
                return operator=(static_cast<float>(other));
            }

            ScalarMatrixAccessor& operator=(ScalarMatrixAccessor&& other)
            {
                return operator=(static_cast<float>(other));
            }

            operator float() const
            {
                return extract<J>(xmm[I]);
            }

            ScalarMatrixAccessor& operator=(float s)
            {
                xmm[I] = insert<J>(xmm[I], s);
                return *this;
            }
        };
    }
}
