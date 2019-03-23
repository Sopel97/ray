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

            template <int J>
            ScalarAccessor& operator=(const ScalarAccessor<J>& other)
            {
                return operator=(static_cast<float>(other));
            }

            template <int J>
            ScalarAccessor& operator=(ScalarAccessor<J>&& other)
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

            template <int J>
            ReadonlyScalarAccessor& operator=(const ScalarAccessor<J>& other) = delete;

            template <int J>
            ReadonlyScalarAccessor& operator=(ScalarAccessor<J>&& other) = delete;

            ReadonlyScalarAccessor& operator=(float s) = delete;
        };
    }
}
