#pragma once

#include "OrthonormalBasis3.h"
#include "Vec3.h"

namespace ray
{
    template <typename T>
    struct Basis3
    {
        Basis3(
            const Vec3<T>& x,
            const Vec3<T>& y,
            const Vec3<T>& z
        ) noexcept :
            m_x(x),
            m_y(y),
            m_z(z)
        {

        }

        [[nodiscard]] OrthonormalBasis3<T> assumeOrthonormal() const
        {
            return OrthonormalBasis3<T>(
                AssumeOrthogonal{},
                m_x.assumeNormalized(),
                m_y.assumeNormalized(),
                m_z.assumeNormalized()
            );
        }

        [[nodiscard]] const Vec3<T>& x() const
        {
            return m_x;
        }

        [[nodiscard]] const Vec3<T>& y() const
        {
            return m_y;
        }

        [[nodiscard]] const Vec3<T>& z() const
        {
            return m_z;
        }

    private:
        Vec3<T> m_x;
        Vec3<T> m_y;
        Vec3<T> m_z;
    };

    using Basis3f = Basis3<float>;
}
