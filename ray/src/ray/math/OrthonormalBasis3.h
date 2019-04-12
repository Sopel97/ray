#pragma once

#include "Handedness3.h"
#include "Vec3.h"

namespace ray
{
    struct AssumeOrthogonal;

    template <typename T>
    struct OrthonormalBasis3
    {
    public:
        [[nodiscard]] static OrthonormalBasis3<T> canonical() noexcept
        {
            return OrthonormalBasis3<T>(
                AssumeOrthogonal{},
                UnitVec3<T>::xAxis(),
                UnitVec3<T>::yAxis(),
                UnitVec3<T>::zAxis()
            );
        }

        // n1 will be reorthogonalized
        OrthonormalBasis3(const UnitVec3<T>& n0, const UnitVec3<T>& n1, Handedness3 handedness) noexcept :
            m_x(n0),
            m_y((Vec3<T>(n1) - projection(n1, n0)).normalized()),
            m_z(handedness == Handedness3::Right ? cross(m_x, m_y).assumeNormalized() : -cross(m_x, m_y).assumeNormalized())
        {

        }

        // n1, n2 will be reorthogonalized
        OrthonormalBasis3(const UnitVec3f& n0, const UnitVec3f& n1, const UnitVec3f& n2) noexcept :
            m_x(n0),
            m_y((Vec3<T>(n1) - projection(n1, n0)).normalized()),
            m_z((Vec3<T>(n2) - projection(n2, n0) - projection(n2, m_y)).normalized())
        {

        }

        // No reorthogonalization takes place
        OrthonormalBasis3(AssumeOrthogonal, const UnitVec3<T>& n0, const UnitVec3<T>& n1, const UnitVec3<T>& n2) noexcept :
            m_x(n0),
            m_y(n1),
            m_z(n2)
        {

        }

        [[nodiscard]] const UnitVec3<T>& x() const
        {
            return m_x;
        }

        [[nodiscard]] const UnitVec3<T>& y() const
        {
            return m_y;
        }

        [[nodiscard]] const UnitVec3<T>& z() const
        {
            return m_z;
        }

    private:
        UnitVec3<T> m_x;
        UnitVec3<T> m_y;
        UnitVec3<T> m_z;
    };

    using OrthonormalBasis3f = OrthonormalBasis3<float>;
}
