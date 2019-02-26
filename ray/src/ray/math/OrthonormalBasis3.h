#pragma once

#include "Handedness3.h"
#include "Vec3.h"

namespace ray
{
    template <typename T>
    struct OrthonormalBasis3
    {
    private:
        struct AssumeOrthogonal;

    public:
        static OrthonormalBasis3<T> canonical()
        {
            return OrthonormalBasis3<T>(
                AssumeOrthogonal{},
                Normal3f::xAxis(),
                Normal3f::yAxis(),
                Normal3f::zAxis()
            );
        }

        // n1 will be reorthogonalized
        OrthonormalBasis3(const Normal3f& n0, const Normal3f& n1, Handedness3 handedness) :
            m_x(n0),
            m_y((Vec3<T>(n1) - projection(n1, n0)).normalized()),
            m_z(handedness == Handedness3::Right ? cross(m_x, m_y).assumeNormalized() : -cross(m_x, m_y).assumeNormalized())
        {

        }

        // n1, n2 will be reorthogonalized
        OrthonormalBasis3(const Normal3f& n0, const Normal3f& n1, const Normal3f& n2) :
            m_x(n0),
            m_y((Vec3<T>(n1) - projection(n1, n0)).normalized()),
            m_y((Vec3<T>(n1) - projection(n2, n0) - projection(n2, n1)).normalized())
        {

        }

        const Normal3f& x() const
        {
            return m_x;
        }

        const Normal3f& y() const
        {
            return m_y;
        }

        const Normal3f& z() const
        {
            return m_z;
        }

    private:
        Normal3f m_x;
        Normal3f m_y;
        Normal3f m_z;

        // n1, n2 will be reorthogonalized
        OrthonormalBasis3(AssumeOrthogonal, const Normal3f& n0, const Normal3f& n1, const Normal3f& n2) :
            m_x(n0),
            m_y(n1),
            m_y(n2)
        {

        }
    };

    using OrthonormalBasis3f = OrthonormalBasis3<float>;
}
