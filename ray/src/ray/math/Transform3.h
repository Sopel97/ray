#pragma once

#include "m128/M128Math.h"
#include "m128/M128MatrixOperations.h"

#include "Basis3.h"
#include "Matrix4.h"
#include "OrthonormalBasis3.h"
#include "Quat4.h"
#include "Vec3.h"

#include <cstdint>

namespace ray
{
    enum struct AffineTransformationComponentMask : std::uint32_t
    {
        None = 0,

        Rotation = 0b001,
        Scale = 0b010,
        Translation = 0b100,

        RotationScale = Rotation | Scale,
        RotationTranslation = Rotation | Translation,
        ScaleTranslation = Scale | Translation,

        All = Rotation | Scale | Translation
    };

    constexpr AffineTransformationComponentMask operator|(AffineTransformationComponentMask lhs, AffineTransformationComponentMask rhs)
    {
        return AffineTransformationComponentMask(static_cast<std::uint32_t>(lhs) | static_cast<std::uint32_t>(rhs));
    }

    constexpr AffineTransformationComponentMask operator&(AffineTransformationComponentMask lhs, AffineTransformationComponentMask rhs)
    {
        return AffineTransformationComponentMask(static_cast<std::uint32_t>(lhs) & static_cast<std::uint32_t>(rhs));
    }

    // whether lhs contains rhs
    constexpr bool contains(AffineTransformationComponentMask lhs, AffineTransformationComponentMask rhs)
    {
        return (lhs & rhs) == lhs;
    }

    template <typename T, AffineTransformationComponentMask MaskV>
    struct AffineTransformation3;

    template <AffineTransformationComponentMask MaskV>
    struct alignas(alignof(__m128)) AffineTransformation3<float, MaskV>;

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::None> : protected Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::None>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::None;

        AffineTransformation3() :
            Matrix4()
        {
        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            return lhs;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return rhs;
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return rhs;
        }

        void invert()
        {
            // nothing to do
        }

        SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose()
        {
            // nothing to do
        }

        // enable casting to more generic matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        operator AffineTransformation3<float, OtherMaskV>() const
        {
            return AffineTransformation3<float, OtherMaskV>(
                m_columns[0],
                m_columns[1],
                m_columns[2],
                m_columns[3]
            );
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Rotation> : protected Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Rotation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Rotation;

        AffineTransformation3() :
            Matrix4()
        {
        }

        explicit AffineTransformation3(const OrthonormalBasis3<float>& basis) :
            Matrix4(basis)
        {
        }

        explicit AffineTransformation3(const Quat4<float>& q) :
            Matrix4(q.normalized())
        {
        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMat3Mat3(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm)).assumeNormalized();
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            // property of the rotation matrix
            m128::transpose3(m_columns);
        }

        SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose()
        {
            m128::transpose3(m_columns);
        }

        // enable casting to more generic matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        operator AffineTransformation3<float, OtherMaskV>() const
        {
            return AffineTransformation3<float, OtherMaskV>(
                m_columns[0],
                m_columns[1],
                m_columns[2],
                m_columns[3]
                );
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Scale> : protected Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Scale>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Scale;

        AffineTransformation3() :
            Matrix4()
        {
        }

        AffineTransformation3(float xs, float ys, float zs) :
            Matrix4(xs, ys, zs)
        {
        }

        explicit AffineTransformation3(const Vec3<float>& s) :
            Matrix4(s.x, s.y, s.z)
        {
        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            // just multiply the diagonal
            SelfType ret{};
            ret.m_values[0][0] = lhs.m_values[0][0] * rhs.m_values[0][0];
            ret.m_values[1][1] = lhs.m_values[1][1] * rhs.m_values[1][1];
            ret.m_values[2][2] = lhs.m_values[2][2] * rhs.m_values[2][2];
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            m_values[0][0] = 1.0f / m_values[0][0];
            m_values[1][1] = 1.0f / m_values[1][1];
            m_values[2][2] = 1.0f / m_values[2][2];
        }

        SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose()
        {
            // nothing to do
        }

        // enable casting to more generic matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        operator AffineTransformation3<float, OtherMaskV>() const
        {
            return AffineTransformation3<float, OtherMaskV>(
                m_columns[0],
                m_columns[1],
                m_columns[2],
                m_columns[3]
            );
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Translation> : protected Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Translation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Translation;

        AffineTransformation3() :
            Matrix4()
        {
        }

        explicit AffineTransformation3(const Vec3<float>& t) :
            Matrix4(t)
        {
        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            return SelfType(
                lhs.m_columns[0],
                lhs.m_columns[1],
                lhs.m_columns[2],
                // truncate to leave it homogeneous
                _mm_add_ps(m128::truncate3(rhs.m_columns[3]), lhs.m_columns[3])
            );
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return rhs + Vec3<float>(lhs.m_columns[3]);
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return rhs + Vec3<float>(lhs.m_columns[3]);
        }

        void invert()
        {
            m_columns[3] = m128::neg(m_columns[3], m128::mask_xyz());
        }

        SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose()
        {
            // nothing to do
        }

        // enable casting to more generic matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        operator AffineTransformation3<float, OtherMaskV>() const
        {
            return AffineTransformation3<float, OtherMaskV>(
                m_columns[0],
                m_columns[1],
                m_columns[2],
                m_columns[3]
                );
        }

    protected:
        AffineTransformation3(__m128 c0, __m128 c1, __m128 c2, __m128 c3) :
            Matrix4(c0, c1, c2, c3)
        {
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::RotationTranslation> : protected Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::RotationTranslation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::RotationTranslation;

        AffineTransformation3() :
            Matrix4()
        {
        }

        AffineTransformation3(const OrthonormalBasis3<float>& basis, const Vec3<float>& t) :
            Matrix4(basis, t)
        {
        }

        AffineTransformation3(const Quat4<float>& q, const Point3<float>& origin) :
            Matrix4(q.normalized(), origin)
        {
        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMatAffineMatAffine(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            m128::invertMatAffineNoScale(m_columns);
        }

        SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose()
        {
            m128::transpose(m_columns);
        }

        // enable casting to more generic matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        operator AffineTransformation3<float, OtherMaskV>() const
        {
            return AffineTransformation3<float, OtherMaskV>(
                m_columns[0],
                m_columns[1],
                m_columns[2],
                m_columns[3]
                );
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::RotationScale> : protected Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::RotationScale>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::RotationScale;

        AffineTransformation3() :
            Matrix4()
        {
        }

        explicit AffineTransformation3(const Basis3<float>& basis) :
            Matrix4(basis)
        {
        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMat3Mat3(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            m128::invertMatAffineNoTrans(m_columns);
        }

        SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose()
        {
            m128::transpose3(m_columns);
        }

        // enable casting to more generic matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        operator AffineTransformation3<float, OtherMaskV>() const
        {
            return AffineTransformation3<float, OtherMaskV>(
                m_columns[0],
                m_columns[1],
                m_columns[2],
                m_columns[3]
                );
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::ScaleTranslation> : protected Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::ScaleTranslation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::ScaleTranslation;

        AffineTransformation3() :
            Matrix4()
        {
        }

        AffineTransformation3(float xs, float ys, float zs, const Vec3<float>& t) :
            Matrix4(xs, ys, zs, t)
        {
        }

        AffineTransformation3(const Vec3<float>& s, const Vec3<float>& t) :
            Matrix4(s.x, s.y, s.z, t)
        {
        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMatAffineMatAffine(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            // could probably be done more efficiently, but it's not important
            m128::invertMatAffine(m_columns);
        }

        SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose()
        {
            m128::transpose(m_columns);
        }

        // enable casting to more generic matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        operator AffineTransformation3<float, OtherMaskV>() const
        {
            return AffineTransformation3<float, OtherMaskV>(
                m_columns[0],
                m_columns[1],
                m_columns[2],
                m_columns[3]
                );
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::All> : protected Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::All>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::All;

        AffineTransformation3() :
            Matrix4()
        {
        }

        AffineTransformation3(const Basis3<float>& basis, const Vec3<float>& t) :
            Matrix4(basis, t)
        {
        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMatAffineMatAffine(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            m128::invertMatAffine(m_columns);
        }

        SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose()
        {
            m128::transpose(m_columns);
        }

        // enable casting to more generic matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        operator AffineTransformation3<float, OtherMaskV>() const
        {
            return AffineTransformation3<float, OtherMaskV>(
                m_columns[0],
                m_columns[1],
                m_columns[2],
                m_columns[3]
                );
        }
    };

    using Identity3f = AffineTransformation3<float, AffineTransformationComponentMask::None>;
    using Rotation3f = AffineTransformation3<float, AffineTransformationComponentMask::Rotation>;
    using Scale3f = AffineTransformation3<float, AffineTransformationComponentMask::Scale>;
    using Translation3f = AffineTransformation3<float, AffineTransformationComponentMask::Translation>;
    using RotationScale3f = AffineTransformation3<float, AffineTransformationComponentMask::RotationScale>;
    using RotationTranslation3f = AffineTransformation3<float, AffineTransformationComponentMask::RotationTranslation>;
    using ScaleTranslation3f = AffineTransformation3<float, AffineTransformationComponentMask::ScaleTranslation>;
    using AffineTransformation3f = AffineTransformation3<float, AffineTransformationComponentMask::All>;

    template <typename T, AffineTransformationComponentMask MaskLhsV, AffineTransformationComponentMask MaskRhsV>
    auto operator*(const AffineTransformation3<T, MaskLhsV>& lhs, const AffineTransformation3<T, MaskRhsV>& rhs)
    {
        static_assert(MaskLhsV != MaskRhsV, "For equal masks there should be an overload");

        static constexpr AffineTransformationComponentMask commonMask = MaskLhsV | MaskRhsV;
        if constexpr (commonMask != MaskLhsV)
        {
            return AffineTransformation3<T, commonMask>(lhs) * rhs;
        }
        else
        {
            return lhs * AffineTransformation3<T, commonMask>(rhs);
        }
    }
}