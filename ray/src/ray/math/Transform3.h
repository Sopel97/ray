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
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::None> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::None>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::None;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(OtherMaskV, mask)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) :
            Matrix4(other)
        {
        }

        AffineTransformation3() :
            Matrix4()
        {
        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            return lhs;
        }

        friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            return rhs;
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

        void transpose3()
        {
            // nothing to do
        }

        SelfType transposed() const
        {
            return *this;
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Rotation> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Rotation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Rotation;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(OtherMaskV, mask)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) :
            Matrix4(other)
        {
        }

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
            m128::transpose3zx(m_columns);
        }

        SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose()
        {
            m128::transpose3zx(m_columns);
        }

        void transpose3()
        {
            m128::transpose3zx(m_columns);
        }

        SelfType transposed() const
        {
            SelfType m(*this);
            m.transpose();
            return m;
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Scale> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Scale>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Scale;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(OtherMaskV, mask)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) :
            Matrix4(other)
        {
        }

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

        friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            const Vec3<float> invS = 1.0f / Vec3<float>(lhs.m_values[0][0], lhs.m_values[1][1], lhs.m_values[2][2]);
            return (Vec3<float>(rhs) * invS).normalized();
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

        void transpose3()
        {
            // nothing to do
        }

        SelfType transposed() const
        {
            return *this;
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Translation> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Translation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Translation;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(OtherMaskV, mask)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) :
            Matrix4(other)
        {
        }

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

        friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            return rhs;
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

        void transpose3()
        {
            m128::transpose3zx(m_columns);
        }

        Matrix4<float> transposed() const
        {
            Matrix4<float> m(*this);
            m.transpose();
            return m;
        }

    protected:
        AffineTransformation3(__m128 c0, __m128 c1, __m128 c2, __m128 c3) :
            Matrix4(c0, c1, c2, c3)
        {
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::RotationTranslation> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::RotationTranslation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::RotationTranslation;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(OtherMaskV, mask)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) :
            Matrix4(other)
        {
        }

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

        friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            // rotation without scaling preserves length
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm)).assumeNormalized();
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
            m128::invertMatAffineNoScalePerpAxes(m_columns);
        }

        SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose3()
        {
            m128::transpose3zx(m_columns);
        }

        Matrix4<float> transposed() const
        {
            Matrix4<float> m(*this);
            m.transpose();
            return m;
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::RotationScale> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::RotationScale>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::RotationScale;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(OtherMaskV, mask)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) :
            Matrix4(other)
        {
        }

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

        friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            __m128 cols[3];
            m128::invertTransposeMat3(lhs.m_columns, cols);
            return Vec3<float>(m128::mulMat3Vec3(cols, rhs.xmm)).normalized();
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
            m128::transpose3zx(m_columns);
        }

        void transpose3()
        {
            m128::transpose3zx(m_columns);
        }

        SelfType transposed() const
        {
            SelfType m(*this);
            m.transpose();
            return m;
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::ScaleTranslation> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::ScaleTranslation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::ScaleTranslation;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(OtherMaskV, mask)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) :
            Matrix4(other)
        {
        }

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

        friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            const Vec3<float> invS = 1.0f / Vec3<float>(lhs.m_values[0][0], lhs.m_values[1][1], lhs.m_values[2][2]);
            return (Vec3<float>(rhs) * invS).normalized();
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

        void transpose3()
        {
            // nothing to do
        }

        Matrix4<float> transposed() const
        {
            Matrix4<float> m(*this);
            m.transpose();
            return m;
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::All> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::All>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::All;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(OtherMaskV, mask)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) :
            Matrix4(other)
        {
        }

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

        friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            __m128 cols[3];
            m128::invertTransposeMat3(lhs.m_columns, cols);
            return Vec3<float>(m128::mulMat3Vec3(cols, rhs.xmm)).normalized();
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        Vec3<float> apply3(const Vec3<float>& rhs) const
        {
            return Vec3<float>(m128::mulMat3Vec3(m_columns, rhs.xmm));
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

        void transpose3()
        {
            m128::transpose3zx(m_columns);
        }

        Matrix4<float> transposed() const
        {
            Matrix4<float> m(*this);
            m.transpose();
            return m;
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

    static_assert(std::is_same_v<decltype(std::declval<Identity3f>() * std::declval<Identity3f>()), Identity3f>);
    static_assert(std::is_same_v<decltype(std::declval<Rotation3f>() * std::declval<Rotation3f>()), Rotation3f>);
    static_assert(std::is_same_v<decltype(std::declval<Scale3f>() * std::declval<Scale3f>()), Scale3f>);
    static_assert(std::is_same_v<decltype(std::declval<Translation3f>() * std::declval<Translation3f>()), Translation3f>);
    static_assert(std::is_same_v<decltype(std::declval<RotationScale3f>() * std::declval<RotationScale3f>()), RotationScale3f>);
    static_assert(std::is_same_v<decltype(std::declval<RotationTranslation3f>() * std::declval<RotationTranslation3f>()), RotationTranslation3f>);
    static_assert(std::is_same_v<decltype(std::declval<ScaleTranslation3f>() * std::declval<ScaleTranslation3f>()), ScaleTranslation3f>);
    static_assert(std::is_same_v<decltype(std::declval<AffineTransformation3f>() * std::declval<AffineTransformation3f>()), AffineTransformation3f>);
    static_assert(std::is_same_v<decltype(std::declval<Matrix4f>() * std::declval<Matrix4f>()), Matrix4f>);

    static_assert(std::is_same_v<decltype(std::declval<Identity3f>() * std::declval<Rotation3f>()), Rotation3f>);
    static_assert(std::is_same_v<decltype(std::declval<Rotation3f>() * std::declval<Translation3f>()), RotationTranslation3f>);
    static_assert(std::is_same_v<decltype(std::declval<Scale3f>() * std::declval<RotationTranslation3f>()), AffineTransformation3f>);
    static_assert(std::is_same_v<decltype(std::declval<RotationScale3f>() * std::declval<Identity3f>()), RotationScale3f>);
    static_assert(std::is_same_v<decltype(std::declval<Matrix4f>() * std::declval<RotationScale3f>()), Matrix4f>);
    static_assert(std::is_same_v<decltype(std::declval<RotationTranslation3f>() * std::declval<Matrix4f>()), Matrix4f>);
    static_assert(std::is_same_v<decltype(std::declval<Identity3f>() * std::declval<ScaleTranslation3f>()), ScaleTranslation3f>);
}