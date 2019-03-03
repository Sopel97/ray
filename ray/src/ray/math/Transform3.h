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

    [[nodiscard]] constexpr AffineTransformationComponentMask operator|(AffineTransformationComponentMask lhs, AffineTransformationComponentMask rhs)
    {
        return AffineTransformationComponentMask(static_cast<std::uint32_t>(lhs) | static_cast<std::uint32_t>(rhs));
    }

    [[nodiscard]] constexpr AffineTransformationComponentMask operator&(AffineTransformationComponentMask lhs, AffineTransformationComponentMask rhs)
    {
        return AffineTransformationComponentMask(static_cast<std::uint32_t>(lhs) & static_cast<std::uint32_t>(rhs));
    }

    // whether lhs contains rhs
    [[nodiscard]] constexpr bool contains(AffineTransformationComponentMask lhs, AffineTransformationComponentMask rhs)
    {
        return (lhs & rhs) == rhs;
    }

    template <typename T, AffineTransformationComponentMask MaskV>
    struct AffineTransformation3;

    template <typename T, AffineTransformationComponentMask MaskV>
    struct AffineTransformation4;

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::None> : Matrix3<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::None>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::None;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) noexcept :
            Matrix3(other)
        {
        }

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask | AffineTransformationComponentMask::Translation, OtherMaskV)>>
        explicit AffineTransformation3(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix3(other)
        {
        }

        AffineTransformation3() noexcept :
            Matrix3()
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            return lhs;
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            return rhs;
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return rhs;
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return rhs;
        }

        void invert()
        {
            // nothing to do
        }

        [[nodiscard]] SelfType inverse() const
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

        [[nodiscard]] SelfType transposed() const
        {
            return *this;
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Rotation> : Matrix3<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Rotation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Rotation;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) noexcept :
            Matrix3(other)
        {
        }

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask | AffineTransformationComponentMask::Translation, OtherMaskV)>>
        explicit AffineTransformation3(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix3(other)
        {
        }

        AffineTransformation3() noexcept :
            Matrix3()
        {
        }

        explicit AffineTransformation3(const OrthonormalBasis3<float>& basis) noexcept :
            Matrix3(basis)
        {
        }

        explicit AffineTransformation3(const Quat4<float>& q) noexcept :
            Matrix3(q.normalized())
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMat3Mat3(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm)).assumeNormalized();
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            // property of the rotation matrix
            m128::transpose3zx(m_columns);
        }

        [[nodiscard]] SelfType inverse() const
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

        [[nodiscard]] SelfType transposed() const
        {
            SelfType m(*this);
            m.transpose();
            return m;
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Scale> : Matrix3<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Scale>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Scale;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) noexcept :
            Matrix3(other)
        {
        }

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask | AffineTransformationComponentMask::Translation, OtherMaskV)>>
        explicit AffineTransformation3(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix3(other)
        {
        }

        AffineTransformation3() noexcept :
            Matrix3()
        {
        }

        AffineTransformation3(float xs, float ys, float zs) noexcept :
            Matrix3(xs, ys, zs)
        {
        }

        explicit AffineTransformation3(const Vec3<float>& s) noexcept :
            Matrix3(s.x, s.y, s.z)
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            // just multiply the diagonal
            SelfType ret{};
            ret.m_values[0][0] = lhs.m_values[0][0] * rhs.m_values[0][0];
            ret.m_values[1][1] = lhs.m_values[1][1] * rhs.m_values[1][1];
            ret.m_values[2][2] = lhs.m_values[2][2] * rhs.m_values[2][2];
            return ret;
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            const Vec3<float> invS = 1.0f / Vec3<float>(lhs.m_values[0][0], lhs.m_values[1][1], lhs.m_values[2][2]);
            return (Vec3<float>(rhs) * invS).normalized();
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            m_values[0][0] = 1.0f / m_values[0][0];
            m_values[1][1] = 1.0f / m_values[1][1];
            m_values[2][2] = 1.0f / m_values[2][2];
        }

        [[nodiscard]] SelfType inverse() const
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

        [[nodiscard]] SelfType transposed() const
        {
            return *this;
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::RotationScale> : Matrix3<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::RotationScale>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::RotationScale;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation3(const AffineTransformation3<float, OtherMaskV>& other) noexcept :
            Matrix3(other)
        {
        }

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask | AffineTransformationComponentMask::Translation, OtherMaskV)>>
        explicit AffineTransformation3(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix3(other)
        {
        }

        AffineTransformation3() noexcept :
            Matrix3()
        {
        }

        explicit AffineTransformation3(const Basis3<float>& basis) noexcept :
            Matrix3(basis)
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMat3Mat3(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            __m128 cols[3];
            m128::invertTransposeMat3(lhs.m_columns, cols);
            return Vec3<float>(m128::mulMat3Vec3(cols, rhs.xmm)).normalized();
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            m128::invertMatAffine(m_columns);
        }

        [[nodiscard]] SelfType inverse() const
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

        [[nodiscard]] SelfType transposed() const
        {
            SelfType m(*this);
            m.transpose();
            return m;
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation4<float, AffineTransformationComponentMask::None> : Matrix4<float>
    {
        using SelfType = AffineTransformation4<float, AffineTransformationComponentMask::None>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::None;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation4(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix4(other)
        {
        }

        AffineTransformation4() noexcept :
            Matrix4()
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            return lhs;
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            return rhs;
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return rhs;
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return rhs;
        }

        void invert()
        {
            // nothing to do
        }

        [[nodiscard]] SelfType inverse() const
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

        [[nodiscard]] SelfType transposed() const
        {
            return *this;
        }

        [[nodiscard]] AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale> withoutTranslation() const
        {
            return AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale>(*this);
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation4<float, AffineTransformationComponentMask::Rotation> : Matrix4<float>
    {
        using SelfType = AffineTransformation4<float, AffineTransformationComponentMask::Rotation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Rotation;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation4(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix4(other)
        {
        }

        AffineTransformation4() noexcept :
            Matrix4()
        {
        }

        explicit AffineTransformation4(const OrthonormalBasis3<float>& basis) noexcept :
            Matrix4(basis)
        {
        }

        explicit AffineTransformation4(const Quat4<float>& q) noexcept :
            Matrix4(q.normalized())
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMat3Mat3(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm)).assumeNormalized();
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            // property of the rotation matrix
            m128::transpose3zx(m_columns);
        }

        [[nodiscard]] SelfType inverse() const
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

        [[nodiscard]] SelfType transposed() const
        {
            SelfType m(*this);
            m.transpose();
            return m;
        }

        [[nodiscard]] AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale> withoutTranslation() const
        {
            return AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale>(*this);
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation4<float, AffineTransformationComponentMask::Scale> : Matrix4<float>
    {
        using SelfType = AffineTransformation4<float, AffineTransformationComponentMask::Scale>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Scale;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation4(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix4(other)
        {
        }

        AffineTransformation4() noexcept:
            Matrix4()
        {
        }

        AffineTransformation4(float xs, float ys, float zs) noexcept :
            Matrix4(xs, ys, zs)
        {
        }

        explicit AffineTransformation4(const Vec3<float>& s) noexcept :
            Matrix4(s.x, s.y, s.z)
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            // just multiply the diagonal
            SelfType ret{};
            ret.m_values[0][0] = lhs.m_values[0][0] * rhs.m_values[0][0];
            ret.m_values[1][1] = lhs.m_values[1][1] * rhs.m_values[1][1];
            ret.m_values[2][2] = lhs.m_values[2][2] * rhs.m_values[2][2];
            return ret;
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            const Vec3<float> invS = 1.0f / Vec3<float>(lhs.m_values[0][0], lhs.m_values[1][1], lhs.m_values[2][2]);
            return (Vec3<float>(rhs) * invS).normalized();
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            m_values[0][0] = 1.0f / m_values[0][0];
            m_values[1][1] = 1.0f / m_values[1][1];
            m_values[2][2] = 1.0f / m_values[2][2];
        }

        [[nodiscard]] SelfType inverse() const
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

        [[nodiscard]] SelfType transposed() const
        {
            return *this;
        }

        [[nodiscard]] AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale> withoutTranslation() const
        {
            return AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale>(*this);
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation4<float, AffineTransformationComponentMask::Translation> : Matrix4<float>
    {
        using SelfType = AffineTransformation4<float, AffineTransformationComponentMask::Translation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Translation;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation4(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix4(other)
        {
        }

        AffineTransformation4() noexcept :
            Matrix4()
        {
        }

        explicit AffineTransformation4(const Vec3<float>& t) noexcept :
            Matrix4(t)
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            return SelfType(
                lhs.m_columns[0],
                lhs.m_columns[1],
                lhs.m_columns[2],
                // truncate to leave it homogeneous
                _mm_add_ps(m128::truncate3(rhs.m_columns[3]), lhs.m_columns[3])
            );
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            return rhs;
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return rhs + Vec3<float>(lhs.m_columns[3]);
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return rhs + Vec3<float>(lhs.m_columns[3]);
        }

        void invert()
        {
            m_columns[3] = m128::neg(m_columns[3], m128::mask_xyz());
        }

        [[nodiscard]] SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose3()
        {
            m128::transpose3zx(m_columns);
        }

        [[nodiscard]] Matrix4<float> transposed() const
        {
            Matrix4<float> m(*this);
            m.transpose();
            return m;
        }

        [[nodiscard]] AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale> withoutTranslation() const
        {
            return AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale>(*this);
        }

    protected:
        AffineTransformation4(__m128 c0, __m128 c1, __m128 c2, __m128 c3) noexcept :
            Matrix4(c0, c1, c2, c3)
        {
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation4<float, AffineTransformationComponentMask::RotationTranslation> : Matrix4<float>
    {
        using SelfType = AffineTransformation4<float, AffineTransformationComponentMask::RotationTranslation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::RotationTranslation;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation4(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix4(other)
        {
        }

        AffineTransformation4() noexcept :
            Matrix4()
        {
        }

        AffineTransformation4(const OrthonormalBasis3<float>& basis, const Vec3<float>& t) noexcept :
            Matrix4(basis, t)
        {
        }

        AffineTransformation4(const Quat4<float>& q, const Point3<float>& origin) noexcept :
            Matrix4(q.normalized(), origin)
        {
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            // rotation without scaling preserves length
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm)).assumeNormalized();
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMatAffineMatAffine(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            m128::invertMatAffineNoScalePerpAxes(m_columns);
        }

        [[nodiscard]] SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose3()
        {
            m128::transpose3zx(m_columns);
        }

        [[nodiscard]] Matrix4<float> transposed() const
        {
            Matrix4<float> m(*this);
            m.transpose();
            return m;
        }

        [[nodiscard]] AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale> withoutTranslation() const
        {
            return AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale>(*this);
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation4<float, AffineTransformationComponentMask::RotationScale> : Matrix4<float>
    {
        using SelfType = AffineTransformation4<float, AffineTransformationComponentMask::RotationScale>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::RotationScale;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation4(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix4(other)
        {
        }

        AffineTransformation4() noexcept :
            Matrix4()
        {
        }

        explicit AffineTransformation4(const Basis3<float>& basis) noexcept :
            Matrix4(basis)
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMat3Mat3(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            __m128 cols[3];
            m128::invertTransposeMat3(lhs.m_columns, cols);
            return Vec3<float>(m128::mulMat3Vec3(cols, rhs.xmm)).normalized();
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            m128::invertMatAffine(m_columns);
        }

        [[nodiscard]] SelfType inverse() const
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

        [[nodiscard]] SelfType transposed() const
        {
            SelfType m(*this);
            m.transpose();
            return m;
        }

        [[nodiscard]] AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale> withoutTranslation() const
        {
            return AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale>(*this);
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation4<float, AffineTransformationComponentMask::ScaleTranslation> : Matrix4<float>
    {
        using SelfType = AffineTransformation4<float, AffineTransformationComponentMask::ScaleTranslation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::ScaleTranslation;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation4(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix4(other)
        {
        }

        AffineTransformation4() noexcept :
            Matrix4()
        {
        }

        AffineTransformation4(float xs, float ys, float zs, const Vec3<float>& t) noexcept :
            Matrix4(xs, ys, zs, t)
        {
        }

        AffineTransformation4(const Vec3<float>& s, const Vec3<float>& t) noexcept :
            Matrix4(s.x, s.y, s.z, t)
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMatAffineMatAffine(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            const Vec3<float> invS = 1.0f / Vec3<float>(lhs.m_values[0][0], lhs.m_values[1][1], lhs.m_values[2][2]);
            return (Vec3<float>(rhs) * invS).normalized();
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            // could probably be done more efficiently, but it's not important
            m128::invertMatAffine(m_columns);
        }

        [[nodiscard]] SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose3()
        {
            // nothing to do
        }

        [[nodiscard]] Matrix4<float> transposed() const
        {
            Matrix4<float> m(*this);
            m.transpose();
            return m;
        }

        [[nodiscard]] AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale> withoutTranslation() const
        {
            return AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale>(*this);
        }
    };

    template <>
    struct alignas(alignof(__m128)) AffineTransformation4<float, AffineTransformationComponentMask::All> : Matrix4<float>
    {
        using SelfType = AffineTransformation4<float, AffineTransformationComponentMask::All>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::All;

        // enable construction from more specific matrix type
        template <AffineTransformationComponentMask OtherMaskV, typename SfinaeT = std::enable_if_t<contains(mask, OtherMaskV)>>
        AffineTransformation4(const AffineTransformation4<float, OtherMaskV>& other) noexcept :
            Matrix4(other)
        {
        }

        AffineTransformation4() noexcept :
            Matrix4()
        {
        }

        AffineTransformation4(const Basis3<float>& basis, const Vec3<float>& t) noexcept :
            Matrix4(basis, t)
        {
        }

        [[nodiscard]] friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            m128::mulMatAffineMatAffine(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        [[nodiscard]] friend Normal3<float> operator*(const SelfType& lhs, const Normal3<float>& rhs)
        {
            __m128 cols[3];
            m128::invertTransposeMat3(lhs.m_columns, cols);
            return Vec3<float>(m128::mulMat3Vec3(cols, rhs.xmm)).normalized();
        }

        [[nodiscard]] friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        [[nodiscard]] friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(m128::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        void invert()
        {
            m128::invertMatAffine(m_columns);
        }

        [[nodiscard]] SelfType inverse() const
        {
            SelfType c(*this);
            c.invert();
            return c;
        }

        void transpose3()
        {
            m128::transpose3zx(m_columns);
        }

        [[nodiscard]] Matrix4<float> transposed() const
        {
            Matrix4<float> m(*this);
            m.transpose();
            return m;
        }

        [[nodiscard]] AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale> withoutTranslation() const
        {
            return AffineTransformation3<float, mask & AffineTransformationComponentMask::RotationScale>(*this);
        }
    };

    using Identity3f = AffineTransformation3<float, AffineTransformationComponentMask::None>;
    using Rotation3f = AffineTransformation3<float, AffineTransformationComponentMask::Rotation>;
    using Scale3f = AffineTransformation3<float, AffineTransformationComponentMask::Scale>;
    using RotationScale3f = AffineTransformation3<float, AffineTransformationComponentMask::RotationScale>;

    using Identity4f = AffineTransformation4<float, AffineTransformationComponentMask::None>;
    using Rotation4f = AffineTransformation4<float, AffineTransformationComponentMask::Rotation>;
    using Scale4f = AffineTransformation4<float, AffineTransformationComponentMask::Scale>;
    using Translation4f = AffineTransformation4<float, AffineTransformationComponentMask::Translation>;
    using RotationScale4f = AffineTransformation4<float, AffineTransformationComponentMask::RotationScale>;
    using RotationTranslation4f = AffineTransformation4<float, AffineTransformationComponentMask::RotationTranslation>;
    using ScaleTranslation4f = AffineTransformation4<float, AffineTransformationComponentMask::ScaleTranslation>;
    using AffineTransformation4f = AffineTransformation4<float, AffineTransformationComponentMask::All>;

    template <typename T, AffineTransformationComponentMask MaskLhsV, AffineTransformationComponentMask MaskRhsV>
    [[nodiscard]] auto operator*(const AffineTransformation4<T, MaskLhsV>& lhs, const AffineTransformation4<T, MaskRhsV>& rhs)
    {
        static_assert(MaskLhsV != MaskRhsV, "For equal masks there should be an overload");

        static constexpr AffineTransformationComponentMask commonMask = MaskLhsV | MaskRhsV;
        if constexpr (commonMask != MaskLhsV)
        {
            return AffineTransformation4<T, commonMask>(lhs) * rhs;
        }
        else
        {
            return lhs * AffineTransformation4<T, commonMask>(rhs);
        }
    }

    static_assert(std::is_same_v<decltype(std::declval<Identity4f>() * std::declval<Identity4f>()), Identity4f>);
    static_assert(std::is_same_v<decltype(std::declval<Rotation4f>() * std::declval<Rotation4f>()), Rotation4f>);
    static_assert(std::is_same_v<decltype(std::declval<Scale4f>() * std::declval<Scale4f>()), Scale4f>);
    static_assert(std::is_same_v<decltype(std::declval<Translation4f>() * std::declval<Translation4f>()), Translation4f>);
    static_assert(std::is_same_v<decltype(std::declval<RotationScale4f>() * std::declval<RotationScale4f>()), RotationScale4f>);
    static_assert(std::is_same_v<decltype(std::declval<RotationTranslation4f>() * std::declval<RotationTranslation4f>()), RotationTranslation4f>);
    static_assert(std::is_same_v<decltype(std::declval<ScaleTranslation4f>() * std::declval<ScaleTranslation4f>()), ScaleTranslation4f>);
    static_assert(std::is_same_v<decltype(std::declval<AffineTransformation4f>() * std::declval<AffineTransformation4f>()), AffineTransformation4f>);
    static_assert(std::is_same_v<decltype(std::declval<Matrix4f>() * std::declval<Matrix4f>()), Matrix4f>);

    static_assert(std::is_same_v<decltype(std::declval<Identity4f>() * std::declval<Rotation4f>()), Rotation4f>);
    static_assert(std::is_same_v<decltype(std::declval<Rotation4f>() * std::declval<Translation4f>()), RotationTranslation4f>);
    static_assert(std::is_same_v<decltype(std::declval<Scale4f>() * std::declval<RotationTranslation4f>()), AffineTransformation4f>);
    static_assert(std::is_same_v<decltype(std::declval<RotationScale4f>() * std::declval<Identity4f>()), RotationScale4f>);
    static_assert(std::is_same_v<decltype(std::declval<Matrix4f>() * std::declval<RotationScale4f>()), Matrix4f>);
    static_assert(std::is_same_v<decltype(std::declval<RotationTranslation4f>() * std::declval<Matrix4f>()), Matrix4f>);
    static_assert(std::is_same_v<decltype(std::declval<Identity4f>() * std::declval<ScaleTranslation4f>()), ScaleTranslation4f>);

    static_assert(std::is_same_v<decltype(static_cast<Scale4f>(std::declval<Identity4f>())), Scale4f>);
    // static_assert(std::is_same_v<decltype(static_cast<Identity4f>(std::declval<Scale4f>())), Identity4f>);

    static_assert(std::is_same_v<decltype(std::declval<Identity4f>().withoutTranslation()), Identity3f>);
    static_assert(std::is_same_v<decltype(std::declval<RotationScale4f>().withoutTranslation()), RotationScale3f>);
    static_assert(std::is_same_v<decltype(std::declval<RotationTranslation4f>().withoutTranslation()), Rotation3f>);
    static_assert(std::is_same_v<decltype(std::declval<Translation4f>().withoutTranslation()), Identity3f>);
    static_assert(std::is_same_v<decltype(std::declval<AffineTransformation4f>().withoutTranslation()), RotationScale3f>);
}