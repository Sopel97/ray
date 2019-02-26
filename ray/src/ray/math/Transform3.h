#pragma once

#include "detail/M128Math.h"
#include "detail/M128MatrixOperations.h"

#include "Matrix4.h"
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
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Rotation> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Rotation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Rotation;

        AffineTransformation3() :
            Matrix4()
        {

        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            detail::mulMat3Mat3(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(detail::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(detail::mulMat3Vec3(lhs.m_columns, rhs.xmm));
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
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Scale> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Scale>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Scale;

        AffineTransformation3() :
            Matrix4()
        {

        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            detail::mulMat3Mat3(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(detail::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(detail::mulMat3Vec3(lhs.m_columns, rhs.xmm));
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
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::Translation> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::Translation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::Translation;

        AffineTransformation3() :
            Matrix4()
        {

        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            return SelfType(
                lhs.m_columns[0],
                lhs.m_columns[1],
                lhs.m_columns[2],
                // truncate to leave it homogeneous
                _mm_add_ps(detail::truncate3(rhs.m_columns[3]), lhs.m_columns[3])
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
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::RotationTranslation> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::RotationTranslation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::RotationTranslation;

        AffineTransformation3() :
            Matrix4()
        {

        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            detail::mulMatAffineMatAffine(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(detail::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(detail::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
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
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::RotationScale> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::RotationScale>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::RotationScale;

        AffineTransformation3() :
            Matrix4()
        {

        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            detail::mulMat3Mat3(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(detail::mulMat3Vec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(detail::mulMat3Vec3(lhs.m_columns, rhs.xmm));
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
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::ScaleTranslation> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::ScaleTranslation>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::ScaleTranslation;

        AffineTransformation3() :
            Matrix4()
        {

        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            detail::mulMatAffineMatAffine(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(detail::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(detail::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
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
    struct alignas(alignof(__m128)) AffineTransformation3<float, AffineTransformationComponentMask::All> : Matrix4<float>
    {
        using SelfType = AffineTransformation3<float, AffineTransformationComponentMask::All>;
        static constexpr AffineTransformationComponentMask mask = AffineTransformationComponentMask::All;

        AffineTransformation3() :
            Matrix4()
        {

        }

        friend SelfType operator*(const SelfType& lhs, const SelfType& rhs)
        {
            SelfType ret{};
            detail::mulMatAffineMatAffine(lhs.m_columns, rhs.m_columns, ret.m_columns);
            return ret;
        }

        friend Vec3<float> operator*(const SelfType& lhs, const Vec3<float>& rhs)
        {
            return Vec3<float>(detail::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
        }

        friend Point3<float> operator*(const SelfType& lhs, const Point3<float>& rhs)
        {
            return Point3<float>(detail::mulMatAffineVec3(lhs.m_columns, rhs.xmm));
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