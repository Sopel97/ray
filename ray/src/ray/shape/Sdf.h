#pragma once

#include "ShapeTraits.h"

#include <ray/math/Vec2.h>
#include <ray/math/Vec3.h>

#include <ray/utility/CloneableUniquePtr.h>
#include <ray/utility/Util.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace ray
{
    struct SdfBase
    {
        [[nodiscard]] virtual float signedDistance(const Point3f& p) const = 0;
        [[nodiscard]] virtual std::unique_ptr<SdfBase> clone() const = 0;
        virtual ~SdfBase() = default;
    };

    template <typename ClippingShapeT>
    struct ClippedSdf
    {
        static_assert(ShapeTraits<ClippingShapeT>::hasVolume);
        static_assert(ShapeTraits<ClippingShapeT>::isBounded);
        static_assert(ShapeTraits<ClippingShapeT>::isLocallyContinuable);

        using ClippingShapeType = ClippingShapeT;

        ClippedSdf(const ClippingShapeType& clippingShape, std::unique_ptr<SdfBase>&& sdf, int maxIters = 64, float accuracy = 0.0001f) :
            m_clippingShape(clippingShape),
            m_sdf(std::move(sdf)),
            m_maxIters(maxIters),
            m_accuracy(accuracy)
        {
        }

        ClippedSdf(const ClippedSdf<ClippingShapeType>& other) :
            m_clippingShape(other.m_clippingShape),
            m_sdf(other.m_sdf->clone()),
            m_maxIters(other.m_maxIters),
            m_accuracy(other.m_accuracy)
        {

        }

        [[nodiscard]] float signedDistance(const Point3f& p) const
        {
            return m_sdf->signedDistance(p);
        }

        [[nodiscard]] Point3f center() const
        {
            return m_clippingShape.center();
        }

        [[nodiscard]] const ClippingShapeType& clippingShape() const
        {
            return m_clippingShape;
        }

        [[nodiscard]] int maxIters() const
        {
            return m_maxIters;
        }

        [[nodiscard]] float accuracy() const
        {
            return m_accuracy;
        }

    private:
        ClippingShapeType m_clippingShape;
        std::unique_ptr<SdfBase> m_sdf;
        int m_maxIters;
        float m_accuracy;
    };

    // tuple<> has non zero size. maybe use inheritance
    template <typename ExprT, typename PartsT>
    struct SdfExpression : SdfBase, private PartsT
    {
        using PartsType = PartsT;
        using ExprType = ExprT;

        template <typename... PartsFwdTs>
        SdfExpression(PartsFwdTs&&... parts) :
            PartsType(std::forward<PartsFwdTs>(parts)...)
        {

        }

        [[nodiscard]] virtual std::unique_ptr<SdfBase> clone() const override
        {
            return std::make_unique<ExprType>(*static_cast<const ExprType*>(this));
        }

        [[nodiscard]] ExprType* operator->()
        {
            return static_cast<ExprType*>(this);
        }

        [[nodiscard]] const ExprType* operator->() const
        {
            return static_cast<const ExprType*>(this);
        }

    protected:
        const PartsType& parts() const
        {
            return *this;
        }
    };

#define DEFINE_SDF_EXPRESSION_0(TypeName, ...) \
    struct TypeName : SdfExpression<TypeName, std::tuple<__VA_ARGS__>> \
    { \
        using BaseType = SdfExpression<TypeName, std::tuple<__VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] virtual float signedDistance(const Point3f& p) const override; \
    protected: \
        template <int I> \
        [[nodiscard]] decltype(auto) get() const \
        { \
            return std::get<I>(parts()); \
        } \
    }; \
    using Poly##TypeName = TypeName; \
    [[nodiscard]] float TypeName::signedDistance(const Point3f& p) const \
    {

#define DEFINE_SDF_EXPRESSION_1(TypeName, ...) \
    template <typename LhsExprT> \
    struct TypeName : SdfExpression<TypeName<LhsExprT>, std::tuple<LhsExprT, __VA_ARGS__>> \
    { \
        using BaseType = SdfExpression<TypeName, std::tuple<LhsExprT, __VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] virtual float signedDistance(const Point3f& p) const override; \
    protected: \
        template <int I> \
        [[nodiscard]] decltype(auto) get() const \
        { \
            return std::get<I + 1>(parts()); \
        } \
        [[nodiscard]] decltype(auto) arg() const \
        { \
            return std::get<0>(parts()); \
        } \
    }; \
    using Poly##TypeName = TypeName<CloneableUniquePtr<SdfBase>>; \
    template <typename LhsExprT> \
    TypeName(LhsExprT, __VA_ARGS__)->TypeName<LhsExprT>; \
    template <typename LhsExprT> \
    TypeName(std::unique_ptr<LhsExprT>, __VA_ARGS__)->TypeName<CloneableUniquePtr<SdfBase>>; \
    template <typename LhsExprT> \
    [[nodiscard]] float TypeName<LhsExprT>::signedDistance(const Point3f& p) const \
    {

#define DEFINE_SDF_EXPRESSION_2(TypeName, ...) \
    template <typename LhsExprT, typename RhsExprT> \
    struct TypeName : SdfExpression<TypeName<LhsExprT, RhsExprT>, std::tuple<LhsExprT, RhsExprT, __VA_ARGS__>> \
    { \
        using BaseType = SdfExpression<TypeName, std::tuple<LhsExprT, RhsExprT, __VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] virtual float signedDistance(const Point3f& p) const override; \
    protected: \
        template <int I> \
        [[nodiscard]] decltype(auto) get() const \
        { \
            return std::get<I + 2>(parts()); \
        } \
        [[nodiscard]] decltype(auto) lhs() const \
        { \
            return std::get<0>(parts()); \
        } \
        [[nodiscard]] decltype(auto) rhs() const \
        { \
            return std::get<1>(parts()); \
        } \
    }; \
    using Poly##TypeName = TypeName<CloneableUniquePtr<SdfBase>, CloneableUniquePtr<SdfBase>>; \
    template <typename LhsExprT, typename RhsExprT> \
    TypeName(LhsExprT, RhsExprT, __VA_ARGS__)->TypeName<LhsExprT, RhsExprT>; \
    template <typename LhsExprT, typename RhsExprT> \
    TypeName(std::unique_ptr<LhsExprT>, std::unique_ptr<RhsExprT>, __VA_ARGS__)->TypeName<CloneableUniquePtr<SdfBase>, CloneableUniquePtr<SdfBase>>; \
    template <typename LhsExprT, typename RhsExprT> \
    [[nodiscard]] float TypeName<LhsExprT, RhsExprT>::signedDistance(const Point3f& p) const \
    {

#define FINALIZE_SDF_EXPRESSION }

    /*
    template <typename LhsExprT, typename RhsExprT>
    struct SdfUnion : SdfExpression<SdfUnion<LhsExprT, RhsExprT>, std::tuple<LhsExprT, RhsExprT>>
    {
        using BaseType = SdfExpression<SdfUnion, std::tuple<LhsExprT, RhsExprT>>;
        using BaseType::BaseType;
        using BaseType::parts;

        [[nodiscard]] virtual float signedDistance(const Point3f& p) const override;

    protected:
        template <int I>
        [[nodiscard]] decltype(auto) get() const
        {
            return std::get<I+2>(parts());
        }

        [[nodiscard]] decltype(auto) lhs() const
        {
            return std::get<0>(parts());
        }

        [[nodiscard]] decltype(auto) rhs() const
        {
            return std::get<1>(parts());
        }
    };
    template <typename LhsExprT, typename RhsExprT>
    SdfUnion(LhsExprT, RhsExprT)->SdfUnion<LhsExprT, RhsExprT>;
    template <typename LhsExprT, typename RhsExprT>
    SdfUnion(std::unique_ptr<LhsExprT>, std::unique_ptr<RhsExprT>)->SdfUnion<CloneableUniquePtr<SdfBase>, CloneableUniquePtr<SdfBase>>;

    template <typename LhsExprT, typename RhsExprT>
    [[nodiscard]] float SdfUnion<LhsExprT, RhsExprT>::signedDistance(const Point3f& p) const
    {
        return std::min(
            lhs()->signedDistance(p),
            rhs()->signedDistance(p)
        );
    }

    // examples
    DEFINE_SDF_EXPRESSION_2(SdfUnion)
        return std::min(
            lhs()->signedDistance(p),
            rhs()->signedDistance(p)
        );
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfUnion2)
        return arg()->signedDistance(p);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfUnion3, float)
        return get<0>();
    FINALIZE_SDF_EXPRESSION
    */

    // http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm

    DEFINE_SDF_EXPRESSION_2(SdfUnion)
        return std::min(
            lhs()->signedDistance(p),
            rhs()->signedDistance(p)
        );
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfDifference)
        return std::max(
            lhs()->signedDistance(p),
            -rhs()->signedDistance(p)
        );
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfIntersection)
        return std::max(
            lhs()->signedDistance(p),
            rhs()->signedDistance(p)
        );
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfSmoothUnion, float)
        const float d1 = lhs()->signedDistance(p);
        const float d2 = rhs()->signedDistance(p);
        const float k = get<0>();
        const float h = std::clamp(0.5f + 0.5f*(d2 - d1) / k, 0.0f, 1.0f);
        return mix(d2, d1, h) - k*h*(1.0f - h);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfSmoothDifference, float)
        const float d1 = lhs()->signedDistance(p);
        const float d2 = -rhs()->signedDistance(p);
        const float k = get<0>();
        const float h = std::clamp(0.5f - 0.5f*(d2 - d1) / k, 0.0f, 1.0f);
        return mix(d2, d1, h) - k * h*(1.0f - h);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfSmoothIntersection, float)
        const float d1 = lhs()->signedDistance(p);
        const float d2 = rhs()->signedDistance(p);
        const float k = get<0>();
        const float h = std::clamp(0.5f - 0.5f*(d2 - d1) / k, 0.0f, 1.0f);
        return mix(d2, d1, h) - k * h*(1.0f - h);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfRound, float)
        const float d = arg()->signedDistance(p);
        const float r = get<0>();
        return d - r;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfOnion, float)
        const float d = arg()->signedDistance(p);
        const float r = get<0>();
        return std::abs(d) - r;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfRepeat, Vec3f)
        const Vec3f& period = get<0>();
        const Vec3f q = mod(p, period) - 0.5f*period;
        return arg()->signedDistance(q);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfTranslation, Vec3f)
        const Vec3f& t = get<0>();
        return arg()->signedDistance(p - t);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfSphere, float)
        // centered at the origin
        const float r = get<0>();
        return Vec3f(p).length() - r;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfBox, Vec3f)
        // centered at the origin
        // b._ is half of the extent in a given direction
        const Vec3f& b = get<0>();
        const Vec3f d = abs(Vec3f(p)) - b;
        return (max(d, Vec3f::broadcast(0.0f))).length() + std::min(std::max(d.x, std::max(d.y, d.z)), 0.0f);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfCapsule, Point3f, Point3f, float)
        const Point3f& a = get<0>();
        const Point3f& b = get<1>();
        const float r = get<2>(); 

        const Vec3f pa = p - a;
        const Vec3f ba = b - a;
        const float h = std::clamp(dot(pa, ba) / dot(ba, ba), 0.0f, 1.0f);
        return (pa - ba * h).length() - r;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfRoundedCone, float, float, float)
        // starts at origin with radius r1
        // end at (0, h, 0) with radius r2
        const float r1 = get<0>();
        const float r2 = get<1>();
        const float h = get<2>();

        const Vec2f q = Vec2f(Vec2f(p.x, p.z).length(), p.y);

        const float b = (r1 - r2) / h;
        const float a = std::sqrt(1.0f - b * b);
        const float k = dot(q, Vec2f(-b, a));

        if (k < 0.0f) return q.length() - r1;
        if (k > a*h) return (q - Vec2f(0.0f, h)).length() - r2;

        return dot(q, Vec2f(a, b)) - r1;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfEllipsoid, Vec3f)
        // centered at the origin
        // with extents of r._
        const Vec3f& r = get<0>();

        const float k0 = (Vec3f(p) / r).length();
        const float k1 = (Vec3f(p) / (r*r)).length();
        return k0 * (k0 - 1.0f) / k1;
    FINALIZE_SDF_EXPRESSION


#undef DEFINE_SDF_EXPRESSION_0
#undef DEFINE_SDF_EXPRESSION_1
#undef DEFINE_SDF_EXPRESSION_2
#undef FINALIZE_SDF_EXPRESSION
}
