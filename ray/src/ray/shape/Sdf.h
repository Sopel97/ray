#pragma once

#include "ShapeTraits.h"

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

#undef DEFINE_SDF_EXPRESSION_0
#undef DEFINE_SDF_EXPRESSION_1
#undef DEFINE_SDF_EXPRESSION_2
#undef FINALIZE_SDF_EXPRESSION
}
