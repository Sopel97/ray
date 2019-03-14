#pragma once

#include "ShapeTraits.h"

#include <ray/math/Vec3.h>

#include <ray/utility/CloneableUniquePtr.h>

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

    template <typename LhsExprT, typename RhsExprT>
    struct SdfUnion : SdfExpression<SdfUnion<LhsExprT, RhsExprT>, std::tuple<LhsExprT, RhsExprT>>
    {
        using BaseType = SdfExpression<SdfUnion, std::tuple<LhsExprT, RhsExprT>>;
        using BaseType::BaseType;
        using BaseType::parts;

        [[nodiscard]] virtual float signedDistance(const Point3f& p) const override
        {
            return std::min(
                get<0>()->signedDistance(p),
                get<1>()->signedDistance(p)
            );
        }

    protected:
        template <int I>
        [[nodiscard]] decltype(auto) get() const
        {
            return std::get<I>(parts());
        }
    };
    template <typename LhsExprT, typename RhsExprT>
    SdfUnion(LhsExprT, RhsExprT)->SdfUnion<LhsExprT, RhsExprT>;
    template <typename LhsExprT, typename RhsExprT>
    SdfUnion(std::unique_ptr<LhsExprT>, std::unique_ptr<RhsExprT>)->SdfUnion<CloneableUniquePtr<SdfBase>, CloneableUniquePtr<SdfBase>>;
}
