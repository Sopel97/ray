#pragma once

#include "ShapeTraits.h"

#include <ray/math/Vec3.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace ray
{
    struct SdfBase
    {
        [[nodiscard]] virtual float signedDistance(const Point3f& p) const = 0;
        [[nodiscard]] virtual std::unique_ptr<SdfBase> clone() const = 0;
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

    template <typename LhsExprT, typename RhsExprT>
    struct SdfUnion : SdfBase
    {
        SdfUnion(LhsExprT lhs, RhsExprT rhs) :
            m_lhs(std::move(lhs)),
            m_rhs(std::move(rhs))
        {

        }

        [[nodiscard]] virtual float signedDistance(const Point3f& p) const override
        {
            return std::min(
                m_lhs.signedDistance(p),
                m_rhs.signedDistance(p)
            );
        }

        [[nodiscard]] virtual std::unique_ptr<SdfBase> clone() const override
        {
            return std::make_unique<SdfUnion>(*this);
        }

    private:
        LhsExprT m_lhs;
        RhsExprT m_rhs;
    };
}
