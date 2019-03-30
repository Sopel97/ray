#pragma once

#include "ShapeTraits.h"

#include <ray/math/Transform3.h>
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

    // This class is always instantiated as a parent of some expression using CRTP
    // Because of that it is safe to use it as if it was ExprType
    // Note that both this and specific SDF functions have normal
    // and Poly___ versions explicitly defined so devirtualization is not needed.
    // Required to prevent unnecessary virtual calls with MSVC.
    template <typename ExprT, typename PartsT>
    struct PolySdfExpression : SdfBase, private PartsT
    {
        using PartsType = PartsT;
        using ExprType = ExprT;

        template <typename... PartsFwdTs>
        PolySdfExpression(PartsFwdTs&&... parts) :
            PartsType(std::forward<PartsFwdTs>(parts)...)
        {

        }

        [[nodiscard]] std::unique_ptr<SdfBase> clone() const override
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

    template <typename ExprT, typename PartsT>
    struct SdfExpression : private PartsT
    {
        using PartsType = PartsT;
        using ExprType = ExprT;

        template <typename... PartsFwdTs>
        SdfExpression(PartsFwdTs&&... parts) :
            PartsType(std::forward<PartsFwdTs>(parts)...)
        {

        }

        [[nodiscard]] std::unique_ptr<ExprType> clone() const
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
#include "detail/SdfExpressionMacroDef.h"

    // http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
    // TODO: consider using approximate versions of more expensive functions since
    //       it's an iterative process and may not get such a big hit in precision

    DEFINE_SDF_EXPRESSION_2(SdfUnion)
        return std::min(
            self.lhs()->signedDistance(p),
            self.rhs()->signedDistance(p)
        );
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfDifference)
        return std::max(
            self.lhs()->signedDistance(p),
            -self.rhs()->signedDistance(p)
        );
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfIntersection)
        return std::max(
            self.lhs()->signedDistance(p),
            self.rhs()->signedDistance(p)
        );
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfSmoothUnion, float)
        const float d1 = self.lhs()->signedDistance(p);
        const float d2 = self.rhs()->signedDistance(p);
        const float k = self.get<0>();
        const float h = std::clamp(0.5f + 0.5f*(d2 - d1) / k, 0.0f, 1.0f);
        return mix(d2, d1, h) - k*h*(1.0f - h);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfSmoothDifference, float)
        const float d1 = self.lhs()->signedDistance(p);
        const float d2 = -self.rhs()->signedDistance(p);
        const float k = self.get<0>();
        const float h = std::clamp(0.5f - 0.5f*(d2 - d1) / k, 0.0f, 1.0f);
        return mix(d2, d1, h) - k * h*(1.0f - h);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfSmoothIntersection, float)
        const float d1 = self.lhs()->signedDistance(p);
        const float d2 = self.rhs()->signedDistance(p);
        const float k = self.get<0>();
        const float h = std::clamp(0.5f - 0.5f*(d2 - d1) / k, 0.0f, 1.0f);
        return mix(d2, d1, h) - k * h*(1.0f - h);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfRound, float)
        const float d = self.arg()->signedDistance(p);
        const float r = self.get<0>();
        return d - r;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfOnion, float)
        const float d = self.arg()->signedDistance(p);
        const float r = self.get<0>();
        return std::abs(d) - r;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfRepeat, Vec3f)
        const Vec3f& period = self.get<0>();
        const Vec3f q = mod(p, period) - 0.5f*period;
        return self.arg()->signedDistance(q);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfTranslation, Vec3f)
        const Vec3f& t = self.get<0>();
        return self.arg()->signedDistance(p - t);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfScale, float)
        // scales by a uniform amount in all directions
        const float s = self.get<0>();
        return self.arg()->signedDistance(p/s)*s;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfSphere, float)
        // centered at the origin
        const float r = self.get<0>();
        return Vec3f(p).length() - r;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfBox, Vec3f)
        // centered at the origin
        // b._ is half of the extent in a given direction
        const Vec3f& b = self.get<0>();
        const Vec3f d = abs(Vec3f(p)) - b;
        return (max(d, Vec3f::broadcast(0.0f))).length() + std::min(std::max(d.x, std::max(d.y, d.z)), 0.0f);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfCapsule, Point3f, Point3f, float)
        const Point3f& a = self.get<0>();
        const Point3f& b = self.get<1>();
        const float r = self.get<2>();

        const Vec3f pa = p - a;
        const Vec3f ba = b - a;
        const float h = std::clamp(dot(pa, ba) / dot(ba, ba), 0.0f, 1.0f);
        return (pa - ba * h).length() - r;
    FINALIZE_SDF_EXPRESSION

    struct SdfRoundedConeParams
    {
        SdfRoundedConeParams(float r1, float r2, float h) :
            r1(r1),
            r2(r2),
            h(h),
            b((r1 - r2) / h),
            a(std::sqrt(1.0f - b * b))
        {

        }

        float r1;
        float r2;
        float h;
        float b;
        float a;
    };

    DEFINE_SDF_EXPRESSION_0(SdfRoundedCone, SdfRoundedConeParams)
        // starts at origin with radius r1
        // end at (0, h, 0) with radius r2
        /*
        // base version
        const float r1 = self.get<0>();
        const float r2 = self.get<1>();
        const float h = self.get<2>();

        const Vec2f q = Vec2f(Vec2f(p.x, p.z).length(), p.y);

        const float b = (r1 - r2) / h;
        const float a = std::sqrt(1.0f - b * b);
        const float k = dot(q, Vec2f(-b, a));

        if (k < 0.0f) return q.length() - r1;
        if (k > a*h) return (q - Vec2f(0.0f, h)).length() - r2;

        return dot(q, Vec2f(a, b)) - r1;
        */

        /*
        // semi-optimized, using precomputed parameters
        const auto& params = self.get<0>();
        const float r1 = params.r1;
        const float r2 = params.r2;
        const float h = params.h;
        const float b = params.b;
        const float a = params.a;

        const float xz_len = Vec2f(p.x, p.z).length();

        //const float b = (r1 - r2) / h;
        //const float a = std::sqrt(1.0f - b * b);
        const float apy = a * p.y;
        const float bxz_len = b * xz_len;

        if (apy < bxz_len) return p.asVector().length() - r1;
        const float k = apy - bxz_len;
        if (k > a*h) return std::sqrt(xz_len * xz_len + (p.y-h)*(p.y-h)) - r2;

        return a * xz_len + b * p.y - r1;
        */

        // fully optimized
        const auto& params = self.get<0>();
        const float r1 = params.r1;
        const float r2 = params.r2;
        const float b = params.b;
        const float a = params.a;
        const float babsb = b * std::abs(b);

        const float xz_dot = p.x * p.x + p.z * p.z;
        const float py = p.y;

        const float apy = a * py;
        const float babsb_xz_dot = babsb * xz_dot;

        // using the fact that a < b <=> a*abs(a) < b*abs(b)
        // b2xz_dot always positive so b2xz_dot = xz_len * abs(xz_len)
        if (apy * std::abs(apy) < babsb_xz_dot) return std::sqrt(xz_dot + py * py) - r1;
        const float xz_len = std::sqrt(xz_dot);
        const float bxz_len = b * xz_len;
        const float k = apy - bxz_len;
        const float h = params.h;
        if (k > a*h) return std::sqrt(xz_dot + (py - h)*(py - h)) - r2;

        return a * xz_len + b * py - r1;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfEllipsoid, Vec3f)
        // centered at the origin
        // with extents of r._
        const Vec3f& r = self.get<0>();

        const float k0 = (p.asVector() / r).length();
        const float k1 = (p.asVector() / (r*r)).length();
        return k0 * (k0 - 1.0f) / k1;
    FINALIZE_SDF_EXPRESSION


    template <typename LhsExprT, typename TransformT> 
    struct SdfTransform : SdfExpression<SdfTransform<LhsExprT, TransformT>, std::tuple<LhsExprT, TransformT>>
    { 
        // TODO: handle other matrices properly
        //       currently we can only handle non-scaling transforms
        static_assert(contains(AffineTransformationComponentMask::RotationTranslation, TransformT::mask));

        using BaseType = SdfExpression<SdfTransform, std::tuple<LhsExprT, TransformT>>;
        using BaseType::BaseType; 
        using BaseType::parts;

        // here we hold the inverse, because it's all we need
        template <typename LhsExprFwdT, typename TransformFwdT>
        SdfTransform(LhsExprFwdT&& expr, TransformFwdT&& transform) :
            SdfExpression(std::forward<LhsExprFwdT>(expr), transform.inverse())
        {
        }

        [[nodiscard]] float signedDistance(const Point3f& p) const override
        {
            const TransformT& transformInverse = get<0>();
            return get()->signedDistance(transformInverse * p);
        }

    protected: 
        template <int I> 
        [[nodiscard]] decltype(auto) get() const 
        { 
            return std::get<I + 1>(parts()); 
        } 

        [[nodiscard]] decltype(auto) arg() const 
        {
            return std::get<0>(parts());
        }
    }; 
    // TODO: change RotationTranslation4f -> Matrix4f when it is supported
    using PolySdfTransform = SdfTransform<CloneableUniquePtr<SdfBase>, RotationTranslation4f>;
    template <typename LhsExprT, typename TransformT>
    SdfTransform(LhsExprT, TransformT)->SdfTransform<LhsExprT, TransformT>;
    template <typename LhsExprT, typename TransformT>
    SdfTransform(std::unique_ptr<LhsExprT>, TransformT)->SdfTransform<CloneableUniquePtr<SdfBase>, TransformT>;

#include "detail/SdfExpressionMacroUndef.h"

}
