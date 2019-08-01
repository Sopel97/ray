#pragma once

#include "ShapeTraits.h"

#include <ray/math/Ray.h>
#include <ray/math/RaycastHit.h>
#include <ray/math/Transform3.h>
#include <ray/math/Vec2.h>
#include <ray/math/Vec3.h>

#include <ray/shape/Sphere.h>

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
        // TODO: generic bounding shape
        // for now only allow spheres as clipping shapes
        // it's a reasonable assumption
        // and it allows reducing virtual calls in iteration loop
        [[nodiscard]] virtual bool raycast(const Ray& ray, const Sphere& bounds, int maxIters, float accuracy, RaycastHit& hit) const = 0;
        [[nodiscard]] virtual std::unique_ptr<SdfBase> clone() const = 0;
        virtual ~SdfBase() = default;
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
        
        // TODO: think what to do with raycast
        //       it's best to have PolyIdentity at the root, so this should never be called
        //       warning would be ideal but there's no way to emit it
        //       don't implement for now

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

        // raycasting is deferred up to here because here we know exact types
        // of all sdf shapes used
        // so we can avoid all virtual calls inside the loop
        [[nodiscard]] bool raycast(const Ray& ray, const Sphere& bounds, int maxIters, float accuracy, RaycastHit& hit) const
        {
            constexpr int numStartupIters = 4;

            Point3f origin = ray.origin();
            UnitVec3f direction = ray.direction();

            // first determine whether we are inside or outside
            // set a multiplier accordingly, -1 if we're inside

            // when we start inside the shape we have to flip the sign
            // of the distance function, because if we didn't
            // we would be going back (negative increments)
            // Effectively when we're inside we're raymarching the shape's inverse
            float sign = 1.0f;
            float depth = 0.0f;

            auto sdfAbsolute = [this, &bounds] (const Point3f& p) {
                const float shape_sd = static_cast<const ExprType&>(*this).signedDistance(p);
                const float clip_sd = bounds.signedDistance(p);
                return std::max(shape_sd, clip_sd);
            };

            const float maxDepth = bounds.maxDistance(origin); // so we know when we can stop and reject
            {
                float sd = sdfAbsolute(origin);
                if (sd < 0.0f)
                {
                    // we have started inside the shape
                    sign = -1.0f;
                    sd = -sd;
                }
                depth += sd;
            }

            auto sdfAsIfOutside = [&sdfAbsolute, sign] (const Point3f& p) {
                return sdfAbsolute(p) * sign;
            };

            auto normal = [&sdfAbsolute, sign](const Point3f& p) {
                constexpr float eps = 0.0001f;

                return Normal3f((Vec3f(
                    sdfAbsolute(Point3f(p.x + eps, p.y, p.z)) - sdfAbsolute(Point3f(p.x - eps, p.y, p.z)),
                    sdfAbsolute(Point3f(p.x, p.y + eps, p.z)) - sdfAbsolute(Point3f(p.x, p.y - eps, p.z)),
                    sdfAbsolute(Point3f(p.x, p.y, p.z + eps)) - sdfAbsolute(Point3f(p.x, p.y, p.z - eps))
                ) * sign).normalized());
            };

            // precaution to prevent early exit when going away from a surface that was just hit
            for (int i = 0; i < numStartupIters; ++i)
            {
                const float sd = sdfAsIfOutside(origin + direction * depth);
                depth += sd;

                if (depth > maxDepth)
                {
                    return false;
                }
            }

            for (int i = 0; i < maxIters; ++i)
            {
                const float sd = sdfAsIfOutside(origin + direction * depth);

                depth += sd;

                if (sd < accuracy)
                {
                    // we have a hit

                    const Point3f point = origin + direction * depth;
                    hit.dist = depth;
                    hit.point = point;
                    hit.normal = normal(point);
                    hit.shapeInPackNo = 0;
                    hit.materialIndex = MaterialIndex(0, 0);
                    hit.isInside = sign < 0.0f; // if we're inside then we have negated the sdf

                    return true;
                }

                if (depth > maxDepth)
                {
                    return false;
                }
            }

            return false;
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
        const float k = self.template get<0>();
        const float h = std::clamp(0.5f + 0.5f*(d2 - d1) / k, 0.0f, 1.0f);
        return mix(d2, d1, h) - k*h*(1.0f - h);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfSmoothDifference, float)
        const float d1 = self.lhs()->signedDistance(p);
        const float d2 = -self.rhs()->signedDistance(p);
        const float k = self.template get<0>();
        const float h = std::clamp(0.5f - 0.5f*(d2 - d1) / k, 0.0f, 1.0f);
        return mix(d2, d1, h) - k * h*(1.0f - h);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_2(SdfSmoothIntersection, float)
        const float d1 = self.lhs()->signedDistance(p);
        const float d2 = self.rhs()->signedDistance(p);
        const float k = self.template get<0>();
        const float h = std::clamp(0.5f - 0.5f*(d2 - d1) / k, 0.0f, 1.0f);
        return mix(d2, d1, h) - k * h*(1.0f - h);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfRound, float)
        const float d = self.arg()->signedDistance(p);
        const float r = self.template get<0>();
        return d - r;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfOnion, float)
        const float d = self.arg()->signedDistance(p);
        const float r = self.template get<0>();
        return std::abs(d) - r;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfRepeat, Vec3f)
        const Vec3f& period = self.template get<0>();
        const Vec3f q = mod(p, period) - 0.5f*period;
        return self.arg()->signedDistance(q);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfTranslation, Vec3f)
        const Vec3f& t = self.template get<0>();
        return self.arg()->signedDistance(p - t);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_1(SdfScale, float)
        // scales by a uniform amount in all directions
        const float s = self.template get<0>();
        return self.arg()->signedDistance(p/s)*s;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfSphere, float)
        // centered at the origin
        const float r = self.template get<0>();
        return Vec3f(p).length() - r;
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfBox, Vec3f)
        // centered at the origin
        // b._ is half of the extent in a given direction
        const Vec3f& b = self.template get<0>();
        const Vec3f d = abs(Vec3f(p)) - b;
        return (max(d, Vec3f::broadcast(0.0f))).length() + std::min(std::max(d.x, std::max(d.y, d.z)), 0.0f);
    FINALIZE_SDF_EXPRESSION

    DEFINE_SDF_EXPRESSION_0(SdfCapsule, Point3f, Point3f, float)
        const Point3f& a = self.template get<0>();
        const Point3f& b = self.template get<1>();
        const float r = self.template get<2>();

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
        const auto& params = self.template get<0>();
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
        const Vec3f& r = self.template get<0>();

        const float k0 = (p.asVector() / r).length();
        const float k1 = (p.asVector() / (r*r)).length();
        return k0 * (k0 - 1.0f) / k1;
    FINALIZE_SDF_EXPRESSION

    /* 
    // TODO: redo with new conventions
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
    */

    template <typename LhsExprT>
    struct PolySdfIdentity : PolySdfExpression<PolySdfIdentity<LhsExprT>, std::tuple<LhsExprT>>
    {
        using BaseType = PolySdfExpression<PolySdfIdentity<LhsExprT>, std::tuple<LhsExprT>>;
        using BaseType::BaseType;
        using BaseType::parts;

        template <typename LhsExprFwdT>
        PolySdfIdentity(LhsExprFwdT&& expr) :
            BaseType(std::forward<LhsExprFwdT>(expr))
        {
        }

        [[nodiscard]] float signedDistance(const Point3f& p) const override
        {
            return arg()->signedDistance(p);
        }

        // defer to the next shape which is hopefully not polymorphic
        // so that there are effectively no virtual calls in the iteration loop
        [[nodiscard]] bool raycast(const Ray& ray, const Sphere& bounds, int maxIters, float accuracy, RaycastHit& hit) const override
        {
            return arg()->raycast(ray, bounds, maxIters, accuracy, hit);
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
    template <typename LhsExprT>
    PolySdfIdentity(LhsExprT)->PolySdfIdentity<LhsExprT>;
    template <typename LhsExprT>
    PolySdfIdentity(std::unique_ptr<LhsExprT>)->PolySdfIdentity<CloneableUniquePtr<SdfBase>>;

#include "detail/SdfExpressionMacroUndef.h"

    template <typename ClippingShapeT>
    struct ClippedSdf
    {
        static_assert(ShapeTraits<ClippingShapeT>::hasVolume);
        static_assert(ShapeTraits<ClippingShapeT>::isBounded);
        static_assert(ShapeTraits<ClippingShapeT>::isLocallyContinuable);

        using ClippingShapeType = ClippingShapeT;

        // use identity sdf as a root so that the iteration loop doesn't have virtual calls
        // identity defers whole raycast
        template <typename ExprT>
        ClippedSdf(const ClippingShapeType& clippingShape, const ExprT& sdf, int maxIters = 64, float accuracy = 0.0001f) :
            m_clippingShape(clippingShape),
            m_sdf(PolySdfIdentity(sdf).clone()),
            m_maxIters(maxIters),
            m_accuracy(accuracy)
        {
        }

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

        [[nodiscard]] bool raycast(const Ray& ray, RaycastHit& hit) const
        {
            return m_sdf->raycast(ray, m_clippingShape, m_maxIters, m_accuracy, hit);
        }

    private:
        ClippingShapeType m_clippingShape;
        std::unique_ptr<SdfBase> m_sdf;
        int m_maxIters;
        float m_accuracy;
    };


}
