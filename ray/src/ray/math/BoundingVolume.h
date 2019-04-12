#pragma once

#include <ray/shape/Box3.h>
#include <ray/shape/Capsule.h>
#include <ray/shape/Cylinder.h>
#include <ray/shape/Disc3.h>
#include <ray/shape/HalfSphere.h>
#include <ray/shape/OrientedBox3.h>
#include <ray/shape/Sdf.h>
#include <ray/shape/ShapeTags.h>
#include <ray/shape/Sphere.h>
#include <ray/shape/TransformedShape3.h>
#include <ray/shape/Triangle3.h>

namespace ray
{
    template <typename T>
    struct SceneObject;

    template <typename BvShapeT>
    struct BoundingVolume;

    template <typename BvShapeT, typename ShapeT>
    [[nodiscard]] decltype(auto) boundingVolume(const ShapeT& shape)
    {
        return BoundingVolume<BvShapeT>::get(shape);
    }

    template <typename BvShapeT, typename ShapeT>
    [[nodiscard]] decltype(auto) boundingVolume(const SceneObject<ShapeT>& obj)
    {
        return BoundingVolume<BvShapeT>::get(obj.shape());
    }
    template <typename BvShapeT>
    [[nodiscard]] decltype(auto) boundingVolume(const SceneObject<CsgShape>& obj)
    {
        return BoundingVolume<BvShapeT>::get(obj);
    }

    template <typename BvShapeT, bool IsBoundedV>
    [[nodiscard]] decltype(auto) boundingVolume(const SceneObject<AnyShape<IsBoundedV>>& obj)
    {
        return BoundingVolume<BvShapeT>::get(obj);
    }

    template <>
    struct BoundingVolume<Box3>
    {
        template <typename ShapeT>
        [[nodiscard]] static decltype(auto) get(const ShapeT& shape)
        {
            return shape.aabb();
        }

        [[nodiscard]] static Box3 get(const Box3& b)
        {
            return b;
        }

        [[nodiscard]] static Box3 get(const Capsule& cap)
        {
            const Vec3f halfExtent = Vec3f::broadcast(cap.radius);
            const Point3f end = cap.begin + cap.axis * cap.length;
            Box3 bb(cap.begin - halfExtent, cap.begin + halfExtent);
            bb.extend(Box3(end - halfExtent, end + halfExtent));
            return bb;
        }

        [[nodiscard]] static Box3 get(const Cylinder& cyl)
        {
            Disc3 dBegin(cyl.begin, Normal3f(cyl.axis), cyl.radius);
            Disc3 dEnd(cyl.begin + cyl.axis * cyl.length, Normal3f(cyl.axis), cyl.radius);
            Box3 bb = get(dBegin);
            bb.extend(get(dEnd));
            return bb;
        }

        [[nodiscard]] static Box3 get(const Disc3& d)
        {
            Vec3f halfExtent = Vec3f::broadcast(d.radius);
            halfExtent *= sqrt(Vec3f::broadcast(1.0f) - d.normal * d.normal);
            return Box3(d.origin - halfExtent, d.origin + halfExtent);
        }

        [[nodiscard]] static Box3 get(const OrientedBox3& obb)
        {
            Box3 bb(obb.origin, obb.origin);
            for (const auto& p : obb.vertices())
            {
                bb.extend(p);
            }
            return bb;
        }

        [[nodiscard]] static Box3 get(const Sphere& s)
        {
            const Vec3f halfExtent = Vec3f::broadcast(s.radius());
            return Box3(s.center() - halfExtent, s.center() + halfExtent);
        }

        [[nodiscard]] static Box3 get(const HalfSphere& s)
        {
            // applies similar (but inverted) method as for disc3
            const Vec3f halfExtent = Vec3f::broadcast(s.radius());
            Box3 bb(s.center() - halfExtent, s.center() + halfExtent);
            const Vec3f diff = (Vec3f::broadcast(1.0f) - sqrt(Vec3f::broadcast(1.0f) - s.normal()*s.normal())) * (-s.radius());
            // diff.x < 0.0f indicates that we have to move bb.max by diff.x
            // diff.x > 0.0f indicates that we have to move bb.min by diff.x
            const Vec3f dmin = Vec3f::blend(Vec3f::broadcast(0.0f), diff, diff < 0);
            const Vec3f dmax = Vec3f::blend(Vec3f::broadcast(0.0f), diff, diff > 0);
            bb.min += dmin;
            bb.max += dmax;
            return bb;
        }

        [[nodiscard]] static Box3 get(const Triangle3& tri)
        {
            const Point3f v1 = tri.v0() + tri.e01();
            const Point3f v2 = tri.v0() + tri.e02();
            const Point3f bmin = min(min(tri.v0(), v1), v2);
            const Point3f bmax = max(max(tri.v0(), v1), v2);
            return Box3(bmin, bmax);
        }

        template <typename TransformT, typename ShapeT>
        [[nodiscard]] static Box3 get(const TransformedShape3<TransformT, ShapeT>& sh)
        {
            auto vs = get(sh.shape).vertices();
            const auto localToWorld = sh.worldToLocal.inverse();
            for (auto& p : vs)
            {
                p = localToWorld * p;
            }

            Point3f a = vs[0];
            Point3f b = vs[0];
            for (int i = 1; i < vs.size(); ++i)
            {
                a = min(a, vs[i]);
                b = max(b, vs[i]);
            }

            return Box3(a, b);
        }

        template <typename ClippingShapeT>
        [[nodiscard]] static Box3 get(const ClippedSdf<ClippingShapeT>& sh)
        {
            return get(sh.clippingShape());
        }
    };
}
