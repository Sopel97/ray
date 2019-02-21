#pragma once

#if defined(RAY_GATHER_PERF_STATS)
#include <ray/perf/PerformanceStats.h>
#endif

#include "Interval.h"
#include "Ray.h"
#include "RaycastHit.h"
#include "Vec3.h"

#include <ray/shape/Box3.h>
#include <ray/shape/ClosedTriangleMesh.h>
#include <ray/shape/Disc3.h>
#include <ray/shape/Triangle3.h>
#include <ray/shape/Plane.h>
#include <ray/shape/Sphere.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <iostream>

#include <xmmintrin.h>
#include <smmintrin.h>

namespace ray
{
    inline bool contains(const Box3& box, const Point3f& point)
    {
        return (((point <= box.max) & (box.min <= point)).packed() & 0b0111) == 0b0111;

        /*
        __m128 mask = _mm_and_ps(
            _mm_cmple_ps(point.xmm, box.max.xmm),
            _mm_cmple_ps(box.min.xmm, point.xmm)
        );
        // has to be true for first 3 components. We don't care about the forth one
        return (_mm_movemask_ps(mask) & 0b0111) == 0b0111;
        */
    }

    // tNearest is the nearest object (not BV) so we don't update it here
    inline bool raycastBv(const Ray& ray, const Box3& box, float tNearest, RaycastBvHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addBvRaycast<Box3>();
#endif

        if (contains(box, ray.origin()))
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gPerfStats.addBvRaycastHit<Box3>();
#endif
            hit.dist = 0.0f;
            return true;
        }
        /*
        const Point3f origin = ray.origin();
        const Vec3f invDir = ray.invDirection();
        const auto sign = ray.signs();
        Point3f min = box.min;
        Point3f max = box.max;
        if (sign[0]) std::swap(min.x, max.x);
        if (sign[1]) std::swap(min.y, max.y);
        if (sign[2]) std::swap(min.z, max.z);

        const Vec3f t0 = (min - origin) * invDir;
        const Vec3f t1 = (max - origin) * invDir;

        float tmin = std::max(t0.x, t0.y);
        float tmax = std::min(t1.x, t1.y);

        if (tmin > tmax) return std::nullopt;

        if (t0.z > tmin)
            tmin = t0.z;
        if (t1.z < tmax)
            tmax = t1.z;

        if (tmin < 0.0f || tmin > tmax) return std::nullopt;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addBvRaycastHit<Box3>();
#endif

        return RaycastBvHit{ tmin };
        //*/
        ///*
        const Vec3f invDir = ray.invDirection();
        const Vec3f t0 = (box.min - ray.origin()) * invDir;
        const Vec3f t1 = (box.max - ray.origin()) * invDir;
        const float tmin = min(t0, t1).max();
        const float tmax = max(t0, t1).min();

        if (tmin <= tmax && tmin < tNearest)
        {
            hit.dist = tmin;
            return true;
        }

        return false;
        //*/
    }

    inline bool raycast(const Ray& ray, const Sphere& sphere, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycast<Sphere>();
#endif
        /*
            O -- ray origin
            D -- ray direction
            C -- sphere origin
            R -- sphere radius

            r: O + tD - C
            o: R^2 = 0

            P = (O - C) -- ray origin with sphere origin in (0, 0, 0)

            (P + tD)^2 - R^2 = 0
            t^2 D^2 + tPD + P^2 - R^2 = 0

            a = D^2 = 1
            b = 2PD
            c = P^2 - R^2
            d = b^2 - 4ac

            t1,2 = (-b +- sqrt(d)) / 2a
        */

        /*
        const Point3f O = ray.origin();
        const Normal3f D = ray.direction();
        const Point3f C = sphere.center();
        const float R = sphere.radius();

        const Vec3f P = O - C;

        const float b = 2.0f * dot(P, D);
        const float c = dot(P, P) - R * R;

        const float d = b * b - 4.0f * c;

        if (d < 0.0f) return std::nullopt;

        const float sqrt_d = std::sqrt(d);
        float t1 = -0.5f * (b + sqrt_d);
        float t2 = -0.5f * (b - sqrt_d);
        */


        /*
            O -- ray origin
            D -- ray direction
            C -- sphere origin
            R -- sphere radius

            r: O + tD - C
            o: R^2 = 0

            L = (C - O) -- |OC|
            t_ca = LD -- ||L||cos(a), where a is the angle between ray direction and the |OC|
            t_ca < 0 means that the angle is >180 degrees, so the sphere is behind

            Let P be the point on the ray, closest to sphere's center.
            Then we have a triangle
            |OP| = t_ca
            |OC| = ||L||
            |PC| = d
            With a right angle by P.
            L^2 = t_ca^2 + d^2
            d^2 = L^2 - t_ca^2

            But there's also a right triangle d, R, t_hc (inside the sphere)
            d^2 + t_hc^2 = R^2
            t_hc = sqrt(R^2 - d^2)

            Then
            t = t_ca +- t_hc
        */

        const Point3f O = ray.origin();
        const Normal3f D = ray.direction();
        const Point3f C = sphere.center();
        const float R = sphere.radius();

        const Vec3f L = C - O;
        const float t_ca = dot(L, D);
        //if (t_ca < 0.0f) return false; // we need to handle cases where ray origin is past the sphere center

        const float d2 = dot(L, L) - t_ca * t_ca;
        if (d2 > R * R) return false;
        const float r = R * R - d2;
        const float t_hc = std::sqrt(r);

        float t = t_ca - t_hc;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycastHit<Sphere>();
#endif
        // select smallest positive
        // we know that at least one is positive
        // and that t2 is greater
        bool isInside = t < 0.0f;
        if (isInside) t = t_ca + t_hc;
        if (t < 0.0f || t >= hit.dist) return false;
        const Point3f hitPoint = O + t * D;
        Normal3f normal = ((hitPoint - C) / R).assumeNormalized();
        if (isInside) normal = -normal;

        hit.dist = t;
        hit.point = hitPoint;
        hit.normal = normal;
        hit.shapeInPackNo = 0;
        hit.materialNo = 0;
        hit.isInside = isInside;
        return true;
    }

    inline bool raycast(const Ray& ray, const Plane& plane, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycast<Plane>();
#endif
        const float nd = dot(ray.direction(), plane.normal);
        const float pn = dot(Vec3f(ray.origin()), plane.normal);
        const float t = (plane.distance - pn) / nd;

        // t must be positive
        if (t >= 0.0f && t < hit.dist)
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gPerfStats.addObjectRaycastHit<Plane>();
#endif
            const Point3f point = ray.origin() + ray.direction() * t;

            hit.dist = t;
            hit.point = point;
            hit.normal = (nd < 0.0f) ? plane.normal : -plane.normal;
            hit.shapeInPackNo = 0;
            hit.materialNo = 0;
            hit.isInside = false;

            return true;
        }

        return false;
    }

    inline bool raycast(const Ray& ray, const Disc3& disc, RaycastHit& hit)
    {
        const float nd = dot(ray.direction(), disc.normal);
        const float pn = dot(Vec3f(ray.origin()), disc.normal);
        const float t = (disc.distance - pn) / nd;

        // t must be positive
        if (t >= 0.0f && t < hit.dist)
        {
            const Point3f point = ray.origin() + ray.direction() * t;
            if (distanceSqr(disc.origin, point) > disc.radius * disc.radius)
                return false;

            hit.dist = t;
            hit.point = point;
            hit.normal = (nd < 0.0f) ? disc.normal : -disc.normal;
            hit.shapeInPackNo = 0;
            hit.materialNo = 0;
            hit.isInside = false;

            return true;
        }

        return false;
    }

    inline bool raycast(const Ray& ray, const Box3& box, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycast<Box3>();
#endif
        const Point3f origin = ray.origin();
        const Vec3f invDir = ray.invDirection();

        const Vec3f t0 = (box.min - ray.origin()) * invDir;
        const Vec3f t1 = (box.max - ray.origin()) * invDir;

        float tmax = max(t0, t1).min();
        if (tmax < 0.0f) return false;
        float tmin = min(t0, t1).max();
        if (tmin > tmax) return false;

        bool isInside = tmin < 0.0f;
        if (isInside)
        {
            tmin = tmax;
        }
        if (tmin >= hit.dist) return false;

        // TODO: make this fast but also reliable. Currently breaks on the edges (sets multiple components).
        Normal3f normal(AssumeNormalized{}, Vec3f::blend(0.0f, -1.0f, (t0 == tmin) | (t1 == tmin)));
        normal.negate(ray.signs());
        
        const Point3f point = ray.origin() + ray.direction() * tmin;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycastHit<Box3>();
#endif
        hit.dist = tmin;
        hit.point = point;
        hit.normal = normal;
        hit.shapeInPackNo = 0;
        hit.materialNo = 0;
        hit.isInside = isInside;

        return true;
    }

    /*
    // this one has faster exit when the triangle is behind or farther than prev hit
    // but otherwise it is slower and requires a precomputed plane
    inline bool raycast(const Ray& ray, const Triangle3& tri, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycast<Triangle3>();
#endif
        
        const Point3f orig = ray.origin();
        const Vec3f invDir = ray.invDirection();
        const Normal3f nor = tri.plane().normal;
        const float dist = tri.plane().distance;

        const float nd = dot(invDir, nor);
        const float pn = dot(Vec3f(orig), nor);
        const float t = (dist - pn) * nd;

        if (t < 0.0f) return false;
        if (t >= hit.dist) return false;

        const Normal3f dir = ray.direction();
        const Point3f point = orig + dir * t;

        const Vec3f v0v1 = tri.e01();
        const Vec3f v0v2 = tri.e02();
        const Vec3f pvec = cross(dir, v0v2);
        const float det = dot(v0v1, pvec);

        // ray and triangle are parallel if det is close to 0
        if (std::abs(det) < 0.00001f) return false;

        const float invDet = 1.0f / det;

        const Vec3f tvec = orig - tri.v0();
        const float u = dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return false;

        const Vec3f qvec = cross(tvec, v0v1);
        const float v = dot(dir, qvec) * invDet;
        if (v < 0 || (v + u) > 1) return false;

        const float w = 1.0f - (v + u);

#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycastHit<Triangle3>();
#endif

        hit.dist = t;
        hit.point = point;
        hit.normal = (tri.normal(0) * u + tri.normal(1) * v + tri.normal(2) * w).assumeNormalized();
        if (nd > 0.0f) hit.normal = -hit.normal;
        hit.shapeInPackNo = 0;
        hit.materialNo = 0;
        hit.isInside = false;

        return true;
    }
    */

    inline bool raycast(const Ray& ray, const Triangle3& tri, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycast<Triangle3>();
#endif

        const Vec3f pvec = cross(ray.direction(), tri.e02());
        const float det = dot(tri.e01(), pvec);

        // ray and triangle are parallel if det is close to 0
        if (std::abs(det) < 0.00001f) return false;

        const float invDet = 1.0f / det;

        const Vec3f tvec = ray.origin() - tri.v0();
        const float v = dot(tvec, pvec) * invDet;
        if (v < 0.0f || v > 1.0f) return false;

        const Vec3f qvec = cross(tvec, tri.e01());
        const float w = dot(ray.direction(), qvec) * invDet;
        const float wv = v + w;
        if (w < 0.0f || wv > 1.0f) return false;

        const float u = 1.0f - wv;

        const float t = dot(tri.e02(), qvec) * invDet;

        if (t < 0.0f) return false;
        if (t >= hit.dist) return false;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycastHit<Triangle3>();
#endif

        hit.dist = t;
        hit.point = ray.origin() + ray.direction() * t;
        hit.normal = (tri.normal(0) * u + tri.normal(1) * v + tri.normal(2) * w).normalized();
        if (det < 0.0f) hit.normal = -hit.normal;
        hit.shapeInPackNo = 0;
        hit.materialNo = 0;
        hit.isInside = false;

        return true;
    }

    inline bool raycast(const Ray& ray, const ClosedTriangleMeshFace& tri, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycast<Triangle3>();
#endif

        const Point3f v0 = tri.vertex(0).point;
        const Vec3f e01 = tri.vertex(1).point - v0;
        const Vec3f e02 = tri.vertex(2).point - v0;
        const Vec3f pvec = cross(ray.direction(), e02);
        const float det = dot(e01, pvec);

        // ray and triangle are parallel if det is close to 0
        if (std::abs(det) < 0.00001f) return false;

        const float invDet = 1.0f / det;

        const Vec3f tvec = ray.origin() - v0;
        const float v = dot(tvec, pvec) * invDet;
        if (v < 0.0f || v > 1.0f) return false;

        const Vec3f qvec = cross(tvec, e01);
        const float w = dot(ray.direction(), qvec) * invDet;
        const float wv = v + w;
        if (w < 0.0f || wv > 1.0f) return false;

        const float u = 1.0f - wv;

        const float t = dot(e02, qvec) * invDet;

        if (t < 0.0f) return false;
        if (t >= hit.dist) return false;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addObjectRaycastHit<Triangle3>();
#endif
        const bool isInside = det < 0.0f;
        hit.dist = t;
        hit.point = ray.origin() + ray.direction() * t;
        hit.normal = (tri.vertex(0).normal * u + tri.vertex(1).normal * v + tri.vertex(2).normal * w).normalized();
        if(isInside) hit.normal = -hit.normal;
        hit.shapeInPackNo = 0;
        hit.materialNo = 0;
        hit.isInside = isInside;

        return true;
    }

    // Interval raycasts

    // TODO: somehow make the data be assigned elsewhere
    template <typename DataT>
    inline bool raycastIntervals(const Ray& ray, const Sphere& sphere, IntervalSet<DataT>& hitIntervals, const DataT& data)
    {
        const Point3f O = ray.origin();
        const Normal3f D = ray.direction();
        const Point3f C = sphere.center();
        const float R = sphere.radius();

        const Vec3f L = C - O;
        const float t_ca = dot(L, D);
        // if (t_ca < 0.0f) return false;

        const float d2 = dot(L, L) - t_ca * t_ca;
        const float r = R * R - d2;
        if (r < 0.0f) return false;
        const float t_hc = std::sqrt(r);

        hitIntervals.pushBack(Interval<DataT>{t_ca - t_hc, t_ca + t_hc, data, data});
        return true;
    }

    template <typename DataT>
    inline bool raycastIntervals(const Ray& ray, const Box3& box, IntervalSet<DataT>& hitIntervals, const DataT& data)
    {
        const Vec3f invDir = ray.invDirection();
        const Vec3f t0 = (box.min - ray.origin()) * invDir;
        const Vec3f t1 = (box.max - ray.origin()) * invDir;
        const float tmin = min(t0, t1).max();
        const float tmax = max(t0, t1).min();

        if (tmin < tmax)
        {
            hitIntervals.pushBack(Interval<DataT>{tmin, tmax, data, data});
            return true;
        }

        return false;
    }
}
