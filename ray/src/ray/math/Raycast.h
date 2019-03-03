#pragma once

#if defined(RAY_GATHER_PERF_STATS)
#include <ray/perf/PerformanceStats.h>
#endif

#include "BoundingVolume.h"
#include "Interval.h"
#include "Ray.h"
#include "RaycastHit.h"
#include "Vec3.h"

#include <ray/material/Material.h>

#include <ray/shape/Box3.h>
#include <ray/shape/ClosedTriangleMesh.h>
#include <ray/shape/Capsule.h>
#include <ray/shape/Cylinder.h>
#include <ray/shape/Disc3.h>
#include <ray/shape/OrientedBox3.h>
#include <ray/shape/Triangle3.h>
#include <ray/shape/Plane.h>
#include <ray/shape/HalfSphere.h>
#include <ray/shape/Sphere.h>
#include <ray/shape/TransformedShape3.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <iostream>

#include <xmmintrin.h>
#include <smmintrin.h>

namespace ray
{
    [[nodiscard]] inline bool contains(const Box3& box, const Point3f& point)
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
    [[nodiscard]] inline bool raycastBv(const Ray& ray, const Box3& box, float tNearest, RaycastBvHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addBvRaycast<Box3>();
#endif

        if (contains(box, ray.origin()))
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addBvRaycastHit<Box3>();
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
        perf::gThreadLocalPerfStats.addBvRaycastHit<Box3>();
#endif

        return RaycastBvHit{ tmin };
        //*/
        ///*
        const Vec3f invDir = ray.invDirection();
        const Vec3f t0 = (box.min - ray.origin()) * invDir;
        const Vec3f t1 = (box.max - ray.origin()) * invDir;
        const float tmax = max(t0, t1).min();
        if (tmax < 0.0f) return false;
        const float tmin = min(t0, t1).max();

        if (tmin <= tmax && tmin < tNearest)
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addBvRaycastHit<Box3>();
#endif
            hit.dist = tmin;
            return true;
        }

        return false;
        //*/
    }

    [[nodiscard]] inline bool raycast(const Ray& ray, const Sphere& sphere, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<Sphere>();
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

        const float b = 2.0f * dot3(P, D);
        const float c = dot3(P, P) - R * R;

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

        // select smallest positive
        // we know that at least one is positive
        // and that t2 is greater
        bool isInside = t < 0.0f;
        if (isInside) t = t_ca + t_hc;
        if (t < 0.0f || t >= hit.dist) return false;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycastHit<Sphere>();
#endif

        const Point3f hitPoint = O + t * D;
        Normal3f normal = ((hitPoint - C) / R).assumeNormalized();
        if (isInside) normal = -normal;

        hit.dist = t;
        hit.point = hitPoint;
        hit.normal = normal;
        hit.shapeInPackNo = 0;
        hit.materialIndex = MaterialIndex(0, 0);
        hit.isInside = isInside;
        return true;
    }

    [[nodiscard]] inline bool raycast(const Ray& ray, const OrientedBox3& obb, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<OrientedBox3>();
#endif

        const Vec3f p = obb.origin - ray.origin();

        const Vec3f f = 1.0f / (obb.worldToLocalRot * ray.direction());

        const Vec3f e = obb.worldToLocalRot * p;

        const Vec3f t0 = (e - obb.halfSize) * f;
        const Vec3f t1 = (e + obb.halfSize) * f;
        const float tmax = max(t0, t1).min();
        if (tmax < 0.0f) return false;
        float tmin = min(t0, t1).max();
        if (tmin > tmax) return false;

        bool isInside = tmin < 0.0f;
        if (isInside)
        {
            tmin = tmax;
        }
        if (tmin >= hit.dist) return false;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycastHit<OrientedBox3>();
#endif

        Normal3f normal(AssumeNormalized{}, Vec3f::blend(0.0f, -1.0f, (t0 == tmin) | (t1 == tmin)));
        normal.negate(f < 0.0f);

        hit.dist = tmin;
        hit.point = ray.origin() + ray.direction() * tmin;
        hit.normal = obb.worldToLocalRot.inverse() * normal;
        hit.shapeInPackNo = 0;
        hit.materialIndex = MaterialIndex(0, 0);
        hit.isInside = isInside;

        return true;
    }

    // half sphere is only surface
    [[nodiscard]] inline bool raycast(const Ray& ray, const HalfSphere& sphere, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<HalfSphere>();
#endif

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

        float tmin = t_ca - t_hc;
        float tmax = t_ca + t_hc;

        Point3f hitPointMin = O + tmin * D;
        Point3f hitPointMax = O + tmax * D;
        if (tmin > 0.0f && tmin < hit.dist && dot(hitPointMin - C, sphere.normal()) > 0.0f)
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addObjectRaycastHit<HalfSphere>();
#endif

            Normal3f normal = ((hitPointMin - C) / R).assumeNormalized();

            hit.dist = tmin;
            hit.point = hitPointMin;
            hit.normal = normal;
            hit.shapeInPackNo = 0;
            hit.materialIndex = MaterialIndex(0);
            hit.isInside = false;
            return true;
        }
        else if (tmax > 0.0f && tmax < hit.dist && dot(hitPointMax - C, sphere.normal()) > 0.0f)
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addObjectRaycastHit<HalfSphere>();
#endif

            Normal3f normal = ((hitPointMax - C) / R).assumeNormalized();

            hit.dist = tmax;
            hit.point = hitPointMax;
            hit.normal = -normal;
            hit.shapeInPackNo = 0;
            hit.materialIndex = MaterialIndex(0);
            hit.isInside = true;
            return true;
        }
        else
        {
            return false;
        }

    }

    [[nodiscard]] inline bool raycast(const Ray& ray, const Plane& plane, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<Plane>();
#endif
        const float nd = dot(ray.direction(), plane.normal);
        const float pn = dot(Vec3f(ray.origin()), plane.normal);
        const float t = (plane.distance - pn) / nd;

        // t must be positive
        if (t >= 0.0f && t < hit.dist)
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addObjectRaycastHit<Plane>();
#endif
            const Point3f point = ray.origin() + ray.direction() * t;

            hit.dist = t;
            hit.point = point;
            hit.normal = (nd < 0.0f) ? plane.normal : -plane.normal;
            hit.shapeInPackNo = 0;
            hit.materialIndex = MaterialIndex(0);
            hit.isInside = false;

            return true;
        }

        return false;
    }

    [[nodiscard]] inline bool raycast(const Ray& ray, const Disc3& disc, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<Disc3>();
#endif

        const float nd = dot(ray.direction(), disc.normal);
        const float pn = dot(Vec3f(ray.origin()), disc.normal);
        const float t = (disc.distance - pn) / nd;

        // t must be positive
        if (t >= 0.0f && t < hit.dist)
        {
            const Point3f point = ray.origin() + ray.direction() * t;
            if (distanceSqr(disc.origin, point) > disc.radius * disc.radius)
                return false;

#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addObjectRaycastHit<Disc3>();
#endif

            hit.dist = t;
            hit.point = point;
            hit.normal = (nd < 0.0f) ? disc.normal : -disc.normal;
            hit.shapeInPackNo = 0;
            hit.materialIndex = MaterialIndex(0);
            hit.isInside = false;

            return true;
        }

        return false;
    }

    [[nodiscard]] inline bool raycast(const Ray& ray, const Box3& box, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<Box3>();
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
        perf::gThreadLocalPerfStats.addObjectRaycastHit<Box3>();
#endif
        hit.dist = tmin;
        hit.point = point;
        hit.normal = normal;
        hit.shapeInPackNo = 0;
        hit.materialIndex = MaterialIndex(0, 0);
        hit.isInside = isInside;

        return true;
    }

    [[nodiscard]] inline bool raycast(const Ray& ray, const Cylinder& cyl, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<Cylinder>();
#endif

        // https://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf
        const Point3f& P = ray.origin();
        const Normal3f& d = ray.direction();
        const Point3f& A = cyl.begin;
        const Normal3f& v = cyl.axis;
        const float r = cyl.radius;

        // infinite cylinder
        const Vec3f dP = P - A;

        const float dv = dot(d, v);
        if (!(std::abs(dv) < 0.00001f))
        {
            // not parallel
            const float dPv = dot(dP, v);
            const Vec3f dvv = Vec3f(d) - dv * v;
            const Vec3f dPvv = Vec3f(dP) - dPv * v;

            const float a = dot(dvv, dvv);
            const float b = 2.0f * dot(dvv, dPvv);
            const float c = dot(dPvv, dPvv) - (cyl.radius * cyl.radius);

            const float delta = b * b - 4.0f*a*c;
            if (delta < 0.0f)
                return false;

            const float sqrtDelta = std::sqrt(delta);

            const float t0 = (-b - sqrtDelta) / (2.0f * a);
            const float t1 = (-b + sqrtDelta) / (2.0f * a);

            const bool isInside = t0 < 0.0f;
            if (isInside && t1 < 0.0f)
            {
                return false;
            }
            const float t = isInside ? t1 : t0;
            if (t >= hit.dist)
                return false;

            const Point3f point = P + d * t;
            const float pl = dot(point - A, v);

            if (pl > 0.0f && pl < cyl.length)
            {
#if defined(RAY_GATHER_PERF_STATS)
                perf::gThreadLocalPerfStats.addObjectRaycastHit<Cylinder>();
#endif

                // we hit the shaft
                const Point3f pointL = A + pl * v;
                const Normal3f normal = ((point - pointL) / r).assumeNormalized();

                hit.dist = t;
                hit.point = point;
                hit.normal = isInside ? -normal : normal;
                hit.shapeInPackNo = 0;
                hit.materialIndex = MaterialIndex(0, 0);
                hit.isInside = isInside;

                return true;
            }
        }

        // parallel or we didn't hit the shaft in the proper range
        // we may hit the caps
        const Disc3 d0(A, -v, cyl.radius);
        const Disc3 d1(A + v * cyl.length, v, cyl.radius);
        if (raycast(ray, d0, hit) || raycast(ray, d1, hit))
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addObjectRaycastHit<Cylinder>();
#endif

            hit.materialIndex = MaterialIndex(1, 0);
            return true;
        }

        return false;
    }

    [[nodiscard]] inline bool raycast(const Ray& ray, const Capsule& cyl, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<Capsule>();
#endif

        // https://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf
        const Point3f& P = ray.origin();
        const Normal3f& d = ray.direction();
        const Point3f& A = cyl.begin;
        const Normal3f& v = cyl.axis;
        const float r = cyl.radius;

        // infinite cylinder
        const Vec3f dP = P - A;

        const float dv = dot(d, v);
        if (!(std::abs(dv) < 0.00001f))
        {
            // not parallel
            const float dPv = dot(dP, v);
            const Vec3f dvv = Vec3f(d) - dv * v;
            const Vec3f dPvv = Vec3f(dP) - dPv * v;

            const float a = dot(dvv, dvv);
            const float b = 2.0f * dot(dvv, dPvv);
            const float c = dot(dPvv, dPvv) - (cyl.radius * cyl.radius);

            const float delta = b * b - 4.0f*a*c;
            if (delta < 0.0f)
                return false;

            const float sqrtDelta = std::sqrt(delta);

            const float t0 = (-b - sqrtDelta) / (2.0f * a);
            const float t1 = (-b + sqrtDelta) / (2.0f * a);

            const bool isInside = t0 < 0.0f;
            if (isInside && t1 < 0.0f)
            {
                return false;
            }
            const float t = isInside ? t1 : t0;
            if (t >= hit.dist)
                return false;

            const Point3f point = P + d * t;
            const float pl = dot(point - A, v);

            if (pl > 0.0f && pl < cyl.length)
            {
#if defined(RAY_GATHER_PERF_STATS)
                perf::gThreadLocalPerfStats.addObjectRaycastHit<Capsule>();
#endif

                // we hit the shaft
                const Point3f pointL = A + pl * v;
                const Normal3f normal = ((point - pointL) / r).assumeNormalized();

                hit.dist = t;
                hit.point = point;
                hit.normal = isInside ? -normal : normal;
                hit.shapeInPackNo = 0;
                hit.materialIndex = MaterialIndex(0, 0);
                hit.isInside = isInside;

                return true;
            }
        }

        // parallel or we didn't hit the shaft in the proper range
        // we may hit the caps
        const HalfSphere d0(A, -v, cyl.radius);
        const HalfSphere d1(A + v * cyl.length, v, cyl.radius);
        if (raycast(ray, d0, hit) || raycast(ray, d1, hit))
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addObjectRaycastHit<Capsule>();
#endif

            hit.materialIndex = MaterialIndex(1, 0);
            return true;
        }

        return false;
    }

    /*
    // this one has faster exit when the triangle is behind or farther than prev hit
    // but otherwise it is slower and requires a precomputed plane
    inline bool raycast(const Ray& ray, const Triangle3& tri, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<Triangle3>();
#endif
        
        const Point3f orig = ray.origin();
        const Vec3f invDir = ray.invDirection();
        const Normal3f nor = tri.plane().normal;
        const float dist = tri.plane().distance;

        const float nd = dot3(invDir, nor);
        const float pn = dot3(Vec3f(orig), nor);
        const float t = (dist - pn) * nd;

        if (t < 0.0f) return false;
        if (t >= hit.dist) return false;

        const Normal3f dir = ray.direction();
        const Point3f point = orig + dir * t;

        const Vec3f v0v1 = tri.e01();
        const Vec3f v0v2 = tri.e02();
        const Vec3f pvec = cross3(dir, v0v2);
        const float det = dot3(v0v1, pvec);

        // ray and triangle are parallel if det is close to 0
        if (std::abs(det) < 0.00001f) return false;

        const float invDet = 1.0f / det;

        const Vec3f tvec = orig - tri.v0();
        const float u = dot3(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return false;

        const Vec3f qvec = cross3(tvec, v0v1);
        const float v = dot3(dir, qvec) * invDet;
        if (v < 0 || (v + u) > 1) return false;

        const float w = 1.0f - (v + u);

#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycastHit<Triangle3>();
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

    [[nodiscard]] inline bool raycast(const Ray& ray, const Triangle3& tri, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<Triangle3>();
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
        perf::gThreadLocalPerfStats.addObjectRaycastHit<Triangle3>();
#endif

        hit.dist = t;
        hit.point = ray.origin() + ray.direction() * t;
        hit.normal = (tri.normal(0) * u + tri.normal(1) * v + tri.normal(2) * w).normalized();
        if (det < 0.0f) hit.normal = -hit.normal;
        hit.shapeInPackNo = 0;
        hit.materialIndex = MaterialIndex(0);
        hit.isInside = false;

        return true;
    }

    [[nodiscard]] inline bool raycast(const Ray& ray, const ClosedTriangleMeshFace& tri, RaycastHit& hit)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addObjectRaycast<ClosedTriangleMeshFace>();
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
        perf::gThreadLocalPerfStats.addObjectRaycastHit<ClosedTriangleMeshFace>();
#endif
        const bool isInside = det < 0.0f;
        hit.dist = t;
        hit.point = ray.origin() + ray.direction() * t;
        hit.normal = (tri.vertex(0).normal * u + tri.vertex(1).normal * v + tri.vertex(2).normal * w).normalized();
        if(isInside) hit.normal = -hit.normal;
        hit.shapeInPackNo = 0;
        hit.materialIndex = MaterialIndex(0, 0);
        hit.isInside = isInside;

        return true;
    }

    // Interval raycasts

    // TODO: somehow make the data be assigned elsewhere
    template <typename DataT>
    [[nodiscard]] inline bool raycastIntervals(const Ray& ray, const Sphere& sphere, IntervalSet<DataT>& hitIntervals, const DataT& data)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addIntervalRaycast<Sphere>();
#endif

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

        if (t_ca + t_hc < 0.0f) return false;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addIntervalRaycastHit<Sphere>();
#endif

        hitIntervals.pushBack(Interval<DataT>{t_ca - t_hc, t_ca + t_hc, data, data});
        return true;
    }

    template <typename DataT>
    [[nodiscard]] inline bool raycastIntervals(const Ray& ray, const Box3& box, IntervalSet<DataT>& hitIntervals, const DataT& data)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addIntervalRaycast<Box3>();
#endif

        const Vec3f invDir = ray.invDirection();
        const Vec3f t0 = (box.min - ray.origin()) * invDir;
        const Vec3f t1 = (box.max - ray.origin()) * invDir;
        const float tmax = max(t0, t1).min();
        if (tmax < 0.0f) return false;
        const float tmin = min(t0, t1).max();

        if (tmin < tmax)
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addIntervalRaycastHit<Box3>();
#endif

            hitIntervals.pushBack(Interval<DataT>{tmin, tmax, data, data});
            return true;
        }

        return false;
    }

    template <typename DataT>
    [[nodiscard]] inline bool raycastIntervals(const Ray& ray, const OrientedBox3& obb, IntervalSet<DataT>& hitIntervals, const DataT& data)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addIntervalRaycast<OrientedBox3>();
#endif

        const Vec3f p = obb.origin - ray.origin();

        const Vec3f f = 1.0f / (obb.worldToLocalRot * ray.direction());

        const Vec3f e = obb.worldToLocalRot * p;

        const Vec3f t0 = (e - obb.halfSize) * f;
        const Vec3f t1 = (e + obb.halfSize) * f;
        const float tmax = max(t0, t1).min();
        if (tmax < 0.0f) return false;
        float tmin = min(t0, t1).max();
        if (tmin > tmax) return false;

        if (tmin < tmax)
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addIntervalRaycastHit<OrientedBox3>();
#endif

            hitIntervals.pushBack(Interval<DataT>{tmin, tmax, data, data});
            return true;
        }

        return false;
    }

    [[nodiscard]] inline bool raycastDist(const Ray& ray, const Disc3& disc, float& t)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addDistRaycast<Disc3>();
#endif

        const float nd = dot(ray.direction(), disc.normal);
        const float pn = dot(Vec3f(ray.origin()), disc.normal);
        t = (disc.distance - pn) / nd;

        const Point3f point = ray.origin() + ray.direction() * t;
        if (distanceSqr(disc.origin, point) > disc.radius * disc.radius)
            return false;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addDistRaycastHit<Disc3>();
#endif

        return true;
    }

    // should return smallest distance (ie. smallest in absolute value)
    [[nodiscard]] inline bool raycastMinMax(const Ray& ray, const HalfSphere& sphere, float& t0, float& t1)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addDistRaycast<HalfSphere>();
#endif

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

        float tmin = t_ca - t_hc;
        float tmax = t_ca + t_hc;
        if (tmax < 0.0f) return false;

        Point3f hitPointMin = O + tmin * D;
        Point3f hitPointMax = O + tmax * D;
        const bool minHit = dot(hitPointMin - C, sphere.normal()) > 0.0f;
        const bool maxHit = dot(hitPointMax - C, sphere.normal()) > 0.0f;
        if (minHit && maxHit)
        {
            t0 = tmin;
            t1 = tmax;
        }
        else if (minHit)
        {
            t0 = t1 = tmin;
        }
        else if (maxHit)
        {
            t0 = t1 = tmax;
        }
        else
        {
            return false;
        }

#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addDistRaycastHit<HalfSphere>();
#endif

        return true;
    }

    template <typename DataT>
    [[nodiscard]] inline bool raycastIntervals(const Ray& ray, const Cylinder& cyl, IntervalSet<DataT>& hitIntervals, const DataT& data)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addIntervalRaycast<Cylinder>();
#endif

        // https://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf
        const Point3f& P = ray.origin();
        const Normal3f& d = ray.direction();
        const Point3f& A = cyl.begin;
        const Normal3f& v = cyl.axis;
        const float r = cyl.radius;

        // infinite cylinder
        const Vec3f dP = P - A;

        const float dv = dot(d, v);
        if (!(std::abs(dv) < 0.00001f))
        {
            // not parallel
            const float dPv = dot(dP, v);
            const Vec3f dvv = Vec3f(d) - dv * v;
            const Vec3f dPvv = Vec3f(dP) - dPv * v;

            const float a = dot(dvv, dvv);
            const float b = 2.0f * dot(dvv, dPvv);
            const float c = dot(dPvv, dPvv) - (cyl.radius * cyl.radius);

            const float delta = b * b - 4.0f*a*c;
            if (delta < 0.0f)
                return false;

            const float sqrtDelta = std::sqrt(delta);

            float t0 = (-b - sqrtDelta) / (2.0f * a);
            float t1 = (-b + sqrtDelta) / (2.0f * a);
            if (t1 < 0.0f) return false;

            const Point3f point0 = P + d * t0;
            const Point3f point1 = P + d * t1;
            const float p0v = dot(point0 - A, v);
            const float p1v = dot(point1 - A, v);

            if (p0v < 0.0f && p1v < 0.0f)
            {
                // we are before A
                // both hits on one side
                // nothing can be hit because the caps are flat
                return false;
            }
            else if (p0v > cyl.length && p1v > cyl.length)
            {
                // we are after B
                // both hits on one side
                // nothing can be hit because the caps are flat
                return false;
            }
            else
            {
                if (p1v < 0.0f)
                {
                    // we are before A for the far hit
                    const Disc3 d0(A, -v, cyl.radius);
                    float dist;
                    if (raycastDist(ray, d0, dist))
                    {
                        t1 = dist;
                        if (dist < 0.0f) return false;
                    }
                    else
                    {
                        return false;
                    }
                }
                else if (p1v > cyl.length)
                {
                    // we are after B for the far hit
                    const Disc3 d1(A + v * cyl.length, v, cyl.radius);
                    float dist;
                    if (raycastDist(ray, d1, dist))
                    {
                        t1 = dist;
                        if (dist < 0.0f) return false;
                    }
                    else
                    {
                        return false;
                    }
                }


                if (p0v < 0.0f)
                {
                    // we are before A for the near hit
                    const Disc3 d0(A, -v, cyl.radius);
                    float dist;
                    if (raycastDist(ray, d0, dist))
                    {
                        t0 = dist;
                    }
                    else
                    {
                        return false;
                    }
                }
                else if (p0v > cyl.length)
                {
                    // we are after B for the near hit
                    const Disc3 d1(A + v * cyl.length, v, cyl.radius);
                    float dist;
                    if (raycastDist(ray, d1, dist))
                    {
                        t0 = dist;
                    }
                    else
                    {
                        return false;
                    }
                }
            }

#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addIntervalRaycastHit<Cylinder>();
#endif

            hitIntervals.pushBack(Interval<DataT>{t0, t1, data, data});

            return true;
        }
        else
        {
            const Disc3 d0(A, -v, cyl.radius);
            const Disc3 d1(A + v * cyl.length, v, cyl.radius);
            float t0, t1;
            if (raycastDist(ray, d0, t0) && raycastDist(ray, d1, t1))
            {
                if (t1 < t0) std::swap(t0, t1);
                if (t1 < 0.0f) return false;

#if defined(RAY_GATHER_PERF_STATS)
                perf::gThreadLocalPerfStats.addIntervalRaycastHit<Cylinder>();
#endif

                hitIntervals.pushBack(Interval<DataT>{t0, t1, data, data});
                return true;
            }
        }

        return false;
    }

    template <typename DataT>
    [[nodiscard]] inline bool raycastIntervals(const Ray& ray, const Capsule& cyl, IntervalSet<DataT>& hitIntervals, const DataT& data)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gThreadLocalPerfStats.addIntervalRaycast<Capsule>();
#endif

        // https://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf
        const Point3f& P = ray.origin();
        const Normal3f& d = ray.direction();
        const Point3f& A = cyl.begin;
        const Normal3f& v = cyl.axis;
        const float r = cyl.radius;

        // infinite cylinder
        const Vec3f dP = P - A;

        const float dv = dot(d, v);
        if (!(std::abs(dv) < 0.00001f))
        {
            // not parallel
            const float dPv = dot(dP, v);
            const Vec3f dvv = Vec3f(d) - dv * v;
            const Vec3f dPvv = Vec3f(dP) - dPv * v;

            const float a = dot(dvv, dvv);
            const float b = 2.0f * dot(dvv, dPvv);
            const float c = dot(dPvv, dPvv) - (cyl.radius * cyl.radius);

            const float delta = b * b - 4.0f*a*c;
            if (delta < 0.0f)
                return false;

            const float sqrtDelta = std::sqrt(delta);

            float t0 = (-b - sqrtDelta) / (2.0f * a);
            float t1 = (-b + sqrtDelta) / (2.0f * a);
            if (t1 < 0.0f) return false;

            const Point3f point0 = P + d * t0;
            const Point3f point1 = P + d * t1;
            const float p0v = dot(point0 - A, v);
            const float p1v = dot(point1 - A, v);

            if (p0v < 0.0f && p1v < 0.0f)
            {
                // we are before A
                // both hits on one side
                const HalfSphere d0(A, -v, cyl.radius);
                float tmin, tmax;
                if (raycastMinMax(ray, d0, tmin, tmax))
                {
                    t0 = tmin;
                    t1 = tmax;
                    if (tmax < 0.0f) return false;

                    return true;
                }
                else
                {
                    return false;
                }
            }
            else if (p0v > cyl.length && p1v > cyl.length)
            {
                // we are after B
                // both hits on one side
                const HalfSphere d1(A + v * cyl.length, v, cyl.radius);
                float tmin, tmax;
                if (raycastMinMax(ray, d1, tmin, tmax))
                {
                    t0 = tmin;
                    t1 = tmax;
                    if (tmax < 0.0f) return false;

                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                if (p1v < 0.0f)
                {
                    // we are before A for the far hit
                    const HalfSphere d0(A, -v, cyl.radius);
                    float tmin, tmax;
                    if (raycastMinMax(ray, d0, tmin, tmax))
                    {
                        t1 = tmax;
                        if (tmax < 0.0f) return false;
                    }
                    else
                    {
                        return false;
                    }
                }
                else if (p1v > cyl.length)
                {
                    // we are after B for the far hit
                    const HalfSphere d1(A + v * cyl.length, v, cyl.radius);
                    float tmin, tmax;
                    if (raycastMinMax(ray, d1, tmin, tmax))
                    {
                        t1 = tmax;
                        if (tmax < 0.0f) return false;
                    }
                    else
                    {
                        return false;
                    }
                }


                if (p0v < 0.0f)
                {
                    // we are before A for the near hit
                    const HalfSphere d0(A, -v, cyl.radius);
                    float tmin, tmax;
                    if (raycastMinMax(ray, d0, tmin, tmax))
                    {
                        t0 = tmin;
                    }
                    else
                    {
                        return false;
                    }
                }
                else if (p0v > cyl.length)
                {
                    // we are after B for the near hit
                    const HalfSphere d1(A + v * cyl.length, v, cyl.radius);
                    float tmin, tmax;
                    if (raycastMinMax(ray, d1, tmin, tmax))
                    {
                        t0 = tmin;
                    }
                    else
                    {
                        return false;
                    }
                }
            }

#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addIntervalRaycastHit<Capsule>();
#endif

            hitIntervals.pushBack(Interval<DataT>{t0, t1, data, data});

            return true;
        }
        else
        {
            // we are parallel
            // we can only go through the caps
            const HalfSphere d0(A, -v, cyl.radius);
            const HalfSphere d1(A + v * cyl.length, v, cyl.radius);
            float t0min, t0max, t1min, t1max;
            if (raycastMinMax(ray, d0, t0min, t0max) && raycastMinMax(ray, d1, t1min, t1max))
            {
                // if we're here then each half sphere was hit exactly once. ie. t0min t0max are the same
                float t0 = t0min;
                float t1 = t1min;
                if (t1 < t0) std::swap(t0, t1);
                if (t1 < 0.0f) return false;

#if defined(RAY_GATHER_PERF_STATS)
                perf::gThreadLocalPerfStats.addIntervalRaycastHit<Capsule>();
#endif

                hitIntervals.pushBack(Interval<DataT>{t0, t1, data, data});
                return true;
            }
        }

        return false;
    }

    template <typename TransformT, typename ShapeT>
    [[nodiscard]] inline bool raycast(const Ray& ray, const TransformedShape3<TransformT, ShapeT>& sh, RaycastHit& hit)
    {
        // We have to:
        //   - transform the ray to shape's local coordinates
        //     - transform origin by inverse
        //     - transform direction by inverse of 3x3 inner matrix (disregarding translation)
        //   - remember the length of the transformed direction - will be used to scale distance
        //   - make a ray as if the direction was unitary
        //   - transform the distance to the previously hit object such that it's as if it were in the same space
        //   - if we hit the sphere:
        //     - transform back to world
        //   - if not:
        //     - transform the previous hit's distance back

        const Vec3f D = sh.worldToLocal.withoutTranslation() * Vec3f(ray.direction()); // it's not a surface normal
        const float DLen = D.length();
        Ray localRay(
            sh.worldToLocal * ray.origin(),
            D.normalized()
        );

        const float oldHitDist = hit.dist;
        hit.dist *= DLen;
        if (raycast(localRay, sh.shape, hit))
        {
            auto inv = sh.worldToLocal.inverse();

            hit.dist /= DLen;
            hit.point = inv * hit.point;
            hit.normal = inv * hit.normal;

            return true;
        }
        else
        {
            hit.dist = oldHitDist;
        }

        return false;
    }

    template <typename TransformT, typename ShapeT, typename DataT>
    [[nodiscard]] inline bool raycastIntervals(const Ray& ray, const TransformedShape3<TransformT, ShapeT>& sh, IntervalSet<DataT>& hitIntervals, const DataT& data)
    {
        // We have to:
        //   - transform the ray to shape's local coordinates
        //     - transform origin by inverse
        //     - transform direction by inverse of 3x3 inner matrix (disregarding translation)
        //   - remember the length of the transformed direction - will be used to scale distance
        //   - make a ray as if the direction was unitary
        //   - if we hit the sphere:
        //     - transform intervals back to the world coordinates
        //   - if not:
        //     - we don't have to do anything

        const Vec3f D = sh.worldToLocal.withoutTranslation() * Vec3f(ray.direction()); // it's not a surface normal
        const float DLen = D.length();
        Ray localRay(
            sh.worldToLocal * ray.origin(),
            D.normalized()
        );

        if (raycastIntervals(localRay, sh.shape, hitIntervals, data))
        {
            hitIntervals.positiveScale(1.0f / DLen);

            return true;
        }

        return false;
    }
}
