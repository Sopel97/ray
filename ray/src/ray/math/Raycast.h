#pragma once

#if defined(RAY_GATHER_PERF_STATS)
#include <ray/perf/PerformanceStats.h>
#endif

#include "Ray.h"
#include "RaycastHit.h"
#include "Vec3.h"

#include <ray/shape/Box3.h>
#include <ray/shape/Plane.h>
#include <ray/shape/Sphere.h>

#include <algorithm>
#include <cmath>
#include <optional>

namespace ray
{
    bool contains(const Box3& box, const Point3f& point)
    {
        return
               point.x >= box.min.x && point.x <= box.max.x
            && point.y >= box.min.y && point.y <= box.max.y
            && point.z >= box.min.z && point.z <= box.max.z;
    }

    std::optional<RaycastBvHit> raycastBv(const Ray& ray, const Box3& box)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addBoxBvRaycast();
#endif

        if (contains(box, ray.origin()))
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gPerfStats.addBoxBvRaycastHit();
#endif
            return RaycastBvHit{ 0.0f };
        }

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
        perf::gPerfStats.addBoxBvRaycastHit();
#endif

        return RaycastBvHit{ tmin };

        /*
        const Vec3f invDir = ray.direction().reciprocal();
        const Vec3f t0 = (box.min - ray.origin()) * invDir;
        const Vec3f t1 = (box.max - ray.origin()) * invDir;
        const float tmin = min(t0, t1).max();
        const float tmax = max(t0, t1).min();

        if (tmin <= tmax)
        {
            return RaycastBvHit{ tmin };
        }

        return std::nullopt;
        */
    }

    std::optional<RaycastHit> raycast(const Ray& ray, const Sphere& sphere)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addSphereRaycast();
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

        // doesn't work if we're inside? surely doesn't work if we're past the sphere origin
        const Point3f O = ray.origin();
        const Normal3f D = ray.direction();
        const Point3f C = sphere.center();
        const float R = sphere.radius();

        const Vec3f L = C - O;
        const float t_ca = dot(L, D);
        if (t_ca < 0.0f) return std::nullopt;

        const float d2 = dot(L, L) - t_ca * t_ca;
        const float r = R*R - d2;
        if (r < 0.0f) return std::nullopt;
        const float t_hc = std::sqrt(r);

        float t = t_ca - t_hc;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addSphereRaycastHit();
#endif
        // select smallest positive
        // we know that at least one is positive
        // and that t2 is greater
        bool isInside = t < 0.0f;
        if (isInside) t = t_ca + t_hc;
        const Point3f hitPoint = O + t * D;
        Normal3f normal = ((hitPoint - C) / R).assumeNormalized();
        if (isInside) normal = -normal;
        return RaycastHit{ hitPoint, normal, 0, 0, isInside };
    }

    std::optional<RaycastHit> raycast(const Ray& ray, const Plane& plane)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addPlaneRaycast();
#endif
        const float nd = dot(ray.direction(), plane.normal);

        // nd must be negative, and not 0
        // if nd is positive, the ray and plane normals
        // point in the same direction. No intersection.
        if (nd >= 0.0f) return std::nullopt;

        const float pn = dot(Vec3f(ray.origin()), plane.normal);
        float t = (plane.distance - pn) / nd;

        // t must be positive
        if (t >= 0.0f) 
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gPerfStats.addPlaneRaycastHit();
#endif
            const Point3f point = ray.origin() + ray.direction() * t;
            return RaycastHit{ point, plane.normal, 0, 0, false };
        }

        return std::nullopt;
    }

    std::optional<RaycastHit> raycast(const Ray& ray, const Box3& box)
    {
#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addBoxRaycast();
#endif
        const Point3f origin = ray.origin();
        const Vec3f invDir = ray.invDirection();
        const auto sign = ray.signs();
        Point3f min = box.min;
        Point3f max = box.max;
        Vec3f normal{};
        if (sign[0]) std::swap(min.x, max.x);
        if (sign[1]) std::swap(min.y, max.y);
        if (sign[2]) std::swap(min.z, max.z);

        const Vec3f t0 = (min - origin) * invDir;
        const Vec3f t1 = (max - origin) * invDir;

        float tmin = std::max(t0.x, t0.y);
        float tmax = std::min(t1.x, t1.y);

        if (t0.x > t0.y)
        {
            // normal on x axis
            normal.x = sign[0] ? 1.0f : -1.0f;
        }
        else
        {
            // normal on y axis
            normal.y = sign[1] ? 1.0f : -1.0f;
        }

        if (tmin > tmax) return std::nullopt;

        if (t0.z > tmin)
        {
            tmin = t0.z;
            normal.x = normal.y = 0.0f;
            normal.z = sign[2] ? 1.0f : -1.0f;
        }
        if (t1.z < tmax)
            tmax = t1.z;

        if (tmin > tmax) return std::nullopt;
        if (tmax < 0.0f) return std::nullopt;

        const bool isInside = tmin < 0.0f;
        const float t = isInside ? tmax : tmin;
        if (isInside) normal = -normal;
        const Point3f point = ray.origin() + ray.direction() * t;

#if defined(RAY_GATHER_PERF_STATS)
        perf::gPerfStats.addBoxRaycastHit();
#endif
        return RaycastHit{ point, normal.assumeNormalized(), 0, 0, isInside };

    }
}
