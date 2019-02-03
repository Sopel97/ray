#pragma once

#include "Ray.h"
#include "RaycastHit.h"
#include "Sphere.h"
#include "Vec3.h"

#include <cmath>
#include <optional>

namespace ray
{
    float smallestPositive(float a, float b)
    {

    }

    std::optional<RaycastHit> raycast(const Ray& ray, const Sphere& sphere)
    {
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

        float t1 = t_ca - t_hc;
        float t2 = t_ca + t_hc;

        // select smallest positive
        // we know that at least one is positive
        // and that t2 is greater
        if (t1 > 0.0f)
        {
            // hit from outside
            const Point3f hitPoint = O + t1 * D;
            const Normal3f normal = (hitPoint - C).normalized();
            const int shapeNo = 0;
            const int materialNo = 0;
            return RaycastHit{ hitPoint, normal, shapeNo, materialNo, false };
        }
        else
        {
            // hit from inside
            const Point3f hitPoint = O + t2 * D;
            const Normal3f normal = (hitPoint - C).normalized();
            const int shapeNo = 0;
            const int materialNo = 0;
            return RaycastHit{ hitPoint, -normal, shapeNo, materialNo, true };
        }

    }
}
