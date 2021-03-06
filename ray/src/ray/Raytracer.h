#pragma once

#if defined(RAY_GATHER_PERF_STATS)
#include <ray/perf/PerformanceStats.h>
#endif

#include <ray/math/Vec2.h>
#include <ray/math/Vec3.h>

#include <ray/sampler/Sampler.h>

#include <ray/scene/LightHandle.h>
#include <ray/scene/Scene.h>
#include <ray/scene/SceneRaycastHit.h>

#include <ray/utility/Util.h>

#include <ray/Camera.h>
#include <ray/Image.h>

namespace ray
{
    struct Raytracer
    {
        struct Options
        {
            Options() {}

            // Used to ensure for example that the hit point is
            // not considered obstructed by the shape it is on
            // due to floating point inaccuracies
            float paddingDistance = 0.002f;
            int maxRayDepth = 5;
            
            // smallest transparenct that is not considered zero
            float transparencyThreshold = 0.01f;
            
            // smallest reflectivity that is still considered reflective
            float reflectivityThreshold = 0.01f;
            
            // smallest diffuse that makes it trace shadow ray
            float diffuseThreshold = 0.0f;
            
            // if raytrace is known to result in less contribution to the final
            // result then the raytrace may be pruned
            float contributionThreshold = 0.01f;

            // not sure if it should be used
            float gamma = 0.43f;

            // if true allows local continuation of tracing
            // when inside a volumetric shape
            bool assumeNoVolumeIntersections = false;
        };

        Raytracer(const Scene& scene, const Options& options = {}) noexcept :
            m_scene(&scene),
            m_options(options)
        {
        }

        template <typename SamplerT = Sampler>
        [[nodiscard]] Image capture(const Camera& camera, const SamplerT& sampler = SamplerT{}) const
        {
            Image img(camera.width(), camera.height());

#if defined(RAY_GATHER_PERF_STATS)
            auto t0 = std::chrono::high_resolution_clock().now();
#endif

            /*
            camera.forEachPixelRay([&](const Ray& ray, int x, int y) {
                img(x, y) = ColorRGBi(trace(ray, ColorRGBf(1.0f, 1.0f, 1.0f)) ^ m_options.gamma);
                }, std::execution::par_unseq);
                */
            sampler.forEachSample(
                camera,
                [&](const Ray& ray) {
                    return trace(ray, ColorRGBf(1.0f, 1.0f, 1.0f));
                },
                [&](const Point2i& imgCoords, const ColorRGBf& color) {
                    img(imgCoords.x, imgCoords.y) = ColorRGBi(color ^ m_options.gamma);
                },
                std::execution::par_unseq
            );

#if defined(RAY_GATHER_PERF_STATS)
            auto t1 = std::chrono::high_resolution_clock().now();
            auto diff = t1 - t0;
            perf::gThreadLocalPerfStats.addTraceTime(diff);
#endif

            return img;
        }

    private:
        const Scene* m_scene;
        Options m_options;

        [[nodiscard]] ColorRGBf trace(const Ray& ray, const ColorRGBf& contribution, int depth = 0, const ResolvedRaycastHit* prevHit = nullptr, bool isInside = false) const
        {

#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addTrace(depth);
#endif

            ResolvableRaycastHit rhit;
            rhit.dist = std::numeric_limits<float>::max();
            bool anyHit = false;
            if (isInside && m_options.assumeNoVolumeIntersections && prevHit && prevHit->isLocallyContinuable)
            {
                // if we're not inside we can't locally continue, even if shape allows that
                anyHit = prevHit->next(ray, rhit);
                if (!anyHit)
                {
                    isInside = false;
                }
            }

            // if it didn't hit locally again (due to fp precision)
            // assume the ray is past the shape (shape was thinner than paddingDistance)
            // mark as if we are not inside anymore
            // do a full raycast
            if (!anyHit)
            {
                anyHit = m_scene->queryNearest(ray, rhit);
            }
            if (!anyHit)
            {
                const MediumMaterial* medium = m_scene->mediumMaterial();
                if (medium)
                {
                    return m_scene->backgroundColor() * exp(-medium->absorbtion * m_scene->backgroundDistance());
                }
                else
                {
                    return m_scene->backgroundColor();
                }
            }

#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addTraceHit(depth);
            perf::gThreadLocalPerfStats.addTraceResolved(depth);
#endif

            const ResolvedRaycastHit hit = rhit.resolve();

            const float reflectionContribution = fresnelReflectAmount(ray, hit);
            const float refractionContribution = ((1.0f - reflectionContribution) * hit.transparency);

            ColorRGBf unabsorbed(1.0f, 1.0f, 1.0f);
            if (isInside && prevHit && hit.mediumMaterial)
            {
                unabsorbed = exp(-hit.mediumMaterial->absorbtion * hit.dist);
            }
            else if (!isInside)
            {
                const MediumMaterial* airMedium = m_scene->mediumMaterial();
                if (airMedium)
                {
                    // we're going through air
                    unabsorbed = exp(-airMedium->absorbtion * hit.dist);
                }
            }

            const ColorRGBf refractionColor = computeRefractionColor(ray, contribution * unabsorbed * refractionContribution, prevHit, hit, depth);
            const ColorRGBf reflectionColor = computeReflectionColor(ray, contribution * unabsorbed * reflectionContribution, prevHit, hit, depth);
            const ColorRGBf diffusionColor = computeDiffusionColor(ray, contribution * unabsorbed, prevHit, hit, depth);

            const ColorRGBf color = combine(
                hit,
                refractionColor * refractionContribution,
                reflectionColor * reflectionContribution,
                diffusionColor
            );

            return color * unabsorbed;
        }

        [[nodiscard]] ColorRGBf combine(
            const ResolvedRaycastHit& hit, 
            const ColorRGBf& refractionColor, 
            const ColorRGBf& reflectionColor, 
            const ColorRGBf& diffusionColor
        ) const
        {
            return hit.surfaceColor * (
                reflectionColor
                + refractionColor
                + diffusionColor) + hit.emissionColor;
        }
        
        [[nodiscard]] float fresnelReflectAmount(const Ray& ray, const ResolvedRaycastHit& hit) const
        {
            auto sqr = [](float f) {return f * f; };

            const MediumMaterial* airMedium = m_scene->mediumMaterial();
            float n1 = airMedium ? airMedium->refractiveIndex : 1.0f;
            float n2 = hit.mediumMaterial ? hit.mediumMaterial->refractiveIndex : n1;
            if (hit.isInside) std::swap(n1, n2);

            // Schlick aproximation
            const float r0 = sqr((n1 - n2) / (n1 + n2));
            float cosX = dot(hit.normal, ray.direction());
            if (n1 > n2)
            {
                const float n = n1 / n2;
                const float sinT2 = n*n*(1.0f - cosX * cosX);
                // Total internal reflection
                if (sinT2 > 1.0f)
                    return 1.0f;
                cosX = std::sqrt(1.0f - sinT2);
            }
            else
            {
                cosX = std::abs(cosX);
            }
            const float x = 1.0f - cosX;
            const float ret = r0 + (1.0f - r0)*x*x*x*x*x;

            // adjust reflect multiplier for object reflectivity
            const float reflectivity = hit.reflectivity;
            return (reflectivity + (1.0f - reflectivity) * ret);
        }

        [[nodiscard]] ColorRGBf computeDiffusionColor(const Ray& ray, const ColorRGBf& contribution, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            // TODO: handle transparency?
            if (!isDiffusive(hit) && !hit.isInside) // if inside we could potentially do it wrong
                return {};

            if (contribution.max() < m_options.contributionThreshold) return {};

            const auto& lights = m_scene->lights();
            const Point3f point = hit.point + hit.normal * m_options.paddingDistance;

#if defined(RAY_GATHER_PERF_STATS)
            perf::gThreadLocalPerfStats.addTrace(depth, lights.size());
#endif

            ColorRGBf color{};
            ResolvableRaycastHit rhit;
            rhit.dist = std::numeric_limits<float>::max();
            const MediumMaterial* airMedium = m_scene->mediumMaterial();
            for (const auto& light : lights)
            {
                const Ray ray = Ray::between(point, light.center());
                if (!m_scene->queryNearest(ray, rhit)) continue;

#if defined(RAY_GATHER_PERF_STATS)
                perf::gThreadLocalPerfStats.addTraceHit(depth);
#endif

                if (rhit.objectId() != light.id()) continue;

#if defined(RAY_GATHER_PERF_STATS)
                perf::gThreadLocalPerfStats.addTraceResolved(depth);
#endif

                ColorRGBf unabsorbed(1.0f, 1.0f, 1.0f);
                if (airMedium)
                {
                    // we're going through air
                    unabsorbed = exp(-airMedium->absorbtion * hit.dist);
                }
                auto lightHit = rhit.resolve();
                color += lightHit.emissionColor * std::max(0.0f, dot(hit.normal, ray.direction())) * unabsorbed;
            }

            return color * hit.diffuse;
        }

        [[nodiscard]] ColorRGBf computeReflectionColor(const Ray& ray, const ColorRGBf& contribution, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            if (!isReflective(hit) || depth > m_options.maxRayDepth)
                return {};

            if (contribution.max() < m_options.contributionThreshold) return {};

            const UnitVec3f reflectionDirection = reflection(ray.direction(), hit.normal);
            const Ray nextRay(hit.point + reflectionDirection * m_options.paddingDistance, reflectionDirection);
            return trace(nextRay, contribution, depth + 1, &hit, hit.isInside);
        }

        [[nodiscard]] ColorRGBf computeRefractionColor(const Ray& ray, const ColorRGBf& contribution, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            if (!isTransparent(hit) || depth > m_options.maxRayDepth)
                return {};

            if (contribution.max() < m_options.contributionThreshold) return {};

            // outside->inside
            if (hit.mediumMaterial)
            {
                const MediumMaterial* airMedium = m_scene->mediumMaterial();
                const float n1 = airMedium ? airMedium->refractiveIndex : 1.0f;
                float eta = n1 / hit.mediumMaterial->refractiveIndex;
                if (hit.isInside) eta = 1.0f / eta;

                // do outside->inside refraction
                const UnitVec3f refractionDirection = refraction(ray.direction(), hit.normal, eta);
                const Ray nextRay(hit.point + refractionDirection * m_options.paddingDistance, refractionDirection);
                return trace(nextRay, contribution, depth + 1, &hit, hit.hasVolume ? !hit.isInside : hit.isInside);
            }
            else
            {
                // if the shape doesn't have volume we don't have to bother with refracting the ray
                const Ray nextRay = ray.translated(ray.direction() * m_options.paddingDistance);
                return trace(nextRay, contribution, depth + 1, &hit, hit.isInside);
            }
        }

        [[nodiscard]] ColorRGBi resolveColor(const ResolvedRaycastHit& hit) const
        {
            return ColorRGBi(hit.surfaceColor);
        }

        [[nodiscard]] bool isTransparent(const ResolvedRaycastHit& hit) const
        {
            return hit.transparency >= m_options.transparencyThreshold;
        }

        [[nodiscard]] bool isReflective(const ResolvedRaycastHit& hit) const
        {
            return hit.reflectivity >= m_options.reflectivityThreshold;
        }

        [[nodiscard]] bool isDiffusive(const ResolvedRaycastHit& hit) const
        {
            return hit.diffuse >= m_options.diffuseThreshold;
        }
    };
}