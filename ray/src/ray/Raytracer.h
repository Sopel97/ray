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
            // Used to ensure for example that the hit point is
            // not considered obstructed by the shape it is on
            // due to floating point inaccuracies
            float paddingDistance = 0.0001f;
            int maxRayDepth = 5;
            float airRefractiveIndex = 1.00027717f;
            
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
            bool assumeNoVolumeIntersections = true;
        };

        Raytracer(const Scene& scene, const Options& options = {}) :
            m_scene(&scene),
            m_options(options)
        {
        }

        template <typename SamplerT = Sampler>
        Image capture(const Camera& camera, const SamplerT& sampler = SamplerT{}) const
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
            perf::gPerfStats.addTraceTime(diff);
#endif

            return img;
        }

    private:
        const Scene* m_scene;
        Options m_options;

        ColorRGBf trace(const Ray& ray, const ColorRGBf& contribution, int depth = 0, const ResolvedRaycastHit* prevHit = nullptr, bool isInside = false) const
        {

#if defined(RAY_GATHER_PERF_STATS)
            perf::gPerfStats.addTrace(depth);
#endif

            float tNearest = std::numeric_limits<float>::max();
            ResolvableRaycastHit rhit;
            bool anyHit =
                isInside && m_options.assumeNoVolumeIntersections && prevHit && prevHit->isLocallyContinuable // if we're not inside we can't locally continue, even if shape allows that
                ? prevHit->next(ray, tNearest, rhit)
                : m_scene->queryNearest(ray, tNearest, rhit);
            if (!anyHit) return m_scene->backgroundColor();

#if defined(RAY_GATHER_PERF_STATS)
            perf::gPerfStats.addTraceHit(depth);
            perf::gPerfStats.addTraceResolved(depth);
#endif

            const ResolvedRaycastHit hit = rhit.resolve();

            const float reflectionContribution = fresnelReflectAmount(ray, hit);
            const float refractionContribution = ((1.0f - reflectionContribution) * hit.material->transparency);

            const ColorRGBf unabsorbed = 
                (isInside && prevHit) 
                ? exp(-hit.material->absorbtion * tNearest) 
                : ColorRGBf(1.0f, 1.0f, 1.0f);

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

        ColorRGBf combine(
            const ResolvedRaycastHit& hit, 
            const ColorRGBf& refractionColor, 
            const ColorRGBf& reflectionColor, 
            const ColorRGBf& diffusionColor
        ) const
        {
            const ColorRGBf textureColor = hit.material->sampleTexture(hit.texCoords);

            return hit.material->surfaceColor * textureColor * (
                reflectionColor
                + refractionColor
                + diffusionColor) + hit.material->emissionColor;
        }
        
        float fresnelReflectAmount(const Ray& ray, const ResolvedRaycastHit& hit) const
        {
            auto sqr = [](float f) {return f * f; };

            float n1 = m_options.airRefractiveIndex;
            float n2 = hit.material->refractiveIndex;
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
            const float reflectivity = hit.material->reflectivity;
            return (reflectivity + (1.0f - reflectivity) * ret);
        }

        ColorRGBf computeDiffusionColor(const Ray& ray, const ColorRGBf& contribution, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            // TODO: handle transparency?
            if (!isDiffusive(*hit.material) && !hit.isInside) // if inside we could potentially do it wrong
                return {};

            if (contribution.max() < m_options.contributionThreshold) return {};

            const auto& lights = m_scene->lights();
            const Point3f point = hit.point + hit.normal * m_options.paddingDistance;

#if defined(RAY_GATHER_PERF_STATS)
            perf::gPerfStats.addTrace(depth, lights.size());
#endif

            ColorRGBf color{};
            ResolvableRaycastHit rhit;
            for (const auto& light : lights)
            {
                const Ray ray = Ray::between(point, light.center());
                float tNearest = std::numeric_limits<float>::max();
                if (!m_scene->queryNearest(ray, tNearest, rhit)) continue;

#if defined(RAY_GATHER_PERF_STATS)
                perf::gPerfStats.addTraceHit(depth);
#endif

                if (rhit.objectId() != light.id()) continue;

#if defined(RAY_GATHER_PERF_STATS)
                perf::gPerfStats.addTraceResolved(depth);
#endif

                auto lightHit = rhit.resolve();
                color += lightHit.material->emissionColor * std::max(0.0f, dot(hit.normal, ray.direction()));
            }

            return color * hit.material->diffuse;
        }

        ColorRGBf computeReflectionColor(const Ray& ray, const ColorRGBf& contribution, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            if (!isReflective(*hit.material) || depth > m_options.maxRayDepth)
                return {};

            if (contribution.max() < m_options.contributionThreshold) return {};

            const Normal3f reflectionDirection = reflection(ray.direction(), hit.normal);
            const Ray nextRay(hit.point + reflectionDirection * m_options.paddingDistance, reflectionDirection);
            return trace(nextRay, contribution, depth + 1, &hit, hit.isInside);
        }

        ColorRGBf computeRefractionColor(const Ray& ray, const ColorRGBf& contribution, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            if (!isTransparent(*hit.material) || depth > m_options.maxRayDepth)
                return {};

            if (contribution.max() < m_options.contributionThreshold) return {};

            // outside->inside
            float eta = m_options.airRefractiveIndex / hit.material->refractiveIndex;
            if (hit.isInside) eta = 1.0f / eta;

            // do outside->inside refraction
            const Normal3f refractionDirection = refraction(ray.direction(), hit.normal, eta);
            const Ray nextRay(hit.point + refractionDirection * m_options.paddingDistance, refractionDirection);
            return trace(nextRay, contribution, depth + 1, &hit, !hit.isInside);
        }

        ColorRGBi resolveColor(const ResolvedRaycastHit& hit) const
        {
            return ColorRGBi(hit.material->surfaceColor);
        }

        bool isTransparent(const Material& material) const
        {
            return material.transparency >= m_options.transparencyThreshold;
        }

        bool isReflective(const Material& material) const
        {
            return material.reflectivity >= m_options.reflectivityThreshold;
        }

        bool isDiffusive(const Material& material) const
        {
            return material.diffuse >= m_options.diffuseThreshold;
        }
    };
}