#pragma once

#if defined(RAY_GATHER_PERF_STATS)
#include "PerformanceStats.h"
#endif

#include "Camera.h"
#include "LightHandle.h"
#include "Image.h"
#include "Scene.h"
#include "RaycastHit.h"
#include "Util.h"

namespace ray
{
    // TODO: make statistics gathering everywhere be a constexpr switch
    //       maybe even use preprocessor switch and a global state
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
            float diffuseThreshold = 0.01f;

            // not sure if it should be used
            float gamma = 0.43f;
        };

        Raytracer(const Scene& scene, const Options& options = {}) :
            m_scene(&scene),
            m_options(options)
        {
        }

        Image capture(const Camera& camera) const
        {
            Image img(camera.width(), camera.height());
#if defined(RAY_GATHER_PERF_STATS)
            auto t0 = std::chrono::high_resolution_clock().now();
#endif
            camera.forEachPixelRay([&img, this](const Ray& ray, int x, int y) {img(x, y) = ColorRGBi(trace(ray) ^ m_options.gamma); }, std::execution::par_unseq);
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

        ColorRGBf trace(const Ray& ray, int depth = 0, const ResolvedRaycastHit* prevHit = nullptr, bool isInside = false) const
        {
#if defined(RAY_GATHER_PERF_STATS)
            perf::gPerfStats.addTrace(depth);
#endif

            std::optional<ResolvableRaycastHit> hitOpt =
                prevHit && prevHit->isLocallyContinuable && isInside // if we're not inside we can't locally continue, even if shape allows that
                ? prevHit->next(ray)
                : m_scene->queryNearest(ray);
            if (!hitOpt) return m_scene->backgroundColor();

#if defined(RAY_GATHER_PERF_STATS)
            perf::gPerfStats.addTraceHit(depth);
            perf::gPerfStats.addTraceResolved(depth);
#endif
            const ResolvedRaycastHit hit = hitOpt->resolve();

            const ColorRGBf refractionColor = computeRefractionColor(ray, prevHit, hit, depth);
            const ColorRGBf reflectionColor = computeReflectionColor(ray, prevHit, hit, depth);
            const ColorRGBf diffusionColor = computeDiffusionColor(ray, prevHit, hit, depth);

            const ColorRGBf color = combineFresnel(ray, hit, refractionColor, reflectionColor, diffusionColor);

            if (hit.isInside && prevHit)
            {
                // do absorbtion
                // Beer's law
                const float dist = distance(hit.point, prevHit->point);
                return exp(-hit.material->absorbtion * dist) * color;
            }
            return color;
        }

        ColorRGBf combineFresnel(const Ray& ray, const ResolvedRaycastHit& hit, const ColorRGBf& refractionColor, const ColorRGBf& reflectionColor, const ColorRGBf& diffusionColor) const
        {
            float n1 = m_options.airRefractiveIndex;
            float n2 = hit.material->refractiveIndex;
            if (hit.isInside) std::swap(n1, n2);
            const float fresnelEffect = fresnelReflectAmount(ray, hit.normal, hit.material->reflectivity, n1, n2);
            const ColorRGBf textureColor = hit.material->sampleTexture(hit.texCoords);

            return hit.material->surfaceColor * textureColor * (
                reflectionColor * (fresnelEffect)
                + refractionColor * ((1.0f - fresnelEffect) * hit.material->transparency)
                + diffusionColor) + hit.material->emissionColor;
        }
        
        float fresnelReflectAmount(const Ray& ray, const Normal3f& normal, float reflectivity, float n1, float n2) const
        {
            auto sqr = [](float f) {return f * f; };

            // Schlick aproximation
            const float r0 = sqr((n1 - n2) / (n1 + n2));
            float cosX = dot(normal, ray.direction());
            if (n1 > n2)
            {
                const float n = n1 / n2;
                const float sinT2 = n * n*(1.0f - cosX * cosX);
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
            return (reflectivity + (1.0f - reflectivity) * ret);
        }

        ColorRGBf computeDiffusionColor(const Ray& ray, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            // TODO: handle transparency?
            if (!isDiffusive(*hit.material) && !hit.isInside) // if inside we could potentially do it wrong
                return {};

            const auto& lights = m_scene->lights();
            const Point3f point = hit.point + hit.normal * m_options.paddingDistance;

#if defined(RAY_GATHER_PERF_STATS)
            perf::gPerfStats.addTrace(depth, lights.size());
#endif

            ColorRGBf color{};
            for (const auto& light : lights)
            {
                const Ray ray = Ray::between(point, light.center());
                std::optional<ResolvableRaycastHit> lightHitOpt = m_scene->queryNearest(ray);
                if (!lightHitOpt) continue;

#if defined(RAY_GATHER_PERF_STATS)
                perf::gPerfStats.addTraceHit(depth);
#endif

                if (lightHitOpt->objectId() != light.id()) continue;

#if defined(RAY_GATHER_PERF_STATS)
                perf::gPerfStats.addTraceResolved(depth);
#endif

                auto lightHit = lightHitOpt->resolve();
                color += lightHit.material->emissionColor * std::max(0.0f, dot(hit.normal, ray.direction()));
            }

            return color * hit.material->diffuse;
        }

        ColorRGBf computeReflectionColor(const Ray& ray, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            if (!isReflective(*hit.material) || depth > m_options.maxRayDepth)
                return {};

            const Normal3f reflectionDirection = reflection(ray.direction(), hit.normal);
            return trace(Ray(hit.point + reflectionDirection * m_options.paddingDistance, reflectionDirection), depth + 1, &hit, hit.isInside);
        }

        ColorRGBf computeRefractionColor(const Ray& ray, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            if (!isTransparent(*hit.material) || depth > m_options.maxRayDepth)
                return {};

            // outside->inside
            float eta = m_options.airRefractiveIndex / hit.material->refractiveIndex;
            if (hit.isInside) eta = 1.0f / eta;

            // do outside->inside refraction
            const Normal3f refractionDirection = refraction(ray.direction(), hit.normal, eta);
            const ColorRGBf color = trace(Ray(hit.point + refractionDirection * m_options.paddingDistance, refractionDirection), depth + 1, &hit, !hit.isInside);
            
            return color;
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