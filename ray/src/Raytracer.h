#pragma once

#include "Camera.h"
#include "Image.h"
#include "Scene.h"
#include "Util.h"

namespace ray
{
    struct Raytracer
    {
        // Used to ensure for example that the hit point is
        // not considered obstructed by the shape it is on
        // due to floating point inaccuracies
        static constexpr float paddingDistance = 0.0001f;
        static constexpr int maxRayDepth = 5;
        static constexpr float airRefractiveIndex = 1.00027717;
        // smallest transparenct that is not considered zero
        static constexpr float transparencyThreshold = 0.01f;
        // smallest reflectivity that is still considered reflective
        static constexpr float reflectivityThreshold = 0.01f;
        // smallest diffuse that makes it trace shadow ray
        static constexpr float diffuseThreshold = 0.01f;

        Raytracer(const Scene& scene) :
            m_scene(&scene)
        {

        }

        Image capture(const Camera& camera) const
        {
            Image img(camera.width(), camera.height());
            camera.forEachPixelRay([&img, this](const Ray& ray, int x, int y) {img(x, y) = ColorRGBi(trace(ray)); }, std::execution::par_unseq);
            return img;
        }

    private:
        const Scene* m_scene;

        ColorRGBf trace(const Ray& ray, int depth = 1) const
        {
            std::optional<ResolvedLocallyContinuableRaycastHit> hitOpt = m_scene->queryNearest(ray);
            if (!hitOpt) return m_scene->backgroundColor();

            // reflection, refraction
            ResolvedLocallyContinuableRaycastHit& hit = *hitOpt;

            ColorRGBf refractionColor = computeRefractionColor(ray, hit, depth);
            ColorRGBf reflectionColor = computeReflectionColor(ray, hit, depth);
            ColorRGBf diffusionColor = computeDiffusionColor(ray, hit);

            return combineFresnel(ray, hit, refractionColor, reflectionColor, diffusionColor) + hit.material->emissionColor;
        }

        ColorRGBf combineFresnel(const Ray& ray, const ResolvedLocallyContinuableRaycastHit& hit, const ColorRGBf& refractionColor, const ColorRGBf& reflectionColor, const ColorRGBf& diffusionColor) const
        {
            const float fresnelEffect = fresnelReflectAmount(ray, hit);

            return hit.material->surfaceColor * (
                reflectionColor * (fresnelEffect * hit.material->reflectivity)
                + refractionColor * ((1.0f - fresnelEffect) * hit.material->transparency)
                + diffusionColor);
        }
        
        float fresnelReflectAmount(const Ray& ray, const ResolvedLocallyContinuableRaycastHit& hit) const
        {
            auto sqr = [](float f) {return f * f; };

            // Schlick aproximation
            const float n1 = airRefractiveIndex;
            const float n2 = hit.material->refractiveIndex;
            const float r0 = sqr((n1 - n2) / (n1 + n2));
            float cosX = dot(hit.normal, ray.direction());
            if (n1 > n2)
            {
                const float n = n1 / n2;
                const float sinT2 = n * n*(1.0f - cosX * cosX);
                // Total internal reflection
                if (sinT2 > 1.0f)
                    return 1.0f;
                cosX = std::sqrt(1.0f - sinT2);
            }
            const float x = 1.0f - cosX;
            const float ret = r0 + (1.0f - r0)*x*x*x*x*x;

            // adjust reflect multiplier for object reflectivity
            const float reflectivity = hit.material->reflectivity;
            //const float transparency = 1.0f - hit.material->opacity;
            return (reflectivity + (1.0f - reflectivity) * ret);
        }

        ColorRGBf computeDiffusionColor(const Ray& ray, const ResolvedLocallyContinuableRaycastHit& hit) const
        {
            if (!isDiffusive(*hit.material))
                return {};

            auto visibleLightHits = m_scene->queryVisibleLights(hit.point + hit.normal * paddingDistance);

            ColorRGBf color{};
            for (auto& lightHit : visibleLightHits)
            {
                const Normal3f lightDirection = (lightHit.point - hit.point).normalized();
                color += lightHit.material->emissionColor * std::max(0.0f, -dot(hit.normal, lightDirection));
            }
            return color * hit.material->diffuse;
        }

        ColorRGBf computeReflectionColor(const Ray& ray, const ResolvedLocallyContinuableRaycastHit& hit, int depth) const
        {
            if (!isReflective(*hit.material) || depth > maxRayDepth)
                return {};

            const Normal3f reflectionDirection = reflection(ray.direction(), hit.normal);
            return trace(Ray(hit.point - hit.normal * paddingDistance, reflectionDirection), depth + 1);
        }

        ColorRGBf computeRefractionColor(const Ray& ray, const ResolvedLocallyContinuableRaycastHit& hit, int depth) const
        {
            if (!isTransparent(*hit.material) || depth > maxRayDepth)
                return {};

            // outside->inside
            const float eta = hit.material->refractiveIndex / airRefractiveIndex;
            
            // do outside->inside refraction
            const Normal3f refractionDirection = refraction(ray.direction(), -hit.normal, eta);
            std::optional<ResolvedRaycastHit> outHitOpt = hit.next(refractionDirection);
            // TODO: fix. Something is wrong. outHitOpt should never be nullopt but is when the incidence angle is low
            if (!outHitOpt) return {};

            // TODO: handle reflection
            // do inside->outside refraction
            ResolvedRaycastHit& outHit = *outHitOpt;
            const Normal3f outRefractionDirection = refraction(refractionDirection, outHit.normal, 1.0f / eta);
            return trace(Ray(outHit.point + outHit.normal * paddingDistance, outRefractionDirection), depth + 1);
        }

        ColorRGBi resolveColor(const ResolvedRaycastHit& hit) const
        {
            return ColorRGBi(hit.material->surfaceColor);
        }

        bool isTransparent(const Material& material) const
        {
            return material.transparency >= transparencyThreshold;
        }

        bool isReflective(const Material& material) const
        {
            return material.reflectivity >= reflectivityThreshold;
        }

        bool isDiffusive(const Material& material) const
        {
            return material.diffuse >= diffuseThreshold;
        }
    };
}