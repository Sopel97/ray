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
        static constexpr ColorRGBf backgroundColor = ColorRGBf(0.0f, 0.0f, 0.0f);
        static constexpr float airRefractiveIndex = 1.00027717;
        // highest opacity that is not yet considered transparent
        static constexpr float opacityThreshold = 0.99f;
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
            if (!hitOpt) return backgroundColor;

            // reflection, refraction
            ResolvedLocallyContinuableRaycastHit& hit = *hitOpt;

            ColorRGBf refractionColor = computeRefractionColor(ray, hit, depth);
            ColorRGBf reflectionColor = computeReflectionColor(ray, hit, depth);
            ColorRGBf diffusionColor = computeDiffusionColor(ray, hit, depth);

            return combineFresnel(ray, hit, refractionColor, reflectionColor, diffusionColor) + hit.material->emissionColor;
        }

        ColorRGBf combineFresnel(const Ray& ray, const ResolvedLocallyContinuableRaycastHit& hit, const ColorRGBf& refractionColor, const ColorRGBf& reflectionColor, const ColorRGBf& diffusionColor) const
        {
            const float transparency = 1.0f - hit.material->opacity;
            const float facingRatio = dot(ray.direction(), hit.normal);
            const float fresnelEffect = mix(std::pow(1.0f - facingRatio, 3), 1.0f, 0.1f);

            return hit.material->surfaceColor * (
                reflectionColor * fresnelEffect
                + refractionColor * (1.0f - fresnelEffect) * transparency
                + diffusionColor);
        }

        ColorRGBf computeDiffusionColor(const Ray& ray, const ResolvedLocallyContinuableRaycastHit& hit, int depth) const
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
            const Normal3f refractionDirection = refraction(ray.direction(), hit.normal, eta);
            std::optional<ResolvedRaycastHit> outHitOpt = hit.next(refractionDirection);
            if (!outHitOpt) return {};

            // do inside->outside refraction
            ResolvedRaycastHit& outHit = *outHitOpt;
            const Normal3f outRefractionDirection = refraction(ray.direction(), outHit.normal, 1.0f / eta);
            return trace(Ray(outHit.point + outHit.normal * paddingDistance, outRefractionDirection), depth + 1);
        }

        ColorRGBi resolveColor(const ResolvedRaycastHit& hit) const
        {
            return ColorRGBi(hit.material->surfaceColor);
        }

        bool isTransparent(const Material& material) const
        {
            return material.opacity < opacityThreshold;
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