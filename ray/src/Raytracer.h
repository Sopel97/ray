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
        static constexpr int maxRayDepth = 6;
        static constexpr float airRefractiveIndex = 1.00027717f;
        // smallest transparenct that is not considered zero
        static constexpr float transparencyThreshold = 0.01f;
        // smallest reflectivity that is still considered reflective
        static constexpr float reflectivityThreshold = 0.01f;
        // smallest diffuse that makes it trace shadow ray
        static constexpr float diffuseThreshold = 0.01f;

        // not sure if it should be used
        static constexpr float gamma = 0.43f;

        Raytracer(const Scene& scene) :
            m_scene(&scene)
        {

        }

        Image capture(const Camera& camera) const
        {
            Image img(camera.width(), camera.height());
            camera.forEachPixelRay([&img, this](const Ray& ray, int x, int y) {img(x, y) = ColorRGBi(trace(ray) ^ gamma); }, std::execution::par_unseq);
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

            return combineFresnel(ray, hit, refractionColor, reflectionColor, diffusionColor, true) + hit.material->emissionColor;
        }

        ColorRGBf trace(const Ray& ray, const ResolvedLocallyContinuableRaycastHit& inHit, int depth = 1) const
        {
            // we are doing the inside part of the refraction
            std::optional<ResolvedRaycastHit> hitOpt = inHit.next(ray);
            if (!hitOpt) return {};

            ResolvedRaycastHit& hit = *hitOpt;
            hit.normal = -hit.normal;

            ColorRGBf refractionColor = computeRefractionColor(ray, inHit, hit, depth);
            ColorRGBf reflectionColor = computeReflectionColor(ray, hit, depth);
            ColorRGBf diffusionColor = computeDiffusionColor(ray, hit);

            return combineFresnel(ray, hit, refractionColor, reflectionColor, diffusionColor, false) + hit.material->emissionColor;
        }

        ColorRGBf combineFresnel(const Ray& ray, const ResolvedRaycastHit& hit, const ColorRGBf& refractionColor, const ColorRGBf& reflectionColor, const ColorRGBf& diffusionColor, bool inside = false) const
        {
            float n1 = airRefractiveIndex;
            float n2 = hit.material->refractiveIndex;
            if (inside) std::swap(n1, n2);
            const float fresnelEffect = fresnelReflectAmount(ray, hit.normal, hit.material->reflectivity, n1, n2);

            /*
            if (hit.objectId == 0)
            {
                // In this particular case, the normal is simular to a point on a unit sphere
                // centred around the origin. We can thus use the normal coordinates to compute
                // the spherical coordinates of Phit.
                // atan2 returns a value in the range [-pi, pi] and we need to remap it to range [0, 1]
                // acosf returns a value in the range [0, pi] and we also need to remap it to the range [0, 1]
                float x = (1.0f + std::atan2(hit.normal.z, hit.normal.x) / pi) * 0.5f;
                float y = std::acosf(hit.normal.y) / pi;
                float scale = 30;
                float pattern = (std::fmodf(x * scale, 1) > 0.5) ^ (fmodf(y * scale * 4.0f, 1) > 0.5) ? 1.0f : 0.8f;

                return hit.material->surfaceColor * pattern * (
                    reflectionColor * (fresnelEffect) * 0.0f
                    +refractionColor * ((1.0f - fresnelEffect) * hit.material->transparency)
                    + diffusionColor);
            }
            */
            return hit.material->surfaceColor * (
                reflectionColor * (fresnelEffect)
                + refractionColor * ((1.0f - fresnelEffect) * hit.material->transparency)
                + diffusionColor);
        }
        
        float fresnelReflectAmount(const Ray& ray, const Normal3f& normal, float reflectivity, float n1, float n2) const
        {
            std::swap(n1, n2);
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

            cosX = std::abs(cosX);
            const float x = 1.0f - cosX;
            const float ret = r0 + (1.0f - r0)*x*x*x*x*x;

            // adjust reflect multiplier for object reflectivity
            return (reflectivity + (1.0f - reflectivity) * ret);
        }

        ColorRGBf computeDiffusionColor(const Ray& ray, const ResolvedRaycastHit& hit) const
        {
            // TODO: handle transparency?
            if (!isDiffusive(*hit.material))
                return {};

            auto visibleLightHits = m_scene->queryVisibleLights(hit.point + hit.normal * paddingDistance);

            ColorRGBf color{};
            for (auto& lightHit : visibleLightHits)
            {
                const Normal3f lightDirection = (lightHit.point - hit.point).normalized();
                color += lightHit.material->emissionColor * std::max(0.0f, dot(hit.normal, lightDirection));
            }
            return color * hit.material->diffuse;
        }

        ColorRGBf computeReflectionColor(const Ray& ray, const ResolvedRaycastHit& hit, int depth) const
        {
            if (!isReflective(*hit.material) || depth > maxRayDepth)
                return {};

            const Normal3f reflectionDirection = reflection(ray.direction(), hit.normal);
            return trace(Ray(hit.point + reflectionDirection * paddingDistance, reflectionDirection), depth + 1);
        }

        ColorRGBf computeRefractionColor(const Ray& ray, const ResolvedLocallyContinuableRaycastHit& hit, int depth) const
        {
            if (!isTransparent(*hit.material) || depth > maxRayDepth)
                return {};

            // outside->inside
            const float eta = airRefractiveIndex / hit.material->refractiveIndex;
            
            // do outside->inside refraction
            const Normal3f refractionDirection = refraction(ray.direction(), hit.normal, eta);
            return trace(Ray(hit.point + refractionDirection * paddingDistance, refractionDirection), hit, depth + 1);
        }

        ColorRGBf computeRefractionColor(const Ray& ray, const ResolvedLocallyContinuableRaycastHit& inHit, const ResolvedRaycastHit& hit, int depth) const
        {
            if (!isTransparent(*hit.material) || depth > maxRayDepth)
                return {};

            // outside->inside
            const float eta = hit.material->refractiveIndex / airRefractiveIndex;

            // do outside->inside refraction
            const Normal3f refractionDirection = refraction(ray.direction(), hit.normal, eta);
            const ColorRGBf color = trace(Ray(hit.point + refractionDirection * paddingDistance, refractionDirection), depth + 1);
            
            // do absorbtion
            // Beer's law
            const float dist = distance(hit.point, inHit.point);
            return exp(-hit.material->absorbtion * dist) * color;
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