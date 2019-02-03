#pragma once

#include "Camera.h"
#include "Image.h"
#include "Scene.h"
#include "RaycastHit.h"
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

        ColorRGBf trace(const Ray& ray, int depth = 1, const ResolvedRaycastHit* prevHit = nullptr, bool isInside = false) const
        {
            std::optional<ResolvedRaycastHit> hitOpt = 
                prevHit && prevHit->isLocallyContinuable && isInside // if we're not inside we can't locally continue, even if shape allows that
                ? prevHit->next(ray) 
                : m_scene->queryNearest(ray);
            if (!hitOpt) return m_scene->backgroundColor();

            ResolvedRaycastHit& hit = *hitOpt;

            ColorRGBf refractionColor = computeRefractionColor(ray, prevHit, hit, depth);
            ColorRGBf reflectionColor = computeReflectionColor(ray, prevHit, hit, depth);
            ColorRGBf diffusionColor = computeDiffusionColor(ray, prevHit, hit);

            ColorRGBf color = combineFresnel(ray, hit, refractionColor, reflectionColor, diffusionColor, hit.isInside) + hit.material->emissionColor;

            if (hit.isInside && prevHit)
            {
                // do absorbtion
                // Beer's law
                const float dist = distance(hit.point, prevHit->point);
                return exp(-hit.material->absorbtion * dist) * color;
            }
            return color;
        }

        ColorRGBf combineFresnel(const Ray& ray, const ResolvedRaycastHit& hit, const ColorRGBf& refractionColor, const ColorRGBf& reflectionColor, const ColorRGBf& diffusionColor, bool inside = false) const
        {
            float n1 = airRefractiveIndex;
            float n2 = hit.material->refractiveIndex;
            if (inside) std::swap(n1, n2);
            const float fresnelEffect = fresnelReflectAmount(ray, hit.normal, hit.material->reflectivity, n1, n2);
            const ColorRGBf textureColor = hit.material->sampleTexture(hit.texCoords);

            return hit.material->surfaceColor * textureColor * (
                reflectionColor * (fresnelEffect)
                + refractionColor * ((1.0f - fresnelEffect) * hit.material->transparency)
                + diffusionColor);
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

        ColorRGBf computeDiffusionColor(const Ray& ray, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit) const
        {
            // TODO: handle transparency?
            if (!isDiffusive(*hit.material) && !hit.isInside) // if inside we could potentially do it wrong
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

        ColorRGBf computeReflectionColor(const Ray& ray, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            if (!isReflective(*hit.material) || depth > maxRayDepth)
                return {};

            const Normal3f reflectionDirection = reflection(ray.direction(), hit.normal);
            return trace(Ray(hit.point + reflectionDirection * paddingDistance, reflectionDirection), depth + 1, &hit, hit.isInside);
        }

        ColorRGBf computeRefractionColor(const Ray& ray, const ResolvedRaycastHit* prevHit, const ResolvedRaycastHit& hit, int depth) const
        {
            if (!isTransparent(*hit.material) || depth > maxRayDepth)
                return {};

            // outside->inside
            float eta = airRefractiveIndex / hit.material->refractiveIndex;
            if (hit.isInside) eta = 1.0f / eta;

            // do outside->inside refraction
            const Normal3f refractionDirection = refraction(ray.direction(), hit.normal, eta);
            const ColorRGBf color = trace(Ray(hit.point + refractionDirection * paddingDistance, refractionDirection), depth + 1, &hit, !hit.isInside);
            
            return color;
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