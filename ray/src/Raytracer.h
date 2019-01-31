#pragma once

#include "Camera.h"
#include "Image.h"
#include "Scene.h"

namespace ray
{
    struct Raytracer
    {
        // Used to ensure for example that the hit point is
        // not considered obstructed by the shape it is on
        // due to floating point inaccuracies
        static constexpr float paddingDistance = 0.0001f;

        Raytracer(const Scene& scene) :
            m_scene(&scene)
        {

        }

        Image capture(const Camera& camera) const
        {
            Image img(camera.width(), camera.height());
            camera.forEachPixelRay([&img, this](const Ray& ray, int x, int y) {img(x, y) = traceResult(ray); });
            return img;
        }

    private:
        const Scene* m_scene;

        ColorRGBi traceResult(const Ray& ray) const
        {
            std::optional<ResolvedRaycastHit> hitOpt = m_scene->queryNearest(ray);
            if (hitOpt)
            {
                ResolvedRaycastHit& hit = *hitOpt;
                if (!isObstructedFromLight(hit.point + hit.normal * paddingDistance))
                {
                    return resolveColor(hit);
                }
            }

            return {};
        }

        bool isObstructedFromLight(const Point3f& point) const
        {
            Ray ray = Ray::between(point, m_scene->lightPosition());
            return m_scene->queryAny(ray).has_value();
        }

        ColorRGBi resolveColor(const ResolvedRaycastHit& hit) const
        {
            return ColorRGBi(hit.material->color);
        }
    };
}