#pragma once

#include "Camera.h"
#include "Image.h"
#include "Scene.h"

namespace ray
{
    struct Raytracer
    {
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
            return ColorRGBi(255, 0, 255);
        }
    };
}