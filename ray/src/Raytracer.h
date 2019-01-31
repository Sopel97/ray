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
            return Image(camera.width(), camera.height());
        }

    private:
        const Scene* m_scene;
    };
}