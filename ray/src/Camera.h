#pragma once

#include <cmath>
#include <iostream>

#include "Angle.h"
#include "Image.h"
#include "Ray.h"
#include "Vec3.h"

namespace ray
{
    struct Camera
    {
        constexpr static float viewportDistance = 1.0f;

        Camera(Image img, const Point3f& position, const Normal3f& direction, const Normal3f& up, Angle fov) :
            m_image(std::move(img)),
            m_position(position),
            m_direction(direction),
            m_up(up),
            m_fov(fov)
        {

        }

        template <typename FuncT>
        void forEachPixelRay(FuncT&& func)
        {
            const float a = aspectRatio();
            const float viewportHeight = 2.0f * viewportDistance * m_fov.tan();
            const float viewportWidth = viewportHeight * a;
            const int viewportWidthPixels = m_image.width();
            const int viewportHeightPixels = m_image.height();
            const float pixelWidth = viewportWidth / static_cast<float>(viewportWidthPixels);
            const float pixelHeight = viewportHeight / static_cast<float>(viewportHeightPixels);
            const Normal3f viewportRight = cross(m_direction, m_up).normalized();
            const Normal3f viewportDown = -cross(viewportRight, m_direction).assumeNormalized(); // 2 orthogonal normals
            const Point3f viewportCenter = m_position + viewportDistance * m_direction;

            // viewportWidth - pixelWidth to end up in the middle of a top left pixel
            const Point3f viewportTopLeft =
                viewportCenter 
                - ((viewportWidth - pixelWidth) / 2.0f * viewportRight) 
                - ((viewportHeight - pixelHeight) / 2.0f * viewportDown);

            for (int y = 0; y < viewportHeightPixels; ++y)
            {
                for (int x = 0; x < viewportWidthPixels; ++x)
                {
                    const Point3f viewportPoint =
                        (viewportTopLeft
                        + static_cast<float>(x) * pixelWidth * viewportRight
                        + static_cast<float>(y) * pixelHeight * viewportDown);

                    const Normal3f direction = (viewportPoint - m_position).normalized();

                    func(Ray(m_position, direction));
                }
            }
        }

        float aspectRatio() const
        {
            return static_cast<float>(m_image.width()) / static_cast<float>(m_image.height());
        }

    private:
        Image m_image;
        Point3f m_position;
        Normal3f m_direction;
        Normal3f m_up;
        Angle m_fov;
    };
}
