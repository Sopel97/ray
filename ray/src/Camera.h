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

        Camera(Image img, const Vec3f& position, const Vec3f& direction, const Vec3f& up, Angle fov) :
            m_image(std::move(img)),
            m_position(position),
            m_direction(direction.normalized()),
            m_up(up.normalized()),
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
            const Vec3f viewportRight = cross(m_direction, m_up).normalized();
            const Vec3f viewportDown = -cross(viewportRight, m_direction);
            const Vec3f viewportCenter = m_position + viewportDistance * m_direction;

            // viewportWidth - pixelWidth to end up in the middle of a top left pixel
            const Vec3f viewportTopLeft = 
                viewportCenter 
                - ((viewportWidth - pixelWidth) / 2.0f * viewportRight) 
                - ((viewportHeight - pixelHeight) / 2.0f * viewportDown);

            for (int y = 0; y < viewportHeightPixels; ++y)
            {
                for (int x = 0; x < viewportWidthPixels; ++x)
                {
                    const Vec3f viewportPoint =
                        (viewportTopLeft
                        + static_cast<float>(x) * pixelWidth * viewportRight
                        + static_cast<float>(y) * pixelHeight * viewportDown);

                    const Vec3f direction = viewportPoint - m_position;

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
        Vec3f m_position;
        Vec3f m_direction;
        Vec3f m_up;
        Angle m_fov;
    };
}
