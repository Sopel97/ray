#pragma once

#include "Angle.h"
#include "IterableNumber.h"
#include "Ray.h"
#include "Vec3.h"

#include <algorithm>
#include <cmath>
#include <execution>
#include <iostream>

namespace ray
{
    struct Camera
    {
        constexpr static float viewportDistance = 1.0f;

        Camera(const Point3f& position, const Normal3f& direction, const Normal3f& up, int width, int height, Angle fov) :
            m_position(position),
            m_direction(direction),
            m_up(up),
            m_width(width),
            m_height(height),
            m_fov(fov)
        {

        }

        template <typename FuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachPixelRay(FuncT&& func, ExecT exec = std::execution::seq) const
        {
            const float a = aspectRatio();
            const float viewportHeight = 2.0f * viewportDistance * m_fov.tan();
            const float viewportWidth = viewportHeight * a;
            const int viewportWidthPixels = m_width;
            const int viewportHeightPixels = m_height;
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

            std::for_each_n(exec, IterableNumber(0), viewportHeightPixels, [&](int y) {
                for (int x = 0; x < viewportWidthPixels; ++x)
                {
                    const Point3f viewportPoint =
                        (viewportTopLeft
                            + static_cast<float>(x) * pixelWidth * viewportRight
                            + static_cast<float>(y) * pixelHeight * viewportDown);

                    const Normal3f direction = (viewportPoint - m_position).normalized();

                    func(Ray(m_position, direction), x, y);
                }
            });
        }

        float aspectRatio() const
        {
            return static_cast<float>(m_width) / static_cast<float>(m_height);
        }

        int width() const
        {
            return m_width;
        }

        int height() const
        {
            return m_height;
        }

    private:
        Point3f m_position;
        Normal3f m_direction;
        Normal3f m_up;
        int m_width;
        int m_height;
        Angle m_fov;
    };
}
