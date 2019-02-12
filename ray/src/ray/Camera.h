#pragma once

#include <ray/math/Angle.h>
#include <ray/math/Ray.h>
#include <ray/math/Vec3.h>

#include <ray/utility/IterableNumber.h>

#include <ray/Viewport.h>

#include <algorithm>
#include <cmath>
#include <execution>

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

        Viewport viewport() const
        {
            Viewport vp;

            vp.height = 2.0f * viewportDistance * m_fov.tan();
            vp.width = vp.height * aspectRatio();
            vp.widthPixels = m_width;
            vp.heightPixels = m_height;
            vp.pixelWidth = vp.width / static_cast<float>(vp.widthPixels);
            vp.pixelHeight = vp.height / static_cast<float>(vp.heightPixels);
            vp.right = cross(m_direction, m_up).normalized();
            vp.down = -cross(vp.right, m_direction).assumeNormalized(); // 2 orthogonal normals
            vp.center = m_position + viewportDistance * m_direction;
            vp.origin = m_position;

            // viewportWidth - pixelWidth to end up in the middle of a top left pixel
            vp.topLeft =
                vp.center
                - ((vp.width - vp.pixelWidth) / 2.0f * vp.right)
                - ((vp.height - vp.pixelHeight) / 2.0f * vp.down);

            return vp;
        }

        template <typename FuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachPixelRay(FuncT&& func, ExecT exec = ExecT{}) const
        {
            const Viewport vp = viewport();

            std::for_each_n(exec, IterableNumber(0), vp.heightPixels, [&](int y) {
                for (int x = 0; x < vp.widthPixels; ++x)
                {
                    func(vp.rayAt(static_cast<float>(x), static_cast<float>(y)), x, y);
                }
            });
        }

        float viewportHeight() const
        {
            return 2.0f * viewportDistance * m_fov.tan();
        }

        float viewportWidth() const
        {
            return viewportHeight() * aspectRatio();
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

        const Point3f& position() const
        {
            return m_position;
        }

        const Normal3f& direction() const
        {
            return m_direction;
        }

        const Normal3f& up() const
        {
            return m_up;
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
