#pragma once

#include <ray/math/Angle2.h>
#include <ray/math/Ray.h>
#include <ray/math/Vec2.h>
#include <ray/math/Vec3.h>

#include <ray/Viewport.h>

#include <algorithm>
#include <cmath>
#include <execution>

namespace ray
{
    struct Camera
    {
        constexpr static float viewportDistance = 1.0f;

        Camera(const Point3f& position, const UnitVec3f& direction, const UnitVec3f& up, int width, int height, Angle2f fov) noexcept :
            m_position(position),
            m_direction(direction),
            m_up(up),
            m_width(width),
            m_height(height),
            m_fov(fov)
        {

        }

        [[nodiscard]] Viewport viewport() const
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

            auto range = IntRange2(Point2i(vp.widthPixels, vp.heightPixels));
            std::for_each(exec, range.begin(), range.end(), [&](const Point2i& xyi) {
                auto[x, y] = xyi;
                func(vp.rayAt(static_cast<float>(x), static_cast<float>(y)), x, y);
            });
        }

        [[nodiscard]] float viewportHeight() const
        {
            return 2.0f * viewportDistance * m_fov.tan();
        }

        [[nodiscard]] float viewportWidth() const
        {
            return viewportHeight() * aspectRatio();
        }

        [[nodiscard]] float aspectRatio() const
        {
            return static_cast<float>(m_width) / static_cast<float>(m_height);
        }

        [[nodiscard]] int width() const
        {
            return m_width;
        }

        [[nodiscard]] int height() const
        {
            return m_height;
        }

        [[nodiscard]] const Point3f& position() const
        {
            return m_position;
        }

        [[nodiscard]] const UnitVec3f& direction() const
        {
            return m_direction;
        }

        [[nodiscard]] const UnitVec3f& up() const
        {
            return m_up;
        }

    private:
        Point3f m_position;
        UnitVec3f m_direction;
        UnitVec3f m_up;
        int m_width;
        int m_height;
        Angle2f m_fov;
    };
}
