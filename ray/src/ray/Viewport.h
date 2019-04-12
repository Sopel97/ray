#pragma once

#include <ray/math/Ray.h>
#include <ray/math/Vec2.h>

namespace ray
{
    struct Viewport
    {
        float width;
        float height;
        int widthPixels;
        int heightPixels;
        float pixelWidth;
        float pixelHeight;
        UnitVec3f right;
        UnitVec3f down;
        Point3f center;
        Point3f topLeft;
        Point3f origin;

        [[nodiscard]] Point3f at(const Point2f& coords) const
        {
            return (topLeft
                   + coords.x * pixelWidth * right
                   + coords.y * pixelHeight * down);
        }

        [[nodiscard]] UnitVec3f directionAt(const Point2f& coords) const
        {
            return (at(coords) - origin).normalized();
        }

        [[nodiscard]] Ray rayAt(const Point2f& coords) const
        {
            return Ray(origin, directionAt(coords));
        }
    };
}
