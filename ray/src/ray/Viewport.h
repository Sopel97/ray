#pragma once

#include <ray/math/Ray.h>

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
        Normal3f right;
        Normal3f down;
        Point3f center;
        Point3f topLeft;
        Point3f origin;

        Point3f at(float x, float y) const
        {
            return (topLeft
                   + x * pixelWidth * right
                   + y * pixelHeight * down);
        }

        Normal3f directionAt(float x, float y) const
        {
            return (at(x, y) - origin).normalized();
        }

        Ray rayAt(float x, float y) const
        {
            return Ray(origin, directionAt(x, y));
        }
    };
}
