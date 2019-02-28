#pragma once

#include "Angle2.h"

namespace ray
{
    template <typename T>
    struct ViewingFrustum3
    {
        Angle2<T> fovy;
        float aspect;
        float near;
        float far;

        ViewingFrustum3(const Angle2<T>& fovy, float aspect, float near, float far) :
            fovy(fovy),
            aspect(aspect),
            near(near),
            far(far)
        {
        }
    };

    using ViewingFrustum3f = ViewingFrustum3<float>;
}
