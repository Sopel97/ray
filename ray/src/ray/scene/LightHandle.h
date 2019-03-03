#pragma once

#include "object/SceneObjectId.h"

#include <ray/math/Vec3.h>

namespace ray
{
    struct LightHandle
    {
        LightHandle(const Point3f& center, SceneObjectId id) noexcept :
            m_center(center),
            m_id(id)
        {

        }

        [[nodiscard]] const Point3f& center() const
        {
            return m_center;
        }

        [[nodiscard]] SceneObjectId id() const
        {
            return m_id;
        }

    private:
        Point3f m_center;
        SceneObjectId m_id;
    };
}
