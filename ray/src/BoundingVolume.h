#pragma once

#include "Box3.h"
#include "Sphere.h"

namespace ray
{
    template <typename BvShapeT>
    struct BoundingVolume;

    template <>
    struct BoundingVolume<Box3>
    {
        static Box3 get(const Sphere& sphere)
        {
            return sphere.aabb();
        }
        static const Box3& get(const Box3& box)
        {
            return box.aabb();
        }
    };

    template <typename BvShapeT, typename ShapeT>
    decltype(auto) boundingVolume(const ShapeT& shape)
    {
        return BoundingVolume<BvShapeT>::get(shape);
    }
}
