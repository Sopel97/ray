#pragma once

#include <ray/shape/Box3.h>
#include <ray/shape/Sphere.h>

namespace ray
{
    template <typename BvShapeT>
    struct BoundingVolume;

    template <>
    struct BoundingVolume<Box3>
    {
        template <typename ShapeT>
        static decltype(auto) get(const ShapeT& shape)
        {
            return shape.aabb();
        }
    };

    template <typename BvShapeT, typename ShapeT>
    decltype(auto) boundingVolume(const ShapeT& shape)
    {
        return BoundingVolume<BvShapeT>::get(shape);
    }

    template <typename BvShapeT, typename ShapeT>
    decltype(auto) boundingVolume(const SceneObject<ShapeT>& obj)
    {
        return BoundingVolume<BvShapeT>::get(obj.shape());
    }

    template <typename BvShapeT>
    decltype(auto) boundingVolume(const SceneObject<BoundedUniqueAnyShape>& obj)
    {
        return BoundingVolume<BvShapeT>::get(obj);
    }

    template <typename BvShapeT>
    decltype(auto) boundingVolume(const SceneObject<BoundedSharedAnyShape>& obj)
    {
        return BoundingVolume<BvShapeT>::get(obj);
    }
}
