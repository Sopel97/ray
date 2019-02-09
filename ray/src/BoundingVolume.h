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
        static Box3 get(const SceneObject<UniqueAnyShape>& obj)
        {
            return obj.aabb();
        }
        static Box3 get(const SceneObject<SharedAnyShape>& obj)
        {
            return obj.aabb();
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
    decltype(auto) boundingVolume(const SceneObject<UniqueAnyShape>& obj)
    {
        return BoundingVolume<BvShapeT>::get(obj);
    }

    template <typename BvShapeT>
    decltype(auto) boundingVolume(const SceneObject<SharedAnyShape>& obj)
    {
        return BoundingVolume<BvShapeT>::get(obj);
    }
}
