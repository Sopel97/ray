#pragma once

#include "SceneObjectArray.h"
#include "ShapeTraits.h"
#include "Sphere.h"

#include <vector>
#include <tuple>

namespace ray
{
    struct Scene
    {
    private:
        // specialized whenever a pack is used
        template <typename ShapeT>
        struct ObjectStorage { using StorageType = SceneObjectArray<ShapeT>; };

        template <typename T>
        using ObjectStorageType = typename ObjectStorage<T>::StorageType;

    public:
        Scene()
        {

        }

        template <typename ShapeT>
        void add(const SceneObject<ShapeT>& so)
        {
            static_assert(ShapeTraits<ShapeT>::numShapes == 1, "Only singular shapes can be added to a scene.");

            objectsOfType<ShapeT>().add(so);
        }

        void setLightPosition(const Point3f& pos)
        {
            m_lightPosition = pos;
        }

        const Point3f& lightPosition() const
        {
            return m_lightPosition;
        }

        ResolvedRaycastHit castRay(const Ray& ray) const
        {
            // TODO:
        }

    private:
        std::tuple<
            ObjectStorageType<Sphere>
        > m_objects;

        Point3f m_lightPosition;

        template <typename ShapeT>
        ObjectStorageType<ShapeT>& objectsOfType()
        {
            return std::get<ObjectStorageType<ShapeT>>(m_objects);
        }
        
        template <typename ShapeT>
        const ObjectStorageType<ShapeT>& objectsOfType() const
        {
            return std::get<ObjectStorageType<ShapeT>>(m_objects);
        }
    };
}
