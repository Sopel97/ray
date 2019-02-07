#pragma once

#include "Raycast.h"
#include "RaycastHit.h"
#include "Scene.h"
#include "SceneObjectArray.h"
#include "ShapeTraits.h"
#include "Sphere.h"
#include "Util.h"

#include <tuple>
#include <type_traits>
#include <vector>

namespace ray
{
    // Holds all objects in one array [per object type].
    // No space partitioning.
    template <typename... ShapeTs>
    struct BlobScene : Scene
    {
        // specialized whenever a pack is used
        template <typename ShapeT>
        struct ObjectStorage { using StorageType = SceneObjectArray<ShapeT>; };

        template <typename T>
        using ObjectStorageType = typename ObjectStorage<T>::StorageType;

    public:
        BlobScene()
        {

        }

        template <typename ShapeT>
        void add(const SceneObject<ShapeT>& so)
        {
            objectsOfType<ShapeT>().add(so);
            if(so.isLight())
            {
                m_lights.emplace_back(so.center(), so.id());
            }
        }

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray, RaycastQueryStats* stats = nullptr) const override
        {
            std::optional<ResolvableRaycastHit> hitOpt = std::nullopt;
            float minDist = std::numeric_limits<float>::max();
            for_each(m_objects, [&](const auto& objects) {
                if (objects.size() > 0)
                {
                    std::optional<ResolvableRaycastHit> hitOptNow = objects.queryNearest(ray, stats);
                    if (hitOptNow)
                    {
                        const float dist = distance(hitOptNow->point, ray.origin());
                        if (dist < minDist)
                        {
                            minDist = dist;
                            hitOpt = std::move(hitOptNow);
                        }
                    }
                }
            });
            if (hitOpt)
            {
                if (stats) ++stats->numUsed;
                return hitOpt;
            }

            return std::nullopt;
        }

        const std::vector<LightHandle>& lights() const override
        {
            return m_lights;
        }

        const ColorRGBf& backgroundColor() const
        {
            return m_backgroundColor;
        }

        void setBackgroundColor(const ColorRGBf& color)
        {
            m_backgroundColor = color;
        }

    private:
        std::tuple<
            ObjectStorageType<ShapeTs>...
        > m_objects;

        std::vector<LightHandle> m_lights;

        ColorRGBf m_backgroundColor;

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
