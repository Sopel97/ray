#pragma once

#include "RaycastHit.h"
#include "SceneObjectArray.h"
#include "ShapeTraits.h"
#include "SceneObjectStorage.h"
#include "Util.h"

#include <optional>
#include <type_traits>

namespace ray
{
    // No space partitioning. Useful for testing.
    // Uses one array for each shape type.
    // Is similar to BlobScene but behaves like SceneObjectStorage
    template <typename... ShapeTs>
    struct SceneObjectBlob : DynamicSceneObjectStorage
    {
    private:
        // specialized whenever a pack is used
        template <typename ShapeT>
        struct ObjectStorage { using StorageType = SceneObjectArray<ShapeT>; };

        template <typename T>
        using ObjectStorageType = typename ObjectStorage<T>::StorageType;

    public:
        template <typename... SceneObjectCollectionsTs>
        SceneObjectBlob(SceneObjectCollectionsTs&&... collections)
        {
            for_each(std::tie(collections...), [&](auto&& collection) {
                for (auto&& object : collection)
                {
                    objectsOfType<remove_cvref_t<decltype(object.shape())>>().add(std::forward<decltype(object)>(object));
                }
            });
        }

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray, RaycastQueryStats* stats = nullptr) const
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

    private:
        std::tuple<
            ObjectStorageType<ShapeTs>...
        > m_objects;

        template <typename ShapeT>
        ObjectStorageType<ShapeT>& objectsOfType()
        {
            return std::get<ObjectStorageType<ShapeT>>(m_objects);
        }
    };
}
