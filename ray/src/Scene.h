#pragma once

#include "Raycast.h"
#include "RaycastHit.h"
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
            if(so.isLight())
            {
                m_lightPositions.emplace_back(so.shape().center());
                m_lightObjectIds.emplace_back(so.id());
            }
        }

        std::optional<ResolvedLocallyContinuableRaycastHit> queryNearest(const Ray& ray) const
        {
            std::optional<ResolvedLocallyContinuableRaycastHit> hitOpt = objectsOfType<Sphere>().queryNearest(ray);
            if (hitOpt) return hitOpt;

            // possibly other types

            return std::nullopt;
        }

        std::optional<ResolvedLocallyContinuableRaycastHit> queryAny(const Ray& ray) const
        {
            std::optional<ResolvedLocallyContinuableRaycastHit> hitOpt = objectsOfType<Sphere>().queryAny(ray);
            if (hitOpt) return hitOpt;

            // possibly other types

            return std::nullopt;
        }

        std::vector<ResolvedLocallyContinuableRaycastHit> queryVisibleLights(const Point3f& point) const
        {
            std::vector<ResolvedLocallyContinuableRaycastHit> visibleLights{};
            for (int i = 0; i < m_lightPositions.size(); ++i)
            {
                const Ray ray = Ray::between(point, m_lightPositions[i]);
                std::optional<ResolvedLocallyContinuableRaycastHit> hitOpt = objectsOfType<Sphere>().queryNearest(ray);
                if (hitOpt)
                {
                    ResolvedLocallyContinuableRaycastHit& hit = *hitOpt;
                    if (hit.objectId != m_lightObjectIds[i]) continue;
                    if (!isLight(*hit.material)) continue;
                    
                    // we hit something and it's exactly the light we were looking for
                    visibleLights.emplace_back(std::move(hit));
                }
            }

            // possibly other types

            return visibleLights;
        }

    private:
        std::tuple<
            ObjectStorageType<Sphere>
        > m_objects;

        std::vector<Point3f> m_lightPositions;
        std::vector<std::uint64_t> m_lightObjectIds;

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

        bool isLight(const Material& mat) const
        {
            return mat.emissionColor.total() > 0.0001f;
        }
    };
}
