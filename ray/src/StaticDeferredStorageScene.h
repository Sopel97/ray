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
    // Uses given space partitioning.
    template <typename StaticSpacePartitionedStorageT>
    struct StaticDeferredStorageScene : Scene
    {
    private:
    public:
        template <typename... SceneObjectCollectionsTs>
        StaticDeferredStorageScene(SceneObjectCollectionsTs&&... collections) :
            m_storage(std::forward<SceneObjectCollectionsTs>(collections)...)
        {
            rememberLights(std::forward<SceneObjectCollectionsTs>(collections)...);
        }

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray) const
        {
            return m_storage.queryNearest(ray);
        }

        std::vector<ResolvableRaycastHit> queryVisibleLights(const Point3f& point) const
        {
            std::vector<ResolvableRaycastHit> visibleLights{};
            for (int i = 0; i < m_lightPositions.size(); ++i)
            {
                const Ray ray = Ray::between(point, m_lightPositions[i]);
                std::optional<ResolvableRaycastHit> hitOpt = queryNearest(ray);
                if (hitOpt)
                {
                    ResolvableRaycastHit& hit = *hitOpt;
                    if (hit.objectId() != m_lightObjectIds[i]) continue;

                    // we hit something and it's exactly the light we were looking for
                    visibleLights.emplace_back(std::move(hit));
                }
            }

            return visibleLights;
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
        StaticSpacePartitionedStorageT m_storage;

        std::vector<Point3f> m_lightPositions;
        std::vector<SceneObjectId> m_lightObjectIds;

        ColorRGBf m_backgroundColor;

        template <typename... SceneObjectCollectionsTs>
        void rememberLights(SceneObjectCollectionsTs&&... collections)
        {
            for_each(std::tie(collections...), [&](const auto& collection) {
                for (const auto& object : collection)
                {
                    if (object.isLight())
                    {
                        m_lightPositions.emplace_back(object.shape().center());
                        m_lightObjectIds.emplace_back(object.id());
                    }
                }
            });
        }
    };
}
