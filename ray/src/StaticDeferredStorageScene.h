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

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray, RaycastQueryStats* stats = nullptr) const
        {
            return m_storage.queryNearest(ray, stats);
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
        StaticSpacePartitionedStorageT m_storage;

        std::vector<LightHandle> m_lights;

        ColorRGBf m_backgroundColor;

        template <typename... SceneObjectCollectionsTs>
        void rememberLights(SceneObjectCollectionsTs&&... collections)
        {
            for_each(std::tie(collections...), [&](const auto& collection) {
                for (const auto& object : collection)
                {
                    if (object.isLight())
                    {
                        m_lights.emplace_back(object.shape().center(), object.id());
                    }
                }
            });
        }
    };
}
