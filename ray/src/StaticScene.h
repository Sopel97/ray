#pragma once

#include "RawSceneObjectBlob.h"
#include "RaycastHit.h"
#include "Scene.h"

#include <tuple>
#include <vector>

namespace ray
{
    // Uses given space partitioning.
    template <typename StaticSpacePartitionedStorageT>
    struct StaticScene : Scene
    {
    private:
    public:
        // TODO: think how to do a move from RawSceneObjectBlob.
        //       problematic since we first have to populate storage
        //       and later get lights
        template <typename... ShapeTs>
        StaticScene(const RawSceneObjectBlob<Shapes<ShapeTs...>>& collection) :
            m_storage(collection)
        {
            rememberLights(collection);
        }

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray) const
        {
            return m_storage.queryNearest(ray);
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

        template <typename... ShapeTs>
        void rememberLights(const RawSceneObjectBlob<Shapes<ShapeTs...>>& collection)
        {
            collection.forEach([&](const auto& object) {
                if (object.isLight())
                {
                    m_lights.emplace_back(object.center(), object.id());
                }
            });
        }
    };

    template <typename ShapesT>
    using StaticBlobScene = StaticScene<SceneObjectBlob<ShapesT>>;
}
