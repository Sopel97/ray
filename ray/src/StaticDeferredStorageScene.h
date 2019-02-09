#pragma once

#include "Raycast.h"
#include "RaycastHit.h"
#include "Scene.h"
#include "RawSceneObjectBlob.h"
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
        template <typename... ShapeTs>
        StaticDeferredStorageScene(const RawSceneObjectBlob<Shapes<ShapeTs...>>& collection) :
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
    using StaticBlobScene = StaticDeferredStorageScene<SceneObjectBlob<ShapesT>>;
}
