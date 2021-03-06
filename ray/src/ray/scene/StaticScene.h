#pragma once

#include "Scene.h"
#include "SceneRaycastHit.h"

#include "object/RawSceneObjectBlob.h"
#include "object/SceneObjectBlob.h"

#include <ray/material/MediumMaterial.h>

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
        template <typename... ShapeTs, typename... ArgTs>
        StaticScene(const RawSceneObjectBlob<Shapes<ShapeTs...>>& collection, ArgTs&&... args) :
            m_storage(collection, std::forward<ArgTs>(args)...)
        {
            m_storage.gatherLights(m_lights);
        }

        template <typename... ShapeTs, typename... ArgTs>
        StaticScene(RawSceneObjectBlob<Shapes<ShapeTs...>>&& collection, ArgTs&&... args) :
            m_storage(std::move(collection), std::forward<ArgTs>(args)...)
        {
            m_storage.gatherLights(m_lights);
        }

        [[nodiscard]] bool queryNearest(const Ray& ray, ResolvableRaycastHit& hit) const override
        {
            return m_storage.queryNearest(ray, hit);
        }

        [[nodiscard]] const std::vector<LightHandle>& lights() const override
        {
            return m_lights;
        }

        [[nodiscard]] const ColorRGBf& backgroundColor() const override
        {
            return m_backgroundColor;
        }

        void setBackgroundColor(const ColorRGBf& color)
        {
            m_backgroundColor = color;
        }

        [[nodiscard]] float backgroundDistance() const override
        {
            return m_backgroundDistance;
        }

        void setBackgroundDistance(float d)
        {
            m_backgroundDistance = d;
        }

        void setMediumMaterial(const MediumMaterial* mediumMaterial)
        {
            m_mediumMaterial = mediumMaterial;
        }

        [[nodiscard]] const MediumMaterial* mediumMaterial() const override
        {
            return m_mediumMaterial;
        }

    private:
        StaticSpacePartitionedStorageT m_storage;

        std::vector<LightHandle> m_lights;

        ColorRGBf m_backgroundColor;
        float m_backgroundDistance;
        const MediumMaterial* m_mediumMaterial;
    };

    template <typename ShapesT>
    using StaticBlobScene = StaticScene<SceneObjectBlob<ShapesT>>;
}
