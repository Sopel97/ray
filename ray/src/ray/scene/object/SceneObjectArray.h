#pragma once

#include "SceneObjectCollection.h"
#include "SceneObject.h"

#include <ray/material/Material.h>
#include <ray/material/SurfaceShader.h>

#include <ray/math/Ray.h>
#include <ray/math/RaycastHit.h>
#include <ray/math/TextureCoordinateResolver.h>

#include <ray/scene/LightHandle.h>
#include <ray/scene/SceneRaycastHit.h>

#include <ray/shape/ShapeTraits.h>

#include <array>
#include <functional>
#include <vector>

namespace ray
{
    namespace detail
    {
        // Specialization for polymorphic shapes
        // We can't do better than just store an array of SceneObject<PolyShapeType>
        // Only supports single shapes as base shape
        template <typename AnyShapeT>
        struct PolymorphicSceneObjectArray : HomogeneousSceneObjectCollection
        {
            using ShapeStorageType = std::vector<SceneObject<AnyShapeT>>;

            PolymorphicSceneObjectArray() noexcept
            {

            }

            PolymorphicSceneObjectArray(const PolymorphicSceneObjectArray&) = default;
            PolymorphicSceneObjectArray(PolymorphicSceneObjectArray&&) noexcept = default;
            PolymorphicSceneObjectArray& operator=(const PolymorphicSceneObjectArray&) = default;
            PolymorphicSceneObjectArray& operator=(PolymorphicSceneObjectArray&&) noexcept = default;

            void add(const SceneObject<AnyShapeT>& so)
            {
                m_objects.emplace_back(so);
            }

            void add(SceneObject<AnyShapeT>&& so)
            {
                m_objects.emplace_back(std::move(so));
            }

            [[nodiscard]] MaterialPtrStorageView materialsView(int shapeNo) const
            {
                return m_objects[shapeNo].materialsView();
            }

            [[nodiscard]] SceneObjectId id(int shapeNo) const
            {
                return m_objects[shapeNo].id();
            }

            [[nodiscard]] int size() const
            {
                return static_cast<int>(m_objects.size());
            }

            [[nodiscard]] bool queryNearest(const Ray& ray, ResolvableRaycastHit& hit) const
            {
                const int size = static_cast<int>(m_objects.size());
                bool anyHit = false;
                for (int shapeNo = 0; shapeNo < size; ++shapeNo)
                {
                    if (m_objects[shapeNo].raycast(ray, hit))
                    {
                        anyHit = true;
                        hit.shapeNo = shapeNo;
                    }
                }

                if (anyHit)
                {
                    hit.owner = this;
                }

                return anyHit;
            }

            [[nodiscard]] bool queryLocal(const Ray& ray, int shapeNo, ResolvableRaycastHit& hit) const override
            {
                if (m_objects[shapeNo].raycast(ray, hit))
                {
                    hit.shapeNo = shapeNo;
                    hit.owner = this;
                    return true;
                }

                return false;
            }

            [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const
            {
                const auto& obj = m_objects[hit.shapeNo];
                ResolvedRaycastHit ret = obj.resolveHit(hit);
                ret.owner = this;
                return ret;
            }

        private:
            ShapeStorageType m_objects;
        };
    }

    // Analogical to SceneObject but shapePacks and materialsView are not interleaved.
    // So there are two separate arrays for shapePacks and materialsView
    // Handles packs by abstracting insertion and access to be done on with granularity of a single shape.
    template <typename ShapeT>
    struct SceneObjectArray : HomogeneousSceneObjectCollection
    {
        using ShapePackType = ShapeT;
        using ShapeTraits = ShapeTraits<ShapePackType>;
        using BaseShapeType = typename ShapeTraits::BaseShapeType;
        static constexpr int numShapesInPack = ShapeTraits::numShapes;
        static constexpr bool isPack = numShapesInPack > 1;
        static constexpr bool hasVolume = ShapeTraits::hasVolume;
        static constexpr bool isBounded = ShapeTraits::isBounded;
        static constexpr bool isLocallyContinuable = ShapeTraits::isLocallyContinuable;
        using ShapeStorageType = std::vector<ShapePackType>;
        using MaterialStorageType = std::vector<MaterialPtrStorageType<BaseShapeType>>;
        using IdStorageType = std::vector<SceneObjectId>;

        SceneObjectArray() noexcept :
            m_size(0)
        {

        }

        SceneObjectArray(const SceneObjectArray&) = default;
        SceneObjectArray(SceneObjectArray&&) noexcept = default;
        SceneObjectArray& operator=(const SceneObjectArray&) = default;
        SceneObjectArray& operator=(SceneObjectArray&&) noexcept = default;

        void add(const SceneObject<BaseShapeType>& so)
        {
            if constexpr (isPack)
            {
                const int subindex = m_size % numShapesInPack;
                ShapePackType& pack = subindex == 0 ? m_shapePacks.emplace_back() : m_shapePacks.back();
                pack.set(subindex, so.shape());
                m_materials.emplace_back(so.materials());
                m_ids.emplace_back(so.id());
                ++m_size;
            }
            else
            {
                m_shapePacks.emplace_back(so.shape());
                m_materials.emplace_back(so.materials());
                m_ids.emplace_back(so.id());
                ++m_size;
            }
        }

        [[nodiscard]] decltype(auto) shape(int shapeNo) const
        {
            if constexpr (isPack)
            {
                // should return a proxy reference
                return m_shapePacks[shapeNo / numShapesInPack].get(shapeNo % numShapesInPack);
            }
            else
            {
                return m_shapePacks[shapeNo];
            }
        }

        [[nodiscard]] MaterialPtrStorageView materialsView(int shapeNo) const
        {
            return m_materials[shapeNo].view();
        }

        [[nodiscard]] SceneObjectId id(int shapeNo) const
        {
            return m_ids[shapeNo];
        }

        [[nodiscard]] int size() const
        {
            return m_size;
        }

        [[nodiscard]] bool queryNearest(const Ray& ray, ResolvableRaycastHit& hit) const
        {
            const int size = static_cast<int>(m_shapePacks.size());
            int nearestHitPackNo{};
            bool anyHit = false;
            for (int packNo = 0; packNo < size; ++packNo)
            {
                if (raycast(ray, m_shapePacks[packNo], hit))
                {
                    anyHit = true;
                    nearestHitPackNo = packNo;
                }
            }

            if (anyHit)
            {
                const int shapeNo = nearestHitPackNo * numShapesInPack + hit.shapeInPackNo;
                hit.shapeNo = shapeNo;
                hit.owner = this;
            }

            return anyHit;
        }

        [[nodiscard]] bool queryLocal(const Ray& ray, int shapeNo, ResolvableRaycastHit& hit) const override
        {
            const int packNo = shapeNo / numShapesInPack;
            if (raycast(ray, m_shapePacks[packNo], hit))
            {
                hit.shapeNo = shapeNo;
                hit.owner = this;
                return true;
            }

            return false;
        }

        [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const
        {
            /*
            const int packNo = hit.shapeNo / numShapesInPack;
            const int shapeInPackNo = hit.shapeNo % numShapesInPack;
            auto[surface, medium] = materialsView(hit.shapeNo).material(hit.materialIndex);
            const TexCoords texCoords = surface->texture ? resolveTexCoords(m_shapePacks[packNo], hit, shapeInPackNo) : TexCoords{ 0.0f, 0.0f };
            return ResolvedRaycastHit(hit.dist, hit.point, hit.normal, texCoords, hit.shapeNo, surface, medium, *this, hit.isInside, hasVolume, isLocallyContinuable);
            */

            const int packNo = hit.shapeNo / numShapesInPack;
            const int shapeInPackNo = hit.shapeNo % numShapesInPack;
            auto[surface, medium] = materialsView(hit.shapeNo).material(hit.materialIndex);
            return ResolvedRaycastHit(
                DefaultSurfaceShader<BaseShapeType>::instance().shade(shape(hit.shapeNo), hit, materialsView(hit.shapeNo)), 
                hit.shapeNo, medium, this, hit.isInside, hasVolume, isLocallyContinuable
            );
        }

        void gatherLights(std::vector<LightHandle>& lights) const
        {
            if constexpr (isBounded)
            {
                for (int i = 0; i < m_size; ++i)
                {
                    if (m_materials[i].isEmissive())
                    {
                        lights.emplace_back(shape(i).center(), m_ids[i]);
                    }
                }
            }
        }

    private:
        ShapeStorageType m_shapePacks;
        MaterialStorageType m_materials;
        IdStorageType m_ids;
        int m_size;
    };

    // only the underlying scene object's structure changes
    template <>
    struct SceneObjectArray<BoundedUniqueAnyShape> : detail::PolymorphicSceneObjectArray<BoundedUniqueAnyShape> {};
    template <>
    struct SceneObjectArray<BoundedSharedAnyShape> : detail::PolymorphicSceneObjectArray<BoundedSharedAnyShape> {};
    template <>
    struct SceneObjectArray<UnboundedUniqueAnyShape> : detail::PolymorphicSceneObjectArray<UnboundedUniqueAnyShape> {};
    template <>
    struct SceneObjectArray<UnboundedSharedAnyShape> : detail::PolymorphicSceneObjectArray<UnboundedSharedAnyShape> {};

    template <>
    struct SceneObjectArray<CsgShape> : HomogeneousSceneObjectCollection
    {
        using ShapeStorageType = std::vector<SceneObject<CsgShape>>;
        using ShapeType = SceneObject<CsgShape>::CsgPrimitiveBase;
        using ShapePtrType = const ShapeType*;

        SceneObjectArray()
        {

        }

        SceneObjectArray(const SceneObjectArray&) = default;
        SceneObjectArray(SceneObjectArray&&) noexcept = default;
        SceneObjectArray& operator=(const SceneObjectArray&) = default;
        SceneObjectArray& operator=(SceneObjectArray&&) noexcept = default;

        void add(const SceneObject<CsgShape>& so)
        {
            m_objects.emplace_back(so);
        }

        void add(SceneObject<CsgShape>&& so)
        {
            m_objects.emplace_back(std::move(so));
        }

        [[nodiscard]] int size() const
        {
            return static_cast<int>(m_objects.size());
        }

        [[nodiscard]] SceneObjectId id(int shapeNo) const
        {
            return m_objects[shapeNo].id();
        }

        [[nodiscard]] bool queryNearest(const Ray& ray, ResolvableRaycastHit& hit) const
        {
            const int size = static_cast<int>(m_objects.size());
            bool anyHit = false;
            for (int shapeNo = 0; shapeNo < size; ++shapeNo)
            {
                if (ShapePtrType shapePtr = m_objects[shapeNo].raycast(ray, hit))
                {
                    anyHit = true;
                    hit.shapeNo = shapeNo;
                    hit.additionalData = static_cast<const void*>(shapePtr);
                }
            }

            if (anyHit)
            {
                hit.owner = this;
            }

            return anyHit;
        }

        [[nodiscard]] bool queryLocal(const Ray& ray, int shapeNo, ResolvableRaycastHit& hit) const override
        {
            if (ShapePtrType shapePtr = m_objects[shapeNo].raycast(ray, hit))
            {
                hit.shapeNo = shapeNo;
                hit.owner = this;
                hit.additionalData = static_cast<const void*>(shapePtr);
                return true;
            }

            return false;
        }

        [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const
        {
            ShapePtrType obj = static_cast<ShapePtrType>(hit.additionalData);
            ResolvedRaycastHit ret = obj->resolveHit(hit);
            ret.owner = this;
            return ret;
        }

        void gatherLights(std::vector<LightHandle>& lights) const
        {
            for (const auto& obj : m_objects)
            {
                if (obj.isLight())
                {
                    lights.emplace_back(obj.center(), obj.id());
                }
            }
        }

    private:
        ShapeStorageType m_objects;
    };
}
