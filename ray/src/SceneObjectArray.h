#pragma once

#include "Material.h"
#include "RaycastQueryStats.h"
#include "SceneObject.h"
#include "SceneObjectCollection.h"
#include "ShapeTraits.h"
#include "TextureCoordinateResolver.h"

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
        struct PolymorphicSceneObjectArray : SceneObjectCollection
        {
            using ShapeStorageType = std::vector<SceneObject<AnyShapeT>>;

            PolymorphicSceneObjectArray()
            {

            }

            void add(const SceneObject<AnyShapeT>& so)
            {
                m_objects.emplace_back(so);
            }

            const Material& material(int shapeNo, int materialNo) const
            {
                return m_objects[shapeNo].material(materialNo);
            }

            SceneObjectId id(int shapeNo) const
            {
                return m_objects[shapeNo].id();
            }

            int size() const
            {
                return static_cast<int>(m_objects.size());
            }

            std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray, RaycastQueryStats* stats = nullptr) const
            {
                const int size = static_cast<int>(m_objects.size());
                std::optional<RaycastHit> nearestHit{};
                float nearestHitDistance = std::numeric_limits<float>::max();
                int nearestHitShapeNo{};
                if (stats) stats->numTests += size;
                for (int shapeNo = 0; shapeNo < size; ++shapeNo)
                {
                    std::optional<RaycastHit> hitOpt = m_objects[shapeNo].raycast(ray);
                    if (hitOpt)
                    {
                        RaycastHit& hit = *hitOpt;
                        const float dist = distance(ray.origin(), hit.point);
                        if (dist < nearestHitDistance)
                        {
                            nearestHit = hit;
                            nearestHitDistance = dist;
                            nearestHitShapeNo = shapeNo;
                        }
                    }
                }

                if (nearestHit)
                {
                    if (stats) ++stats->numHits;
                    RaycastHit& hit = *nearestHit;
                    return resolveHitPartially(hit, nearestHitShapeNo);
                }

                return std::nullopt;
            }

            std::optional<ResolvableRaycastHit> queryLocal(const Ray& ray, int shapeNo, RaycastQueryStats* stats = nullptr) const override
            {
                if (stats) ++stats->numTests;
                std::optional<RaycastHit> hitOpt = m_objects[shapeNo].raycast(ray);
                if (hitOpt)
                {
                    if (stats) ++stats->numHits;
                    RaycastHit& hit = *hitOpt;
                    return resolveHitPartially(hit, shapeNo);
                }

                return std::nullopt;
            }

            ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const
            {
                const auto& obj = m_objects[hit.shapeNo];
                const Material& mat = material(hit.shapeNo, hit.materialNo);
                const TexCoords texCoords = mat.texture ? m_objects[hit.shapeNo].resolveTexCoords(hit, 0) : TexCoords{ 0.0f, 0.0f };
                return ResolvedRaycastHit(hit.point, hit.normal, texCoords, hit.shapeNo, mat, *this, hit.isInside, obj.hasVolume());
            }

        private:
            ShapeStorageType m_objects;

            ResolvableRaycastHit resolveHitPartially(const RaycastHit& hit, int shapeNo) const
            {
                return ResolvableRaycastHit(hit.point, hit.normal, shapeNo, hit.materialNo, *this, hit.isInside);
            }
        };
    }

    // Analogical to SceneObject but shapePacks and materials are not interleaved.
    // So there are two separate arrays for shapePacks and materials
    // Handles packs by abstracting insertion and access to be done on with granularity of a single shape.
    template <typename ShapeT>
    struct SceneObjectArray : SceneObjectCollection
    {
        using ShapePackType = ShapeT;
        using ShapeTraits = ShapeTraits<ShapePackType>;
        using BaseShapeType = typename ShapeTraits::BaseShapeType;
        static constexpr int numShapesInPack = ShapeTraits::numShapes;
        static constexpr bool isPack = numShapesInPack > 1;
        static constexpr int numMaterialsPerShape = ShapeTraits::numMaterialsPerShape;
        static constexpr bool hasVolume = ShapeTraits::hasVolume;
        using ShapeStorageType = std::vector<ShapePackType>;
        using MaterialStorageType = std::vector<std::array<const Material*, numMaterialsPerShape>>;
        using IdStorageType = std::vector<SceneObjectId>;

        SceneObjectArray() :
            m_size(0)
        {

        }

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

        decltype(auto) shape(int shapeNo) const
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

        const Material& material(int shapeNo, int materialNo) const
        {
            return *(m_materials[shapeNo][materialNo]);
        }

        SceneObjectId id(int shapeNo) const
        {
            return m_ids[shapeNo];
        }

        int size() const
        {
            return m_size;
        }

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray, RaycastQueryStats* stats = nullptr) const
        {
            const int size = static_cast<int>(m_shapePacks.size());
            std::optional<RaycastHit> nearestHit{};
            float nearestHitDistance = std::numeric_limits<float>::max();
            int nearestHitPackNo{};
            if (stats) stats->numTests += size;
            for (int packNo = 0; packNo < size; ++packNo)
            {
                std::optional<RaycastHit> hitOpt = raycast(ray, m_shapePacks[packNo]);
                if (hitOpt)
                {
                    RaycastHit& hit = *hitOpt;
                    const float dist = distance(ray.origin(), hit.point);
                    if (dist < nearestHitDistance)
                    {
                        nearestHit = hit;
                        nearestHitDistance = dist;
                        nearestHitPackNo = packNo;
                    }
                }
            }

            if (nearestHit)
            {
                if (stats) ++stats->numHits;
                RaycastHit& hit = *nearestHit;
                const int shapeNo = nearestHitPackNo * numShapesInPack + hit.shapeInPackNo;
                return resolveHitPartially(hit, shapeNo);
            }

            return std::nullopt;
        }

        std::optional<ResolvableRaycastHit> queryLocal(const Ray& ray, int shapeNo, RaycastQueryStats* stats = nullptr) const override
        {
            if (stats) ++stats->numTests;
            const int packNo = shapeNo / numShapesInPack;
            std::optional<RaycastHit> hitOpt = raycast(ray, m_shapePacks[packNo]);
            if (hitOpt)
            {
                if (stats) ++stats->numHits;
                RaycastHit& hit = *hitOpt;
                return resolveHitPartially(hit, shapeNo);
            }

            return std::nullopt;
        }

        ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const
        {
            const int packNo = hit.shapeNo / numShapesInPack;
            const int shapeInPackNo = hit.shapeNo % numShapesInPack;
            const Material& mat = material(hit.shapeNo, hit.materialNo);
            const TexCoords texCoords = mat.texture ? resolveTexCoords(m_shapePacks[packNo], hit, shapeInPackNo) : TexCoords{ 0.0f, 0.0f };
            return ResolvedRaycastHit(hit.point, hit.normal, texCoords, hit.shapeNo, mat, *this, hit.isInside, hasVolume);
        }

    private:
        ShapeStorageType m_shapePacks;
        MaterialStorageType m_materials;
        IdStorageType m_ids;
        int m_size;

        ResolvableRaycastHit resolveHitPartially(const RaycastHit& hit, int shapeNo) const
        {
            return ResolvableRaycastHit(hit.point, hit.normal, shapeNo, hit.materialNo, *this, hit.isInside);
        }
    };

    // only the underlying scene object's structure changes
    template <>
    struct SceneObjectArray<UniqueAnyShape> : detail::PolymorphicSceneObjectArray<UniqueAnyShape> {};
    template <>
    struct SceneObjectArray<SharedAnyShape> : detail::PolymorphicSceneObjectArray<SharedAnyShape> {};
}
