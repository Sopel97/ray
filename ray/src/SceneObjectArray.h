#pragma once

#include "Material.h"
#include "SceneObject.h"
#include "SceneObjectCollection.h"
#include "ShapeTraits.h"
#include "TextureCoordinateResolver.h"

#include <array>
#include <functional>
#include <vector>

namespace ray
{
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
        using IdStorageType = std::vector<std::uint64_t>;
        using ConstIterator = typename ShapeStorageType::const_iterator;

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

        const ShapeStorageType& shapePacks() const
        {
            return m_shapePacks;
        }

        const Material& material(int shapeNo, int materialNo) const
        {
            return *(m_materials[shapeNo][materialNo]);
        }

        std::uint64_t id(int shapeNo) const
        {
            return m_ids[shapeNo];
        }

        const MaterialStorageType& materials() const
        {
            return m_materials;
        }

        int size() const
        {
            return m_size;
        }

        ConstIterator begin() const
        {
            using std::begin;
            return begin(m_shapePacks);
        }

        ConstIterator end() const
        {
            using std::end;
            return end(m_shapePacks);
        }

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray) const
        {
            const int size = static_cast<int>(m_shapePacks.size());
            std::optional<RaycastHit> nearestHit{};
            float nearestHitDistance = std::numeric_limits<float>::max();
            int nearestHitPackNo{};
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
                RaycastHit& hit = *nearestHit;
                const int shapeNo = nearestHitPackNo * numShapesInPack + hit.shapeInPackNo;
                return resolveHitPartially(hit, nearestHitPackNo, shapeNo);
            }

            return std::nullopt;
        }

        std::optional<ResolvableRaycastHit> queryAny(const Ray& ray) const
        {
            const int size = static_cast<int>(m_shapePacks.size());
            for (int packNo = 0; packNo < size; ++packNo)
            {
                std::optional<RaycastHit> hitOpt = raycast(ray, m_shapePacks[packNo]);
                if (hitOpt)
                {
                    RaycastHit& hit = *hitOpt;
                    const int shapeNo = packNo * numShapesInPack + hit.shapeInPackNo;
                    return resolveHitPartially(hit, packNo, shapeNo);
                }
            }

            return std::nullopt;
        }

        std::optional<ResolvableRaycastHit> queryLocal(const Ray& ray, int shapeNo) const override
        {
            const int packNo = shapeNo / numShapesInPack;
            std::optional<RaycastHit> hitOpt = raycast(ray, m_shapePacks[packNo]);
            if (hitOpt)
            {
                RaycastHit& hit = *hitOpt;
                return resolveHitPartially(hit, packNo, shapeNo);
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

        SceneObjectId objectId(int shapeNo) const override
        {
            return id(shapeNo);
        }

    private:
        ShapeStorageType m_shapePacks;
        MaterialStorageType m_materials;
        IdStorageType m_ids;
        int m_size;

        ResolvableRaycastHit resolveHitPartially(const RaycastHit& hit, int packNo, int shapeNo) const
        {
            return ResolvableRaycastHit(hit.point, hit.normal, shapeNo, hit.materialNo, *this, hit.isInside);
        }
    };
}
