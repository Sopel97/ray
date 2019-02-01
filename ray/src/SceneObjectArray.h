#pragma once

#include "Material.h"
#include "SceneObject.h"
#include "ShapeTraits.h"

#include <array>
#include <functional>
#include <vector>

namespace ray
{
    // Analogical to SceneObject but shapes and materials are not interleaved.
    // So there are two separate arrays for shapes and materials
    // Handles packs by abstracting insertion and access to be done on with granularity of a single shape.
    template <typename ShapeT>
    struct SceneObjectArray
    {
        using ShapeType = ShapeT;
        using ShapeTraits = ShapeTraits<ShapeType>;
        using BaseShapeType = typename ShapeTraits::BaseShapeType;
        static constexpr int numShapesInPack = ShapeTraits::numShapes;
        static constexpr bool isPack = numShapesInPack > 1;
        static constexpr int numMaterialsPerShape = ShapeTraits::numMaterialsPerShape;
        using ShapeStorageType = std::vector<ShapeType>;
        using MaterialStorageType = std::vector<std::array<Material*, numMaterialsPerShape>>;
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
                ShapeType& pack = subindex == 0 ? m_shapes.emplace_back() : m_shapes.back();
                pack.set(subindex, so.shape());
                m_materials.emplace_back(so.materials());
                m_ids.emplace_back(so.id());
                ++m_size;
            }
            else
            {
                m_shapes.emplace_back(so.shape());
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
                return m_shapes[shapeNo / numShapesInPack].get(shapeNo % numShapesInPack);
            }
            else
            {
                return m_shapes[shapeNo];
            }
        }

        const ShapeStorageType& shapes() const
        {
            return m_shapes;
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
            return begin(m_shapes);
        }

        ConstIterator end() const
        {
            using std::end;
            return end(m_shapes);
        }

        std::optional<ResolvedLocallyContinuableRaycastHit> queryNearest(const Ray& ray) const
        {
            const int size = static_cast<int>(m_shapes.size());
            std::optional<RaycastHit> nearestHit{};
            float nearestHitDistance = std::numeric_limits<float>::max();
            int nearestHitPackNo{};
            for (int packNo = 0; packNo < size; ++packNo)
            {
                std::optional<RaycastHit> hitOpt = raycastOutside(ray, m_shapes[packNo]);
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
                const int shapeNo = nearestHitPackNo * numShapesInPack + hit.shapeNo;
                return ResolvedLocallyContinuableRaycastHit(hit.point, hit.normal, material(shapeNo, hit.materialNo), id(shapeNo), createLocalRaycaster(shapeNo));
            }

            return std::nullopt;
        }

        std::optional<ResolvedLocallyContinuableRaycastHit> queryAny(const Ray& ray) const
        {
            const int size = static_cast<int>(m_shapes.size());
            for (int packNo = 0; packNo < size; ++packNo)
            {
                std::optional<RaycastHit> hitOpt = raycastOutside(ray, m_shapes[packNo]);
                if (hitOpt)
                {
                    RaycastHit& hit = *hitOpt;
                    const int shapeNo = packNo * numShapesInPack + hit.shapeNo;
                    return ResolvedLocallyContinuableRaycastHit(hit.point, hit.normal, material(shapeNo, hit.materialNo), id(shapeNo), createLocalRaycaster(shapeNo));
                }
            }

            return std::nullopt;
        }

        std::optional<ResolvedRaycastHit> queryInsideSpecificShape(const Ray& ray, int shapeNo) const
        {
            const int packNo = shapeNo / numShapesInPack;
            std::optional<RaycastHit> hitOpt = raycastInside(ray, m_shapes[packNo]);
            if (hitOpt)
            {
                RaycastHit& hit = *hitOpt;
                return ResolvedRaycastHit(hit.point, hit.normal, material(shapeNo, hit.materialNo), id(shapeNo));
            }

            return std::nullopt;
        }

    private:
        ShapeStorageType m_shapes;
        MaterialStorageType m_materials;
        IdStorageType m_ids;
        int m_size;

        std::function<std::optional<ResolvedRaycastHit>(const ResolvedLocallyContinuableRaycastHit& prevHit, const Normal3f&)> createLocalRaycaster(int shapeNo) const
        {
            return [this, shapeNo](const ResolvedLocallyContinuableRaycastHit& prevHit, const Normal3f& direction) {
                return this->queryInsideSpecificShape(Ray(prevHit.point, direction), shapeNo);
            };
        }
    };
}
