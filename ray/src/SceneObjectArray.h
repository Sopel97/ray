#pragma once

#include "Material.h"
#include "SceneObject.h"
#include "ShapeTraits.h"

#include <array>
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
                ++m_size;
            }
            else
            {
                m_shapes.emplace_back(so.shape());
                m_materials.emplace_back(so.materials());
                ++m_size;
            }
        }

        const ShapeType& shape(int shapeNo) const
        {
            if constexpr (isPack)
            {
                // TODO: can't just return a reference
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

        const MaterialStorageType& materials() const
        {
            return m_materials;
        }

        int size() const
        {
            return m_size;
        }

    private:
        ShapeStorageType m_shapes;
        MaterialStorageType m_materials;
        int m_size;
    };
}