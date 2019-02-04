#pragma once

#include "Material.h"
#include "ShapeTraits.h"

#include <array>

namespace ray
{
    static std::uint64_t nextSceneObjectId;

    // Stores a shape and its material[s].
    template <typename ShapeT>
    struct SceneObject
    {
        using ShapePackType = ShapeT;
        using ShapeTraits = ShapeTraits<ShapePackType>;
        using BaseShapeType = typename ShapeTraits::BaseShapeType;
        static constexpr int numShapesInPack = ShapeTraits::numShapes;
        static constexpr bool isPack = numShapesInPack > 1;
        static constexpr int numMaterialsPerShape = ShapeTraits::numMaterialsPerShape;
        using MaterialStorageType = std::array<const Material*, numMaterialsPerShape>;

        static_assert(!isPack, "A single scene object must not be a pack. Use SceneObjectArray.");

        SceneObject(const ShapePackType& shape, const MaterialStorageType& materials) :
            m_shape(shape),
            m_materials(materials),
            m_id(nextSceneObjectId++)
        {

        }

        const ShapePackType& shape() const
        {
            return m_shape;
        }

        const Material& material(int i) const
        {
            return *(m_materials[i]);
        }

        const MaterialStorageType& materials() const
        {
            return m_materials;
        }

        bool isLight() const
        {
            for (const auto& mat : m_materials)
            {
                if (mat->emissionColor.total() > 0.0001f) return true;
            }
            return false;
        }

        std::uint64_t id() const
        {
            return m_id;
        }

    private:
        ShapePackType m_shape;
        MaterialStorageType m_materials;
        std::uint64_t m_id;
    };
}
