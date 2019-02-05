#pragma once

#include "Material.h"
#include "ShapeTraits.h"

#include <array>
#include <atomic>

namespace ray
{
    using SceneObjectId = std::uint64_t;

    static inline std::atomic<SceneObjectId> nextSceneObjectId = 0;

    template <typename ShapeT, bool IsPolymorphicV = ShapeTraits<ShapeT>::isPolymorphic>
    struct SceneObject;

    // Stores a shape and its material[s].
    template <typename ShapeT>
    struct SceneObject<ShapeT, false>
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
            m_id(nextSceneObjectId.fetch_add(1))
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

        SceneObjectId id() const
        {
            return m_id;
        }

    private:
        ShapePackType m_shape;
        MaterialStorageType m_materials;
        SceneObjectId m_id;
    };

    // Stores a polymorphic shape
    // The polymorphic shape has to store its materials (because the count may vary).
    // Only supports single shapes
    template <typename ShapeT>
    struct SceneObject<ShapeT, true>
    {
        using ShapeType = ShapeT;
        using BaseShapeType = typename ShapeTraits<ShapeType>::BaseShapeType;
        static constexpr int numShapesInPack = ShapeTraits<ShapeType>::numShapes;
        static constexpr bool isPack = numShapesInPack > 1;

        static_assert(!isPack, "A single scene object must not be a pack. Use SceneObjectArray.");

        template <typename PolyShapeT>
        SceneObject(const PolyShapeT& shape, const std::array<const Material*, ShapeTraits<PolyShapeT>::numMaterialsPerShape>& materials) :
            m_shape(shape, materials),
            m_id(nextSceneObjectId.fetch_add(1))
        {

        }

        const ShapeType& shape() const
        {
            return m_shape;
        }

        const Material& material(int i) const
        {
            return m_shape.material(i);
        }

        bool isLight() const
        {
            return m_shape.isLight();
        }

        SceneObjectId id() const
        {
            return m_id;
        }

    private:
        ShapeType m_shape;
        SceneObjectId m_id;
    };
}
