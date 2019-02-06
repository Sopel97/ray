#pragma once

#include "RaycastHit.h"
#include "SceneObjectId.h"
#include "Material.h"
#include "ShapeTraits.h"
#include "TexCoords.h"
#include "Vec3.h"

#include <array>
#include <atomic>
#include <optional>

namespace ray
{
    struct Ray;
    struct ResolvableRaycastHit;

    struct UniqueAnyShape;
    struct SharedAnyShape;

    static inline std::atomic<SceneObjectId> nextSceneObjectId = 0;

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
            m_id(nextSceneObjectId.fetch_add(1))
        {

        }

        decltype(auto) center() const
        {
            return m_shape.center();
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

    // TODO: think how to merge the following cases nicely

    // Stores any shape in a polymorphic wrapper.
    // Only supports single shapes
    // Copy performs deep copy of the shape, material array, and id
    template <>
    struct SceneObject<UniqueAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            virtual Point3f center() const = 0;
            virtual std::optional<RaycastHit> raycast(const Ray& ray) const = 0;
            virtual std::unique_ptr<PolymorphicSceneObjectBase> clone() const = 0;
            virtual TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const = 0;
            virtual bool hasVolume() const = 0;
            virtual const Material& material(int materialNo) const = 0;
            virtual bool isLight() const = 0;
            virtual SceneObjectId id() const = 0;
        };

        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            using MaterialStorageType = std::array<const Material*, numMaterialsPerShape>;
            static_assert(!isPack, "Only a single object can be made polymorphic.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(nextSceneObjectId.fetch_add(1))
            {

            }

            Point3f center() const override
            {
                return m_shape.center();
            }
            std::optional<RaycastHit> raycast(const Ray& ray) const override
            {
                return ::ray::raycast(ray, m_shape);
            }
            std::unique_ptr<PolymorphicSceneObjectBase> clone() const override
            {
                return std::make_unique<PolymorphicSceneObject<ShapeT>>(*this);
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ::ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
            }
            bool hasVolume() const override
            {
                return ShapeTraits<ShapeT>::hasVolume;
            }
            const Material& material(int materialNo) const override
            {
                return *(m_materials[materialNo]);
            }
            bool isLight() const override
            {
                for (const auto& mat : m_materials)
                {
                    if (mat->emissionColor.total() > 0.0001f) return true;
                }
                return false;
            }
            SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SceneObjectId m_id;
        };

    public:
        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const std::array<const Material*, ShapeTraits<ShapeT>::numMaterialsPerShape>& materials) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(materials, shape))
        {

        }

        SceneObject(const SceneObject& other) :
            m_obj(other.m_obj->clone())
        {

        }

        SceneObjectId id() const
        {
            return m_obj->id();
        }

        Point3f center() const
        {
            return m_obj->center();
        }

        std::optional<RaycastHit> raycast(const Ray& ray) const
        {
            return m_obj->raycast(ray);
        }

        TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const
        {
            return m_obj->resolveTexCoords(hit, shapeInPackNo);
        }

        bool hasVolume() const
        {
            return m_obj->hasVolume();
        }

        const Material& material(int materialNo) const
        {
            return m_obj->material(materialNo);
        }

        bool isLight() const
        {
            return m_obj->isLight();
        }

    private:
        std::unique_ptr<PolymorphicSceneObjectBase> m_obj;
    };

    // Stores any shape in a polymorphic wrapper.
    // Only supports single shapes
    // A copy is shallow, ie. all data is shared.
    template <>
    struct SceneObject<SharedAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            virtual Point3f center() const = 0;
            virtual std::optional<RaycastHit> raycast(const Ray& ray) const = 0;
            virtual TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const = 0;
            virtual bool hasVolume() const = 0;
            virtual const Material& material(int materialNo) const = 0;
            virtual bool isLight() const = 0;
            virtual SceneObjectId id() const = 0;
        };

        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            using MaterialStorageType = std::array<const Material*, numMaterialsPerShape>;
            static_assert(!isPack, "Only a single object can be made polymorphic.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(nextSceneObjectId.fetch_add(1))
            {

            }

            Point3f center() const override
            {
                return m_shape.center();
            }
            std::optional<RaycastHit> raycast(const Ray& ray) const override
            {
                return ::ray::raycast(ray, m_shape);
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ::ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
            }
            bool hasVolume() const override
            {
                return ShapeTraits<ShapeT>::hasVolume;
            }
            const Material& material(int materialNo) const override
            {
                return *(m_materials[materialNo]);
            }
            bool isLight() const override
            {
                for (const auto& mat : m_materials)
                {
                    if (mat->emissionColor.total() > 0.0001f) return true;
                }
                return false;
            }
            SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SceneObjectId m_id;
        };

    public:
        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const std::array<const Material*, ShapeTraits<ShapeT>::numMaterialsPerShape>& materials) :
            m_obj(std::make_shared<PolymorphicSceneObject<ShapeT>>(materials, shape))
        {

        }

        SceneObjectId id() const
        {
            return m_obj->id();
        }

        Point3f center() const
        {
            return m_obj->center();
        }

        std::optional<RaycastHit> raycast(const Ray& ray) const
        {
            return m_obj->raycast(ray);
        }

        TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const
        {
            return m_obj->resolveTexCoords(hit, shapeInPackNo);
        }

        bool hasVolume() const
        {
            return m_obj->hasVolume();
        }

        const Material& material(int materialNo) const
        {
            return m_obj->material(materialNo);
        }

        bool isLight() const
        {
            return m_obj->isLight();
        }

    private:
        std::shared_ptr<PolymorphicSceneObjectBase> m_obj;
    };
}
