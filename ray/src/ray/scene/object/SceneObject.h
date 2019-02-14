#pragma once

#include "SceneObjectId.h"

#include <ray/material/Material.h>
#include <ray/material/TexCoords.h>

#include <ray/math/RaycastHit.h>
#include <ray/math/Vec3.h>

#include <ray/scene/SceneRaycastHit.h>

#include <ray/shape/Box3.h>
#include <ray/shape/ShapeTraits.h>

#include <array>
#include <atomic>
#include <memory>
#include <optional>

namespace ray
{
    template <typename ShapeT>
    using MaterialArrayType = std::array<const Material*, ShapeTraits<ShapeT>::numMaterialsPerShape>;

    struct Ray;
    struct ResolvableRaycastHit;

    struct BoundedUniqueAnyShape;
    struct BoundedSharedAnyShape;
    struct UnboundedUniqueAnyShape;
    struct UnboundedSharedAnyShape;

    namespace detail
    {
        static inline std::atomic<SceneObjectId> gNextSceneObjectId = 0;
    }

    // Stores a shape and its material[s].
    template <typename ShapeT>
    struct SceneObject
    {
        using ShapeType = ShapeT;
        using ShapeTraits = ShapeTraits<ShapeType>;
        using BaseShapeType = typename ShapeTraits::BaseShapeType;
        static constexpr int numShapesInPack = ShapeTraits::numShapes;
        static constexpr bool isPack = numShapesInPack > 1;
        static constexpr int numMaterialsPerShape = ShapeTraits::numMaterialsPerShape;
        using MaterialStorageType = MaterialArrayType<ShapeType>;

        static_assert(!isPack, "A single scene object must not be a pack. Use SceneObjectArray.");

        SceneObject(const ShapeType& shape, const MaterialStorageType& materials) :
            m_shape(shape),
            m_materials(materials),
            m_id(detail::gNextSceneObjectId.fetch_add(1))
        {

        }

        decltype(auto) center() const
        {
            return m_shape.center();
        }

        decltype(auto) aabb() const
        {
            return m_shape.aabb();
        }

        const MaterialStorageType& materials() const
        {
            return m_materials;
        }

        const ShapeType& shape() const
        {
            return m_shape;
        }

        const Material& material(int i) const
        {
            return *(m_materials[i]);
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
        ShapeType m_shape;
        MaterialStorageType m_materials;
        SceneObjectId m_id;
    };

    // TODO: think how to merge the following cases nicely

    // Stores any shape in a polymorphic wrapper.
    // Only supports single shapes
    // Copy performs deep copy of the shape, material array, and id
    template <>
    struct SceneObject<BoundedUniqueAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            virtual Point3f center() const = 0;
            virtual std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const = 0;
            virtual std::unique_ptr<PolymorphicSceneObjectBase> clone() const = 0;
            virtual TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const = 0;
            virtual bool hasVolume() const = 0;
            virtual Box3 aabb() const = 0;
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
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            using MaterialStorageType = MaterialArrayType<ShapeT>;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(isBounded, "Must be bounded.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }

            Point3f center() const override
            {
                return m_shape.center();
            }
            Box3 aabb() const override
            {
                return m_shape.aabb();
            }
            std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const override
            {
                return ray::raycast(ray, m_shape, tNearest);
            }
            std::unique_ptr<PolymorphicSceneObjectBase> clone() const override
            {
                return std::make_unique<PolymorphicSceneObject<ShapeT>>(*this);
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
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
        using ShapeType = BoundedUniqueAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialArrayType<ShapeT>& materials) :
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

        Box3 aabb() const
        {
            return m_obj->aabb();
        }

        std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const
        {
            return m_obj->raycast(ray, tNearest);
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
    struct SceneObject<BoundedSharedAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            virtual Point3f center() const = 0;
            virtual std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const = 0;
            virtual TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const = 0;
            virtual bool hasVolume() const = 0;
            virtual Box3 aabb() const = 0;
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
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            using MaterialStorageType = MaterialArrayType<ShapeT>;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(isBounded, "Must be bounded.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }

            Point3f center() const override
            {
                return m_shape.center();
            }
            Box3 aabb() const override
            {
                return m_shape.aabb();
            }
            std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const override
            {
                return ray::raycast(ray, m_shape, tNearest);
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
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
        using ShapeType = BoundedSharedAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialArrayType<ShapeT>& materials) :
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

        Box3 aabb() const
        {
            return m_obj->aabb();
        }

        std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const
        {
            return m_obj->raycast(ray, tNearest);
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


    // Stores any shape in a polymorphic wrapper.
    // Only supports single shapes
    // Copy performs deep copy of the shape, material array, and id
    template <>
    struct SceneObject<UnboundedUniqueAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            virtual std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const = 0;
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
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            using MaterialStorageType = MaterialArrayType<ShapeT>;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(!isBounded, "Must not be bounded.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }
            std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const override
            {
                return ray::raycast(ray, m_shape, tNearest);
            }
            std::unique_ptr<PolymorphicSceneObjectBase> clone() const override
            {
                return std::make_unique<PolymorphicSceneObject<ShapeT>>(*this);
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
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
        using ShapeType = UnboundedUniqueAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialArrayType<ShapeT>& materials) :
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

        std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const
        {
            return m_obj->raycast(ray, tNearest);
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
    struct SceneObject<UnboundedSharedAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            virtual std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const = 0;
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
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            using MaterialStorageType = MaterialArrayType<ShapeT>;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(!isBounded, "Must not be bounded.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }
            std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const override
            {
                return ray::raycast(ray, m_shape, tNearest);
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
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
        using ShapeType = UnboundedSharedAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialArrayType<ShapeT>& materials) :
            m_obj(std::make_shared<PolymorphicSceneObject<ShapeT>>(materials, shape))
        {

        }

        SceneObjectId id() const
        {
            return m_obj->id();
        }

        std::optional<RaycastHit> raycast(const Ray& ray, float& tNearest) const
        {
            return m_obj->raycast(ray, tNearest);
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
