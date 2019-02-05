#pragma once

#include "RaycastHit.h"
#include "Util.h"
#include "Vec3.h"

#include <optional>
#include <memory>
#include <utility>

namespace ray
{
    struct Ray;
    struct RaycastHit;

    struct PolymorphicShapeBase
    {
        virtual Point3f center() const = 0;
        virtual std::optional<RaycastHit> raycast(const Ray& ray) const = 0;
        virtual std::unique_ptr<PolymorphicShapeBase> clone() const = 0;
        virtual TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const = 0;
        virtual bool hasVolume() const = 0;
        virtual const Material& material(int materialNo) const = 0;
        virtual bool isLight() const = 0;
    };

    template <typename ShapeT>
    struct PolymorphicShape : PolymorphicShapeBase, ShapeT
    {
        static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
        static constexpr bool isPack = numShapesInPack > 1;
        static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
        using MaterialStorageType = std::array<const Material*, numMaterialsPerShape>;
        static_assert(!isPack, "A single scene object must not be a pack. Use SceneObjectArray.");

        template <typename... ArgsTs>
        PolymorphicShape(const MaterialStorageType& materials, ArgsTs&&... args) :
            ShapeT(std::forward<ArgsTs>(args)...),
            m_materials(materials)
        {
            
        }

        Point3f center() const override
        {
            return ShapeT::center();
        }
        std::optional<RaycastHit> raycast(const Ray& ray) const override
        {
            return ::ray::raycast(ray, *this);
        }
        std::unique_ptr<PolymorphicShapeBase> clone() const override
        {
            return std::make_unique<PolymorphicShape<ShapeT>>(*this);
        }
        TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
        {
            return ::ray::resolveTexCoords(*this, hit, shapeInPackNo);
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

    private:
        MaterialStorageType m_materials;
    };

    struct AnyShape
    {
        template <typename ShapeT, int MaterialCountV = ShapeTraits<ShapeT>::numMaterialsPerShape>
        AnyShape(ShapeT&& shape, const std::array<const Material*, MaterialCountV>& materials) :
            m_shape(new PolymorphicShape<remove_cvref_t<ShapeT>>(materials, std::forward<ShapeT>(shape)))
        {

        }

        AnyShape(const AnyShape& other) :
            m_shape(other.m_shape->clone())
        {

        }

        Point3f center() const
        {
            return m_shape->center();
        }

        std::optional<RaycastHit> raycast(const Ray& ray) const
        {
            return m_shape->raycast(ray);
        }

        TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const
        {
            return m_shape->resolveTexCoords(hit, shapeInPackNo);
        }

        bool hasVolume() const
        {
            return m_shape->hasVolume();
        }

        const Material& material(int materialNo) const
        {
            return m_shape->material(materialNo);
        }

        bool isLight() const
        {
            return m_shape->isLight();
        }

    private:
        std::unique_ptr<PolymorphicShapeBase> m_shape;
    };

    inline TexCoords resolveTexCoords(const AnyShape& shape, const ResolvableRaycastHit& hit, int shapeInPackNo)
    {
        return shape.resolveTexCoords(hit, shapeInPackNo);
    }

    inline std::optional<RaycastHit> raycast(const Ray& ray, const AnyShape& shape)
    {
        return shape.raycast(ray);
    }
}
