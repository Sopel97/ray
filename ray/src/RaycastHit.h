#pragma once

#include "SceneObject.h"
#include "SceneObjectCollection.h"
#include "Material.h"
#include "Ray.h"
#include "TexCoords.h"
#include "Vec3.h"

#include <cstdint>
#include <functional>
#include <optional>

namespace ray
{
    struct SceneObjectCollction;
    struct ResolvedRaycastHit;

    struct RaycastHit
    {
        Point3f point;
        Normal3f normal;
        int shapeInPackNo;
        int materialNo;
        bool isInside;
    };

    struct ResolvableRaycastHit
    {
        Point3f point;
        Normal3f normal;
        int shapeNo;
        int materialNo;
        bool isInside;
        const SceneObjectCollection* owner;

        ResolvedRaycastHit resolve() const;

        SceneObjectId objectId() const
        {
            return owner->objectId(shapeNo);
        }
    };

    struct ResolvedRaycastHit
    {
        ResolvedRaycastHit(const Point3f& point, const Normal3f& normal, const Material& material) :
            point(point),
            normal(normal),
            material(&material),
            owner(nullptr),
            shapeNo(0),
            isLocallyContinuable(false)
        {

        }

        ResolvedRaycastHit(const Point3f& point, const Normal3f& normal, const TexCoords& texCoords, const Material& material, bool isInside, const SceneObjectCollection& owner, int shapeNo, bool local) :
            point(point),
            normal(normal),
            texCoords(texCoords),
            material(&material),
            isInside(isInside),
            owner(&owner),
            shapeNo(shapeNo),
            isLocallyContinuable(local)
        {

        }

        Point3f point;
        Normal3f normal;
        TexCoords texCoords;
        const Material* material;
        bool isInside;
        const SceneObjectCollection* owner;
        int shapeNo;
        bool isLocallyContinuable;

        std::optional<ResolvableRaycastHit> next(const Ray& ray) const
        {
            if (!owner || !isLocallyContinuable) return std::nullopt;

            return owner->queryLocal(ray, shapeNo);
        }

        SceneObjectId objectId() const
        {
            return owner->objectId(shapeNo);
        }
    };
}
