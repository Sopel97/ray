#pragma once

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
        std::uint64_t objectId;
        bool isInside;
        const SceneObjectCollection* owner;

        ResolvedRaycastHit resolve() const;
    };

    struct ResolvedRaycastHit
    {
        ResolvedRaycastHit(const Point3f& point, const Normal3f& normal, const Material& material, std::uint64_t objectId) :
            point(point),
            normal(normal),
            material(&material),
            objectId(objectId),
            owner(nullptr),
            shapeNo(0),
            isLocallyContinuable(false)
        {

        }

        ResolvedRaycastHit(const Point3f& point, const Normal3f& normal, const TexCoords& texCoords, const Material& material, std::uint64_t objectId, bool isInside, const SceneObjectCollection& owner, int shapeNo, bool local) :
            point(point),
            normal(normal),
            texCoords(texCoords),
            material(&material),
            objectId(objectId),
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
        std::uint64_t objectId;
        bool isInside;
        const SceneObjectCollection* owner;
        int shapeNo;
        bool isLocallyContinuable;

        std::optional<ResolvableRaycastHit> next(const Ray& ray) const
        {
            if (!owner || !isLocallyContinuable) return std::nullopt;

            return owner->queryLocal(ray, shapeNo);
        }
    };
}
