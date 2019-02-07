#pragma once

#include "SceneObjectId.h"
#include "Material.h"
#include "Ray.h"
#include "TexCoords.h"
#include "Vec3.h"

#include <cstdint>
#include <functional>
#include <optional>

namespace ray
{
    struct SceneObjectCollection;
    struct ResolvedRaycastHit;
    struct RaycastQueryStats;

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
        ResolvableRaycastHit(const Point3f& point, const Normal3f& normal, int shapeNo, int materialNo, const SceneObjectCollection& owner, bool isInside) :
            point(point),
            normal(normal),
            shapeNo(shapeNo),
            materialNo(materialNo),
            owner(&owner),
            isInside(isInside)
        {

        }

        Point3f point;
        Normal3f normal;
        int shapeNo;
        int materialNo;
        const SceneObjectCollection* owner;
        bool isInside;

        ResolvedRaycastHit resolve() const;

        SceneObjectId objectId() const;
    };

    struct ResolvedRaycastHit
    {
        ResolvedRaycastHit(const Point3f& point, const Normal3f& normal, const TexCoords& texCoords, int shapeNo, const Material& material, const SceneObjectCollection& owner, bool isInside, bool local) :
            point(point),
            normal(normal),
            texCoords(texCoords),
            shapeNo(shapeNo),
            material(&material),
            owner(&owner),
            isInside(isInside),
            isLocallyContinuable(local)
        {

        }

        Point3f point;
        Normal3f normal;
        TexCoords texCoords;
        int shapeNo;
        const Material* material;
        const SceneObjectCollection* owner;
        bool isInside;
        bool isLocallyContinuable;

        std::optional<ResolvableRaycastHit> next(const Ray& ray, RaycastQueryStats* stats = nullptr) const;

        SceneObjectId objectId() const;
    };
}
