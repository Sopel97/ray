#pragma once

#include "object/SceneObjectId.h"

#include <ray/material/TexCoords.h>

#include <ray/math/Ray.h>
#include <ray/math/RaycastHit.h>
#include <ray/math/Vec3.h>

#include <cstdint>
#include <functional>
#include <optional>

namespace ray
{
    struct HomogeneousSceneObjectCollection;
    struct ResolvedRaycastHit;
    struct Material;

    struct ResolvableRaycastHit : RaycastHit
    {
        int shapeNo;
        const HomogeneousSceneObjectCollection* owner;

        ResolvedRaycastHit resolve() const;

        SceneObjectId objectId() const;
    };

    struct ResolvedRaycastHit
    {
        ResolvedRaycastHit(
            const Point3f& point,
            const Normal3f& normal,
            const TexCoords& texCoords,
            int shapeNo,
            const Material& material,
            const HomogeneousSceneObjectCollection& owner,
            bool isInside,
            bool local
        ) :
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
        const HomogeneousSceneObjectCollection* owner;
        bool isInside;
        bool isLocallyContinuable;

        bool next(const Ray& ray, float& tNearest, ResolvableRaycastHit& hit) const;

        SceneObjectId objectId() const;
    };
}
