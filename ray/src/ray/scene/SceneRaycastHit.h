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
    struct SurfaceMaterial;
    struct MediumMaterial;

    struct ResolvableRaycastHit : RaycastHit
    {
        int shapeNo;
        const HomogeneousSceneObjectCollection* owner;
        const void* additionalData = nullptr;

        ResolvedRaycastHit resolve() const;

        SceneObjectId objectId() const;
    };

    struct ResolvedRaycastHit
    {
        ResolvedRaycastHit(
            float dist,
            const Point3f& point,
            const Normal3f& normal,
            const TexCoords& texCoords,
            int shapeNo,
            const SurfaceMaterial* surfaceMaterial,
            const MediumMaterial* mediumMaterial,
            const HomogeneousSceneObjectCollection& owner,
            bool isInside,
            bool hasVolume,
            bool local
        ) :
            dist(dist),
            point(point),
            normal(normal),
            texCoords(texCoords),
            shapeNo(shapeNo),
            surfaceMaterial(surfaceMaterial),
            mediumMaterial(mediumMaterial),
            owner(&owner),
            isInside(isInside),
            hasVolume(hasVolume),
            isLocallyContinuable(local)
        {

        }

        float dist;
        Point3f point;
        Normal3f normal;
        TexCoords texCoords;
        int shapeNo;
        const SurfaceMaterial* surfaceMaterial;
        const MediumMaterial* mediumMaterial;
        const HomogeneousSceneObjectCollection* owner;
        bool isInside;
        bool hasVolume;
        bool isLocallyContinuable;

        bool next(const Ray& ray, ResolvableRaycastHit& hit) const;

        SceneObjectId objectId() const;
    };
}
