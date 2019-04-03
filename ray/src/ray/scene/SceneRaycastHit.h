#pragma once

#include "object/SceneObjectId.h"

#include <ray/material/SurfaceShaderOutput.h>
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

        [[nodiscard]] ResolvedRaycastHit resolve() const;

        [[nodiscard]] SceneObjectId objectId() const;
    };

    struct ResolvedRaycastHit : SurfaceShaderOutput
    {
        ResolvedRaycastHit(
            const SurfaceShaderOutput& shaderOutput,
            int shapeNo,
            const MediumMaterial* mediumMaterial,
            const HomogeneousSceneObjectCollection* owner,
            bool isInside,
            bool hasVolume,
            bool local
        ) :
            SurfaceShaderOutput(shaderOutput),
            shapeNo(shapeNo),
            mediumMaterial(mediumMaterial),
            owner(owner),
            isInside(isInside),
            hasVolume(hasVolume),
            isLocallyContinuable(local)
        {

        }
        
        int shapeNo;
        const MediumMaterial* mediumMaterial;
        const HomogeneousSceneObjectCollection* owner;
        bool isInside;
        bool hasVolume;
        bool isLocallyContinuable;

        [[nodiscard]] bool next(const Ray& ray, ResolvableRaycastHit& hit) const;

        [[nodiscard]] SceneObjectId objectId() const;
    };
}
