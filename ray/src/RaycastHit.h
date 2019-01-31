#pragma once

#include "Material.h"
#include "Vec3.h"

namespace ray
{
    struct RaycastHit
    {
        Point3f point;
        Normal3f normal;
        int shapeNo;
        int materialNo;
    };

    template <typename ShapeT>
    struct ResolvedTypedRaycastHit
    {
        Point3f point;
        Normal3f normal;
        const ShapeT* shape;
        const Material* material;
    };

    struct ResolvedUntypedRaycastHit
    {
        template <typename ShapeT>
        ResolvedUntypedRaycastHit(const ResolvedTypedRaycastHit<ShapeT>& typed) :
            point(typed.point),
            normal(typed.normal),
            material(typed.material)
        {

        }

        Point3f point;
        Normal3f normal;
        const Material* material;
    };

    struct ResolvedUntypedInOutRaycastHit
    {
        ResolvedUntypedRaycastHit in;
        ResolvedUntypedRaycastHit out;
    };
}
