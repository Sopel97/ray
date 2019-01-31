#pragma once

#include "Material.h"
#include "Vec3.h"

#include <functional>
#include <optional>

namespace ray
{
    struct RaycastHit
    {
        Point3f point;
        Normal3f normal;
        int shapeNo;
        int materialNo;
    };

    struct ResolvedRaycastHit
    {
        ResolvedRaycastHit(const Point3f& point, const Normal3f& normal, const Material& material) :
            point(point),
            normal(normal),
            material(&material)
        {

        }

        Point3f point;
        Normal3f normal;
        const Material* material;
    };

    // allows fast queries from inside the shape
    struct ResolvedLocallyContinuableRaycastHit : ResolvedRaycastHit
    {
        template <typename FuncT>
        ResolvedLocallyContinuableRaycastHit(const Point3f& point, const Normal3f& normal, const Material& material, FuncT&& func) :
            ResolvedRaycastHit(point, normal, material),
            m_continuation(std::forward<FuncT>(func))
        {

        }

        std::optional<ResolvedRaycastHit> next(const Normal3f& direction) const
        {
            return m_continuation(*this, direction);
        }

    private:
        std::function<std::optional<ResolvedRaycastHit>(const ResolvedLocallyContinuableRaycastHit& prevHit, const Normal3f&)> m_continuation;
    };
}
