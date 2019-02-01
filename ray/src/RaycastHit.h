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
        ResolvedRaycastHit(const Point3f& point, const Normal3f& normal, const Material& material, std::uint64_t objectId) :
            point(point),
            normal(normal),
            material(&material),
            objectId(objectId)
        {

        }

        Point3f point;
        Normal3f normal;
        const Material* material;
        std::uint64_t objectId;
    };

    // allows fast queries from inside the shape
    struct ResolvedLocallyContinuableRaycastHit : ResolvedRaycastHit
    {
        template <typename FuncT>
        ResolvedLocallyContinuableRaycastHit(const Point3f& point, const Normal3f& normal, const Material& material, std::uint64_t objectId, FuncT&& func) :
            ResolvedRaycastHit(point, normal, material, objectId),
            m_continuation(std::forward<FuncT>(func))
        {

        }

        std::optional<ResolvedRaycastHit> next(const Ray& ray) const
        {
            return m_continuation(*this, ray);
        }

    private:
        std::function<std::optional<ResolvedRaycastHit>(const ResolvedLocallyContinuableRaycastHit& prevHit, const Ray&)> m_continuation;
    };
}
