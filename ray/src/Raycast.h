#pragma once

#include "Ray.h"
#include "RaycastHit.h"
#include "Sphere.h"
#include "Vec3.h"

#include <optional>

namespace ray
{
    std::optional<RaycastHit> raycast(const Ray& ray, const Sphere& sphere)
    {
        // TODO: this
        return std::nullopt;
    }
}
