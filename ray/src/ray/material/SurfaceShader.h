#pragma once

#include "Color.h"
#include "MaterialPtrStorage.h"
#include "SurfaceMaterial.h"
#include "SurfaceShaderOutput.h"

#include <ray/math/RaycastHit.h>
#include <ray/math/Vec3.h>

namespace ray
{
    template <typename ShapeT>
    struct SurfaceShader
    {
        [[nodiscard]] virtual SurfaceShaderOutput shade(const ShapeT& shape, const RaycastHit& hit, const MaterialPtrStorageView& surfaceMaterial) const = 0;
    };

    template <typename ShapeT>
    struct DefaultSurfaceShader : SurfaceShader<ShapeT>
    {
        using BaseType = SurfaceShader<ShapeT>;
        using ShapeType = ShapeT;

        [[nodiscard]] static DefaultSurfaceShader<ShapeT>& instance()
        {
            static DefaultSurfaceShader<ShapeT> s_instance{};
            return s_instance;
        }

        [[nodiscard]] SurfaceShaderOutput shade(const ShapeT& shape, const RaycastHit& hit, const MaterialPtrStorageView& materials) const override
        {
            const SurfaceMaterial& surfaceMaterial = materials.surfaceMaterial(hit.materialIndex.surfaceMaterialNo());
            ColorRGBf surfaceColor = surfaceMaterial.surfaceColor;
            if (surfaceMaterial.texture)
            {
                const TexCoords texCoords = resolveTexCoords(shape, hit);
                surfaceColor *= surfaceMaterial.sampleTexture(texCoords);
            }

            return SurfaceShaderOutput{
                hit.point,
                hit.normal,
                hit.dist,
                surfaceColor,
                surfaceMaterial.emissionColor,
                surfaceMaterial.transparency,
                surfaceMaterial.reflectivity,
                surfaceMaterial.diffuse
            };
        }

    private:
        DefaultSurfaceShader() noexcept = default;
    };

    template <typename ShapeT>
    inline const DefaultSurfaceShader<ShapeT>& defaultShader = DefaultSurfaceShader<ShapeT>::instance();
}
