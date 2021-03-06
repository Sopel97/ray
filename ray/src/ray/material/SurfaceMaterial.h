#pragma once

#include "Color.h"
#include "TexCoords.h"
#include "Texture.h"

namespace ray
{
    struct SurfaceMaterial
    {
        ColorRGBf surfaceColor;
        ColorRGBf emissionColor;
        float transparency;
        float reflectivity;
        float diffuse;
        const Texture* texture;

        SurfaceMaterial() noexcept :
            surfaceColor{},
            emissionColor{},
            transparency{},
            reflectivity{},
            diffuse{},
            texture(nullptr)
        {
        }

        SurfaceMaterial(
            const ColorRGBf& surfaceColor,
            const ColorRGBf& emissionColor,
            float transparency,
            float reflectivity,
            float diffuse,
            const Texture* texture = nullptr
        ) noexcept :
            surfaceColor(surfaceColor),
            emissionColor(emissionColor),
            transparency(transparency),
            reflectivity(reflectivity),
            diffuse(diffuse),
            texture(texture)
        {
        }

        SurfaceMaterial(const SurfaceMaterial&) noexcept = default;
        SurfaceMaterial(SurfaceMaterial&&) noexcept = default;
        SurfaceMaterial& operator=(const SurfaceMaterial&) noexcept = default;
        SurfaceMaterial& operator=(SurfaceMaterial&&) noexcept = default;

        SurfaceMaterial& withSurfaceColor(const ColorRGBf& color)
        {
            surfaceColor = color;
            return *this;
        }

        SurfaceMaterial& withEmissionColor(const ColorRGBf& color)
        {
            emissionColor = color;
            return *this;
        }

        SurfaceMaterial& withTransparency(float f)
        {
            transparency = f;
            return *this;
        }

        SurfaceMaterial& withReflectivity(float f)
        {
            reflectivity = f;
            return *this;
        }

        SurfaceMaterial& withDiffuse(float f)
        {
            diffuse = f;
            return *this;
        }

        SurfaceMaterial& withTexture(const Texture& tex)
        {
            texture = &tex;
            return *this;
        }

        [[nodiscard]] ColorRGBf sampleTexture(const TexCoords& coords) const
        {
            if (!texture) return ColorRGBf(1.0f, 1.0f, 1.0f);

            return texture->sample(coords);
        }
    };
}
