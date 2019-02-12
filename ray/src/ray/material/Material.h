#pragma once

#include "Color.h"
#include "TexCoords.h"
#include "Texture.h"

namespace ray
{
    struct Material
    {
        ColorRGBf surfaceColor;
        ColorRGBf emissionColor;
        float transparency;
        float refractiveIndex;
        float reflectivity;
        float diffuse;
        ColorRGBf absorbtion;
        const Texture* texture;

        Material() :
            surfaceColor{},
            emissionColor{},
            transparency{},
            refractiveIndex(1.0f),
            reflectivity{},
            diffuse{},
            absorbtion{},
            texture(nullptr)
        {
        }

        Material(
            const ColorRGBf& surfaceColor,
            const ColorRGBf& emissionColor,
            float transparency,
            float refractiveIndex,
            float reflectivity,
            float diffuse,
            const ColorRGBf& absorbtion,
            const Texture* texture = nullptr
        ) :
            surfaceColor(surfaceColor),
            emissionColor(emissionColor),
            transparency(transparency),
            refractiveIndex(refractiveIndex),
            reflectivity(reflectivity),
            diffuse(diffuse),
            absorbtion(absorbtion),
            texture(texture)
        {
        }

        Material(const Material&) = default;
        Material(Material&&) = default;
        Material& operator=(const Material&) = default;
        Material& operator=(Material&&) = default;

        Material& withSurfaceColor(const ColorRGBf& color)
        {
            surfaceColor = color;
            return *this;
        }

        Material& withEmissionColor(const ColorRGBf& color)
        {
            emissionColor = color;
            return *this;
        }

        Material& withTransparency(float f)
        {
            transparency = f;
            return *this;
        }

        Material& withRefractiveIndex(float f)
        {
            refractiveIndex = f;
            return *this;
        }

        Material& withReflectivity(float f)
        {
            reflectivity = f;
            return *this;
        }

        Material& withDiffuse(float f)
        {
            diffuse = f;
            return *this;
        }

        Material& withAbsorbtion(const ColorRGBf& color)
        {
            absorbtion = color;
            return *this;
        }

        Material& withTexture(const Texture& tex)
        {
            texture = &tex;
        }

        ColorRGBf sampleTexture(const TexCoords& coords) const
        {
            if (!texture) return ColorRGBf(1.0f, 1.0f, 1.0f);

            return texture->sample(coords);
        }
    };
}
