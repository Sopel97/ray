#pragma once

#include "Color.h"
#include "TexCoords.h"
#include "Texture.h"

namespace ray
{
    struct MediumMaterial
    {
        ColorRGBf absorbtion;
        float refractiveIndex;

        MediumMaterial() noexcept :
            absorbtion{},
            refractiveIndex(1.0f)
        {
        }

        MediumMaterial(
            const ColorRGBf& absorbtion,
            float refractiveIndex
        ) noexcept :
            absorbtion(absorbtion),
            refractiveIndex(refractiveIndex)
        {
        }

        MediumMaterial(const MediumMaterial&) noexcept = default;
        MediumMaterial(MediumMaterial&&) noexcept = default;
        MediumMaterial& operator=(const MediumMaterial&) noexcept = default;
        MediumMaterial& operator=(MediumMaterial&&) noexcept = default;

        MediumMaterial& withRefractiveIndex(float f)
        {
            refractiveIndex = f;
            return *this;
        }

        MediumMaterial& withAbsorbtion(const ColorRGBf& color)
        {
            absorbtion = color;
            return *this;
        }
    };
}
