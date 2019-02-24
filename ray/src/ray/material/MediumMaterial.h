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

        MediumMaterial() :
            absorbtion{},
            refractiveIndex(1.0f)
        {
        }

        MediumMaterial(
            const ColorRGBf& absorbtion,
            float refractiveIndex
        ) :
            absorbtion(absorbtion),
            refractiveIndex(refractiveIndex)
        {
        }

        MediumMaterial(const MediumMaterial&) = default;
        MediumMaterial(MediumMaterial&&) = default;
        MediumMaterial& operator=(const MediumMaterial&) = default;
        MediumMaterial& operator=(MediumMaterial&&) = default;

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
