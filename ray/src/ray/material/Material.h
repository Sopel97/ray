#pragma once

#include "Color.h"
#include "TexCoords.h"
#include "Texture.h"

#include "MediumMaterial.h"
#include "SurfaceMaterial.h"

namespace ray
{
    struct MaterialIndex
    {
        static constexpr int none = -1;

        MaterialIndex() = default;

        explicit MaterialIndex(int s, int m = none) :
            m_surfaceMaterialNo(s),
            m_mediumMaterialNo(m)
        {

        }

        int surfaceMaterialNo() const
        {
            return m_surfaceMaterialNo;
        }

        int mediumMaterialNo() const
        {
            return m_mediumMaterialNo;
        }

    private:
        int m_surfaceMaterialNo;
        int m_mediumMaterialNo;
    };
}
