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

        MaterialIndex() noexcept = default;

        explicit MaterialIndex(int s, int m = none) noexcept :
            m_surfaceMaterialNo(s),
            m_mediumMaterialNo(m)
        {

        }

        [[nodiscard]] int surfaceMaterialNo() const
        {
            return m_surfaceMaterialNo;
        }

        [[nodiscard]] int mediumMaterialNo() const
        {
            return m_mediumMaterialNo;
        }

    private:
        int m_surfaceMaterialNo;
        int m_mediumMaterialNo;
    };
}
