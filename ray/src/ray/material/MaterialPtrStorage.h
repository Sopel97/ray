#pragma once

#include "Material.h"

#include <array>
#include <utility>
#include <iostream>

namespace ray
{
    // size is not stored as it is not needed, should be resolved from other sources
    struct MaterialPtrStorageView
    {
        MaterialPtrStorageView(const SurfaceMaterial* const * surfaceMaterials, const MediumMaterial* const * mediumMaterials) noexcept :
            m_surfaceMaterials(surfaceMaterials),
            m_mediumMaterials(mediumMaterials)
        {

        }

        MaterialPtrStorageView(const MaterialPtrStorageView&) noexcept = default;
        MaterialPtrStorageView(MaterialPtrStorageView&&) noexcept = default;

        MaterialPtrStorageView& operator=(const MaterialPtrStorageView&) noexcept = default;
        MaterialPtrStorageView& operator=(MaterialPtrStorageView&&) noexcept = default;

        [[nodiscard]] const SurfaceMaterial& surfaceMaterial(int i) const
        {
            return *m_surfaceMaterials[i];
        }

        [[nodiscard]] const MediumMaterial& mediumMaterial(int i) const
        {
            return *m_mediumMaterials[i];
        }

        [[nodiscard]] std::pair<const SurfaceMaterial*, const MediumMaterial*> material(const MaterialIndex& idx) const
        {
            const int s = idx.surfaceMaterialNo();
            const int m = idx.mediumMaterialNo();

            if (m >= 0)
            {
                return { m_surfaceMaterials[s], m_mediumMaterials[m] };
            }
            else
            {
                return { m_surfaceMaterials[s], nullptr };
            }
        }

    private:
        const SurfaceMaterial* const * m_surfaceMaterials;
        const MediumMaterial* const * m_mediumMaterials;
    };

    template <int NumSurfaceMaterialsV, int NumMediumMaterialsV>
    struct MaterialPtrStorage
    {
        static_assert(NumSurfaceMaterialsV >= 1 && NumMediumMaterialsV >= 1);

        using SurfaceMaterialStorage = std::array<const SurfaceMaterial*, NumSurfaceMaterialsV>;
        using MediumMaterialStorage = std::array<const MediumMaterial*, NumMediumMaterialsV>;

        MaterialPtrStorage(const SurfaceMaterialStorage& surfaceMaterials, const MediumMaterialStorage& mediumMaterials) noexcept :
            m_surfaceMaterials(surfaceMaterials),
            m_mediumMaterials(mediumMaterials)
        {

        }

        [[nodiscard]] const SurfaceMaterial& surfaceMaterial(int i) const
        {
            return *m_surfaceMaterials[i];
        }

        [[nodiscard]] const MediumMaterial& mediumMaterial(int i) const
        {
            return *m_mediumMaterials[i];
        }

        [[nodiscard]] std::pair<const SurfaceMaterial*, const MediumMaterial*> at(const MaterialIndex& idx) const
        {
            const int s = idx.surfaceMaterialNo();
            const int m = idx.mediumMaterialNo();

            if (m >= 0)
            {
                return { m_surfaceMaterials[s], m_mediumMaterials[m] };
            }
            else
            {
                return { m_surfaceMaterials[s], nullptr };
            }
        }

        [[nodiscard]] MaterialPtrStorageView view() const
        {
            return MaterialPtrStorageView(m_surfaceMaterials.data(), m_mediumMaterials.data());
        }

        [[nodiscard]] bool isEmissive() const
        {
            for (const auto& mat : m_surfaceMaterials)
            {
                if (mat->emissionColor.total() > 0.00001f) return true;
            }

            return false;
        }

    private:
        SurfaceMaterialStorage m_surfaceMaterials;
        MediumMaterialStorage m_mediumMaterials;
    };

    template <int NumSurfaceMaterialsV>
    struct MaterialPtrStorage<NumSurfaceMaterialsV, 0>
    {
        static_assert(NumSurfaceMaterialsV >= 1);

        using SurfaceMaterialStorage = std::array<const SurfaceMaterial*, NumSurfaceMaterialsV>;

        MaterialPtrStorage(const SurfaceMaterialStorage& surfaceMaterials) noexcept :
            m_surfaceMaterials(surfaceMaterials)
        {

        }

        [[nodiscard]] const SurfaceMaterial& surfaceMaterial(int i) const
        {
            return *m_surfaceMaterials[i];
        }

        [[nodiscard]] MaterialPtrStorageView view() const
        {
            return MaterialPtrStorageView(m_surfaceMaterials.data(), nullptr);
        }

        [[nodiscard]] bool isEmissive() const
        {
            for (const auto& mat : m_surfaceMaterials)
            {
                if (mat->emissionColor.total() > 0.00001f) return true;
            }

            return false;
        }

    private:
        SurfaceMaterialStorage m_surfaceMaterials;
    };
}
