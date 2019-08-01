#pragma once

#include "Material.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>

namespace ray
{
    struct MaterialDatabase
    {
        MaterialDatabase() noexcept {}
        MaterialDatabase(const MaterialDatabase&) = delete;
        MaterialDatabase(MaterialDatabase&&) = default;
        MaterialDatabase& operator=(const MaterialDatabase&) = delete;
        MaterialDatabase& operator=(MaterialDatabase&&) = default;

        template <typename... ArgsTs>
        const SurfaceMaterial& emplaceSurface(std::string name, ArgsTs&&... args)
        {
            return *(m_surfaces.try_emplace(std::move(name), std::make_unique<SurfaceMaterial>(std::forward<ArgsTs>(args)...)).first->second);
        }
        template <typename... ArgsTs>
        const MediumMaterial& emplaceMedium(std::string name, ArgsTs&&... args)
        {
            return *(m_mediums.try_emplace(std::move(name), std::make_unique<MediumMaterial>(std::forward<ArgsTs>(args)...)).first->second);
        }

        [[nodiscard]] const SurfaceMaterial& getSurface(std::string_view name) const
        {
            return *(m_surfaces.find(name)->second);
        }

        [[nodiscard]] const MediumMaterial& get(std::string_view name) const
        {
            return *(m_mediums.find(name)->second);
        }

    private:
        std::map<std::string, std::unique_ptr<SurfaceMaterial>, std::less<>> m_surfaces;
        std::map<std::string, std::unique_ptr<MediumMaterial>, std::less<>> m_mediums;
    };
}
