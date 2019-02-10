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
        MaterialDatabase() {}
        MaterialDatabase(const MaterialDatabase&) = delete;
        MaterialDatabase(MaterialDatabase&&) = default;
        MaterialDatabase& operator=(const MaterialDatabase&) = delete;
        MaterialDatabase& operator=(MaterialDatabase&&) = default;

        template <typename... ArgsTs>
        const Material& emplace(std::string name, ArgsTs&&... args)
        {
            return *(m_textures.try_emplace(std::move(name), std::make_unique<Material>(std::forward<ArgsTs>(args)...)).first->second);
        }

        const Material& get(std::string_view name) const
        {
            return *(m_textures.find(name)->second);
        }

    private:
        std::map<std::string, std::unique_ptr<Material>, std::less<>> m_textures;
    };
}
