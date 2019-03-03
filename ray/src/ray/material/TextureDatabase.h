#pragma once

#include "Texture.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>

namespace ray
{
    struct TextureDatabase
    {
        TextureDatabase() noexcept {}
        TextureDatabase(const TextureDatabase&) = delete;
        TextureDatabase(TextureDatabase&&) noexcept = default;
        TextureDatabase& operator=(const TextureDatabase&) = delete;
        TextureDatabase& operator=(TextureDatabase&&) noexcept = default;

        template <typename TexT, typename... ArgsTs>
        const Texture& emplace(std::string name, ArgsTs&&... args)
        {
            return *(m_textures.try_emplace(std::move(name), std::make_unique<TexT>(std::forward<ArgsTs>(args)...)).first->second);
        }

        [[nodiscard]] const Texture& get(std::string_view name) const
        {
            return *(m_textures.find(name)->second);
        }

    private:
        std::map<std::string, std::unique_ptr<Texture>, std::less<>> m_textures;
    };
}
