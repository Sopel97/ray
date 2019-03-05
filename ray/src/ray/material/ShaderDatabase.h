#pragma once

#include "SurfaceShader.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>

namespace ray
{
    // TODO: Maybe make it dynamic with type_index instead of static globabl storage
    //       speed is not important here
    struct ShaderDatabase
    {
    private:
        template <typename ShapeT>
        struct SpecificShaderDatabase
        {
            using StorageType = std::map<std::string, std::unique_ptr<SurfaceShader<ShapeT>>, std::less<>>;

            template <typename ShaderT, typename... ArgsTs>
            static const SurfaceShader<ShapeT>& emplace(std::string&& name, ArgsTs&&... args)
            {
                return *(storage().try_emplace(std::move(name), std::make_unique<ShaderT>(std::forward<ArgsTs>(args)...)).first->second);
            }

            [[nodiscard]] static const SurfaceShader<ShapeT>& get(std::string_view name)
            {
                return *(storage().find(name)->second);
            }

        private:
            static StorageType& storage()
            {
                static StorageType s_storage;
                return s_storage;
            }
        };

    public:
        ShaderDatabase() noexcept {}
        ShaderDatabase(const ShaderDatabase&) = delete;
        ShaderDatabase(ShaderDatabase&&) noexcept = default;
        ShaderDatabase& operator=(const ShaderDatabase&) = delete;
        ShaderDatabase& operator=(ShaderDatabase&&) noexcept = default;

        template <typename ShaderT, typename... ArgsTs>
        const auto& emplace(std::string name, ArgsTs&&... args)
        {
            return SpecificShaderDatabase<typename ShaderT::ShapeType>::template emplace<ShaderT>(std::move(name), std::forward<ArgsTs>(args)...);
        }

        template <typename ShapeT>
        [[nodiscard]] const SurfaceShader<ShapeT>& get(std::string_view name) const
        {
            return SpecificShaderDatabase<ShapeT>::get(name);
        }
    };
}
