#pragma once

#include "SceneObjectArray.h"
#include "ShapeTraits.h"

namespace ray
{
    struct PackedSceneObjectStorageProvider
    {
        template <typename ShapeT>
        struct Storage
        {
            using Type = SceneObjectArray<typename ShapeTraits<ShapeT>::ShapePackType>;
        };

        template <>
        struct Storage<SharedAnyShape>
        {
            using Type = SceneObjectArray<SharedAnyShape>;
        };

        template <>
        struct Storage<UniqueAnyShape>
        {
            using Type = SceneObjectArray<UniqueAnyShape>;
        };

        template <typename ShapeT>
        using ArrayType = typename Storage<ShapeT>::Type;
    };

    struct RawSceneObjectStorageProvider
    {
        template <typename ShapeT>
        using ArrayType = SceneObjectArray<ShapeT>;
    };
}
