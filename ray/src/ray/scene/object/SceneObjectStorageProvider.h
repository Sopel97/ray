#pragma once

#include "SceneObjectArray.h"

#include <ray/shape/ShapeTraits.h>

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
        struct Storage<CsgShape>
        {
            using Type = SceneObjectArray<CsgShape>;
        };

        template <bool IsBoundedV>
        struct Storage<AnyShape<IsBoundedV>>
        {
            using Type = SceneObjectArray<AnyShape<IsBoundedV>>;
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
