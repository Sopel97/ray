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
        struct Storage<BoundedSharedAnyShape>
        {
            using Type = SceneObjectArray<BoundedSharedAnyShape>;
        };

        template <>
        struct Storage<BoundedUniqueAnyShape>
        {
            using Type = SceneObjectArray<BoundedUniqueAnyShape>;
        };

        template <>
        struct Storage<UnboundedSharedAnyShape>
        {
            using Type = SceneObjectArray<UnboundedSharedAnyShape>;
        };

        template <>
        struct Storage<UnboundedUniqueAnyShape>
        {
            using Type = SceneObjectArray<UnboundedUniqueAnyShape>;
        };

        template <>
        struct Storage<CsgShape>
        {
            using Type = SceneObjectArray<CsgShape>;
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
