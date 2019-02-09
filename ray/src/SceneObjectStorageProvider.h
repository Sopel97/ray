#pragma once

#include "SceneObjectArray.h"
#include "ShapeTraits.h"

namespace ray
{
    struct PackedSceneObjectStorageProvider
    {
        template <typename ShapeT>
        using ArrayType = SceneObjectArray<typename ShapeTraits<ShapeT>::ShapePackType>;
    };

    struct RawSceneObjectStorageProvider
    {
        template <typename ShapeT>
        using ArrayType = SceneObjectArray<ShapeT>;
    };
}
