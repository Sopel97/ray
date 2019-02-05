#pragma once

namespace ray
{
    struct Sphere;
    struct AnyShape;

    template <typename ShapeT>
    struct ShapeTraits;

    template <>
    struct ShapeTraits<Sphere>
    {
        using ShapePackType = Sphere;
        using BaseShapeType = Sphere; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numMaterialsPerShape = 1;
        static constexpr bool hasVolume = true;
        static constexpr bool isPolymorphic = false;
    };

    // Polymorphic shapes that are packs are currently not supported
    template <>
    struct ShapeTraits<AnyShape>
    {
        using ShapePackType = AnyShape;
        using BaseShapeType = AnyShape;
        static constexpr int numShapes = 1;
        static constexpr bool isPolymorphic = true;
    };
}
