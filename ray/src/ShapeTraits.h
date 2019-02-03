#pragma once

namespace ray
{
    struct Sphere;

    template <typename ShapeT>
    struct ShapeTraits;

    template <>
    struct ShapeTraits<Sphere>
    {
        using ShapeType = Sphere;
        using BaseShapeType = Sphere; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numMaterialsPerShape = 1;
        static constexpr bool hasVolume = true;
    };
}
