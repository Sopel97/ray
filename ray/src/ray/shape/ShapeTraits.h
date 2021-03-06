#pragma once

#include "ShapeTags.h"

namespace ray
{
    struct Box3;
    struct Plane;
    struct Sphere;
    struct Triangle3;
    struct Disc3;
    struct Capsule;
    struct Cylinder;
    struct OrientedBox3;
    struct ClosedTriangleMeshFace;
    template <typename TransformT, typename ShapeT>
    struct TransformedShape3;
    template <typename ClippingShapeT>
    struct ClippedSdf;

    template <typename ShapeT>
    struct ShapeTraits;

    template <>
    struct ShapeTraits<Box3>
    {
        using ShapePackType = Box3;
        using BaseShapeType = Box3; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = 1;
        static constexpr int numMediumMaterialsPerShape = 1;
        static constexpr bool hasVolume = true;
        static constexpr bool isLocallyContinuable = true;
        static constexpr bool isBounded = true;
    };

    template <>
    struct ShapeTraits<Plane>
    {
        using ShapePackType = Plane;
        using BaseShapeType = Plane; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = 1;
        static constexpr int numMediumMaterialsPerShape = 0;
        static constexpr bool hasVolume = false;
        static constexpr bool isLocallyContinuable = false;
        static constexpr bool isBounded = false;
    };

    template <>
    struct ShapeTraits<Sphere>
    {
        using ShapePackType = Sphere;
        using BaseShapeType = Sphere; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = 1;
        static constexpr int numMediumMaterialsPerShape = 1;
        static constexpr bool hasVolume = true;
        static constexpr bool isLocallyContinuable = true;
        static constexpr bool isBounded = true;
    };

    template <>
    struct ShapeTraits<Cylinder>
    {
        using ShapePackType = Cylinder;
        using BaseShapeType = Cylinder; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = 2;
        static constexpr int numMediumMaterialsPerShape = 1;
        static constexpr bool hasVolume = true;
        static constexpr bool isLocallyContinuable = true;
        static constexpr bool isBounded = true;
    };

    template <>
    struct ShapeTraits<Capsule>
    {
        using ShapePackType = Capsule;
        using BaseShapeType = Capsule; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = 2;
        static constexpr int numMediumMaterialsPerShape = 1;
        static constexpr bool hasVolume = true;
        static constexpr bool isLocallyContinuable = true;
        static constexpr bool isBounded = true;
    };

    template <>
    struct ShapeTraits<OrientedBox3>
    {
        using ShapePackType = OrientedBox3;
        using BaseShapeType = OrientedBox3; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = 1;
        static constexpr int numMediumMaterialsPerShape = 1;
        static constexpr bool hasVolume = true;
        static constexpr bool isLocallyContinuable = true;
        static constexpr bool isBounded = true;
    };

    template <>
    struct ShapeTraits<Triangle3>
    {
        using ShapePackType = Triangle3;
        using BaseShapeType = Triangle3; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = 1;
        static constexpr int numMediumMaterialsPerShape = 0;
        static constexpr bool hasVolume = false;
        static constexpr bool isLocallyContinuable = false;
        static constexpr bool isBounded = true;
    };

    template <typename TransformT, typename ShapeT>
    struct ShapeTraits<TransformedShape3<TransformT, ShapeT>>
    {
        using ShapePackType = TransformedShape3<TransformT, ShapeT>;
        using BaseShapeType = TransformedShape3<TransformT, ShapeT>; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = ShapeTraits<ShapeT>::numSurfaceMaterialsPerShape;
        static constexpr int numMediumMaterialsPerShape = ShapeTraits<ShapeT>::numMediumMaterialsPerShape;
        static constexpr bool hasVolume = ShapeTraits<ShapeT>::hasVolume;
        static constexpr bool isLocallyContinuable = ShapeTraits<ShapeT>::isLocallyContinuable;
        static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
    };

    template <>
    struct ShapeTraits<Disc3>
    {
        using ShapePackType = Disc3;
        using BaseShapeType = Disc3; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = 1;
        static constexpr int numMediumMaterialsPerShape = 0;
        static constexpr bool hasVolume = false;
        static constexpr bool isLocallyContinuable = false;
        static constexpr bool isBounded = true;
    };

    template <>
    struct ShapeTraits<ClosedTriangleMeshFace>
    {
        using ShapePackType = ClosedTriangleMeshFace;
        using BaseShapeType = ClosedTriangleMeshFace; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = 1;
        static constexpr int numMediumMaterialsPerShape = 1;
        static constexpr bool hasVolume = true;
        // TODO: think if it can be somehow made semi-locally continuable
        //       the raytracer and storage would have to know that it's a mesh and know all its faces
        //       Specialize SceneObjectArray?
        static constexpr bool isLocallyContinuable = false;
        static constexpr bool isBounded = true;
    };

    template <typename ClippingShapeT>
    struct ShapeTraits<ClippedSdf<ClippingShapeT>>
    {
        using ShapePackType = ClippedSdf<ClippingShapeT>;
        using BaseShapeType = ClippedSdf<ClippingShapeT>; // for a pack it should be an underlying shape
        static constexpr int numShapes = 1; // >1 means that it's a pack (and should behave like a pack of BaseShapeType)
        static constexpr int numSurfaceMaterialsPerShape = 1;
        static constexpr int numMediumMaterialsPerShape = 1;
        static constexpr bool hasVolume = true;
        static constexpr bool isLocallyContinuable = true;
        static constexpr bool isBounded = true;
    };

    template <>
    struct ShapeTraits<CsgShape>
    {
        static constexpr bool isBounded = true;
    };

    template <bool IsBoundedV>
    struct ShapeTraits<AnyShape<IsBoundedV>>
    {
        static constexpr bool isBounded = IsBoundedV;
    };

    struct ShapePredicates
    {
        struct IsBounded
        {
            template <typename ShapeT>
            static constexpr bool value = ShapeTraits<ShapeT>::isBounded;
        };

        struct IsUnbounded
        {
            template <typename ShapeT>
            static constexpr bool value = !ShapeTraits<ShapeT>::isBounded;
        };
    };
}
