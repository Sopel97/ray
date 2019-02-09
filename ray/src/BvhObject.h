#pragma once

#include "NamedTypePacks.h"
#include "BvhNode.h"

#include <vector>
#include <memory>

namespace ray
{
    template <typename...>
    struct BoundedStaticBvhObject;

    template <typename BvShapeT, typename StorageProviderT, typename... ShapeTs>
    struct BoundedStaticBvhObject<BvhParams<Shapes<ShapeTs...>, BvShapeT, StorageProviderT>> 
    {
        using BvhParamsT = BvhParams<Shapes<ShapeTs...>, BvShapeT, StorageProviderT>;
        using LeafNodeType = StaticBvhLeafNode<BvhParamsT>;

        virtual void addTo(LeafNodeType& leaf) const = 0;
        virtual const BvShapeT& boundingVolume() const = 0;
        virtual const Box3& aabb() const = 0;
        virtual const Point3f& center() const = 0;
    };

    template <typename BvhParamsT>
    using BoundedStaticBvhObjectVector = std::vector<std::unique_ptr<BoundedStaticBvhObject<BvhParamsT>>>;
    template <typename BvhParamsT>
    using BoundedStaticBvhObjectVectorIterator = typename BoundedStaticBvhObjectVector<BvhParamsT>::iterator;
}
