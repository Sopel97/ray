#pragma once

#include "BvhNode.h"

#include <vector>
#include <memory>

namespace ray
{
    template <typename BvShapeT, typename... ShapeTs>
    struct BoundedStaticBvhObject
    {
        using LeafNodeType = StaticBvhLeafNode<BvShapeT, ShapeTs...>;

        virtual void addTo(LeafNodeType& leaf) const = 0;
        virtual const BvShapeT& boundingVolume() const = 0;
        virtual const Box3& aabb() const = 0;
        virtual const Point3f& center() const = 0;
    };

    template <typename BvShapeT, typename... ShapeTs>
    using BoundedStaticBvhObjectVector = std::vector<std::unique_ptr<BoundedStaticBvhObject<BvShapeT, ShapeTs...>>>;
    template <typename BvShapeT, typename... ShapeTs>
    using BoundedStaticBvhObjectVectorIterator = typename BoundedStaticBvhObjectVector<BvShapeT, ShapeTs...>::iterator;
}
