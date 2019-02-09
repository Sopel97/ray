#pragma once

#include "NamedTypePacks.h"
#include "Box3.h"
#include "BvhNode.h"
#include "BvhObject.h"
#include "StaticBvhObjectPartitioner.h"

namespace ray
{
    template <typename...>
    struct StaticBvhObjectMedianPartitioner;

    template <typename BvShapeT, typename... ShapeTs>
    struct StaticBvhObjectMedianPartitioner<Shapes<ShapeTs...>, BvShapeT> : StaticBvhObjectPartitioner
    {
        using AllShapes = Shapes<ShapeTs...>;

        using BoundedBvhObject = BoundedStaticBvhObject<BvShapeT, AllShapes>;

        using BoundedBvhObjectVector = BoundedStaticBvhObjectVector<BvShapeT, AllShapes>;
        using BoundedBvhObjectVectorIterator = BoundedStaticBvhObjectVectorIterator<BvShapeT, AllShapes>;

        using Point3fMemberPtr = float(Point3f::*);

        // must include last
        std::vector<BoundedBvhObjectVectorIterator> partition(BoundedBvhObjectVectorIterator first, BoundedBvhObjectVectorIterator last) const
        {
            Point3fMemberPtr cmpAxis = biggestExtentAxis(first, last);

            const int size = static_cast<int>(std::distance(first, last));
            auto mid = std::next(first, size / 2);
            std::nth_element(first, mid, last, [cmpAxis](const auto& lhs, const auto& rhs) {
                return lhs->center().*cmpAxis < rhs->center().*cmpAxis;
                });

            const float partitionPoint = (*mid)->center().*cmpAxis;
            return { std::partition(first, last, [partitionPoint, cmpAxis](const auto& em) {
                return em->center().*cmpAxis < partitionPoint;
                }) , last};
        }

        Box3 aabb(BoundedBvhObjectVectorIterator first, BoundedBvhObjectVectorIterator last) const
        {
            if (first == last) return {};
            Box3 bb = (*first)->aabb();
            ++first;
            while (first != last)
            {
                bb.extend((*first)->aabb());
                ++first;
            }
            return bb;
        }

        Point3fMemberPtr biggestExtentAxis(BoundedBvhObjectVectorIterator first, BoundedBvhObjectVectorIterator last) const
        {
            Box3 bb = aabb(first, last);
            const Vec3f extent = bb.extent();
            if (extent.x > extent.y && extent.x > extent.z) return &Point3f::x;
            if (extent.y > extent.x && extent.y > extent.z) return &Point3f::y;
            return &Point3f::z;
        }
    };
}
