#pragma once

#include "NamedTypePacks.h"
#include "Box3.h"
#include "BvhNode.h"
#include "BvhObject.h"
#include "StaticBvhObjectPartitioner.h"

#include <vector>

namespace ray
{
    struct StaticBvhObjectMedianPartitioner
    {
        template <typename...>
        struct For;

        template <typename BvShapeT, typename StorageProviderT, typename... ShapeTs>
        struct For<BvhParams<Shapes<ShapeTs...>, BvShapeT, StorageProviderT>> : StaticBvhObjectPartitioner
        {
            using BvhParamsT = BvhParams<Shapes<ShapeTs...>, BvShapeT, StorageProviderT>;
            using AllShapes = Shapes<ShapeTs...>;

            using BoundedBvhObject = BoundedStaticBvhObject<BvhParamsT>;

            using BoundedBvhObjectVector = BoundedStaticBvhObjectVector<BvhParamsT>;
            using BoundedBvhObjectVectorIterator = BoundedStaticBvhObjectVectorIterator<BvhParamsT>;

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
                    }) , last };
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
    };
}
