#pragma once

#include "BvhObject.h"
#include "BvhParams.h"
#include "StaticBvhObjectPartitioner.h"

#include <ray/math/Vec3.h>

#include <ray/shape/Shapes.h>

#include <algorithm>
#include <vector>

namespace ray
{
    struct StaticBvhObjectMeanPartitioner
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

            For(int order = 1) :
                m_numParts(1 << order)
            {

            }

            // must include last
            // can assume that distance(first, last) > 1
            std::vector<BoundedBvhObjectVectorIterator> partition(BoundedBvhObjectVectorIterator first, BoundedBvhObjectVectorIterator last) const
            {
                return partition(first, last, m_numParts);
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
                const Vec3f extent = aabb(first, last).extent();
                if (extent.x > extent.y && extent.x > extent.z) return &Point3f::x;
                if (extent.y > extent.x && extent.y > extent.z) return &Point3f::y;
                return &Point3f::z;
            }

        private:
            int m_numParts;

            // must include last
            std::vector<BoundedBvhObjectVectorIterator> partition(BoundedBvhObjectVectorIterator first, BoundedBvhObjectVectorIterator last, int p) const
            {
                auto flp2 = [](int x) {
                    int y;
                    do {
                        y = x;
                        x = x & (x - 1);
                    } while (x);
                    return y;
                };

                const Point3fMemberPtr cmpAxis = biggestExtentAxis(first, last);

                const int size = static_cast<int>(std::distance(first, last));
                const int parts = flp2(std::min(size, p));
                const auto mid = partitionHalf(first, last);
                if (parts <= 2)
                {
                    return { mid, last };
                }
                else
                {
                    std::vector<BoundedBvhObjectVectorIterator> p1 = partition(first, mid, parts / 2);
                    std::vector<BoundedBvhObjectVectorIterator> p2 = partition(mid, last, parts / 2);
                    p1.insert(p1.end(), p2.begin(), p2.end());
                    return p1;
                }
            }

            BoundedBvhObjectVectorIterator partitionHalf(BoundedBvhObjectVectorIterator first, BoundedBvhObjectVectorIterator last) const
            {
                const Point3fMemberPtr cmpAxis = biggestExtentAxis(first, last);

                const int size = static_cast<int>(std::distance(first, last));
                const auto mid = std::next(first, size / 2);

                const float partitionPoint = std::accumulate(first, last, 0.0f, [cmpAxis](float acc, const auto& lhs) {
                    return lhs->center().*cmpAxis + acc;
                    }) / size;

                return std::partition(first, last, [partitionPoint, cmpAxis](const auto& em) {
                        return em->center().*cmpAxis < partitionPoint;
                    });
            }
        };
    };
}
