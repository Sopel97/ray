#pragma once

#if defined(RAY_GATHER_PERF_STATS)
#include <ray/perf/PerformanceStats.h>
#endif

#include "BvhNode.h"
#include "BvhObject.h"
#include "BvhParams.h"

#include <ray/math/BoundingVolume.h>

#include <ray/scene/SceneRaycastHit.h>
#include <ray/scene/object/RawSceneObjectBlob.h>
#include <ray/scene/object/SceneObjectBlob.h>
#include <ray/scene/object/SceneObjectCollection.h>

#include <ray/shape/Box3.h>
#include <ray/shape/Shapes.h>
#include <ray/shape/ShapeTraits.h>

#include <ray/utility/Util.h>

#include <memory>
#include <optional>
#include <queue>
#include <type_traits>
#include <vector>

namespace ray
{
    template <typename... Ts>
    struct StaticBvh;

    template <typename PartitionerMakerT, typename BvShapeT, typename StorageProviderT, typename... ShapeTs>
    struct StaticBvh<BvhParams<Shapes<ShapeTs...>, BvShapeT, StorageProviderT>, PartitionerMakerT> : StaticHeterogeneousSceneObjectCollection
    {
        static constexpr int maxDepth = 16;
        static constexpr int maxObjectsPerNode = 1;

        using AllShapes = Shapes<ShapeTs...>;
        using BoundedShapes = FilterShapes<AllShapes, ShapePredicates::IsBounded>;
        using UnboundedShapes = FilterShapes<AllShapes, ShapePredicates::IsUnbounded>;
        using BvhParamsT = BvhParams<BoundedShapes, BvShapeT, StorageProviderT>;
        using PartitionerT = typename PartitionerMakerT::template For<BvhParamsT>;
        using LeafNodeType = StaticBvhLeafNode<BvhParamsT>;
        using PartitionNodeType = StaticBvhPartitionNode<BvShapeT>;

        using BoundedBvhObject = BoundedStaticBvhObject<BvhParamsT>;

        using BoundedBvhObjectVector = BoundedStaticBvhObjectVector<BvhParamsT>;
        using BoundedBvhObjectVectorIterator = BoundedStaticBvhObjectVectorIterator<BvhParamsT>;

        template <typename... PartitionerArgsTs>
        StaticBvh(const RawSceneObjectBlob<AllShapes>& blob, PartitionerArgsTs&&... args) :
            m_partitioner(std::forward<PartitionerArgsTs>(args)...)
        {
#if defined(RAY_GATHER_PERF_STATS)
            auto t0 = std::chrono::high_resolution_clock().now();
#endif
            construct(blob);
#if defined(RAY_GATHER_PERF_STATS)
            auto t1 = std::chrono::high_resolution_clock().now();
            auto diff = t1 - t0;
            perf::gPerfStats.addConstructionTime(diff);
#endif
        }

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray) const
        {
            BvhNodeHitQueue<BvShapeT> queue;
            queue.push(StaticBvhNodeHit(0.0f, *m_root));
            float nearestHitDist = std::numeric_limits<float>::max();
            std::optional<ResolvableRaycastHit> nearestHitOpt = m_unboundedObjects.queryNearest(ray);
            if (nearestHitOpt)
            {
                nearestHitDist = distance(ray.origin(), nearestHitOpt->point);
            }

            while (!queue.empty())
            {
                StaticBvhNodeHit entry = queue.top();
                queue.pop();
                if (entry.dist >= nearestHitDist) break;

                std::optional<ResolvableRaycastHit> hitOpt = entry.node->nextHit(ray, queue);
                if (hitOpt)
                {
                    const float dist = distance(ray.origin(), hitOpt->point);
                    if (dist < nearestHitDist)
                    {
                        nearestHitOpt = std::move(*hitOpt);
                        nearestHitDist = dist;
                    }
                }
            }

            return nearestHitOpt;
        }

    private:
        std::unique_ptr<StaticBvhNode<BvShapeT>> m_root;
        SceneObjectBlob<UnboundedShapes, StorageProviderT> m_unboundedObjects;
        PartitionerT m_partitioner;

        template <typename ShapeT>
        struct SpecificBvhObject : BoundedBvhObject
        {
            SpecificBvhObject(const SceneObject<ShapeT>& obj) :
                m_object(&obj),
                m_boundingVolume(ray::boundingVolume<BvShapeT>(obj)),
                m_aabb(obj.aabb()),
                m_center(obj.center())
            {

            }

            void addTo(LeafNodeType& leaf) const override
            {
                leaf.add(object());
            }

            const SceneObject<ShapeT>& object() const
            {
                return *m_object;
            }

            const BvShapeT& boundingVolume() const override
            {
                return m_boundingVolume;
            }

            const Box3& aabb() const override
            {
                return m_aabb;
            }

            const Point3f& center() const override
            {
                return m_center;
            }

        private:
            const SceneObject<ShapeT>* m_object;
            BvShapeT m_boundingVolume;
            Box3 m_aabb;
            Point3f m_center;
        };

        template <typename... SceneObjectCollectionTs>
        void construct(const RawSceneObjectBlob<AllShapes>& blob)
        {
            BoundedBvhObjectVector allObjects;
            blob.forEach([&](auto&& object) {
                using ObjectType = remove_cvref_t<decltype(object)>;
                using ShapeType = typename ObjectType::ShapeType;
                if constexpr (ShapeTraits<ShapeType>::isBounded)
                {
                    allObjects.emplace_back(std::make_unique<SpecificBvhObject<ShapeType>>(object));
                }
                else
                {
                    m_unboundedObjects.add(object);
                }
            });

            m_root = makeNode(allObjects.begin(), allObjects.end());
        }

        BvShapeT boundingVolume(BoundedBvhObjectVectorIterator first, BoundedBvhObjectVectorIterator last) const
        {
            if (first == last) return {};
            BvShapeT bb = (*first)->boundingVolume();
            ++first;
            while (first != last)
            {
                bb.extend((*first)->boundingVolume());
                ++first;
            }
            return bb;
        }

        std::unique_ptr<LeafNodeType> makeLeafNode(BoundedBvhObjectVectorIterator first, BoundedBvhObjectVectorIterator last) const
        {
            auto leaf = std::make_unique<LeafNodeType>();
            while (first != last)
            {
                (*first)->addTo(*leaf);
                ++first;
            }
            return leaf;
        }

        std::unique_ptr<PartitionNodeType> makePartitionNode(BoundedBvhObjectVectorIterator first, BoundedBvhObjectVectorIterator last, int depth) const
        {
            // call recucively
            std::vector<BoundedBvhObjectVectorIterator> ends = m_partitioner.partition(first, last);
            auto node = std::make_unique<PartitionNodeType>();
            for (const auto& partEnd : ends)
            {
                BvShapeT bv = boundingVolume(first, partEnd);
                node->addChild(makeNode(first, partEnd, depth + 1), std::move(bv));
                first = partEnd;
            }
            return node;
        }

        std::unique_ptr<StaticBvhNode<BvShapeT>> makeNode(BoundedBvhObjectVectorIterator first, BoundedBvhObjectVectorIterator last, int depth = 0) const
        {
            const int size = static_cast<int>(std::distance(first, last));
            if (size <= maxObjectsPerNode || depth >= maxDepth)
            {
                return makeLeafNode(first, last);
            }
            else
            {
                return makePartitionNode(first, last, depth);
            }
        }
    };
}