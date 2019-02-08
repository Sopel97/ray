#pragma once

#if defined(RAY_GATHER_PERF_STATS)
#include "PerformanceStats.h"
#endif

#include "Box3.h"
#include "RaycastHit.h"
#include "SceneObjectArray.h"
#include "SceneObjectBlob.h"
#include "ShapeTraits.h"
#include "SceneObjectStorage.h"
#include "Util.h"

#include <algorithm>
#include <optional>
#include <memory>
#include <type_traits>
#include <vector>
#include <queue>

namespace ray
{
    struct BoundedBvhNode;
    struct BvhNodeHit;

    using BvhNodeHitQueue = std::priority_queue<BvhNodeHit, std::vector<BvhNodeHit>, std::greater<BvhNodeHit>>;

    struct StaticSceneObjectBvhNode
    {
        virtual std::optional<ResolvableRaycastHit> nextHit(const Ray& ray, BvhNodeHitQueue& queue) const = 0;
    };

    struct BoundedBvhNode
    {
        BoundedBvhNode(std::unique_ptr<StaticSceneObjectBvhNode>&& node, const Box3& bb) :
            node(std::move(node)),
            aabb(bb)
        {
        }

        std::unique_ptr<StaticSceneObjectBvhNode> node;
        Box3 aabb;
    };

    struct BvhNodeHit
    {
        BvhNodeHit(float dist, const StaticSceneObjectBvhNode& node) :
            dist(dist),
            node(&node)
        {
        }

        float dist;
        const StaticSceneObjectBvhNode* node;

        friend bool operator<(const BvhNodeHit& lhs, const BvhNodeHit& rhs) noexcept
        {
            return lhs.dist < rhs.dist;
        }
        friend bool operator>(const BvhNodeHit& lhs, const BvhNodeHit& rhs) noexcept
        {
            return lhs.dist > rhs.dist;
        }
    };

    template <typename... ShapeTs>
    struct StaticSceneObjectBvhLeafNode : StaticSceneObjectBvhNode
    {
        template <typename ShapeT>
        void add(const SceneObject<ShapeT>& so)
        {
            m_objects.add(so);
        }

        std::optional<ResolvableRaycastHit> nextHit(const Ray& ray, BvhNodeHitQueue& queue) const override
        {
            return m_objects.queryNearest(ray);
        }
        
    private:
        SceneObjectBlob<ShapeTs...> m_objects;
    };

    template <typename... ShapeTs>
    struct StaticSceneObjectBvhPartitionNode : StaticSceneObjectBvhNode
    {
        // provide memory from outside to prevent a lot of small allocations
        std::optional<ResolvableRaycastHit> nextHit(const Ray& ray, BvhNodeHitQueue& hits) const override
        {
            for (const auto& child : m_children)
            {
                std::optional<RaycastBvHit> hitOpt = raycastBv(ray, child.aabb);
                if (hitOpt)
                {
                    hits.emplace(hitOpt->dist, *child.node);
                }
            }
            return std::nullopt;
        }

        void addChild(std::unique_ptr<StaticSceneObjectBvhNode>&& node, const Box3& bb)
        {
            m_children.emplace_back(std::move(node), bb);
        }

    private:
        std::vector<BoundedBvhNode> m_children;
    };

    template <typename... ShapeTs>
    struct StaticSceneObjectBvh : StaticSceneObjectStorage
    {
        static constexpr int maxDepth = 16;
        static constexpr int maxObjectsPerNode = 1;

        using LeafNodeType = StaticSceneObjectBvhLeafNode<ShapeTs...>;
        using PartitionNodeType = StaticSceneObjectBvhPartitionNode<ShapeTs...>;

        template <typename... SceneObjectCollectionTs>
        StaticSceneObjectBvh(SceneObjectCollectionTs&&... collections)
        {
#if defined(RAY_GATHER_PERF_STATS)
            auto t0 = std::chrono::high_resolution_clock().now();
#endif
            construct(std::forward<SceneObjectCollectionTs>(collections)...);
#if defined(RAY_GATHER_PERF_STATS)
            auto t1 = std::chrono::high_resolution_clock().now();
            auto diff = t1 - t0;
            perf::gPerfStats.addConstructionTime(diff);
#endif
        }

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray) const
        {
            BvhNodeHitQueue queue;
            queue.push(BvhNodeHit(0.0f, *m_root));
            float nearestHitDist = std::numeric_limits<float>::max();
            std::optional<ResolvableRaycastHit> nearestHitOpt = std::nullopt;
            while (!queue.empty())
            {
                BvhNodeHit entry = queue.top();
                queue.pop();
                if (entry.dist >= nearestHitDist) break;

                std::optional<ResolvableRaycastHit> hitOpt = entry.node->nextHit(ray, queue);
                if (hitOpt)
                {
                    const float dist = distance(ray.origin(), hitOpt->point);
                    if (dist < nearestHitDist)
                    {
                        nearestHitOpt = *hitOpt;
                        nearestHitDist = dist;
                    }
                }
            }

            return nearestHitOpt;
        }

    private:
        std::unique_ptr<StaticSceneObjectBvhNode> m_root;

        struct Object
        {
            virtual void addTo(LeafNodeType& leaf) const = 0;
            virtual const Box3& aabb() const = 0;
            virtual const Point3f& center() const = 0;
        };

        using ObjectVector = std::vector<std::unique_ptr<Object>>;
        using ObjectVectorIterator = typename ObjectVector::iterator;

        template <typename ShapeT>
        struct SpecificObject : Object
        {
            SpecificObject(const SceneObject<ShapeT>& obj) :
                m_object(&obj),
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
            Box3 m_aabb;
            Point3f m_center;
        };

        void constructFrom(std::vector<std::unique_ptr<Object>>& objects)
        {
            m_root = makeNode(objects.begin(), objects.end());
        }

        template <typename... SceneObjectCollectionTs>
        void construct(SceneObjectCollectionTs&&... collections)
        {
            std::vector<std::unique_ptr<Object>> allObjects;
            for_each(std::tie(std::forward<SceneObjectCollectionTs>(collections)...), [&](auto&& collection) {
                for (auto&& object : collection)
                {
                    using ObjectType = remove_cvref_t<decltype(object)>;
                    using ShapeType = remove_cvref_t<decltype(object.shape())>;
                    allObjects.emplace_back(std::make_unique<SpecificObject<ShapeType>>(std::forward<ObjectType>(object)));
                }
            });

            constructFrom(allObjects);
        }

        Box3 aabb(ObjectVectorIterator first, ObjectVectorIterator last) const
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

        using Point3fMemberPtr = float(Point3f::*);
        Point3fMemberPtr biggestExtentAxis(ObjectVectorIterator first, ObjectVectorIterator last) const
        {
            Box3 bb = aabb(first, last);
            const Vec3f extent = bb.extent();
            if (extent.x > extent.y && extent.x > extent.z) return &Point3f::x;
            if (extent.y > extent.x && extent.y > extent.z) return &Point3f::y;
            return &Point3f::z;
        }

        ObjectVectorIterator partitionObjects(ObjectVectorIterator first, ObjectVectorIterator last) const
        {
            Point3fMemberPtr cmpAxis = biggestExtentAxis(first, last);

            const int size = static_cast<int>(std::distance(first, last));
            auto mid = std::next(first, size / 2);
            std::nth_element(first, mid, last, [cmpAxis](const auto& lhs, const auto& rhs) {
                return lhs->center().*cmpAxis < rhs->center().*cmpAxis;
            });
         
            const float partitionPoint = (*mid)->center().*cmpAxis;
            return std::partition(first, last, [partitionPoint, cmpAxis](const auto& em) {
                return em->center().*cmpAxis < partitionPoint;
            });
        }

        std::unique_ptr<LeafNodeType> makeLeafNode(ObjectVectorIterator first, ObjectVectorIterator last) const
        {
            auto leaf = std::make_unique<LeafNodeType>();
            while (first != last)
            {
                (*first)->addTo(*leaf);
                ++first;
            }
            return leaf;
        }

        std::unique_ptr<PartitionNodeType> makePartitionNode(ObjectVectorIterator first, ObjectVectorIterator last, int depth) const
        {
            // partition into two sets (TODO: config)
            // call recucively
            ObjectVectorIterator mid = partitionObjects(first, last);
            Box3 bb1 = aabb(first, mid);
            Box3 bb2 = aabb(mid, last);

            auto node = std::make_unique<PartitionNodeType>();
            node->addChild(makeNode(first, mid, depth + 1), std::move(bb1));
            node->addChild(makeNode(mid, last, depth + 1), std::move(bb2));
            return node;
        }

        std::unique_ptr<StaticSceneObjectBvhNode> makeNode(ObjectVectorIterator first, ObjectVectorIterator last, int depth = 0) const
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
