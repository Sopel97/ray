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
    struct StaticSceneObjectBvhNode
    {
        virtual bool isLeaf() const = 0;
    };

    template <typename... ShapeTs>
    struct StaticSceneObjectBvhLeaf : StaticSceneObjectBvhNode
    {
        template <typename ShapeT>
        void add(const SceneObject<ShapeT>& so)
        {
            m_objects.add(so);
        }

        bool isLeaf() const override
        {
            return true;
        }

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray, RaycastQueryStats* stats = nullptr) const
        {
            return m_objects.queryNearest(ray, stats);
        }
        
    private:
        SceneObjectBlob<ShapeTs...> m_objects;
    };

    template <typename... ShapeTs>
    struct StaticSceneObjectBvhPartitionNode : StaticSceneObjectBvhNode
    {
        struct Entry
        {
            Entry(std::unique_ptr<StaticSceneObjectBvhNode>&& node, const Box3& bb) :
                node(std::move(node)),
                aabb(bb)
            {
            }

            std::unique_ptr<StaticSceneObjectBvhNode> node;
            Box3 aabb;
        };

        struct EntryHit
        {
            EntryHit(float dist, const StaticSceneObjectBvhNode& node) :
                dist(dist),
                node(&node)
            {
            }

            float dist;
            const StaticSceneObjectBvhNode* node;

            bool operator<(const EntryHit& rhs) const noexcept
            {
                return dist < rhs.dist;
            }
            bool operator>(const EntryHit& rhs) const noexcept
            {
                return dist > rhs.dist;
            }
        };

        // provide memory from outside to prevent a lot of small allocations
        void gatherHits(const Ray& ray, std::priority_queue<EntryHit, std::vector<EntryHit>, std::greater<EntryHit>>& hits) const
        {
            for (const auto& child : m_children)
            {
                std::optional<RaycastBvHit> hitOpt = raycastBv(ray, child.aabb);
                if (hitOpt)
                {
                    hits.emplace(hitOpt->dist, *child.node);
                }
            }
        }

        bool isLeaf() const override
        {
            return false;
        }

        void addChild(std::unique_ptr<StaticSceneObjectBvhNode>&& node, const Box3& bb)
        {
            m_children.emplace_back(std::move(node), bb);
        }

    private:
        std::vector<Entry> m_children;
    };

    template <typename... ShapeTs>
    struct StaticSceneObjectBvh : StaticSceneObjectStorage
    {
        static constexpr int maxDepth = 16;
        static constexpr int maxObjectsPerNode = 8;
    private:
        // specialized whenever a pack is used
        template <typename ShapeT>
        struct ObjectStorage { using StorageType = SceneObjectArray<ShapeT>; };

        template <typename T>
        using ObjectStorageType = typename ObjectStorage<T>::StorageType;

    public:
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

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray, RaycastQueryStats* stats = nullptr) const
        {
            using EntryHit = typename StaticSceneObjectBvhPartitionNode<ShapeTs...>::EntryHit;
            std::priority_queue<EntryHit, std::vector<EntryHit>, std::greater<EntryHit>> queue;
            queue.push(EntryHit(0.0f, *m_root));
            float nearestHitDist = std::numeric_limits<float>::max();
            std::optional<ResolvableRaycastHit> nearestHitOpt = std::nullopt;
            while (!queue.empty())
            {
                EntryHit entry = queue.top();
                queue.pop();
                if (entry.dist >= nearestHitDist) break;

                if (entry.node->isLeaf())
                {
                    const auto& leaf = static_cast<const StaticSceneObjectBvhLeaf<ShapeTs...>&>(*entry.node);
                    std::optional<ResolvableRaycastHit> hitOpt = leaf.queryNearest(ray, stats);
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
                else
                {
                    const auto& node = static_cast<const StaticSceneObjectBvhPartitionNode<ShapeTs...>&>(*entry.node);
                    node.gatherHits(ray, queue);
                }
            }

            return nearestHitOpt;
        }

    private:
        std::unique_ptr<StaticSceneObjectBvhNode> m_root;

        struct Object
        {
            virtual void addTo(StaticSceneObjectBvhLeaf<ShapeTs...>& leaf) const = 0;
            virtual const Box3& aabb() const = 0;
            virtual const Point3f& center() const = 0;
        };

        template <typename ShapeT>
        struct SpecificObject : Object
        {
            SpecificObject(const SceneObject<ShapeT>& obj) :
                m_object(&obj),
                m_aabb(obj.aabb()),
                m_center(obj.center())
            {

            }

            void addTo(StaticSceneObjectBvhLeaf<ShapeTs...>& leaf) const override
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
            for_each(std::tie(collections...), [&](auto&& collection) {
                for (auto&& object : collection)
                {
                    allObjects.emplace_back(std::make_unique<SpecificObject<remove_cvref_t<decltype(object.shape())>>>(object));
                }
            });

            constructFrom(allObjects);
        }

        Box3 aabb(typename std::vector<std::unique_ptr<Object>>::iterator first, typename std::vector<std::unique_ptr<Object>>::iterator last) const
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
        Point3fMemberPtr biggestExtentAxis(typename std::vector<std::unique_ptr<Object>>::iterator first, typename std::vector<std::unique_ptr<Object>>::iterator last) const
        {
            Box3 bb = aabb(first, last);
            const Vec3f extent = bb.extent();
            if (extent.x > extent.y && extent.x > extent.z) return &Point3f::x;
            if (extent.y > extent.x && extent.y > extent.z) return &Point3f::y;
            return &Point3f::z;
        }

        std::unique_ptr<StaticSceneObjectBvhNode> makeNode(typename std::vector<std::unique_ptr<Object>>::iterator first, typename std::vector<std::unique_ptr<Object>>::iterator last, int depth = 0) const
        {
            const int size = static_cast<int>(std::distance(first, last));
            if (size <= maxObjectsPerNode || depth >= maxDepth)
            {
                // create a leaf
                auto leaf = std::make_unique<StaticSceneObjectBvhLeaf<ShapeTs...>>();
                while (first != last)
                {
                    (*first)->addTo(*leaf);
                    ++first;
                }
                return leaf;
            }
            else
            {
                // partition into two sets (TODO: config)
                // call recucively

                Point3fMemberPtr cmpAxis = biggestExtentAxis(first, last);
                auto mid = std::next(first, size / 2);
                std::nth_element(first, mid, last, [cmpAxis](const std::unique_ptr<Object>& lhs, const std::unique_ptr<Object>& rhs) {
                    return lhs->center().*cmpAxis < rhs->center().*cmpAxis;
                });
                const float partitionPoint = (*mid)->center().*cmpAxis;
                mid = std::partition(first, last, [partitionPoint, cmpAxis](const std::unique_ptr<Object>& em) {
                    return em->center().*cmpAxis < partitionPoint;
                });

                Box3 bb1 = aabb(first, mid);
                Box3 bb2 = aabb(mid, last);

                auto node = std::make_unique<StaticSceneObjectBvhPartitionNode<ShapeTs...>>();
                node->addChild(makeNode(first, mid, depth + 1), bb1);
                node->addChild(makeNode(mid, last, depth + 1), bb2);
                return node;
            }
        }
    };
}
