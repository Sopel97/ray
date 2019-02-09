#pragma once

#include "Raycast.h"
#include "SceneObjectBlob.h"
#include "RaycastHit.h"

#include <queue>
#include <optional>
#include <memory>

namespace ray
{
    template <typename BvShapeT>
    struct BoundedStaticBvhNode;
    template <typename BvShapeT>
    struct StaticBvhNodeHit;

    template <typename BvShapeT>
    using BvhNodeHitQueue = std::priority_queue<StaticBvhNodeHit<BvShapeT>, std::vector<StaticBvhNodeHit<BvShapeT>>, std::greater<StaticBvhNodeHit<BvShapeT>>>;

    template <typename BvShapeT>
    struct StaticBvhNode
    {
        virtual std::optional<ResolvableRaycastHit> nextHit(const Ray& ray, BvhNodeHitQueue<BvShapeT>& queue) const = 0;
    };

    template <typename BvShapeT>
    struct BoundedStaticBvhNode
    {
        BoundedStaticBvhNode(std::unique_ptr<StaticBvhNode<BvShapeT>>&& node, const BvShapeT& bv) :
            node(std::move(node)),
            boundingVolume(bv)
        {
        }

        std::unique_ptr<StaticBvhNode<BvShapeT>> node;
        BvShapeT boundingVolume;
    };

    template <typename BvShapeT>
    struct StaticBvhNodeHit
    {
        StaticBvhNodeHit(float dist, const StaticBvhNode<BvShapeT>& node) :
            dist(dist),
            node(&node)
        {
        }

        float dist;
        const StaticBvhNode<BvShapeT>* node;

        friend bool operator<(const StaticBvhNodeHit& lhs, const StaticBvhNodeHit& rhs) noexcept
        {
            return lhs.dist < rhs.dist;
        }
        friend bool operator>(const StaticBvhNodeHit& lhs, const StaticBvhNodeHit& rhs) noexcept
        {
            return lhs.dist > rhs.dist;
        }
    };

    template <typename...>
    struct StaticBvhLeafNode;

    template <typename BvShapeT, typename... ShapeTs>
    struct StaticBvhLeafNode<BvShapeT, Shapes<ShapeTs...>> : StaticBvhNode<BvShapeT>
    {
        using AllShapes = Shapes<ShapeTs...>;

        template <typename ShapeT>
        void add(const SceneObject<ShapeT>& so)
        {
            m_objects.add(so);
        }

        std::optional<ResolvableRaycastHit> nextHit(const Ray& ray, BvhNodeHitQueue<BvShapeT>& queue) const override
        {
            return m_objects.queryNearest(ray);
        }

    private:
        SceneObjectBlob<AllShapes> m_objects;
    };

    template <typename BvShapeT>
    struct StaticBvhPartitionNode : StaticBvhNode<BvShapeT>
    {
        // provide memory from outside to prevent a lot of small allocations
        std::optional<ResolvableRaycastHit> nextHit(const Ray& ray, BvhNodeHitQueue<BvShapeT>& hits) const override
        {
            for (const auto& child : m_children)
            {
                std::optional<RaycastBvHit> hitOpt = raycastBv(ray, child.boundingVolume);
                if (hitOpt)
                {
                    hits.emplace(hitOpt->dist, *child.node);
                }
            }
            return std::nullopt;
        }

        void addChild(std::unique_ptr<StaticBvhNode<BvShapeT>>&& node, const Box3& bb)
        {
            m_children.emplace_back(std::move(node), bb);
        }

    private:
        std::vector<BoundedStaticBvhNode<BvShapeT>> m_children;
    };
}
