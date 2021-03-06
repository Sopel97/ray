#pragma once

#include "BvhParams.h"

#include <ray/math/Ray.h>
#include <ray/math/Raycast.h>
#include <ray/math/RaycastHit.h>

#include <ray/scene/LightHandle.h>
#include <ray/scene/SceneRaycastHit.h>
#include <ray/scene/object/SceneObjectBlob.h>

#include <ray/shape/Shapes.h>

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
    using BvhNodeHitQueue = 
        std::priority_queue<
            StaticBvhNodeHit<BvShapeT>, 
            std::vector<StaticBvhNodeHit<BvShapeT>>, 
            std::greater<StaticBvhNodeHit<BvShapeT>>
        >;

    template <typename BvShapeT>
    struct StaticBvhNode
    {
        [[nodiscard]] virtual bool nextHit(const Ray& ray, BvhNodeHitQueue<BvShapeT>& queue, ResolvableRaycastHit& hit) const = 0;
        virtual void gatherLights(std::vector<LightHandle>& lights) const = 0;
        virtual ~StaticBvhNode() = default;
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

        [[nodiscard]] friend bool operator<(const StaticBvhNodeHit& lhs, const StaticBvhNodeHit& rhs) noexcept
        {
            return lhs.dist < rhs.dist;
        }
        [[nodiscard]] friend bool operator>(const StaticBvhNodeHit& lhs, const StaticBvhNodeHit& rhs) noexcept
        {
            return lhs.dist > rhs.dist;
        }
    };

    template <typename...>
    struct StaticBvhLeafNode;

    template <typename BvShapeT, typename StorageProviderT, typename... ShapeTs>
    struct StaticBvhLeafNode<BvhParams<Shapes<ShapeTs...>, BvShapeT, StorageProviderT>> : StaticBvhNode<BvShapeT>
    {
        using BvhParamsT = BvhParams<Shapes<ShapeTs...>, BvShapeT, StorageProviderT>;
        using AllShapes = Shapes<ShapeTs...>;
        using BvShapeType = BvShapeT;

        template <typename ShapeT>
        void add(const SceneObject<ShapeT>& so)
        {
            m_objects.add(so);
        }

        template <typename ShapeT>
        void add(SceneObject<ShapeT>&& so)
        {
            m_objects.add(std::move(so));
        }

        [[nodiscard]] bool nextHit(const Ray& ray, BvhNodeHitQueue<BvShapeT>& queue, ResolvableRaycastHit& hit) const override
        {
            return m_objects.queryNearest(ray, hit);
        }

        void gatherLights(std::vector<LightHandle>& lights) const override
        {
            m_objects.gatherLights(lights);
        }

    private:
        SceneObjectBlob<AllShapes, StorageProviderT> m_objects;
    };


    template <typename BvShapeT>
    struct StaticBvhPartitionNode : StaticBvhNode<BvShapeT>
    {
        struct Child
        {
            Child(std::unique_ptr<StaticBvhNode<BvShapeT>>&& node, const BvShapeT& bv) :
                node(std::move(node)),
                boundingVolume(bv)
            {
            }

            Child(std::unique_ptr<StaticBvhNode<BvShapeT>>&& node, BvShapeT&& bv) :
                node(std::move(node)),
                boundingVolume(std::move(bv))
            {
            }

            std::unique_ptr<StaticBvhNode<BvShapeT>> node;
            BvShapeT boundingVolume;
        };

        // provide memory from outside to prevent a lot of small allocations
        [[nodiscard]] bool nextHit(const Ray& ray, BvhNodeHitQueue<BvShapeT>& hits, ResolvableRaycastHit& hit) const override
        {
            RaycastBvHit bvhit;
            for (const auto& child : m_children)
            {
                if (raycastBv(ray, child.boundingVolume, hit.dist, bvhit))
                {
                    hits.push(StaticBvhNodeHit(bvhit.dist, *child.node));
                }
            }
            return false;
        }

        void addChild(std::unique_ptr<StaticBvhNode<BvShapeT>>&& node, const BvShapeT& bv)
        {
            m_children.emplace_back(std::move(node), bv);
        }

        void addChild(std::unique_ptr<StaticBvhNode<BvShapeT>>&& node, BvShapeT&& bv)
        {
            m_children.emplace_back(std::move(node), std::move(bv));
        }

        void gatherLights(std::vector<LightHandle>& lights) const override
        {
            for (const auto& child : m_children)
            {
                child.node->gatherLights(lights);
            }
        }

    private:
        std::vector<Child> m_children;
    };
}
