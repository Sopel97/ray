#pragma once

#include "SceneObjectId.h"

#include <ray/material/Material.h>
#include <ray/material/MaterialPtrStorage.h>
#include <ray/material/SurfaceShader.h>
#include <ray/material/TexCoords.h>

#include <ray/math/BoundingVolume.h>
#include <ray/math/Interval.h>
#include <ray/math/Raycast.h>
#include <ray/math/RaycastHit.h>
#include <ray/math/Vec3.h>

#include <ray/scene/SceneRaycastHit.h>

#include <ray/shape/Box3.h>
#include <ray/shape/ShapeTags.h>
#include <ray/shape/ShapeTraits.h>

#include <array>
#include <atomic>
#include <optional>
#include <memory>

namespace ray
{
    template <typename ShapeT>
    using MaterialPtrStorageType = MaterialPtrStorage<
        ShapeTraits<ShapeT>::numSurfaceMaterialsPerShape,
        ShapeTraits<ShapeT>::numMediumMaterialsPerShape
    >;

    namespace detail
    {
        static inline std::atomic<SceneObjectId> gNextSceneObjectId = 0;
    }

    // Stores a shape and its material[s].
    template <typename ShapeT>
    struct SceneObject
    {
        using ShapeType = ShapeT;
        using ShapeTraits = ShapeTraits<ShapeType>;
        using BaseShapeType = typename ShapeTraits::BaseShapeType;
        static constexpr bool isPack = ShapeTraits::numShapes > 1;
        static constexpr bool isBounded = ShapeTraits::isBounded;
        using MaterialStorageType = MaterialPtrStorageType<ShapeType>;
        using MaterialStorageViewType = MaterialPtrStorageView;
        using SurfaceShaderType = SurfaceShader<ShapeT>;
        using SurfaceShaderPtrType = const SurfaceShaderType*;

        static_assert(!isPack, "A single scene object must not be a pack. Use SceneObjectArray.");

        SceneObject(const ShapeType& shape, const MaterialStorageType& materials) :
            m_shape(shape),
            m_materials(materials),
            m_shader(&DefaultSurfaceShader<ShapeT>::instance()),
            m_id(detail::gNextSceneObjectId.fetch_add(1))
        {

        }

        SceneObject(const ShapeType& shape, const MaterialStorageType& materials, const SurfaceShaderPtrType& shader) :
            m_shape(shape),
            m_materials(materials),
            m_shader(&shader),
            m_id(detail::gNextSceneObjectId.fetch_add(1))
        {

        }

        SceneObject(const SceneObject&) = default;
        SceneObject(SceneObject&&) noexcept = default;
        SceneObject& operator=(const SceneObject&) = default;
        SceneObject& operator=(SceneObject&&) noexcept = default;

        [[nodiscard]] decltype(auto) center() const
        {
            return m_shape.center();
        }

        [[nodiscard]] decltype(auto) aabb() const
        {
            return boundingVolume<Box3>(m_shape);
        }

        [[nodiscard]] const MaterialStorageType& materials() const
        {
            return m_materials;
        }

        [[nodiscard]] MaterialStorageViewType materialsView() const
        {
            return m_materials.view();
        }

        [[nodiscard]] const ShapeType& shape() const
        {
            return m_shape;
        }

        [[nodiscard]] const SurfaceShaderType& shader() const
        {
            return *m_shader;
        }

        [[nodiscard]] bool isLight() const
        {
            return isBounded && m_materials.isEmissive();
        }

        [[nodiscard]] SceneObjectId id() const
        {
            return m_id;
        }

    private:
        ShapeType m_shape;
        MaterialStorageType m_materials;
        SurfaceShaderPtrType m_shader;
        SceneObjectId m_id;
    };

    // Stores a tree of polymorphic volumetric shapes with boolean operations
    // Supports shape packs as single shapes
    // A copy is shallow, ie. all data is shared.
    template <>
    struct SceneObject<CsgShape>
    {
    public:
        struct CsgPrimitiveBase;

    private:
        struct CsgIntervalData
        {
            const CsgPrimitiveBase* shape;
            bool invert;
        };

        using CsgHitIntervals = IntervalSet<CsgIntervalData>;
        using CsgHitIntervalsStack = std::vector<IntervalSet<CsgIntervalData>>;
        using CsgHitIntervalsStackIter = typename CsgHitIntervalsStack::iterator;

        struct CsgNode
        {
            [[nodiscard]] virtual bool raycastIntervals(const Ray& ray, CsgHitIntervals& hitIntervals, CsgHitIntervalsStackIter scratchHitIntervals, bool invert = false) const = 0;
            [[nodiscard]] virtual Box3 aabb() const = 0;
            [[nodiscard]] virtual int depth() const = 0;
        };

    public:
        struct CsgPrimitiveBase : CsgNode
        {
            using MaterialStorageViewType = MaterialPtrStorageView;

            [[nodiscard]] virtual bool raycast(const Ray& ray, RaycastHit& hit) const = 0;
            [[nodiscard]] virtual ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const = 0;
            [[nodiscard]] virtual MaterialStorageViewType materialsView() const = 0;
            [[nodiscard]] virtual bool isLight() const = 0;
            [[nodiscard]] virtual SceneObjectId id() const = 0;
        };

    private:
        template <typename ShapeT>
        struct CsgPrimitiveImpl : CsgPrimitiveBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            static constexpr bool hasVolume = ShapeTraits<ShapeT>::hasVolume;
            static constexpr bool isLocallyContinuable = ShapeTraits<ShapeT>::isLocallyContinuable;
            using MaterialStorageType = MaterialPtrStorageType<ShapeT>;
            using MaterialStorageViewType = MaterialPtrStorageView;
            using SurfaceShaderType = SurfaceShader<ShapeT>;
            using SurfaceShaderPtrType = const SurfaceShaderType*;

            static_assert(isBounded, "Must be bounded.");
            static_assert(hasVolume, "Must have volume.");
            static_assert(isLocallyContinuable, "Must be locally continuable.");

            CsgPrimitiveImpl(const ShapeT& shape, const MaterialStorageType& materials, const SurfaceShaderType& shader) :
                m_shape(shape),
                m_materials(materials),
                m_shader(&shader),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }
            [[nodiscard]] Box3 aabb() const override
            {
                return boundingVolume<Box3>(m_shape);
            }
            [[nodiscard]] bool raycast(const Ray& ray, RaycastHit& hit) const override
            {
                return ray::raycast(ray, m_shape, hit);
            }
            [[nodiscard]] bool raycastIntervals(const Ray& ray, CsgHitIntervals& hitIntervals, CsgHitIntervalsStackIter scratchHitIntervals, bool invert = false) const override
            {
                hitIntervals.clear();
                const bool anyInterval = ray::raycastIntervals(ray, m_shape, hitIntervals);
                if (anyInterval)
                {
                    hitIntervals.setData(CsgIntervalData{ this, invert });
                    return true;
                }
                
                return false;
            }
            [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const override
            {
                auto[surface, medium] = materialsView().material(hit.materialIndex);
                return ResolvedRaycastHit(
                    m_shader->shade(m_shape, hit, materialsView()),
                    hit.shapeNo, medium, nullptr, hit.isInside, true, true
                );
            }
            [[nodiscard]] MaterialStorageViewType materialsView() const override
            {
                return m_materials.view();
            }
            [[nodiscard]] bool isLight() const override
            {
                return m_materials.isEmissive();
            }
            [[nodiscard]] SceneObjectId id() const override
            {
                return m_id;
            }

            [[nodiscard]] int depth() const override
            {
                return 1;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SurfaceShaderPtrType m_shader;
            SceneObjectId m_id;
        };

        struct CsgBinaryOperation : CsgNode
        {
            CsgBinaryOperation(std::shared_ptr<CsgNode> lhs, std::shared_ptr<CsgNode> rhs) :
                m_lhs(std::move(lhs)),
                m_rhs(std::move(rhs)),
                m_aabb(aabb()),
                m_depth(std::max(m_lhs->depth(), m_rhs->depth()) + 1)
            {
            }

            [[nodiscard]] Box3 aabb() const override
            {
                Box3 b = m_lhs->aabb();
                b.extend(m_rhs->aabb());
                return b;
            }

            [[nodiscard]] int depth() const override
            {
                return m_depth;
            }

        protected:
            std::shared_ptr<CsgNode> m_lhs;
            std::shared_ptr<CsgNode> m_rhs;
            Box3 m_aabb;
            int m_depth;
        };

        struct CsgUnion : CsgBinaryOperation
        {
            using CsgBinaryOperation::CsgBinaryOperation;

            [[nodiscard]] bool raycastIntervals(const Ray& ray, CsgHitIntervals& hitIntervals, CsgHitIntervalsStackIter scratchHitIntervals, bool invert = false) const override
            {
                RaycastBvHit bvHit;
                if (!ray::raycastBv(ray, m_aabb, std::numeric_limits<float>::max(), bvHit))
                {
                    hitIntervals.clear();
                    return false;
                }
                if (!m_lhs->raycastIntervals(ray, hitIntervals, std::next(scratchHitIntervals), invert))
                {
                    return m_rhs->raycastIntervals(ray, hitIntervals, std::next(scratchHitIntervals), invert);
                }
                else if(m_rhs->raycastIntervals(ray, *scratchHitIntervals, std::next(scratchHitIntervals), invert))
                {
                    hitIntervals |= *scratchHitIntervals;
                }
                return !hitIntervals.isEmpty();
            }
        };

        struct CsgIntersection : CsgBinaryOperation
        {
            using CsgBinaryOperation::CsgBinaryOperation;

            [[nodiscard]] bool raycastIntervals(const Ray& ray, CsgHitIntervals& hitIntervals, CsgHitIntervalsStackIter scratchHitIntervals, bool invert = false) const override
            {
                RaycastBvHit bvHit;
                if (!ray::raycastBv(ray, m_aabb, std::numeric_limits<float>::max(), bvHit))
                {
                    hitIntervals.clear();
                    return false;
                }
                if (!m_rhs->raycastIntervals(ray, *scratchHitIntervals, std::next(scratchHitIntervals), invert))
                {
                    hitIntervals.clear();
                    return false;
                }
                if (!m_lhs->raycastIntervals(ray, hitIntervals, std::next(scratchHitIntervals), invert))
                {
                    return false;
                }

                hitIntervals &= *scratchHitIntervals;
                return !hitIntervals.isEmpty();
            }
        };

        struct CsgDifference : CsgBinaryOperation
        {
            using CsgBinaryOperation::CsgBinaryOperation;

            [[nodiscard]] bool raycastIntervals(const Ray& ray, CsgHitIntervals& hitIntervals, CsgHitIntervalsStackIter scratchHitIntervals, bool invert = false) const override
            {
                RaycastBvHit bvHit;
                if (!ray::raycastBv(ray, m_aabb, std::numeric_limits<float>::max(), bvHit))
                {
                    hitIntervals.clear();
                    return false;
                }
                if (!m_lhs->raycastIntervals(ray, hitIntervals, std::next(scratchHitIntervals), invert))
                {
                    return false;
                }
                if(m_rhs->raycastIntervals(ray, *scratchHitIntervals, std::next(scratchHitIntervals), !invert))
                {
                    hitIntervals -= *scratchHitIntervals;
                }
                return !hitIntervals.isEmpty();
            }
        };

        SceneObject(std::shared_ptr<CsgNode> op) :
            m_obj(op),
            m_id(detail::gNextSceneObjectId.fetch_add(1))
        {

        }

    public:
        using ShapeType = CsgShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialPtrStorageType<ShapeT>& materials, const SurfaceShader<ShapeT>& shader) :
            m_obj(std::make_shared<CsgPrimitiveImpl<ShapeT>>(shape, materials, shader)),
            m_id(detail::gNextSceneObjectId.fetch_add(1))
        {

        }

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialPtrStorageType<ShapeT>& materials) :
            m_obj(std::make_unique<CsgPrimitiveImpl<ShapeT>>(shape, materials, defaultShader<ShapeT>))
        {

        }

        SceneObject(const SceneObject&) = default;
        SceneObject(SceneObject&&) noexcept = default;
        SceneObject& operator=(const SceneObject&) = default;
        SceneObject& operator=(SceneObject&&) noexcept = default;

        [[nodiscard]] const CsgPrimitiveBase* raycast(const Ray& ray, RaycastHit& hit) const
        {
            thread_local CsgHitIntervalsStack allHitIntervals;

            const int depth = m_obj->depth();
            if (allHitIntervals.size() < depth)
            {
                // technically +1 is not needed because identity doesn't use scratch, but just to be safe
                allHitIntervals.resize(depth + 1);
            }
            auto& hitIntervals = allHitIntervals[0];
            auto scratchHitIntervalsIter = std::next(allHitIntervals.begin());
            hitIntervals.clear();
            const bool anyHit = m_obj->raycastIntervals(ray, hitIntervals, scratchHitIntervalsIter);

            for (const auto& interval : hitIntervals)
            {
                // NOTE: we'll see how it goes, kinda hacky
                // Instead of remembering all information about all hits
                // we only remember the object and the distance
                // so that we can later reraycast it from close by hoping to 
                // get what we want
                // If this doesn't work properly then a specialized raycast function
                // for each shape that returns a hit closest to the given
                // distance will be required. Or just collect all hits on the way - may be too expensive.
                // NOTE: for now, after testing, seems ok
                if (interval.max > 0.0f)
                {
                    constexpr float padding = 0.001f;
                    const CsgIntervalData& data = interval.min > 0.0f ? interval.minData : interval.maxData;
                    // apply padding because we are raycasting it again, we don't want to miss it
                    const float dist = (interval.min > 0.0f ? interval.min : interval.max) - padding;
                    const Ray offsetRay = ray.translated(ray.direction() * dist);
                    const CsgPrimitiveBase* shape = nullptr;
                    hit.dist -= dist;
                    if (data.shape->raycast(offsetRay, hit))
                    {
                        // technically we should always hit, but just to be safe from floating point inaccuracy
                        shape = data.shape;
                        if (data.invert)
                        {
                            hit.isInside = !hit.isInside;
                        }
                    }
                    hit.dist += dist;

                    return shape;
                }
            }

            return nullptr;
        }

        [[nodiscard]] decltype(auto) aabb() const
        {
            return m_obj->aabb();
        }

        [[nodiscard]] decltype(auto) center() const
        {
            return aabb().center();
        }

        [[nodiscard]] bool hasVolume() const
        {
            return true;
        }

        [[nodiscard]] bool isLocallyContinuable() const
        {
            return true;
        }

        // TODO: maybe this properly
        [[nodiscard]] bool isLight() const
        {
            return false;
        }

        [[nodiscard]] SceneObjectId id() const
        {
            return m_id;
        }

        [[nodiscard]] friend SceneObject<CsgShape> operator|(const SceneObject<CsgShape>& lhs, const SceneObject<CsgShape>& rhs)
        {
            return SceneObject<CsgShape>(std::make_shared<CsgUnion>(lhs.m_obj, rhs.m_obj));
        }

        [[nodiscard]] friend SceneObject<CsgShape> operator&(const SceneObject<CsgShape>& lhs, const SceneObject<CsgShape>& rhs)
        {
            return SceneObject<CsgShape>(std::make_shared<CsgIntersection>(lhs.m_obj, rhs.m_obj));
        }

        [[nodiscard]] friend SceneObject<CsgShape> operator-(const SceneObject<CsgShape>& lhs, const SceneObject<CsgShape>& rhs)
        {
            return SceneObject<CsgShape>(std::make_shared<CsgDifference>(lhs.m_obj, rhs.m_obj));
        }

    private:
        std::shared_ptr<CsgNode> m_obj;
        SceneObjectId m_id;
    };

    // TODO: think how to merge the following cases nicely

    // Stores any shape in a polymorphic wrapper.
    // Only supports single shapes
    // Copy performs deep copy of the shape, material array, and id
    template <>
    struct SceneObject<BoundedUniqueAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            using MaterialStorageViewType = MaterialPtrStorageView;

            [[nodiscard]] virtual Point3f center() const = 0;
            [[nodiscard]] virtual bool raycast(const Ray& ray, RaycastHit& hit) const = 0;
            [[nodiscard]] virtual std::unique_ptr<PolymorphicSceneObjectBase> clone() const = 0;
            [[nodiscard]] virtual ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const = 0;
            [[nodiscard]] virtual bool hasVolume() const = 0;
            [[nodiscard]] virtual bool isLocallyContinuable() const = 0;
            [[nodiscard]] virtual Box3 aabb() const = 0;
            [[nodiscard]] virtual MaterialStorageViewType materials() const = 0;
            [[nodiscard]] virtual bool isLight() const = 0;
            [[nodiscard]] virtual SceneObjectId id() const = 0;
        };

        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(isBounded, "Must be bounded.");
            using MaterialStorageType = MaterialPtrStorageType<ShapeT>;
            using MaterialStorageViewType = MaterialPtrStorageView;
            using SurfaceShaderType = SurfaceShader<ShapeT>;
            using SurfaceShaderPtrType = const SurfaceShaderType*;

            template <typename... ArgsTs>
            PolymorphicSceneObject(const ShapeT& shape, const MaterialStorageType& materials, const SurfaceShaderType& shader) :
                m_shape(shape),
                m_materials(materials),
                m_shader(&shader),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }

            [[nodiscard]] Point3f center() const override
            {
                return m_shape.center();
            }
            [[nodiscard]] Box3 aabb() const override
            {
                return boundingVolume<Box3>(m_shape);
            }
            [[nodiscard]] bool raycast(const Ray& ray, RaycastHit& hit) const override
            {
                return ray::raycast(ray, m_shape, hit);
            }
            [[nodiscard]] std::unique_ptr<PolymorphicSceneObjectBase> clone() const override
            {
                return std::make_unique<PolymorphicSceneObject<ShapeT>>(*this);
            }
            [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const override
            {
                auto[surface, medium] = materialsView(hit.shapeNo).material(hit.materialIndex);
                return ResolvedRaycastHit(
                    m_shader->shade(m_shape, hit, materialsView()),
                    hit.shapeNo, medium, nullptr, hit.isInside, hasVolume(), isLocallyContinuable()
                );
            }
            [[nodiscard]] bool hasVolume() const override
            {
                return ShapeTraits<ShapeT>::hasVolume;
            }
            [[nodiscard]] bool isLocallyContinuable() const override
            {
                return ShapeTraits<ShapeT>::isLocallyContinuable;
            }
            [[nodiscard]] MaterialStorageViewType material() const override
            {
                return m_materials.view();
            }
            [[nodiscard]] bool isLight() const override
            {
                return m_materials.isEmissive();
            }
            [[nodiscard]] SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SurfaceShaderPtrType m_shader;
            SceneObjectId m_id;
        };

    public:
        using ShapeType = BoundedUniqueAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialPtrStorageType<ShapeT>& materials, const SurfaceShader<ShapeT>& shader) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(shape, materials, shader))
        {

        }

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialPtrStorageType<ShapeT>& materials) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(shape, materials, defaultShader<ShapeT>))
        {

        }

        SceneObject(const SceneObject& other) :
            m_obj(other.m_obj->clone())
        {

        }
        SceneObject& operator=(const SceneObject& other)
        {
            m_obj = other.m_obj->clone();
        }

        SceneObject(SceneObject&&) noexcept = default;
        SceneObject& operator=(SceneObject&&) noexcept = default;

        [[nodiscard]] SceneObjectId id() const
        {
            return m_obj->id();
        }

        [[nodiscard]] Point3f center() const
        {
            return m_obj->center();
        }

        [[nodiscard]] Box3 aabb() const
        {
            return m_obj->aabb();
        }

        [[nodiscard]] bool raycast(const Ray& ray, RaycastHit& hit) const
        {
            return m_obj->raycast(ray, hit);
        }

        [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const
        {
            return m_obj->resolveHit(hit);
        }

        [[nodiscard]] bool hasVolume() const
        {
            return m_obj->hasVolume();
        }
        [[nodiscard]] bool isLocallyContinuable() const
        {
            return m_obj->isLocallyContinuable();
        }

        [[nodiscard]] MaterialPtrStorageView materialsView() const
        {
            return m_obj->materials();
        }

        [[nodiscard]] bool isLight() const
        {
            return m_obj->isLight();
        }

    private:
        std::unique_ptr<PolymorphicSceneObjectBase> m_obj;
    };

    // Stores any shape in a polymorphic wrapper.
    // Only supports single shapes
    // A copy is shallow, ie. all data is shared.
    template <>
    struct SceneObject<BoundedSharedAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            using MaterialStorageViewType = MaterialPtrStorageView;

            [[nodiscard]] virtual Point3f center() const = 0;
            [[nodiscard]] virtual bool raycast(const Ray& ray, RaycastHit& hit) const = 0;
            [[nodiscard]] virtual ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const = 0;
            [[nodiscard]] virtual bool hasVolume() const = 0;
            [[nodiscard]] virtual bool isLocallyContinuable() const = 0;
            [[nodiscard]] virtual Box3 aabb() const = 0;
            [[nodiscard]] virtual MaterialStorageViewType materials() const = 0;
            [[nodiscard]] virtual bool isLight() const = 0;
            [[nodiscard]] virtual SceneObjectId id() const = 0;
        };

        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(isBounded, "Must be bounded.");
            using MaterialStorageType = MaterialPtrStorageType<ShapeT>;
            using MaterialStorageViewType = MaterialPtrStorageView;
            using SurfaceShaderType = SurfaceShader<ShapeT>;
            using SurfaceShaderPtrType = const SurfaceShaderType*;

            template <typename... ArgsTs>
            PolymorphicSceneObject(const ShapeT& shape, const MaterialStorageType& materials, const SurfaceShaderType& shader) :
                m_shape(shape),
                m_materials(materials),
                m_shader(&shader),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }

            [[nodiscard]] Point3f center() const override
            {
                return m_shape.center();
            }
            [[nodiscard]] Box3 aabb() const override
            {
                return boundingVolume<Box3>(m_shape);
            }
            [[nodiscard]] bool raycast(const Ray& ray, RaycastHit& hit) const override
            {
                return ray::raycast(ray, m_shape, hit);
            }
            [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const override
            {
                auto[surface, medium] = materialsView(hit.shapeNo).material(hit.materialIndex);
                return ResolvedRaycastHit(
                    m_shader->shade(m_shape, hit, materialsView()),
                    hit.shapeNo, medium, nullptr, hit.isInside, hasVolume(), isLocallyContinuable()
                );
            }
            [[nodiscard]] bool hasVolume() const override
            {
                return ShapeTraits<ShapeT>::hasVolume;
            }
            [[nodiscard]] bool isLocallyContinuable() const override
            {
                return ShapeTraits<ShapeT>::isLocallyContinuable;
            }
            [[nodiscard]] MaterialStorageViewType materials() const override
            {
                return m_materials.view();
            }
            [[nodiscard]] bool isLight() const override
            {
                return m_materials.isEmissive();
            }
            [[nodiscard]] SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SurfaceShaderPtrType m_shader;
            SceneObjectId m_id;
        };

    public:
        using ShapeType = BoundedSharedAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialPtrStorageType<ShapeT>& materials, const SurfaceShader<ShapeT>& shader) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(shape, materials, shader))
        {

        }

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialPtrStorageType<ShapeT>& materials) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(shape, materials, defaultShader<ShapeT>))
        {

        }

        SceneObject(const SceneObject&) = default;
        SceneObject(SceneObject&&) noexcept = default;
        SceneObject& operator=(const SceneObject&) = default;
        SceneObject& operator=(SceneObject&&) noexcept = default;

        [[nodiscard]] SceneObjectId id() const
        {
            return m_obj->id();
        }

        [[nodiscard]] Point3f center() const
        {
            return m_obj->center();
        }

        [[nodiscard]] Box3 aabb() const
        {
            return m_obj->aabb();
        }

        [[nodiscard]] bool raycast(const Ray& ray, RaycastHit& hit) const
        {
            return m_obj->raycast(ray, hit);
        }

        [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const
        {
            return m_obj->resolveHit(hit);
        }

        [[nodiscard]] bool hasVolume() const
        {
            return m_obj->hasVolume();
        }

        [[nodiscard]] bool isLocallyContinuable() const
        {
            return m_obj->isLocallyContinuable();
        }

        [[nodiscard]] MaterialPtrStorageView materialsView() const
        {
            return m_obj->materials();
        }

        [[nodiscard]] bool isLight() const
        {
            return m_obj->isLight();
        }

    private:
        std::shared_ptr<PolymorphicSceneObjectBase> m_obj;
    };


    // Stores any shape in a polymorphic wrapper.
    // Only supports single shapes
    // Copy performs deep copy of the shape, material array, and id
    template <>
    struct SceneObject<UnboundedUniqueAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            using MaterialStorageViewType = MaterialPtrStorageView;

            [[nodiscard]] virtual bool raycast(const Ray& ray, RaycastHit& hit) const = 0;
            [[nodiscard]] virtual std::unique_ptr<PolymorphicSceneObjectBase> clone() const = 0;
            [[nodiscard]] virtual ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const = 0;
            [[nodiscard]] virtual bool hasVolume() const = 0;
            [[nodiscard]] virtual bool isLocallyContinuable() const = 0;
            [[nodiscard]] virtual MaterialStorageViewType materials() const = 0;
            [[nodiscard]] virtual SceneObjectId id() const = 0;
        };

        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(!isBounded, "Must not be bounded.");
            using MaterialStorageType = MaterialPtrStorageType<ShapeT>;
            using MaterialStorageViewType = MaterialPtrStorageView;
            using SurfaceShaderType = SurfaceShader<ShapeT>;
            using SurfaceShaderPtrType = const SurfaceShaderType*;

            template <typename... ArgsTs>
            PolymorphicSceneObject(const ShapeT& shape, const MaterialStorageType& materials, const SurfaceShaderType& shader) :
                m_shape(shape),
                m_materials(materials),
                m_shader(&shader),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }

            [[nodiscard]] bool raycast(const Ray& ray, RaycastHit& hit) const override
            {
                return ray::raycast(ray, m_shape, hit);
            }
            [[nodiscard]] std::unique_ptr<PolymorphicSceneObjectBase> clone() const override
            {
                return std::make_unique<PolymorphicSceneObject<ShapeT>>(*this);
            }
            [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const override
            {
                auto[surface, medium] = materialsView(hit.shapeNo).material(hit.materialIndex);
                return ResolvedRaycastHit(
                    m_shader->shade(m_shape, hit, materialsView()),
                    hit.shapeNo, medium, nullptr, hit.isInside, hasVolume(), isLocallyContinuable()
                );
            }
            [[nodiscard]] bool hasVolume() const override
            {
                return ShapeTraits<ShapeT>::hasVolume;
            }
            [[nodiscard]] bool isLocallyContinuable() const override
            {
                return ShapeTraits<ShapeT>::isLocallyContinuable;
            }
            [[nodiscard]] MaterialStorageViewType material() const override
            {
                return m_materials.view();
            }
            [[nodiscard]] SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SurfaceShaderPtrType m_shader;
            SceneObjectId m_id;
        };

    public:
        using ShapeType = UnboundedUniqueAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialPtrStorageType<ShapeT>& materials, const SurfaceShader<ShapeT>& shader) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(shape, materials, shader))
        {

        }

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialPtrStorageType<ShapeT>& materials) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(shape, materials, defaultShader<ShapeT>))
        {

        }

        SceneObject(const SceneObject& other) :
            m_obj(other.m_obj->clone())
        {

        }
        SceneObject& operator=(const SceneObject& other)
        {
            m_obj = other.m_obj->clone();
        }

        SceneObject(SceneObject&&) noexcept = default;
        SceneObject& operator=(SceneObject&&) noexcept = default;

        [[nodiscard]] SceneObjectId id() const
        {
            return m_obj->id();
        }

        [[nodiscard]] bool raycast(const Ray& ray, RaycastHit& hit) const
        {
            return m_obj->raycast(ray, hit);
        }

        [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const
        {
            return m_obj->resolveHit(hit);
        }

        [[nodiscard]] bool hasVolume() const
        {
            return m_obj->hasVolume();
        }

        [[nodiscard]] bool isLocallyContinuable() const
        {
            return m_obj->isLocallyContinuable();
        }

        [[nodiscard]] MaterialPtrStorageView materialsView() const
        {
            return m_obj->materials();
        }

        [[nodiscard]] bool isLight() const
        {
            // unbounded objects cannot be light emitters
            return false;
        }

    private:
        std::unique_ptr<PolymorphicSceneObjectBase> m_obj;
    };

    // Stores any shape in a polymorphic wrapper.
    // Only supports single shapes
    // A copy is shallow, ie. all data is shared.
    template <>
    struct SceneObject<UnboundedSharedAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            using MaterialStorageViewType = MaterialPtrStorageView;

            [[nodiscard]] virtual bool raycast(const Ray& ray, RaycastHit& hit) const = 0;
            [[nodiscard]] virtual ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const = 0;
            [[nodiscard]] virtual bool hasVolume() const = 0;
            [[nodiscard]] virtual bool isLocallyContinuable() const = 0;
            [[nodiscard]] virtual MaterialStorageViewType materials() const = 0;
            [[nodiscard]] virtual SceneObjectId id() const = 0;
        };

        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(!isBounded, "Must not be bounded.");
            using MaterialStorageType = MaterialPtrStorageType<ShapeT>;
            using MaterialStorageViewType = MaterialPtrStorageView;
            using SurfaceShaderType = SurfaceShader<ShapeT>;
            using SurfaceShaderPtrType = const SurfaceShaderType*;

            template <typename... ArgsTs>
            PolymorphicSceneObject(const ShapeT& shape, const MaterialStorageType& materials, const SurfaceShaderType& shader) :
                m_shape(shape),
                m_materials(materials),
                m_shader(&shader),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }

            [[nodiscard]] bool raycast(const Ray& ray, RaycastHit& hit) const override
            {
                return ray::raycast(ray, m_shape, hit);
            }
            [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const override
            {
                auto[surface, medium] = materialsView(hit.shapeNo).material(hit.materialIndex);
                return ResolvedRaycastHit(
                    m_shader->shade(m_shape, hit, materialsView()),
                    hit.shapeNo, medium, nullptr, hit.isInside, hasVolume(), isLocallyContinuable()
                );
            }
            [[nodiscard]] bool hasVolume() const override
            {
                return ShapeTraits<ShapeT>::hasVolume;
            }
            [[nodiscard]] bool isLocallyContinuable() const override
            {
                return ShapeTraits<ShapeT>::isLocallyContinuable;
            }
            [[nodiscard]] MaterialStorageViewType materials() const override
            {
                return m_materials.view();
            }
            [[nodiscard]] SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SurfaceShaderPtrType m_shader;
            SceneObjectId m_id;
        };

    public:
        using ShapeType = UnboundedSharedAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialPtrStorageType<ShapeT>& materials, const SurfaceShader<ShapeT>& shader) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(shape, materials, shader))
        {

        }

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialPtrStorageType<ShapeT>& materials) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(shape, materials, defaultShader<ShapeT>))
        {

        }

        SceneObject(const SceneObject&) = default;
        SceneObject(SceneObject&&) noexcept = default;
        SceneObject& operator=(const SceneObject&) = default;
        SceneObject& operator=(SceneObject&&) noexcept = default;

        [[nodiscard]] SceneObjectId id() const
        {
            return m_obj->id();
        }

        [[nodiscard]] bool raycast(const Ray& ray, RaycastHit& hit) const
        {
            return m_obj->raycast(ray, hit);
        }

        [[nodiscard]] ResolvedRaycastHit resolveHit(const ResolvableRaycastHit& hit) const
        {
            return m_obj->resolveHit(hit);
        }

        [[nodiscard]] bool hasVolume() const
        {
            return m_obj->hasVolume();
        }

        [[nodiscard]] bool isLocallyContinuable() const
        {
            return m_obj->isLocallyContinuable();
        }

        [[nodiscard]] MaterialPtrStorageView materialsView() const
        {
            return m_obj->materials();
        }

        [[nodiscard]] bool isLight() const
        {
            // unbounded objects cannot be light emitters
            return false;
        }

    private:
        std::shared_ptr<PolymorphicSceneObjectBase> m_obj;
    };
}
