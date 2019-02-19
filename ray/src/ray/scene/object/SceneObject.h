#pragma once

#include "SceneObjectId.h"

#include <ray/material/Material.h>
#include <ray/material/TexCoords.h>

#include <ray/math/Interval.h>
#include <ray/math/Raycast.h>
#include <ray/math/RaycastHit.h>
#include <ray/math/Vec3.h>

#include <ray/scene/SceneRaycastHit.h>

#include <ray/shape/Box3.h>
#include <ray/shape/ShapeTraits.h>

#include <array>
#include <atomic>
#include <optional>
#include <memory>

namespace ray
{
    template <typename ShapeT>
    using MaterialArrayType = std::array<const Material*, ShapeTraits<ShapeT>::numMaterialsPerShape>;

    struct Ray;
    struct ResolvableRaycastHit;

    struct BoundedUniqueAnyShape;
    struct BoundedSharedAnyShape;
    struct UnboundedUniqueAnyShape;
    struct UnboundedSharedAnyShape;
    struct CsgShape;

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
        static constexpr int numShapesInPack = ShapeTraits::numShapes;
        static constexpr bool isPack = numShapesInPack > 1;
        static constexpr int numMaterialsPerShape = ShapeTraits::numMaterialsPerShape;
        using MaterialStorageType = MaterialArrayType<ShapeType>;

        static_assert(!isPack, "A single scene object must not be a pack. Use SceneObjectArray.");

        SceneObject(const ShapeType& shape, const MaterialStorageType& materials) :
            m_shape(shape),
            m_materials(materials),
            m_id(detail::gNextSceneObjectId.fetch_add(1))
        {

        }

        decltype(auto) center() const
        {
            return m_shape.center();
        }

        decltype(auto) aabb() const
        {
            return m_shape.aabb();
        }

        const MaterialStorageType& materials() const
        {
            return m_materials;
        }

        const ShapeType& shape() const
        {
            return m_shape;
        }

        const Material& material(int i) const
        {
            return *(m_materials[i]);
        }

        bool isLight() const
        {
            for (const auto& mat : m_materials)
            {
                if (mat->emissionColor.total() > 0.0001f) return true;
            }
            return false;
        }

        SceneObjectId id() const
        {
            return m_id;
        }

    private:
        ShapeType m_shape;
        MaterialStorageType m_materials;
        SceneObjectId m_id;
    };

    // Stores a tree of polymorphic volumetric shapes with boolean operations
    // Supports shape packs as single shapes
    // A copy is shallow, ie. all data is shared.
    // TODO: Working on full intervals is costly.
    //       Could benefit from a local BVH, we already have a nice tree structure.
    template <>
    struct SceneObject<CsgShape>
    {
        struct PolymorphicSceneObjectBase;

        struct Data
        {
            const PolymorphicSceneObjectBase* shape;
            bool invert;
        };

        using HitIntervals = IntervalSet<Data>;
        using HitIntervalsIter = std::vector<IntervalSet<Data>>::iterator;

        struct PolymorphicSceneObjectBase
        {
            virtual Point3f center() const = 0;
            virtual bool raycast(const Ray& ray, RaycastHit& hit) const = 0;
            virtual bool raycastIntervals(const Ray& ray, HitIntervals& hitIntervals, bool invert = false) const = 0;
            virtual TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const = 0;
            virtual Box3 aabb() const = 0;
            virtual const Material& material(int materialNo) const = 0;
            virtual bool isLight() const = 0;
            virtual SceneObjectId id() const = 0;
        };

    private:
        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            static constexpr bool hasVolume = ShapeTraits<ShapeT>::hasVolume;
            static constexpr bool isLocallyContinuable = ShapeTraits<ShapeT>::isLocallyContinuable;
            using MaterialStorageType = MaterialArrayType<ShapeT>;

            static_assert(isBounded, "Must be bounded.");
            static_assert(hasVolume, "Must have volume.");
            static_assert(isLocallyContinuable, "Must be locally continuable.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }

            Point3f center() const override
            {
                return m_shape.center();
            }
            Box3 aabb() const override
            {
                return m_shape.aabb();
            }
            bool raycast(const Ray& ray, RaycastHit& hit) const override
            {
                return ray::raycast(ray, m_shape, hit);
            }
            bool raycastIntervals(const Ray& ray, HitIntervals& hitIntervals, bool invert = false) const override
            {
                return ray::raycastIntervals<Data>(ray, m_shape, hitIntervals, { this, invert });
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
            }
            const Material& material(int materialNo) const override
            {
                return *(m_materials[materialNo]);
            }
            bool isLight() const override
            {
                for (const auto& mat : m_materials)
                {
                    if (mat->emissionColor.total() > 0.0001f) return true;
                }
                return false;
            }
            SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SceneObjectId m_id;
        };

        struct CsgOperation
        {
            virtual bool raycastIntervals(const Ray& ray, HitIntervals& hitIntervals, HitIntervalsIter scratchHitIntervals, bool invert = false) const = 0;
            virtual Box3 aabb() const = 0;
            virtual int depth() const = 0;
        };

        struct CsgIdentity : CsgOperation
        {
            CsgIdentity(std::shared_ptr<PolymorphicSceneObjectBase>&& obj) :
                m_obj(std::move(obj))
            {
            }

            bool raycastIntervals(const Ray& ray, HitIntervals& hitIntervals, HitIntervalsIter scratchHitIntervals, bool invert = false) const override
            {
                hitIntervals.clear();
                return m_obj->raycastIntervals(ray, hitIntervals, invert);
            }

            Box3 aabb() const override
            {
                return m_obj->aabb();
            }

            int depth() const override
            {
                return 1;
            }

        private:
            std::shared_ptr<PolymorphicSceneObjectBase> m_obj;
        };

        struct CsgUnion : CsgOperation
        {
            CsgUnion(std::shared_ptr<CsgOperation> lhs, std::shared_ptr<CsgOperation> rhs) :
                m_lhs(std::move(lhs)),
                m_rhs(std::move(rhs)),
                m_aabb(aabb()),
                m_depth(std::max(m_lhs->depth(), m_rhs->depth()) + 1)
            {
            }

            bool raycastIntervals(const Ray& ray, HitIntervals& hitIntervals, HitIntervalsIter scratchHitIntervals, bool invert = false) const override
            {
                RaycastBvHit bvHit;
                if (!ray::raycastBv(ray, m_aabb, std::numeric_limits<float>::max(), bvHit))
                {
                    hitIntervals.clear();
                    return false;
                }
                auto rlhs = m_lhs->raycastIntervals(ray, hitIntervals, std::next(scratchHitIntervals), invert);
                if (!rlhs)
                {
                    auto rrhs = m_rhs->raycastIntervals(ray, hitIntervals, std::next(scratchHitIntervals), invert);
                }
                else
                {
                    auto rrhs = m_rhs->raycastIntervals(ray, *scratchHitIntervals, std::next(scratchHitIntervals), invert);
                    hitIntervals |= *scratchHitIntervals;
                }
                return !hitIntervals.isEmpty();
            }

            Box3 aabb() const override
            {
                Box3 b = m_lhs->aabb();
                b.extend(m_rhs->aabb());
                return b;
            }

            int depth() const override
            {
                return m_depth;
            }

        private:
            std::shared_ptr<CsgOperation> m_lhs;
            std::shared_ptr<CsgOperation> m_rhs;
            Box3 m_aabb;
            int m_depth;
        };

        struct CsgIntersection : CsgOperation
        {
            CsgIntersection(std::shared_ptr<CsgOperation> lhs, std::shared_ptr<CsgOperation> rhs) :
                m_lhs(std::move(lhs)),
                m_rhs(std::move(rhs)),
                m_aabb(aabb()),
                m_depth(std::max(m_lhs->depth(), m_rhs->depth()) + 1)
            {
            }

            bool raycastIntervals(const Ray& ray, HitIntervals& hitIntervals, HitIntervalsIter scratchHitIntervals, bool invert = false) const override
            {
                RaycastBvHit bvHit;
                if (!ray::raycastBv(ray, m_aabb, std::numeric_limits<float>::max(), bvHit))
                {
                    hitIntervals.clear();
                    return false;
                }
                auto rrhs = m_rhs->raycastIntervals(ray, *scratchHitIntervals, std::next(scratchHitIntervals), invert);
                if (!rrhs)
                {
                    hitIntervals.clear();
                    return false;
                }
                auto rlhs = m_lhs->raycastIntervals(ray, hitIntervals, std::next(scratchHitIntervals), invert);
                if (!rlhs) return false;

                hitIntervals &= *scratchHitIntervals;
                return !hitIntervals.isEmpty();
            }

            Box3 aabb() const override
            {
                Box3 b = m_lhs->aabb();
                b.extend(m_rhs->aabb());
                return b;
            }

            int depth() const override
            {
                return m_depth;
            }

        private:
            std::shared_ptr<CsgOperation> m_lhs;
            std::shared_ptr<CsgOperation> m_rhs;
            Box3 m_aabb;
            int m_depth;
        };

        struct CsgDifference : CsgOperation
        {
            CsgDifference(std::shared_ptr<CsgOperation> lhs, std::shared_ptr<CsgOperation> rhs) :
                m_lhs(std::move(lhs)),
                m_rhs(std::move(rhs)),
                m_aabb(aabb()),
                m_depth(std::max(m_lhs->depth(), m_rhs->depth()) + 1)
            {
            }

            bool raycastIntervals(const Ray& ray, HitIntervals& hitIntervals, HitIntervalsIter scratchHitIntervals, bool invert = false) const override
            {
                RaycastBvHit bvHit;
                if (!ray::raycastBv(ray, m_aabb, std::numeric_limits<float>::max(), bvHit))
                {
                    hitIntervals.clear();
                    return false;
                }
                auto rlhs = m_lhs->raycastIntervals(ray, hitIntervals, std::next(scratchHitIntervals), invert);
                if (!rlhs) return false;
                auto rrhs = m_rhs->raycastIntervals(ray, *scratchHitIntervals, std::next(scratchHitIntervals), !invert);
                if(rrhs)
                {
                    hitIntervals -= *scratchHitIntervals;
                }
                return !hitIntervals.isEmpty();
            }

            Box3 aabb() const override
            {
                Box3 b = m_lhs->aabb();
                b.extend(m_rhs->aabb());
                return b;
            }

            int depth() const override
            {
                return m_depth;
            }

        private:
            std::shared_ptr<CsgOperation> m_lhs;
            std::shared_ptr<CsgOperation> m_rhs;
            Box3 m_aabb;
            int m_depth;
        };

        SceneObject(std::shared_ptr<CsgOperation> op) :
            m_obj(op),
            m_id(detail::gNextSceneObjectId.fetch_add(1))
        {

        }

    public:
        using ShapeType = CsgShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialArrayType<ShapeT>& materials) :
            m_obj(std::make_shared<CsgIdentity>(std::make_shared<PolymorphicSceneObject<ShapeT>>(materials, shape))),
            m_id(detail::gNextSceneObjectId.fetch_add(1))
        {

        }

        const PolymorphicSceneObjectBase* raycast(const Ray& ray, RaycastHit& hit) const
        {
            thread_local std::vector<IntervalSet<Data>> allHitIntervals;

            const int depth = m_obj->depth();
            if (allHitIntervals.size() < depth)
            {
                // technically +1 is not needed because identity doesn't use scratch, but just to be safe
                allHitIntervals.resize(depth + 1);
            }
            auto& hitIntervals = allHitIntervals[0];
            auto scratchHitIntervalsIter = std::next(allHitIntervals.begin());
            hitIntervals.clear();
            m_obj->raycastIntervals(ray, hitIntervals, scratchHitIntervalsIter);
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
                if (interval.min > 0.0f)
                {
                    hit.dist -= (interval.min - 0.001f);
                    bool b = interval.minData.shape->raycast(ray.translated(ray.direction() * (interval.min - 0.001f)), hit);
                    hit.dist += (interval.min - 0.001f);
                    if (b && interval.minData.invert) hit.isInside = !hit.isInside;
                    return b ? interval.minData.shape : nullptr;
                }
                else if (interval.max > 0.0f)
                {
                    hit.dist -= (interval.max - 0.001f);
                    bool b = interval.maxData.shape->raycast(ray.translated(ray.direction() * (interval.max - 0.001f)), hit);
                    hit.dist += (interval.max - 0.001f);
                    if (b && interval.maxData.invert) hit.isInside = !hit.isInside;
                    return b ? interval.maxData.shape : nullptr;
                }
            }
            return nullptr;
        }

        decltype(auto) aabb() const
        {
            return m_obj->aabb();
        }

        decltype(auto) center() const
        {
            return aabb().center();
        }

        bool hasVolume() const
        {
            return true;
        }

        bool isLocallyContinuable() const
        {
            return true;
        }

        // TODO: maybe this properly
        bool isLight() const
        {
            return false;
        }

        SceneObjectId id() const
        {
            return m_id;
        }

        friend SceneObject<CsgShape> operator|(const SceneObject<CsgShape>& lhs, const SceneObject<CsgShape>& rhs)
        {
            return SceneObject<CsgShape>(std::make_shared<CsgUnion>(lhs.m_obj, rhs.m_obj));
        }

        friend SceneObject<CsgShape> operator&(const SceneObject<CsgShape>& lhs, const SceneObject<CsgShape>& rhs)
        {
            return SceneObject<CsgShape>(std::make_shared<CsgIntersection>(lhs.m_obj, rhs.m_obj));
        }

        friend SceneObject<CsgShape> operator-(const SceneObject<CsgShape>& lhs, const SceneObject<CsgShape>& rhs)
        {
            return SceneObject<CsgShape>(std::make_shared<CsgDifference>(lhs.m_obj, rhs.m_obj));
        }

    private:
        std::shared_ptr<CsgOperation> m_obj;
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
            virtual Point3f center() const = 0;
            virtual bool raycast(const Ray& ray, RaycastHit& hit) const = 0;
            virtual std::unique_ptr<PolymorphicSceneObjectBase> clone() const = 0;
            virtual TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const = 0;
            virtual bool hasVolume() const = 0;
            virtual bool isLocallyContinuable() const = 0;
            virtual Box3 aabb() const = 0;
            virtual const Material& material(int materialNo) const = 0;
            virtual bool isLight() const = 0;
            virtual SceneObjectId id() const = 0;
        };

        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            using MaterialStorageType = MaterialArrayType<ShapeT>;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(isBounded, "Must be bounded.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }

            Point3f center() const override
            {
                return m_shape.center();
            }
            Box3 aabb() const override
            {
                return m_shape.aabb();
            }
            bool raycast(const Ray& ray, RaycastHit& hit) const override
            {
                return ray::raycast(ray, m_shape, hit);
            }
            std::unique_ptr<PolymorphicSceneObjectBase> clone() const override
            {
                return std::make_unique<PolymorphicSceneObject<ShapeT>>(*this);
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
            }
            bool hasVolume() const override
            {
                return ShapeTraits<ShapeT>::hasVolume;
            }
            bool isLocallyContinuable() const override
            {
                return ShapeTraits<ShapeT>::isLocallyContinuable;
            }
            const Material& material(int materialNo) const override
            {
                return *(m_materials[materialNo]);
            }
            bool isLight() const override
            {
                for (const auto& mat : m_materials)
                {
                    if (mat->emissionColor.total() > 0.0001f) return true;
                }
                return false;
            }
            SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SceneObjectId m_id;
        };

    public:
        using ShapeType = BoundedUniqueAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialArrayType<ShapeT>& materials) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(materials, shape))
        {

        }

        SceneObject(const SceneObject& other) :
            m_obj(other.m_obj->clone())
        {

        }

        SceneObjectId id() const
        {
            return m_obj->id();
        }

        Point3f center() const
        {
            return m_obj->center();
        }

        Box3 aabb() const
        {
            return m_obj->aabb();
        }

        bool raycast(const Ray& ray, RaycastHit& hit) const
        {
            return m_obj->raycast(ray, hit);
        }

        TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const
        {
            return m_obj->resolveTexCoords(hit, shapeInPackNo);
        }

        bool hasVolume() const
        {
            return m_obj->hasVolume();
        }
        bool isLocallyContinuable() const
        {
            return m_obj->isLocallyContinuable();
        }

        const Material& material(int materialNo) const
        {
            return m_obj->material(materialNo);
        }

        bool isLight() const
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
            virtual Point3f center() const = 0;
            virtual bool raycast(const Ray& ray, RaycastHit& hit) const = 0;
            virtual TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const = 0;
            virtual bool hasVolume() const = 0;
            virtual bool isLocallyContinuable() const = 0;
            virtual Box3 aabb() const = 0;
            virtual const Material& material(int materialNo) const = 0;
            virtual bool isLight() const = 0;
            virtual SceneObjectId id() const = 0;
        };

        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            using MaterialStorageType = MaterialArrayType<ShapeT>;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(isBounded, "Must be bounded.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }

            Point3f center() const override
            {
                return m_shape.center();
            }
            Box3 aabb() const override
            {
                return m_shape.aabb();
            }
            bool raycast(const Ray& ray, RaycastHit& hit) const override
            {
                return ray::raycast(ray, m_shape, hit);
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
            }
            bool hasVolume() const override
            {
                return ShapeTraits<ShapeT>::hasVolume;
            }
            bool isLocallyContinuable() const override
            {
                return ShapeTraits<ShapeT>::isLocallyContinuable;
            }
            const Material& material(int materialNo) const override
            {
                return *(m_materials[materialNo]);
            }
            bool isLight() const override
            {
                for (const auto& mat : m_materials)
                {
                    if (mat->emissionColor.total() > 0.0001f) return true;
                }
                return false;
            }
            SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SceneObjectId m_id;
        };

    public:
        using ShapeType = BoundedSharedAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialArrayType<ShapeT>& materials) :
            m_obj(std::make_shared<PolymorphicSceneObject<ShapeT>>(materials, shape))
        {

        }

        SceneObjectId id() const
        {
            return m_obj->id();
        }

        Point3f center() const
        {
            return m_obj->center();
        }

        Box3 aabb() const
        {
            return m_obj->aabb();
        }

        bool raycast(const Ray& ray, RaycastHit& hit) const
        {
            return m_obj->raycast(ray, hit);
        }

        TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const
        {
            return m_obj->resolveTexCoords(hit, shapeInPackNo);
        }

        bool hasVolume() const
        {
            return m_obj->hasVolume();
        }

        bool isLocallyContinuable() const
        {
            return m_obj->isLocallyContinuable();
        }

        const Material& material(int materialNo) const
        {
            return m_obj->material(materialNo);
        }

        bool isLight() const
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
            virtual bool raycast(const Ray& ray, RaycastHit& hit) const = 0;
            virtual std::unique_ptr<PolymorphicSceneObjectBase> clone() const = 0;
            virtual TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const = 0;
            virtual bool hasVolume() const = 0;
            virtual bool isLocallyContinuable() const = 0;
            virtual const Material& material(int materialNo) const = 0;
            virtual bool isLight() const = 0;
            virtual SceneObjectId id() const = 0;
        };

        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            using MaterialStorageType = MaterialArrayType<ShapeT>;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(!isBounded, "Must not be bounded.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }
            bool raycast(const Ray& ray, RaycastHit& hit) const override
            {
                return ray::raycast(ray, m_shape, hit);
            }
            std::unique_ptr<PolymorphicSceneObjectBase> clone() const override
            {
                return std::make_unique<PolymorphicSceneObject<ShapeT>>(*this);
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
            }
            bool hasVolume() const override
            {
                return ShapeTraits<ShapeT>::hasVolume;
            }
            bool isLocallyContinuable() const override
            {
                return ShapeTraits<ShapeT>::isLocallyContinuable;
            }
            const Material& material(int materialNo) const override
            {
                return *(m_materials[materialNo]);
            }
            bool isLight() const override
            {
                for (const auto& mat : m_materials)
                {
                    if (mat->emissionColor.total() > 0.0001f) return true;
                }
                return false;
            }
            SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SceneObjectId m_id;
        };

    public:
        using ShapeType = UnboundedUniqueAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialArrayType<ShapeT>& materials) :
            m_obj(std::make_unique<PolymorphicSceneObject<ShapeT>>(materials, shape))
        {

        }

        SceneObject(const SceneObject& other) :
            m_obj(other.m_obj->clone())
        {

        }

        SceneObjectId id() const
        {
            return m_obj->id();
        }

        bool raycast(const Ray& ray, RaycastHit& hit) const
        {
            return m_obj->raycast(ray, hit);
        }

        TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const
        {
            return m_obj->resolveTexCoords(hit, shapeInPackNo);
        }

        bool hasVolume() const
        {
            return m_obj->hasVolume();
        }

        bool isLocallyContinuable() const
        {
            return m_obj->isLocallyContinuable();
        }

        const Material& material(int materialNo) const
        {
            return m_obj->material(materialNo);
        }

        bool isLight() const
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
    struct SceneObject<UnboundedSharedAnyShape>
    {
    private:
        struct PolymorphicSceneObjectBase
        {
            virtual bool raycast(const Ray& ray, RaycastHit& hit) const = 0;
            virtual TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const = 0;
            virtual bool hasVolume() const = 0;
            virtual bool isLocallyContinuable() const = 0;
            virtual const Material& material(int materialNo) const = 0;
            virtual bool isLight() const = 0;
            virtual SceneObjectId id() const = 0;
        };

        template <typename ShapeT>
        struct PolymorphicSceneObject : PolymorphicSceneObjectBase
        {
            static constexpr int numShapesInPack = ShapeTraits<ShapeT>::numShapes;
            static constexpr bool isPack = numShapesInPack > 1;
            static constexpr int numMaterialsPerShape = ShapeTraits<ShapeT>::numMaterialsPerShape;
            static constexpr bool isBounded = ShapeTraits<ShapeT>::isBounded;
            using MaterialStorageType = MaterialArrayType<ShapeT>;
            static_assert(!isPack, "Only a single object can be made polymorphic.");
            static_assert(!isBounded, "Must not be bounded.");

            template <typename... ArgsTs>
            PolymorphicSceneObject(const MaterialStorageType& materials, ArgsTs&&... args) :
                m_shape(std::forward<ArgsTs>(args)...),
                m_materials(materials),
                m_id(detail::gNextSceneObjectId.fetch_add(1))
            {

            }
            bool raycast(const Ray& ray, RaycastHit& hit) const override
            {
                return ray::raycast(ray, m_shape, hit);
            }
            TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const override
            {
                return ray::resolveTexCoords(m_shape, hit, shapeInPackNo);
            }
            bool hasVolume() const override
            {
                return ShapeTraits<ShapeT>::hasVolume;
            }
            bool isLocallyContinuable() const override
            {
                return ShapeTraits<ShapeT>::isLocallyContinuable;
            }
            const Material& material(int materialNo) const override
            {
                return *(m_materials[materialNo]);
            }
            bool isLight() const override
            {
                for (const auto& mat : m_materials)
                {
                    if (mat->emissionColor.total() > 0.0001f) return true;
                }
                return false;
            }
            SceneObjectId id() const override
            {
                return m_id;
            }

        private:
            ShapeT m_shape;
            MaterialStorageType m_materials;
            SceneObjectId m_id;
        };

    public:
        using ShapeType = UnboundedSharedAnyShape;

        template <typename ShapeT>
        SceneObject(const ShapeT& shape, const MaterialArrayType<ShapeT>& materials) :
            m_obj(std::make_shared<PolymorphicSceneObject<ShapeT>>(materials, shape))
        {

        }

        SceneObjectId id() const
        {
            return m_obj->id();
        }

        bool raycast(const Ray& ray, RaycastHit& hit) const
        {
            return m_obj->raycast(ray, hit);
        }

        TexCoords resolveTexCoords(const ResolvableRaycastHit& hit, int shapeInPackNo) const
        {
            return m_obj->resolveTexCoords(hit, shapeInPackNo);
        }

        bool hasVolume() const
        {
            return m_obj->hasVolume();
        }

        bool isLocallyContinuable() const
        {
            return m_obj->isLocallyContinuable();
        }

        const Material& material(int materialNo) const
        {
            return m_obj->material(materialNo);
        }

        bool isLight() const
        {
            return m_obj->isLight();
        }

    private:
        std::shared_ptr<PolymorphicSceneObjectBase> m_obj;
    };
}
