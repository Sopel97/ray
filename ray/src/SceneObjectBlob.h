#pragma once

#include "NamedTypePacks.h"
#include "RawSceneObjectBlob.h"
#include "RaycastHit.h"
#include "SceneObjectCollection.h"
#include "SceneObjectStorageProvider.h"
#include "ShapeTraits.h"
#include "Util.h"

#include <optional>
#include <tuple>
#include <type_traits>

namespace ray
{
    template <typename...>
    struct SceneObjectBlob;

    // No space partitioning. Useful for testing.
    // Uses one array for each shape type.
    template <typename SceneObjectStorageProviderT, typename... ShapeTs>
    struct SceneObjectBlob<Shapes<ShapeTs...>, SceneObjectStorageProviderT> : DynamicHeterogeneousSceneObjectCollection
    {
        using AllShapes = Shapes<ShapeTs...>;

    private:
        template <typename ShapeT>
        using ObjectStorageType = typename SceneObjectStorageProviderT::template ArrayType<ShapeT>;

    public:
        SceneObjectBlob() :
            m_objects{}
        {
        }

        template <typename... ShapeTs>
        SceneObjectBlob(const RawSceneObjectBlob<Shapes<ShapeTs...>>& blob)
        {
            blob.forEach([&](auto&& object) {
                add(object);
            });
        }

        template <typename... ShapeTs>
        SceneObjectBlob(RawSceneObjectBlob<Shapes<ShapeTs...>>&& blob)
        {
            blob.forEach([&](auto&& object) {
                add(std::move(object));
                });
        }

        template <typename ShapeT>
        void add(const SceneObject<ShapeT>& so)
        {
            objectsOfType<ShapeT>().add(so);
        }

        template <typename ShapeT>
        void add(SceneObject<ShapeT>&& so)
        {
            objectsOfType<ShapeT>().add(std::move(so));
        }

        std::optional<ResolvableRaycastHit> queryNearest(const Ray& ray) const
        {
            std::optional<ResolvableRaycastHit> hitOpt = std::nullopt;
            float minDist = std::numeric_limits<float>::max();
            for_each(m_objects, [&](const auto& objects) {
                if (objects.size() > 0)
                {
                    std::optional<ResolvableRaycastHit> hitOptNow = objects.queryNearest(ray);
                    if (hitOptNow)
                    {
                        const float dist = distance(hitOptNow->point, ray.origin());
                        if (dist < minDist)
                        {
                            minDist = dist;
                            hitOpt = std::move(*hitOptNow);
                        }
                    }
                }
            });

            return hitOpt;
        }

    private:
        std::tuple<
            ObjectStorageType<ShapeTs>...
        > m_objects;

        template <typename ShapeT>
        ObjectStorageType<ShapeT>& objectsOfType()
        {
            return std::get<ObjectStorageType<ShapeT>>(m_objects);
        }

        template <typename ShapeT>
        const ObjectStorageType<ShapeT>& objectsOfType() const
        {
            return std::get<ObjectStorageType<ShapeT>>(m_objects);
        }
    };

    template <typename ShapesT>
    using UnpackedSceneObjectBlob = SceneObjectBlob<ShapesT, RawSceneObjectStorageProvider>;

    template <typename ShapesT>
    using PackedSceneObjectBlob = SceneObjectBlob<ShapesT, PackedSceneObjectStorageProvider>;
}
