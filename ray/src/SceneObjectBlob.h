#pragma once

#include "NamedTypePacks.h"
#include "SceneObjectStorageProvider.h"
#include "RaycastHit.h"
#include "RawSceneObjectBlob.h"
#include "SceneObjectCollection.h"
#include "SceneObjectArray.h"
#include "ShapeTraits.h"
#include "Util.h"

#include <optional>
#include <type_traits>

namespace ray
{
    template <typename...>
    struct SceneObjectBlob;

    // No space partitioning. Useful for testing.
    // Uses one array for each shape type.
    // Is similar to BlobScene but behaves like SceneObjectStorage
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
                using SceneObjectType = remove_cvref_t<decltype(object)>;
                using ShapeType = typename SceneObjectType::ShapeType;
                objectsOfType<ShapeType>().add(object);
            });
        }

        template <typename ShapeT>
        void add(const SceneObject<ShapeT>& so)
        {
            objectsOfType<ShapeT>().add(so);
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
    };

    template <typename ShapesT>
    using UnpackedSceneObjectBlob = SceneObjectBlob<ShapesT, RawSceneObjectStorageProvider>;

    template <typename ShapesT>
    using PackedSceneObjectBlob = SceneObjectBlob<ShapesT, PackedSceneObjectStorageProvider>;
}
