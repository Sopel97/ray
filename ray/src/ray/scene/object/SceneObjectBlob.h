#pragma once

#include "RawSceneObjectBlob.h"
#include "SceneObjectCollection.h"
#include "SceneObjectStorageProvider.h"

#include <ray/scene/SceneRaycastHit.h>

#include <ray/shape/Shapes.h>
#include <ray/shape/ShapeTraits.h>

#include <ray/utility/Util.h>

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

        [[nodiscard]] bool queryNearest(const Ray& ray, ResolvableRaycastHit& hit) const
        {
            bool anyHit = false;
            for_each(m_objects, [&](const auto& objects) {
                if (objects.size() > 0)
                {
                    anyHit |= objects.queryNearest(ray, hit);
                }
            });

            return anyHit;
        }

    private:
        std::tuple<
            ObjectStorageType<ShapeTs>...
        > m_objects;

        template <typename ShapeT>
        [[nodiscard]] ObjectStorageType<ShapeT>& objectsOfType()
        {
            return std::get<ObjectStorageType<ShapeT>>(m_objects);
        }

        template <typename ShapeT>
        [[nodiscard]] const ObjectStorageType<ShapeT>& objectsOfType() const
        {
            return std::get<ObjectStorageType<ShapeT>>(m_objects);
        }
    };

    template <typename ShapesT>
    using UnpackedSceneObjectBlob = SceneObjectBlob<ShapesT, RawSceneObjectStorageProvider>;

    template <typename ShapesT>
    using PackedSceneObjectBlob = SceneObjectBlob<ShapesT, PackedSceneObjectStorageProvider>;
}
