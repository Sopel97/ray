#pragma once

#include "NamedTypePacks.h"
#include "SceneObjectCollection.h"
#include "Util.h"

#include <optional>
#include <tuple>
#include <vector>

namespace ray
{
    template <typename...>
    struct RawSceneObjectBlob;

    template <typename... ShapeTs>
    struct RawSceneObjectBlob<Shapes<ShapeTs...>> : DynamicHeterogeneousSceneObjectCollection
    {
        using AllShapes = Shapes<ShapeTs...>;

        template <typename ShapeT>
        using ObjectStorageType = std::vector<SceneObject<ShapeT>>;

        template <typename... SceneObjectCollectionTs>
        RawSceneObjectBlob(SceneObjectCollectionTs&&... collections)
        {
            for_each(std::forward_as_tuple(std::forward<SceneObjectCollectionTs>(collections)...), [&](auto&& collection) {
                for (auto&& object : collection)
                {
                    using SceneObjectType = remove_cvref_t<decltype(object)>;
                    add(std::forward<SceneObjectType>(object));
                }
                });
        }

        template <typename ShapeT>
        void add(const SceneObject<ShapeT>& so)
        {
            objectsOfType<ShapeT>().emplace_back(so);
        }

        template <typename ShapeT>
        void add(SceneObject<ShapeT>&& so)
        {
            objectsOfType<ShapeT>().emplace_back(std::move(so));
        }

        template <typename FuncT>
        void forEach(FuncT func) const
        {
            for_each(m_objects, [&](auto&& collection) {
                for (auto&& object : collection)
                {
                    func(object);
                }
            });
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
}
