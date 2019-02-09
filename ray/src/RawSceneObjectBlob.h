#pragma once

#include "NamedTypePacks.h"
#include "SceneObjectStorageProvider.h"
#include "RaycastHit.h"
#include "SceneObjectCollection.h"
#include "SceneObjectArray.h"
#include "ShapeTraits.h"
#include "Util.h"

#include <optional>
#include <type_traits>

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

    public:
        template <typename... SceneObjectCollectionsTs>
        RawSceneObjectBlob(SceneObjectCollectionsTs&&... collections)
        {
            for_each(std::tie(collections...), [&](auto&& collection) {
                for (auto&& object : collection)
                {
                    using SceneObjectType = remove_cvref_t<decltype(object)>;
                    using ShapeType = typename SceneObjectType::ShapeType;
                    objectsOfType<ShapeType>().emplace_back(std::forward<SceneObjectType>(object));
                }
                });
        }

        template <typename ShapeT>
        void add(const SceneObject<ShapeT>& so)
        {
            objectsOfType<ShapeT>().emplace_back(so);
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
    };

}
