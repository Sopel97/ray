#pragma once

#include <type_traits>

namespace ray
{
    template <typename T>
    struct Matrix4;

    template <typename TransformT, typename ShapeT>
    struct TransformedShape3
    {
        TransformT worldToLocal;
        ShapeT shape;

        TransformedShape3(const TransformT& worldToLocal, const ShapeT& shape) :
            worldToLocal(worldToLocal),
            shape(shape)
        {
        }

        [[nodiscard]] decltype(auto) center() const
        {
            return worldToLocal.inverse() * shape.center();
        }
    };

    template <typename TransformLhsT, typename TransformRhsT, typename ShapeT>
    auto operator*(const TransformLhsT& lhs, const TransformedShape3<TransformRhsT, ShapeT>& rhs)
    {
        return TransformedShape3<decltype(std::declval<TransformLhsT>() * std::declval<TransformRhsT>()), ShapeT>(lhs * rhs.worldToLocal, rhs.shape);
    }

    template <typename TransformLhsT, typename ShapeT, typename SFINAE = std::enable_if_t<std::is_base_of_v<Matrix4<float>, TransformLhsT>>>
    auto operator*(const TransformLhsT& lhs, const ShapeT& rhs)
    {
        return TransformedShape3<TransformLhsT, ShapeT>(lhs, rhs);
    }

    // the following pass but require unnessesary includes
    //static_assert(std::is_same_v<decltype(std::declval<Identity4f>() * std::declval<Rotation4f>()), Rotation4f>);
    //static_assert(std::is_same_v<decltype(std::declval<Rotation4f>() * std::declval<Sphere>()), TransformedShape3<Rotation4f, Sphere>>);
    //static_assert(std::is_same_v<decltype(std::declval<Scale4f>() * std::declval<TransformedShape3<Rotation4f, Sphere>>()), TransformedShape3<RotationScale4f, Sphere>>);
}
