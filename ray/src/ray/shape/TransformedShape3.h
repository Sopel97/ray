#pragma once

namespace ray
{
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
}
