#pragma once

namespace ray
{
    template <typename TransformT, typename ShapeT>
    struct TransformedShape3
    {
        TransformT worldToLocal;
        ShapeT shape;

        decltype(auto) center() const
        {
            return worldToLocal.inverse() * shape.center();
        }
    };
}
