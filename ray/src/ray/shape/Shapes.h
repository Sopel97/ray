#pragma once

#include <type_traits>

namespace ray
{
    template <typename... ShapeTs>
    struct Shapes;

    template <typename...>
    struct CatShapesImpl;

    template <typename... LhsShapeTs, typename... RhsShapeTs>
    struct CatShapesImpl<Shapes<LhsShapeTs...>, Shapes<RhsShapeTs...>>
    {
        using type = Shapes<LhsShapeTs..., RhsShapeTs...>;
    };

    template <typename LhsShapesT, typename RhsShapesT>
    using CatShapes = typename CatShapesImpl<LhsShapesT, RhsShapesT>::type;

    template <typename...>
    struct FilterShapesImpl;

    template <typename PredicateT>
    struct FilterShapesImpl<Shapes<>, PredicateT>
    {
        using type = Shapes<>;
    };

    template <typename PredicateT, typename HeadShapeT, typename... TailShapeTs>
    struct FilterShapesImpl<Shapes<HeadShapeT, TailShapeTs...>, PredicateT>
    {
        using type = std::conditional_t<
            PredicateT::template value<HeadShapeT>,
            CatShapes<Shapes<HeadShapeT>, typename FilterShapesImpl<Shapes<TailShapeTs...>, PredicateT>::type>,
            typename FilterShapesImpl<Shapes<TailShapeTs...>, PredicateT>::type
        >;
    };

    template <typename ShapesT, typename PredicateT>
    using FilterShapes = typename FilterShapesImpl<ShapesT, PredicateT>::type;
}
