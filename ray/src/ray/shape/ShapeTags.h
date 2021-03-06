#pragma once

namespace ray
{
    // Tags that indicate shapes that require special treatment
    // ie. they cannot be represented with constexpr assumptions
    // which are required by ShapeTraits.
    // Also implementing them under the name of "Shape" would violate
    // the priciple that shapes don't hold their own material -
    // it's SceneObject's job.
    // SceneObject, SceneObjectArray should specialize for them

    struct CsgShape;

    template <bool IsBoundedV>
    struct AnyShape;
    using AnyBoundedShape = AnyShape<true>;
    using AnyUnboundedShape = AnyShape<false>;
}
