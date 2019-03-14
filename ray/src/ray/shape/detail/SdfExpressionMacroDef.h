#pragma once

#include "SdfExpressionMacroUndef.h"

#define DEFINE_SDF_EXPRESSION_0(TypeName, ...) \
    struct TypeName : SdfExpression<TypeName, std::tuple<__VA_ARGS__>> \
    { \
        using BaseType = SdfExpression<TypeName, std::tuple<__VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] virtual float signedDistance(const Point3f& p) const override; \
    protected: \
        template <int I> \
        [[nodiscard]] decltype(auto) get() const \
        { \
            return std::get<I>(parts()); \
        } \
    }; \
    using Poly##TypeName = TypeName; \
    [[nodiscard]] float TypeName::signedDistance(const Point3f& p) const \
    {

#define DEFINE_SDF_EXPRESSION_1(TypeName, ...) \
    template <typename LhsExprT> \
    struct TypeName : SdfExpression<TypeName<LhsExprT>, std::tuple<LhsExprT, __VA_ARGS__>> \
    { \
        using BaseType = SdfExpression<TypeName, std::tuple<LhsExprT, __VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] virtual float signedDistance(const Point3f& p) const override; \
    protected: \
        template <int I> \
        [[nodiscard]] decltype(auto) get() const \
        { \
            return std::get<I + 1>(parts()); \
        } \
        [[nodiscard]] decltype(auto) arg() const \
        { \
            return std::get<0>(parts()); \
        } \
    }; \
    using Poly##TypeName = TypeName<CloneableUniquePtr<SdfBase>>; \
    template <typename LhsExprT> \
    TypeName(LhsExprT, __VA_ARGS__)->TypeName<LhsExprT>; \
    template <typename LhsExprT> \
    TypeName(std::unique_ptr<LhsExprT>, __VA_ARGS__)->TypeName<CloneableUniquePtr<SdfBase>>; \
    template <typename LhsExprT> \
    [[nodiscard]] float TypeName<LhsExprT>::signedDistance(const Point3f& p) const \
    {

#define DEFINE_SDF_EXPRESSION_2(TypeName, ...) \
    template <typename LhsExprT, typename RhsExprT> \
    struct TypeName : SdfExpression<TypeName<LhsExprT, RhsExprT>, std::tuple<LhsExprT, RhsExprT, __VA_ARGS__>> \
    { \
        using BaseType = SdfExpression<TypeName, std::tuple<LhsExprT, RhsExprT, __VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] virtual float signedDistance(const Point3f& p) const override; \
    protected: \
        template <int I> \
        [[nodiscard]] decltype(auto) get() const \
        { \
            return std::get<I + 2>(parts()); \
        } \
        [[nodiscard]] decltype(auto) lhs() const \
        { \
            return std::get<0>(parts()); \
        } \
        [[nodiscard]] decltype(auto) rhs() const \
        { \
            return std::get<1>(parts()); \
        } \
    }; \
    using Poly##TypeName = TypeName<CloneableUniquePtr<SdfBase>, CloneableUniquePtr<SdfBase>>; \
    template <typename LhsExprT, typename RhsExprT> \
    TypeName(LhsExprT, RhsExprT, __VA_ARGS__)->TypeName<LhsExprT, RhsExprT>; \
    template <typename LhsExprT, typename RhsExprT> \
    TypeName(std::unique_ptr<LhsExprT>, std::unique_ptr<RhsExprT>, __VA_ARGS__)->TypeName<CloneableUniquePtr<SdfBase>, CloneableUniquePtr<SdfBase>>; \
    template <typename LhsExprT, typename RhsExprT> \
    [[nodiscard]] float TypeName<LhsExprT, RhsExprT>::signedDistance(const Point3f& p) const \
    {

#define FINALIZE_SDF_EXPRESSION }
