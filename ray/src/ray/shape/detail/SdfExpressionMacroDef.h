#pragma once

#include "SdfExpressionMacroUndef.h"

#define DEFINE_SDF_EXPRESSION_0(TypeName, ...) \
    template <typename T> \
    [[nodiscard]] float TypeName##ImplEval (T&& self, const Point3f& p); \
    template <bool IsPolyV> \
    struct TypeName##Impl : PolySdfExpression<TypeName##Impl <IsPolyV>, std::tuple<__VA_ARGS__>> \
    { \
        using BaseType = PolySdfExpression<TypeName##Impl, std::tuple<__VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] float signedDistance(const Point3f& p) const override {return TypeName##ImplEval (*this, p);} \
        template <int I> \
        [[nodiscard]] decltype(auto) get() const \
        { \
            return std::get<I>(parts()); \
        } \
    }; \
    template <> \
    struct TypeName##Impl<false> : SdfExpression<TypeName##Impl <false>, std::tuple<__VA_ARGS__>> \
    { \
        using BaseType = SdfExpression<TypeName##Impl, std::tuple<__VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] float signedDistance(const Point3f& p) const {return TypeName##ImplEval (*this, p);} \
        template <int I> \
        [[nodiscard]] decltype(auto) get() const \
        { \
            return std::get<I>(parts()); \
        } \
    }; \
    using Poly##TypeName = TypeName##Impl <true>; \
    using TypeName = TypeName##Impl <false>; \
    template <typename T> \
    [[nodiscard]] float TypeName##ImplEval (T&& self, const Point3f& p) \
    {

#define DEFINE_SDF_EXPRESSION_1(TypeName, ...) \
    template <typename T> \
    [[nodiscard]] float TypeName##ImplEval (T&& self, const Point3f& p); \
    template <typename LhsExprT> \
    struct Poly##TypeName : PolySdfExpression<Poly##TypeName <LhsExprT>, std::tuple<LhsExprT, __VA_ARGS__>> \
    { \
        using BaseType = PolySdfExpression<Poly##TypeName, std::tuple<LhsExprT, __VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] float signedDistance(const Point3f& p) const override {return TypeName##ImplEval (*this, p);} \
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
    template <typename LhsExprT> \
    struct TypeName : SdfExpression<TypeName <LhsExprT>, std::tuple<LhsExprT, __VA_ARGS__>> \
    { \
        using BaseType = SdfExpression<TypeName, std::tuple<LhsExprT, __VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] float signedDistance(const Point3f& p) const {return TypeName##ImplEval (*this, p);} \
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
    template <typename LhsExprT> \
    TypeName(LhsExprT, __VA_ARGS__)->TypeName <LhsExprT>; \
    template <typename LhsExprT> \
    TypeName(std::unique_ptr<LhsExprT>, __VA_ARGS__)->TypeName <CloneableUniquePtr<SdfBase>>; \
    template <typename LhsExprT> \
    Poly##TypeName(LhsExprT, __VA_ARGS__)->Poly##TypeName <LhsExprT>; \
    template <typename LhsExprT> \
    Poly##TypeName(std::unique_ptr<LhsExprT>, __VA_ARGS__)->Poly##TypeName <CloneableUniquePtr<SdfBase>>; \
    template <typename T> \
    [[nodiscard]] float TypeName##ImplEval (T&& self, const Point3f& p) \
    {

#define DEFINE_SDF_EXPRESSION_2(TypeName, ...) \
    template <typename T> \
    [[nodiscard]] float TypeName##ImplEval (T&& self, const Point3f& p); \
    template <typename LhsExprT, typename RhsExprT> \
    struct Poly##TypeName : PolySdfExpression<Poly##TypeName <LhsExprT, RhsExprT>, std::tuple<LhsExprT, RhsExprT, __VA_ARGS__>> \
    { \
        using BaseType = PolySdfExpression<Poly##TypeName, std::tuple<LhsExprT, RhsExprT, __VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] float signedDistance(const Point3f& p) const override {return TypeName##ImplEval (*this, p);} \
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
    template <typename LhsExprT, typename RhsExprT> \
    struct TypeName : SdfExpression<TypeName <LhsExprT, RhsExprT>, std::tuple<LhsExprT, RhsExprT, __VA_ARGS__>> \
    { \
        using BaseType = SdfExpression<TypeName, std::tuple<LhsExprT, RhsExprT, __VA_ARGS__>>; \
        using BaseType::BaseType; \
        using BaseType::parts; \
        [[nodiscard]] float signedDistance(const Point3f& p) const {return TypeName##ImplEval (*this, p);} \
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
    template <typename LhsExprT, typename RhsExprT> \
    Poly##TypeName(LhsExprT, RhsExprT, __VA_ARGS__)->Poly##TypeName<LhsExprT, RhsExprT>; \
    template <typename LhsExprT, typename RhsExprT> \
    Poly##TypeName(std::unique_ptr<LhsExprT>, std::unique_ptr<RhsExprT>, __VA_ARGS__)->Poly##TypeName<CloneableUniquePtr<SdfBase>, CloneableUniquePtr<SdfBase>>; \
    template <typename LhsExprT, typename RhsExprT> \
    TypeName(LhsExprT, RhsExprT, __VA_ARGS__)->TypeName<LhsExprT, RhsExprT>; \
    template <typename LhsExprT, typename RhsExprT> \
    TypeName(std::unique_ptr<LhsExprT>, std::unique_ptr<RhsExprT>, __VA_ARGS__)->TypeName<CloneableUniquePtr<SdfBase>, CloneableUniquePtr<SdfBase>>; \
    template <typename T> \
    [[nodiscard]] float TypeName##ImplEval (T&& self, const Point3f& p) \
    {

#define FINALIZE_SDF_EXPRESSION }
