#pragma once

#include "UtilityMacroUndef.h"

#include <cassert>

#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)

#define RAY_ASSUME(expr) if(!(expr)) { __builtin_unreachable(); }

#elif defined(_MSC_VER)

#define RAY_ASSUME(expr) (__assume(expr))

#else

#define RAY_ASSUME(expr) ((void)0)

#endif


#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)

#define RAY_ASSERT(expr) if(!(expr)) { __builtin_unreachable(); }

#elif defined(_MSC_VER)

#if defined(_DEBUG)
#define RAY_ASSERT(expr) (assert(expr))
#else
#define RAY_ASSERT(expr) (__assume(expr))
#endif

#else

#define RAY_ASSERT(expr) ((void)0)

#endif


#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)

#define RAY_FORCEINLINE __attribute__((always_inline))

#elif defined(_MSC_VER)

#define RAY_FORCEINLINE __forceinline

#else

#define RAY_FORCEINLINE inline

#endif


#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)

#define RAY_UNREACHABLE() (__builtin_unreachable())

#elif defined(_MSC_VER)

#if defined(_DEBUG)
#define RAY_UNREACHABLE() (assert(false))
#else
#define RAY_UNREACHABLE() (__assume(0))
#endif

#else

#define RAY_UNREACHABLE() ((void)0)

#endif