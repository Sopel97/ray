#include "M128Shuffle.h"

// use Y for the last component, because will more often hit a more optimizable case
#define RAY_GEN_MEMBER_SWIZZLE3(T, X, Y, Z) T X##Y##Z() const { return T(m128::perm<m128::lane::##X, m128::lane::##Y, m128::lane::##Z, m128::lane::##Y>(xmm)); }
#define RAY_GEN_MEMBER_SWIZZLE3_ALL(T) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, x, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, x, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, x, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, x, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, y, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, y, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, y, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, y, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, z, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, z, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, z, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, z, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, w, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, w, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, w, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, x, w, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, x, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, x, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, x, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, x, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, y, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, y, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, y, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, y, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, z, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, z, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, z, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, z, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, w, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, w, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, w, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, y, w, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, x, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, x, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, x, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, x, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, y, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, y, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, y, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, y, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, z, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, z, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, z, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, z, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, w, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, w, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, w, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, z, w, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, x, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, x, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, x, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, x, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, y, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, y, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, y, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, y, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, z, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, z, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, z, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, z, w) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, w, x) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, w, y) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, w, z) \
RAY_GEN_MEMBER_SWIZZLE3(T, w, w, w) 



#define RAY_GEN_MEMBER_SWIZZLE4(T, X, Y, Z, W) T X##Y##Z##W() const { return T(m128::perm<m128::lane::##X, m128::lane::##Y, m128::lane::##Z, m128::lane::##W>(xmm)); }
#define RAY_GEN_MEMBER_SWIZZLE4_ALL(T) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, x, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, y, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, z, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, x, w, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, x, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, y, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, z, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, y, w, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, x, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, y, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, z, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, z, w, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, x, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, y, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, z, w, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, x, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, x, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, x, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, x, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, y, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, y, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, y, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, y, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, z, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, z, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, z, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, z, w) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, w, x) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, w, y) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, w, z) \
RAY_GEN_MEMBER_SWIZZLE4(T, w, w, w, w) 