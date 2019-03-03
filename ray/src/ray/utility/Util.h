#pragma once

#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <xmmintrin.h>

namespace ray
{
    namespace detail
    {
        template <typename TupleT, typename FuncT, std::size_t ...IndicesVs>
        void for_each_impl(TupleT&& tuple, FuncT&& f, std::index_sequence<IndicesVs...>) {
            using swallow = int[];
            (void)swallow {
                1,
                    (f(std::get<IndicesVs>(std::forward<TupleT>(tuple))), void(), int{})...
            };
        }
    }

    [[nodiscard]] inline float mix(float a, float b, float mix)
    {
        return b * mix + a * (1.0f - mix);
    }

    template <typename TupleT, typename FuncT>
    void for_each(TupleT&& tuple, FuncT&& f) {
        constexpr std::size_t N = std::tuple_size<std::remove_reference_t<TupleT>>::value;
        detail::for_each_impl(std::forward<TupleT>(tuple), std::forward<FuncT>(f),
            std::make_index_sequence<N>{});
    }

    /*
    // std::sqrt appears actually faster than anything else
    // will be reused for vectorized raycasts
    inline float rcp(float x)
    {
        return _mm_cvtss_f32(_mm_rcp_ss(_mm_set_ps1(x)));
    }

    inline float fastSqrt(float a)
    {
        //return std::sqrt(a);
        //return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ps1(a)));
        
        // f(x) = x*x - a
        // f(x) = 0
        // f'(x) = 2x
        // x_0 = rcp(rsqrt(a))
        // NR
        // x_1 = x_0 - f(x_0)/f'(x_0) = x_0 - ((x_0*x_0 - a)/(2*x_0))
        const float x_0 = _mm_cvtss_f32(_mm_rcp_ss(_mm_rsqrt_ss(_mm_set_ss(a))));
        const float x_1 = (x_0 + a / x_0) * 0.5f;
        return x_1;
    }
    */

    template <typename T>
    using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;
}
