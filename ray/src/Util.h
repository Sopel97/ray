#pragma once

#include <cstddef>
#include <tuple>
#include <utility>

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

    float mix(float a, float b, float mix)    
    {
        return b * mix + a * (1.0f - mix);
    }

    template <typename TupleT, typename FuncT>
    void for_each(TupleT&& tuple, FuncT&& f) {
        constexpr std::size_t N = std::tuple_size<std::remove_reference_t<TupleT>>::value;
        detail::for_each_impl(std::forward<TupleT>(tuple), std::forward<FuncT>(f),
            std::make_index_sequence<N>{});
    }
}
