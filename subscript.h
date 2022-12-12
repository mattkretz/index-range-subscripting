/*{ Copyright © 2022 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH
                     Matthias Kretz <m.kretz@gsi.de>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the names of contributing organizations nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
}*/

#include <concepts>
#include <cassert>
#include <ranges>
#include <span>
#include <type_traits>

template <auto N>
  inline constexpr std::integral_constant<decltype(N), N> Const = {};

namespace detail
{
  template <typename T>
    concept index_like = (std::integral<T> && !std::same_as<T, bool>)
                           || (std::is_class_v<T> && requires(int* p, T i)
    {
      { p[i] } -> std::same_as<int&>;
      { p[i + 1] } -> std::same_as<int&>;
    });

  template <typename T>
    concept constexpr_index
      = index_like<T>
          && std::integral<typename T::value_type>
          && requires(T x) {
            typename std::integral_constant<typename T::value_type, T::value>;
          };

  template <typename T>
    concept positive_index
      = std::unsigned_integral<T> || (constexpr_index<T> && T::value >= 0);

  static_assert( positive_index<unsigned>);
  static_assert(!positive_index<int>);
  static_assert( positive_index<decltype(Const<1>)>);
  static_assert(!positive_index<decltype(Const< -1>)>);

  static_assert( constexpr_index<decltype(Const<1>)>);
  static_assert(!constexpr_index<int>);

  template <bool Cond, typename T>
    using maybe_const = std::conditional_t<Cond, const T, T>;
}

/**
 * range[a, b]
 *
 * a: if a >= 0: offset from begin (0 is begin)
 *    if a < 0: offset from end (inclusive; -1 is last element before end)
 *
 * b: if b >= 0: size of resulting range/span
 *    if b < 0: offset from end (inclusive)
 *
 * Examples:
 * [ 0,  2]: elements [0] and [1]
 * [ 4,  3]: elements [4], [5], and [6]
 * [-1, -1]: the last element
 * [-1,  1]: the last element
 * [-4, -2]: the three elements [-4], [-3], and [-2] (where [-1] is the last element)
 * [-1, -2]: empty
 * [-1, -3]: ill-formed
 * [-1,  2]: ill-formed
 * [ 0, -2]: all elements but the last
 * [ 1,  0]: empty
 * [-1,  0]: empty
 */

#ifdef __GXX_NONMEMBER_SUBSCRIPT__
#define SUBOP operator[]
#else
#define SUBOP operator_sub
#endif

// empty range if Size == 0
template <std::ranges::range R, detail::constexpr_index Size>
  requires(Size::value == 0)
  constexpr std::ranges::empty_view<std::ranges::range_value_t<R>>
  SUBOP(R&&, detail::index_like auto, Size)
  { return {}; }

// empty range if First index one behind Last index (requires both to be negative)
template <std::ranges::range R, detail::constexpr_index First, detail::constexpr_index Last>
  requires(First::value < 0 && Last::value < 0 && First::value > Last::value)
  constexpr std::ranges::empty_view<std::ranges::range_value_t<R>>
  SUBOP(R&&, First, Last)
  {
    static_assert(First::value == Last::value + 1, "index range results in negative size");
    return {};
  }

namespace detail
{
  template <detail::index_like First, detail::index_like Last>
    consteval auto span_size()
    {
      if constexpr (not detail::constexpr_index<Last>)
        return std::dynamic_extent;
      else if constexpr (Last::value > 0)
        return Last::value;
      else if constexpr (not detail::constexpr_index<First>)
        return std::dynamic_extent;
      else if constexpr (First::value < 0 && Last::value < 0)
        {
          if constexpr (First::value <= Last::value)
            return Last::value + 1 - First::value;
          // ill-formed (SFINAE)
        }
      else
        return std::dynamic_extent;
    }
}

// otherwise, given a contiguous_range
// if resulting size is known: span with static extent
// - First >= 0 && Last > 0 => size = Last
// - First <  0 && Last < 0 => size = Last + 1 - First
// else span with dynamic extent
template <std::ranges::contiguous_range R, detail::index_like First, detail::index_like Last>
  constexpr std::span<detail::maybe_const<
                        not std::ranges::output_range<R, std::ranges::range_value_t<R>>,
                        std::ranges::range_value_t<R>>, detail::span_size<First, Last>()>
  SUBOP(R&& range, First first, Last last)
  {
    constexpr auto size = detail::span_size<First, Last>();
    using Span = std::span<detail::maybe_const<
                             not std::ranges::output_range<R, std::ranges::range_value_t<R>>,
                             std::ranges::range_value_t<R>>, size>;
    if constexpr (size == std::dynamic_extent)
      {
        if constexpr (detail::constexpr_index<First> && detail::constexpr_index<Last>)
          {
            if constexpr (first >= 0)
              {
                static_assert(last < 0); // >=0 is static extent
                return {range.begin() + first, range.end() + last};
              }
            else
              {
                static_assert(last >= 0); // <0 is static extent
                return {range.end() + first, range.end() + first + last};
              }
          }
        else if (first >= 0 && last >= 0) // always true if both are unsigned
          return {std::ranges::data(range) + first, std::size_t(last)};
        else if (first >= 0 && last < 0)
          return {range.begin() + first, range.end() + last};
        else if (first < 0 && last >= 0)
          return {range.end() + first, range.end() + first + last};
        else
          return {range.end() + first, range.end() + last};
      }
    else if constexpr (detail::positive_index<First>)
      return Span{std::ranges::data(range) + first, size};
    else // implies First < 0
      {
        static_assert(First::value + static_cast<typename First::value_type>(size) <= 0,
                      "index range exceeds bounds");
        return Span{range.end() + first, size};
      }
  }

// otherwise, given a random_access_range, return a subrange
template <std::ranges::random_access_range R, detail::index_like First, detail::index_like Last>
  requires std::ranges::sized_range<R> && (not std::ranges::contiguous_range<R>)
  constexpr std::ranges::subrange<std::ranges::iterator_t<R>, std::ranges::iterator_t<R>,
                                  std::ranges::subrange_kind::sized>
  SUBOP(R&& range, First first, Last last)
  {
    using D = std::ranges::range_difference_t<R>;
    using I = std::ranges::iterator_t<R>;
    using U = std::make_unsigned_t<std::ranges::range_difference_t<R>>;
    const auto range_size = std::ranges::size(range);
    const auto size = last >= 0 ? last
                                : first >= 0 ? range_size - first + last + 1
                                             : last + 1 - first;
    assert (size >= 0);
    I b = std::ranges::begin(range);
    std::ranges::advance(b, first >= 0 ? first : range_size + first);
    I e = b;
    std::ranges::advance(e, size);
    return {b, e, U(size)};
  }

