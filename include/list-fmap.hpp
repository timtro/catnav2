#pragma once

#include <algorithm>
#include <array>
#include <experimental/type_traits>
#include <list>
#include <numeric>
#include <type_traits>
#include <vector>

#include <boost/hana/functional/flip.hpp>

namespace dtl_ {
  template <class T>
  using has_reserve_t =
      decltype(std::declval<T &>().reserve(std::declval<size_t>()));

  template <typename T>
  constexpr bool has_reserve_v =
      std::experimental::is_detected_v<has_reserve_t, T>;
}  // namespace dtl_

// fmap : (A → B) → ListF<A> → ListF<B>
//        ~~~~~~~
//           f
//  F = std::vector | std::list | ⋯
//
// This version specifies on a collection functor, ListF,  which can be
// constructed with a typename (F<typename>). This includes std::vector and
// std::list. If ListF has a reserve(·) member like std::vector does, it is
// called to avoid needless allocation.
template <template <typename...> typename ListF, typename A,
          typename... FCtorArgs, typename F>
auto fmap(F &f, const ListF<A, FCtorArgs...> &as) {
  ListF<std::invoke_result_t<F, A>> bs;

  if constexpr (dtl_::has_reserve_v<decltype(bs)>) bs.reserve(as.size());

  std::transform(cbegin(as), cend(as), std::back_inserter(bs),
                 std::forward<F>(f));
  return bs;
}

template <typename F, typename A, size_t N>
auto fmap(F &f, const std::array<A, N> &as) {
  std::array<std::invoke_result_t<F, A>, N> bs;

  std::transform(cbegin(as), cend(as), begin(bs), f);
  return bs;
}

template <typename F, typename A, size_t N>
auto fmap(F &f, const A (&as)[N]) {
  std::array<std::invoke_result_t<F, A>, N> bs;

  std::transform(cbegin(as), cend(as), begin(bs), f);
  return bs;
}

// foldl :: (B -> A -> B) -> B -> F<A> -> B
//          ~~~~~~~~~~~~     ~    ~~~~
//               f           b0    as
//  F = std::vector | std::list
//
template <typename F, typename B, typename A, typename Actr>
B foldl(const F &f, const B &b0, const std::vector<A, Actr> &as) {
  return std::accumulate(cbegin(as), cend(as), b0, f);
}

template <typename F, typename B, typename A, typename Actr>
B foldl(const F &f, const B &b0, const std::list<A, Actr> &as) {
  return std::accumulate(cbegin(as), cend(as), b0, f);
}

// foldr :: (A -> B -> B) -> B -> F<A> -> B
//          ~~~~~~~~~~~~     ~    ~~~~
//               f           b0   as
//  F = std::vector | std::list
//
template <typename F, typename B, typename A, typename Actr>
B foldr(const F &f, const B &b0, const std::vector<A, Actr> &as) {
  return std::accumulate(crbegin(as), crend(as), b0, boost::hana::flip(f));
}

template <typename F, typename B, typename A, typename Actr>
B foldr(const F &f, const B &b0, const std::list<A, Actr> &as) {
  return std::accumulate(crbegin(as), crend(as), b0, boost::hana::flip(f));
}
