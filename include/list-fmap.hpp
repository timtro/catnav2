#pragma once

#include <list>
#include <type_traits>
#include <vector>

#include <boost/hana/functional/flip.hpp>

// fmap : (A → B) → F<A> → F<B>
//        ~~~~~~~
//           f
//  F = std::vector | std::list
//
template <typename T, typename A, typename F>
auto fmap(F &f, const std::vector<T, A> &as) {
  using MapedToType = std::invoke_result_t<F, T &>;
  std::vector<MapedToType> bs;
  bs.reserve(as.size());

  std::transform(cbegin(as), cend(as), std::back_inserter(bs),
                 std::forward<F>(f));

  return bs;
}

template <typename T, typename A, typename F>
auto fmap(F &f, const std::list<T, A> &as) {
  using MapedToType = std::invoke_result_t<F, T &>;
  std::list<MapedToType> bs;

  std::transform(cbegin(as), cend(as), std::back_inserter(bs),
                 std::forward<F>(f));

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
