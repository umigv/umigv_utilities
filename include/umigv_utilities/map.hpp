#ifndef UMIGV_MAP_HPP
#define UMIGV_MAP_HPP

// range helper; provided a range or iterator pair and a callable object
// (e.g. a function pointer, lambda, functor), will return the result of calling
// the function on the object

#include "types.hpp" // umigv::isize

#include "detail/invoke.hpp" // umigv::detail::invoke,
                             // umigv::detail::InvokeResultT

#include <iterator> // std::begin, std::end, std::iterator_traits
#include <type_traits> // std::result_of_t
#include <utility> // std::forward, std::initializer_list, std::declval

namespace umigv {

template <typename Begin, typename End, typename Function>
class MappedRange {
    using DereferenceT = decltype((*std::declval<Begin>()));
    using ReturnT = detail::InvokeResultT<Function, DereferenceT>;

public:
    using difference_type = isize;
    using value_type = typename std::decay<ReturnT>::type;
    using pointer = const value_type*;
    using reference = ReturnT;
    using iterator_category = std::forward_iterator_tag;

    template <typename F>
    MappedRange(Begin begin, End end, F &&f)
        : begin_{ begin }, end_{ end },
          function_{ std::forward<F>(f) }
    { }

    MappedRange& begin() noexcept {
        return *this;
    }

    MappedRange& end() noexcept {
        return *this;
    }

    MappedRange& operator++() {
        ++begin_;

        return *this;
    }

    friend bool operator==(const MappedRange lhs,
                           const MappedRange rhs) noexcept {
        return (lhs.begin_ == lhs.end_) and (lhs.end_ == rhs.end_);
    }

    friend bool operator!=(const MappedRange lhs,
                           const MappedRange rhs) noexcept {
        return (lhs.begin_ != lhs.end_) or (lhs.end_ != rhs.end_);
    }

    reference operator*() {
        return detail::invoke(function_, *begin_);
    }

private:
    Begin begin_;
    End end_;
    Function function_;
};

template <typename Range, typename Function>
auto map(Range &&range, Function &&function) {
    using std::begin;
    using std::end;

    using BeginT = decltype(begin(std::forward<Range>(range)));
    using EndT = decltype(end(std::forward<Range>(range)));
    using RangeT = MappedRange<BeginT, EndT, Function>;

    return RangeT{ begin(std::forward<Range>(range)),
                   end(std::forward<Range>(range)),
                   std::forward<Function>(function) };
}

template <typename T, typename Function>
auto map(const std::initializer_list<T> list, Function &&function) {
    using BeginT = decltype(begin(list));
    using EndT = decltype(end(list));
    using RangeT = MappedRange<BeginT, EndT, Function>;

    return RangeT{ std::begin(list), std::end(list),
                   std::forward<Function>(function) };
}

template <typename Begin, typename End, typename Function>
auto map(Begin &&begin, End &&end, Function &&function) {
    using RangeT = MappedRange<Begin, End, Function>;

    return RangeT{ std::forward<Begin>(begin), std::forward<End>(end),
                   std::forward<Function>(function) };
}

} // namespace umigv

#endif
