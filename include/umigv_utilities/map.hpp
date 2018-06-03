#ifndef UMIGV_MAP_HPP
#define UMIGV_MAP_HPP

// range helper; provided a range or iterator pair and a callable object
// (e.g. a function pointer, lambda, functor), will return the result of calling
// the function on the object

#include "umigv_utilities/types.hpp"
#include "umigv_utilities/invoke.hpp"
#include "umigv_utilities/traits.hpp"
#include "umigv_utilities/utility.hpp"

#include <iterator> // std::begin, std::end, std::iterator_traits
#include <type_traits> // std::common_type_t
#include <utility> // std::forward, std::initializer_list, std::declval

namespace umigv {

template <
    typename I, typename C,
    std::enable_if_t<
        is_input_iterator_v<I>
        && is_invocable_v<const C&, iterator_reference_t<I>>, int
    > = 0
>
class MappedRange;

template <
    typename I, typename C,
    std::enable_if_t<
        is_input_iterator_v<I>
        && is_invocable_v<const C&, iterator_reference_t<I>>, int
    > = 0
>
class MappedRangeIterator {
public:
    using difference_type = isize;
    using value_type = std::decay_t<invoke_result_t<const C&, iterator_reference_t<I>>>;
    using pointer = const value_type*;
    using reference = invoke_result_t<const C&, iterator_reference_t<I>>;
    using iterator_category =
        std::conditional_t<is_forward_iterator_v<I>
                           && std::is_default_constructible<C>::value,
                           std::forward_iterator_tag, std::input_iterator_tag>;

    constexpr MappedRangeIterator& operator++() {
        if (current_ == last_) {
            throw std::out_of_range{ "MappedRangeIterator::operator++" };
        }

        ++current_;

        return *this;
    }

    constexpr reference operator*() const {
        if (current_ == last_) {
            throw std::out_of_range{ "MappedRangeIterator::operator*" };
        }

        return invoke(callable_, *current_);
    }

    constexpr friend bool operator==(const MappedRangeIterator &lhs,
                                     const MappedRangeIterator &rhs) {
        if (lhs.last_ == rhs.last_) {
            throw std::out_of_range{
                "operator==(const MappedRangeIterator&, "
                "const MappedRangeIterator&)"
            };
        }

        return lhs.current_ == rhs.current_;
    }

    constexpr friend bool operator!=(const MappedRangeIterator &lhs,
                                     const MappedRangeIterator &rhs) {
        return !(lhs == rhs);
    }

private:
    friend MappedRange<I, C>;

    constexpr MappedRangeIterator(I current, I last,  C callable)
    noexcept(std::is_nothrow_move_constructible<I>::value
             && std::is_nothrow_move_constructible<C>::value)
    : current_{ std::move(current) }, last_{ std::move(last) },
      callable_{ std::move(callable) } { }

    I current_;
    I last_;
    C callable_;
};

template <
    typename I, typename C,
    std::enable_if_t<
        is_input_iterator_v<I>
        && is_invocable_v<const C&, iterator_reference_t<I>>, int
    >
>
class MappedRange {
public:
    using iterator = MappedRangeIterator<I, C>;
    using difference_type = iterator_difference_type_t<iterator>;

    MappedRange(I first, I last, C callable)
        : first_{ std::move(first) }, last_{ std::move(last) },
          callable_{ std::move(callable) }
    { }

    iterator begin() const
    noexcept(std::is_nothrow_constructible<iterator, I, I, C>::value) {
        return iterator{ first_, last_, callable_ };
    }

    iterator end() const
    noexcept(std::is_nothrow_constructible<iterator, I, I, C>::value) {
        return iterator{ last_, last_, callable_ };
    }

private:
    I first_;
    I last_;
    C callable_;
};

template <typename Iterator, typename Function>
typename MappedRange<Iterator, Function>::Iterator
begin(const MappedRange<Iterator, Function> &range)
    noexcept(noexcept(
        std::declval<const MappedRange<Iterator, Function>&>().begin()
    ))
{
    return range.begin();
}

template <typename Iterator, typename Function>
typename MappedRange<Iterator, Function>::Iterator
end(const MappedRange<Iterator, Function> &range)
    noexcept(noexcept(
        std::declval<const MappedRange<Iterator, Function>&>().end()
    ))
{
    return range.end();
}

template <typename Range, typename Function,
          std::enable_if_t<is_range_v<Range>, int> = 0>
auto map(Range &&range, Function &&function) {
    using IteratorT = begin_result_t<Range>;
    using RangeT = MappedRange<IteratorT, Function>;

    return RangeT{ ::adl::begin(std::forward<Range>(range)),
                   ::adl::end(std::forward<Range>(range)),
                   std::forward<Function>(function) };
}

template <typename Begin, typename End, typename Function,
          typename = std::common_type_t<Begin, End>>
auto map(Begin begin, End end, Function &&function) {
    using IteratorT = std::common_type_t<Begin, End>;
    using RangeT = MappedRange<IteratorT, Function>;

    return RangeT{ begin, end, std::forward<Function>(function) };
}

template <typename T, typename Function>
auto map(const std::initializer_list<T> list, Function &&function) {
    using std::begin;
    using std::end;

    using IteratorT = decltype(begin(list));
    using RangeT = MappedRange<IteratorT, Function>;

    return RangeT{ begin(list), end(list), std::forward<Function>(function) };
}

} // namespace umigv

#endif
