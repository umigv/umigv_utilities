#ifndef UMIGV_ZIP_HPP
#define UMIGV_ZIP_HPP

// defines umigv::zip, which takes a variadic number of iterable ranges and zips them together

#include "umigv_utilities/fold.hpp"
#include "umigv_utilities/traits.hpp"
#include "umigv_utilities/types.hpp"

#include <iterator>
#include <memory>
#include <tuple>
#include <type_traits>

#include <boost/optional.hpp>

namespace umigv {

template <typename IterTuple, usize ...Is>
class ZippedRange;

template <typename Iterators, usize ...Is>
class ZippedRangeIterator {
public:
    friend ZippedRange<Iterators, Is...>;

    using difference_type = isize;
    using value_type = std::tuple<iterator_reference_t<
        std::tuple_element_t<Is, Iterators>
    >...>;
    using pointer = const value_type*;
    using reference = const value_type&;
    using iterator_category = std::input_iterator_tag;

    constexpr reference operator*() const {
        if (any_equal_last()) {
            throw std::out_of_range{ "ZippedRangeIterator::operator*" };
        }

        deref_.emplace(*std::get<Is>(current_)...);

        return deref_.value();
    }

    constexpr pointer operator->() const {
        return std::addressof(**this);
    }

    constexpr ZippedRangeIterator& operator++() {
        if (any_equal_last()) {
            throw std::out_of_range{ "ZippedRangeIterator::operator++" };
        }

        auto increment_op = [](auto &&x) {
            return ++std::forward<decltype(x)>(x);
        };

        unary_fold(increment_op, std::get<Is>(current_)...);

        return *this;
    }

    constexpr friend bool operator==(const ZippedRangeIterator lhs,
                                     const ZippedRangeIterator rhs) {
        if (!boolean_and(
            (std::get<Is>(lhs.last_) == std::get<Is>(rhs.last_))...
        )) {
            throw std::out_of_range{
                "operator==(ZippedRangeIterator, ZippedRangeIterator)"
            };
        }

        return boolean_or(
            (std::get<Is>(lhs.current_) == std::get<Is>(rhs.current_))...
        );
    }

    constexpr friend bool operator!=(const ZippedRangeIterator lhs,
                                     const ZippedRangeIterator rhs) {
        if (!boolean_and(
            (std::get<Is>(lhs.last_) == std::get<Is>(rhs.last_))...
        )) {
            throw std::out_of_range{
                "operator!=(ZippedRangeIterator, ZippedRangeIterator)"
            };
        }

        return boolean_and(
            (std::get<Is>(lhs.current_) != std::get<Is>(rhs.current_))...
        );
    }

private:
    constexpr ZippedRangeIterator(Iterators current, Iterators last)
    noexcept(std::is_nothrow_move_constructible<Iterators>::value)
    : current_{ std::move(current) }, last_{ std::move(last) } { }

    constexpr bool any_equal_last() const
    noexcept(conjunction_v<is_nothrow_equality_comparable<
        std::tuple_element_t<Is, value_type>
    >...>) {
        const auto equal_op = [](auto &&lhs, auto &&rhs) -> decltype(auto) {
            return std::forward<decltype(lhs)>(lhs)
                   == std::forward<decltype(rhs)>(rhs);
        };

        const auto or_op = [](auto &&lhs, auto &&rhs) -> decltype(auto) {
            return std::forward<decltype(lhs)>(lhs)
                   || std::forward<decltype(rhs)>(rhs);
        };

        return binary_fold_pairs(or_op, equal_op, current_, last_);
    }

    Iterators current_;
    Iterators last_;
    mutable boost::optional<value_type> deref_;
};

template <typename Iterators, usize ...Is>
class ZippedRange {
    template <typename T>
    using DereferenceT = decltype((*std::declval<T>()));

public:
    using iterator = ZippedRangeIterator<Iterators, Is...>;
    using difference_type = iterator_difference_type<iterator>;

    constexpr ZippedRange(Iterators first, Iterators last)
    noexcept(std::is_nothrow_move_constructible<Iterators>::value)
    : first_{ std::move(first) }, last_{ std::move(last) } { }

    constexpr iterator begin() const {
        return { first_, last_ };
    }

    constexpr iterator end() const {
        return { last_, last_ };
    }

private:
    Iterators first_;
    Iterators last_;
};

template <typename IteratorTuple, usize ...Is>
typename ZippedRange<IteratorTuple, Is...>::Iterator
begin(const ZippedRange<IteratorTuple, Is...> &range)
    noexcept(noexcept(
        std::declval<ZippedRange<IteratorTuple, Is...>>().begin()
    ))
{
    return range.begin();
}

template <typename IteratorTuple, usize ...Is>
typename ZippedRange<IteratorTuple, Is...>::Iterator
end(const ZippedRange<IteratorTuple, Is...> &range)
    noexcept(noexcept(
        std::declval<ZippedRange<IteratorTuple, Is...>>().end()
    ))
{
    return range.end();
}

template <typename BeginTuple, typename EndTuple,
          typename = std::common_type_t<BeginTuple, EndTuple>, usize ...Is>
auto zip_impl(BeginTuple &&begins, EndTuple &&ends,
              std::index_sequence<Is...>) {
    using IteratorTupleT = std::common_type_t<BeginTuple, EndTuple>;
    using RangeT = ZippedRange<IteratorTupleT, Is...>;

    return RangeT{ std::forward<BeginTuple>(begins),
                   std::forward<EndTuple>(ends) };
}

template <typename ...Ranges>
auto zip(Ranges &&...ranges) {
    using std::begin;
    using std::end;

    return zip_impl(std::make_tuple(begin(std::forward<Ranges>(ranges))...),
                    std::make_tuple(end(std::forward<Ranges>(ranges))...),
                    std::index_sequence_for<Ranges...>{ });
}

} // namespace umigv

#endif
