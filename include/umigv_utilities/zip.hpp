#ifndef UMIGV_ZIP_HPP
#define UMIGV_ZIP_HPP

// defines umigv::zip, which takes a variadic number of iterable ranges and zips them together

#include "types.hpp" // umigv::isize
#include "detail/boolean.hpp" // umigv::detail::boolean_and, umigv::detail::equal

#include <type_traits>
#include <iterator>
#include <tuple> // std::tuple, std::tuple_element_t

namespace umigv {

template <typename BeginTuple, typename EndTuple, usize ...Is>
class ZippedRange {
    template <typename T>
    using DereferenceT = decltype((*std::declval<T>()));

public:
    using difference_type = isize;
    using value_type = std::tuple<DereferenceT<
        std::tuple_element_t<Is, BeginTuple>
    >...>;
    using pointer = const value_type*;
    using reference = const value_type&;
    using iterator_category = std::forward_iterator_tag;

    template <typename B, typename E>
    ZippedRange(B &&begins, E &&ends) : begins_{ std::forward<B>(begins) },
                                        ends_{ std::forward<E>(ends) } { }

    ZippedRange& begin() noexcept {
        return *this;
    }

    ZippedRange& end() noexcept {
        return *this;
    }

    ZippedRange& operator++() {
        __attribute__((unused)) auto i = { (++std::get<Is>(begins_), 0)... };

        return *this;
    }

    friend bool operator==(const ZippedRange lhs,
                           const ZippedRange rhs)
        noexcept(noexcept(lhs.ends_ == rhs.ends_
                          and detail::boolean_or(
                              (std::get<Is>(lhs.begins_)
                              == std::get<Is>(lhs.ends_))...
                          ))) {
        return lhs.ends_ == rhs.ends_
               and detail::boolean_or((std::get<Is>(lhs.begins_)
                                       == std::get<Is>(lhs.ends_))...);
    }

    friend bool operator!=(const ZippedRange lhs,
                           const ZippedRange rhs)
        noexcept(noexcept(lhs.ends_ != rhs.ends_
                          or detail::boolean_and(
                              (std::get<Is>(lhs.begins_)
                              != std::get<Is>(lhs.ends_))...
                          ))) {
        return lhs.ends_ != rhs.ends_
               or detail::boolean_and((std::get<Is>(lhs.begins_)
                                       != std::get<Is>(lhs.ends_))...);
    }

    value_type operator*() const {
        return value_type{ *std::get<Is>(begins_)... };
    }

private:
    BeginTuple begins_;
    EndTuple ends_;
};

template <typename Begin1, typename Begin2, typename End1, typename End2>
class ZippedRange<std::tuple<Begin1, Begin2>, std::tuple<End1, End2>, 0, 1> {
    template <typename T>
    using DereferenceT = decltype((*std::declval<T>()));

public:
    using difference_type = isize;
    using value_type = std::pair<DereferenceT<Begin1>, DereferenceT<Begin2>>;
    using pointer = const value_type*;
    using reference = const value_type&;
    using iterator_category = std::forward_iterator_tag;

    template <typename B, typename E>
    ZippedRange(B &&begins, E &&ends)
        : begins_{ std::get<0>(std::forward<B>(begins)),
                   std::get<1>(std::forward<B>(begins)) },
          ends_{ std::get<0>(std::forward<E>(ends)),
                 std::get<1>(std::forward<E>(ends)) }
    { }

    ZippedRange& begin() noexcept {
        return *this;
    }

    ZippedRange& end() noexcept {
        return *this;
    }

    ZippedRange& operator++() {
        ++begins_.first;
        ++begins_.second;

        return *this;
    }

    friend bool operator==(const ZippedRange lhs,
                           const ZippedRange rhs) noexcept {
        return ((lhs.begins_.first == lhs.ends_.first)
                or (lhs.begins_.second == lhs.ends_.second))
               and (lhs.ends_ == rhs.ends_);
    }

    friend bool operator!=(const ZippedRange lhs,
                           const ZippedRange rhs) noexcept {
        return ((lhs.begins_.first != lhs.ends_.first)
                and (lhs.begins_.second != lhs.ends_.second))
               or (lhs.ends_ != rhs.ends_);
    }

    value_type operator*() const {
        return { *begins_.first, *begins_.second };
    }

private:
    std::pair<Begin1, Begin2> begins_;
    std::pair<End1, End2> ends_;
};

template <typename BeginTuple, typename EndTuple, usize ...N>
decltype(auto) begin(ZippedRange<BeginTuple, EndTuple, N...> &range) noexcept {
    return range.begin();
}

template <typename BeginTuple, typename EndTuple, usize ...N>
decltype(auto) end(ZippedRange<BeginTuple, EndTuple, N...> &range) noexcept {
    return range.end();
}

template <usize ...N, typename BeginTuple, typename EndTuple>
auto zip_impl(BeginTuple &&begins, EndTuple &&ends, std::index_sequence<N...>) {
    using std::begin;
    using std::end;

    using RangeT = ZippedRange<BeginTuple, EndTuple, N...>;

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
