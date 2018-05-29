#ifndef UMIGV_ZIP_HPP
#define UMIGV_ZIP_HPP

// defines umigv::zip, which takes a variadic number of iterable ranges and zips them together

#include "umigv_utilities/types.hpp"
#include "umigv_utilities/detail/boolean.hpp"

#include <type_traits>
#include <iterator>
#include <tuple> // std::tuple, std::tuple_element_t

namespace umigv {

template <typename IteratorTuple, usize ...Is>
class ZippedRange {
    template <typename T>
    using DereferenceT = decltype((*std::declval<T>()));

public:
    class Iterator {
    public:
        friend ZippedRange;

        using difference_type = isize;
        using value_type = std::tuple<DereferenceT<
            std::tuple_element_t<Is, IteratorTuple>
        >...>;
        using pointer = const value_type*;
        using reference = const value_type&;
        using iterator_category = std::forward_iterator_tag;

        Iterator& operator++() {
            __attribute__((unused)) auto i =
                { (++std::get<Is>(current_), 0)... };

            return *this;
        }

        value_type operator*() const {
            return value_type{ *std::get<Is>(current_)... };
        }

        friend bool operator==(const Iterator lhs, const Iterator rhs)
            noexcept(noexcept(
                detail::boolean_or((std::get<Is>(std::declval<IteratorTuple>())
                                    == std::get<Is>(
                                        std::declval<IteratorTuple>()
                                    ))...)
            ))
        {
            return detail::boolean_or((std::get<Is>(lhs.current_)
                                       == std::get<Is>(rhs.current_))...);
        }

        friend bool operator!=(const Iterator lhs, const Iterator rhs)
            noexcept(noexcept(
                detail::boolean_and((std::get<Is>(std::declval<IteratorTuple>())
                                     != std::get<Is>(
                                         std::declval<IteratorTuple>()
                                     ))...)
            ))
        {
            return detail::boolean_and((std::get<Is>(lhs.current_)
                                        != std::get<Is>(rhs.current_))...);
        }

    private:
        explicit Iterator(IteratorTuple current)
            noexcept(noexcept(
                IteratorTuple{ std::move(std::declval<IteratorTuple>()) }
            ))
            : current_{ std::move(current) }
        { }

        IteratorTuple current_;
    };

    ZippedRange(IteratorTuple begins, IteratorTuple ends)
        noexcept(noexcept(
            IteratorTuple{ std::move(std::declval<IteratorTuple>()) }
        ))
        : begins_{ std::move(begins) },
          ends_{ std::move(ends) }
    { }

    Iterator begin() const
        noexcept(noexcept(Iterator{ std::declval<IteratorTuple>() }))
    {
        return Iterator{ begins_ };
    }

    Iterator end() const
        noexcept(noexcept(Iterator{ std::declval<IteratorTuple>() }))
    {
        return Iterator{ ends_ };
    }

private:
    IteratorTuple begins_;
    IteratorTuple ends_;
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
