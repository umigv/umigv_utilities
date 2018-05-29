#ifndef UMIGV_MAP_HPP
#define UMIGV_MAP_HPP

// range helper; provided a range or iterator pair and a callable object
// (e.g. a function pointer, lambda, functor), will return the result of calling
// the function on the object

#include "umigv_utilities/types.hpp"
#include "umigv_utilities/invoke.hpp"

#include <iterator> // std::begin, std::end, std::iterator_traits
#include <type_traits> // std::common_type_t
#include <utility> // std::forward, std::initializer_list, std::declval

namespace umigv {

template <typename It, typename Function>
class MappedRange {
    using DereferenceT = decltype((*std::declval<It>()));
    using ReturnT = invoke_result_t<Function, DereferenceT>;

public:
    class Iterator {
    public:
        friend MappedRange;

        using difference_type = isize;
        using value_type = typename std::decay<ReturnT>::type;
        using pointer = const value_type*;
        using reference = ReturnT;
        using iterator_category = std::forward_iterator_tag;

        Iterator& operator++() noexcept(noexcept(++std::declval<It&>())) {
            ++current_;
            return *this;
        }

        reference operator*() const
        noexcept(is_nothrow_invocable_v<Function, iterator_reference_t<It>>) {
            return invoke(function_, *current_);
        }

        friend bool operator==(const Iterator lhs, const Iterator rhs)
            noexcept(noexcept(std::declval<It>() == std::declval<It>()))
        {
            return lhs.current_ == rhs.current_;
        }

        friend bool operator!=(const Iterator lhs, const Iterator rhs)
            noexcept(noexcept(std::declval<It>() != std::declval<It>()))
        {
            return lhs.current_ != rhs.current_;
        }

    private:
        Iterator(It current, Function function)
            noexcept(
                noexcept(It{ std::move(std::declval<It>()) })
                and noexcept(Function{ std::move(std::declval<Function>()) })
            ) : current_{ std::move(current) },
                function_{ std::move(function) }
        { }

        It current_;
        Function function_;
    };

    template <typename F>
    MappedRange(It begin, It end, F &&f)
        : begin_{ std::move(begin) }, end_{ std::move(end) },
          function_{ std::forward<F>(f) }
    { }

    Iterator begin() const
        noexcept(noexcept(Iterator{ std::declval<It>(),
                                    std::declval<Function>() }))
    {
        return Iterator{ begin_, function_ };
    }

    Iterator end() const
        noexcept(noexcept(Iterator{ std::declval<It>(),
                                    std::declval<Function>() }))
    {
        return Iterator{ end_, function_ };
    }

private:
    It begin_;
    It end_;
    Function function_;
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

template <typename Range, typename Function>
auto map(Range &&range, Function &&function) {
    using std::begin;
    using std::end;

    using IteratorT = decltype(begin(std::forward<Range>(range)));
    using RangeT = MappedRange<IteratorT, Function>;

    return RangeT{ begin(std::forward<Range>(range)),
                   end(std::forward<Range>(range)),
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
