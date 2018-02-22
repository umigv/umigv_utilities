#ifndef UMIGV_FILTER_HPP
#define UMIGV_FILTER_HPP

// dereferencing filter returns only elements that evaluate true by a predicate

#include "types.hpp" // umigv::isize
#include "detail/invoke.hpp" // umigv::detail::invoke

#include <iterator> // std::begin, std::end, std::iterator_traits
#include <type_traits> // std::common_type_t
#include <utility> // std::forward

namespace umigv {

template <typename It, typename Predicate>
class FilteredRange {
public:
    class Iterator {
    public:
        friend FilteredRange;

        using difference_type = isize;
        using value_type = typename std::iterator_traits<It>::value_type;
        using pointer = typename std::iterator_traits<It>::pointer;
        using reference = typename std::iterator_traits<It>::reference;
        using iterator_category = std::forward_iterator_tag;

        Iterator& operator++()
            noexcept(noexcept(++std::declval<It&>())
                     and noexcept(std::declval<It>() == std::declval<It>())
                     and noexcept(detail::invoke(std::declval<Predicate>(),
                                                 *std::declval<It>())))
        {
            while (current_ != end_) {
                ++current_;

                if (detail::invoke(predicate_, *current_)) {
                    break;
                }
            }

            return *this;
        }

        reference operator*() const noexcept(noexcept(*std::declval<It>())) {
            return *current_;
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
        Iterator(It current, It end, Predicate predicate)
            noexcept(noexcept(It{ std::move(std::declval<It>()) })
                     and noexcept(Predicate{
                        std::move(std::declval<Predicate>())
                     }))
            : current_{ std::move(current) }, end_{ std::move(end) },
              predicate_{ std::move(predicate) }
        { }

        It current_;
        It end_;
        Predicate predicate_;
    };

    FilteredRange(It begin, It end, Predicate predicate)
        noexcept(noexcept(It{ std::move(std::declval<It>()) })
                 and noexcept(Predicate{
                    std::move(std::declval<Predicate>())
                 }))
        : begin_{ std::move(begin) }, end_{ std::move(end) },
          predicate_{ std::move(predicate) }
    {
        while (not detail::invoke(predicate_, *begin_) and begin_ != end_) {
            ++begin_;
        }
    }

    Iterator begin() const
        noexcept(noexcept(Iterator{ std::declval<It>(), std::declval<It>(),
                                    std::declval<Predicate>() }))
    {
        return Iterator{ begin_, end_, predicate_ };
    }

    Iterator end() const
        noexcept(noexcept(Iterator{ std::declval<It>(), std::declval<It>(),
                                    std::declval<Predicate>() }))
    {
        return Iterator{ end_, end_, predicate_ };
    }

private:
    It begin_;
    It end_;
    Predicate predicate_;
};

template <typename Iterator, typename Predicate>
typename FilteredRange<Iterator, Predicate>::Iterator
begin(const FilteredRange<Iterator, Predicate> &range)
    noexcept(noexcept(
        std::declval<FilteredRange<Iterator, Predicate>>().begin()
    ))
{
    return range.begin();
}

template <typename Iterator, typename Predicate>
typename FilteredRange<Iterator, Predicate>::Iterator
end(const FilteredRange<Iterator, Predicate> &range)
    noexcept(noexcept(
        std::declval<FilteredRange<Iterator, Predicate>>().end()
    ))
{
    return range.end();
}

template <typename Range, typename Predicate>
auto filter(Range &&range, Predicate &&predicate) {
    using std::begin;
    using std::end;

    using IteratorT = decltype((begin(std::forward<Range>(range))));
    using RangeT = FilteredRange<IteratorT, Predicate>;

    return RangeT{ begin(std::forward<Range>(range)),
                   end(std::forward<Range>(range)),
                   std::forward<Predicate>(predicate) };
}

template <typename T, typename Predicate>
auto filter(const std::initializer_list<T> list, Predicate &&predicate) {
    using std::begin;
    using std::end;

    using IteratorT = decltype((begin(list)));
    using RangeT = FilteredRange<IteratorT, Predicate>;

    return RangeT{ begin(list), end(list), std::forward<Predicate>(predicate) };
}

template <typename Begin, typename End, typename Predicate,
          typename = std::common_type_t<Begin, End>>
auto filter(Begin begin, End end, Predicate &&predicate) {
    using IteratorT = std::common_type_t<Begin, End>;
    using RangeT = FilteredRange<IteratorT, Predicate>;

    return RangeT{ std::forward<Begin>(begin), std::forward<End>(end),
                   std::forward<Predicate>(predicate) };
}

} // namespace umigv

#endif
