#ifndef UMIGV_ENUMERATE_HPP
#define UMIGV_ENUMERATE_HPP

// range helper; provided a range or iterator pair, will enumerate each
// element upon dereferencing

#include "types.hpp" // umigv::usize, umigv::isize

#include <iterator> // std::begin, std::end, std::iterator_traits
#include <utility> // std::forward, std::initializer_list, std::pair

namespace umigv {

// lazy evaluation iterator that return a pair
template <typename Count, typename Begin, typename End>
class EnumeratedRange {
public:
    using difference_type = isize;
    using value_type = std::pair<Count, decltype((*std::declval<Begin>()))>;
    using pointer = const value_type*;
    using reference = const value_type&;
    using iterator_category = std::forward_iterator_tag;

    EnumeratedRange(Begin begin, End end) : begin_{ begin }, end_{ end },
                                            index_{ 0 } { }

    EnumeratedRange& begin() noexcept {
        return *this;
    }

    EnumeratedRange& end() noexcept {
        return *this;
    }

    EnumeratedRange& operator++() {
        ++index_;
        ++begin_;

        return *this;
    }

    friend bool operator==(const EnumeratedRange lhs,
                           const EnumeratedRange rhs) noexcept {
        return (lhs.begin_ == lhs.end_) and (lhs.end_ == rhs.end_);
    }

    friend bool operator!=(const EnumeratedRange lhs,
                           const EnumeratedRange rhs) noexcept {
        return (lhs.begin_ != lhs.end_) or (lhs.end_ != rhs.end_);
    }

    value_type operator*() const {
        return { index_, *begin_ };
    }

private:
    Begin begin_;
    End end_;
    Count index_;
};

template <typename Count, typename Begin, typename End>
decltype(auto) begin(EnumeratedRange<Count, Begin, End> &range) noexcept {
    return range.begin();
}

template <typename Count, typename Begin, typename End>
decltype(auto) end(EnumeratedRange<Count, Begin, End> &range) noexcept {
    return range.end();
}

template <typename Count = usize, typename Range>
auto enumerate(Range &&range) {
    using std::begin;
    using std::end;

    using BeginT = decltype(begin(std::forward<Range>(range)));
    using EndT = decltype(end(std::forward<Range>(range)));
    using RangeT = EnumeratedRange<Count, BeginT, EndT>;

    return RangeT{ begin(std::forward<Range>(range)),
                   end(std::forward<Range>(range)) };
}

template <typename Count = usize, typename Begin, typename End>
auto enumerate(Begin &&begin, End &&end) {
    using RangeT = EnumeratedRange<Count, Begin, End>;

    return RangeT{ std::forward<Begin>(begin), std::forward<End>(end) };
}

template <typename Count = usize, typename T>
auto enumerate(const std::initializer_list<T> list) {
    using IteratorT = decltype(std::begin(list));
    using RangeT = EnumeratedRange<Count, IteratorT, IteratorT>;

    return RangeT{ std::begin(list), std::end(list) };
}

} // namespace umigv

#endif
