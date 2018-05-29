#ifndef UMIGV_TRAITS_HPP
#define UMIGV_TRAITS_HPP

// type traits header; defines custom type traits

#include <iterator>
#include <map>
#include <type_traits>
#include <vector>

namespace umigv {

template <typename ...Ts>
using void_t = void;

template <typename T>
struct is_rosparam_literal : std::false_type { };

template <typename T>
constexpr inline bool is_rosparam_literal_v = is_rosparam_literal<T>::value;

template <>
struct is_rosparam_literal<bool> : std::true_type { };

template <>
struct is_rosparam_literal<int> : std::true_type { };

template <>
struct is_rosparam_literal<float> : std::true_type { };

template <>
struct is_rosparam_literal<double> : std::true_type { };

template <>
struct is_rosparam_literal<std::string> : std::true_type { };

template <typename T>
struct is_rosparam : is_rosparam_literal<T> { };

template <typename T>
constexpr inline bool is_rosparam_v = is_rosparam<T>::value;

template <typename T>
struct is_rosparam<std::vector<T>> : is_rosparam_literal<T> { };

template <typename T>
struct is_rosparam<std::map<std::string, T>> : is_rosparam_literal<T> { };

template <typename T, typename = void>
struct is_iterator : std::false_type { };

template <typename T>
struct is_iterator<
    T,
    void_t<typename std::iterator_traits<T>::difference_type,
           typename std::iterator_traits<T>::value_type,
           typename std::iterator_traits<T>::pointer,
           typename std::iterator_traits<T>::reference,
           typename std::iterator_traits<T>::iterator_category>
> : std::true_type { };

template <typename T>
constexpr inline bool is_iterator_v = is_iterator<T>::value;

namespace detail {

template <typename T, bool is_iterator>
struct iterator_difference_type { };

template <typename T>
struct iterator_difference_type<T, true> {
    using type = typename std::iterator_traits<T>::difference_type;
};

template <typename T, bool is_iterator>
struct iterator_value_type { };

template <typename T>
struct iterator_value_type<T, true> {
    using type = typename std::iterator_traits<T>::value_type;
};

template <typename T, bool is_iterator>
struct iterator_pointer { };

template <typename T>
struct iterator_pointer<T, true> {
    using type = typename std::iterator_traits<T>::pointer;
};

template <typename T, bool is_iterator>
struct iterator_reference { };

template <typename T>
struct iterator_reference<T, true> {
    using type = typename std::iterator_traits<T>::reference;
};

template <typename T, bool is_iterator>
struct iterator_category { };

template <typename T>
struct iterator_category<T, true> {
    using type = typename std::iterator_traits<T>::iterator_category;
};

} // namespace detail

template <typename T>
struct iterator_difference_type
: detail::iterator_difference_type<T, is_iterator_v<T>> { };

template <typename T>
using iterator_difference_type_t = typename iterator_difference_type<T>::type;

template <typename T>
struct iterator_value_type
: detail::iterator_value_type<T, is_iterator_v<T>> { };

template <typename T>
using iterator_value_type_t = typename iterator_value_type<T>::type;

template <typename T>
struct iterator_pointer
: detail::iterator_pointer<T, is_iterator_v<T>> { };

template <typename T>
using iterator_pointer_t = typename iterator_pointer<T>::type;

template <typename T>
struct iterator_reference
: detail::iterator_reference<T, is_iterator_v<T>> { };

template <typename T>
using iterator_reference_t = typename iterator_reference<T>::type;

template <typename T>
struct iterator_category
: detail::iterator_category<T, is_iterator_v<T>> { };

template <typename T>
using iterator_category_t = typename iterator_category<T>::type;

namespace detail {

template <typename T, bool is_iterator>
struct is_input_iterator : std::false_type { };

template <typename T>
struct is_input_iterator<T, true>
: std::conditional_t<
    std::is_base_of_v<std::input_iterator_tag, iterator_category_t<T>>,
    std::true_type, std::false_type
> { };

template <typename T, bool is_iterator>
struct is_output_iterator : std::false_type { };

template <typename T>
struct is_output_iterator<T, true>
: std::conditional_t<
    std::is_base_of_v<std::output_iterator_tag, iterator_category_t<T>>,
    std::true_type, std::false_type
> { };

template <typename T, bool is_iterator>
struct is_forward_iterator : std::false_type { };

template <typename T>
struct is_forward_iterator<T, true>
: std::conditional_t<
    std::is_base_of_v<std::forward_iterator_tag, iterator_category_t<T>>,
    std::true_type, std::false_type
> { };

template <typename T, bool is_iterator>
struct is_bidirectional_iterator : std::false_type { };

template <typename T>
struct is_bidirectional_iterator<T, true>
: std::conditional_t<
    std::is_base_of_v<std::bidirectional_iterator_tag, iterator_category_t<T>>,
    std::true_type, std::false_type
> { };

template <typename T, bool is_iterator>
struct is_random_access_iterator : std::false_type { };

template <typename T>
struct is_random_access_iterator<T, true>
: std::conditional_t<
    std::is_base_of_v<std::random_access_iterator_tag, iterator_category_t<T>>,
    std::true_type, std::false_type
> { };

} // namespace detail

template <typename T>
struct is_input_iterator : detail::is_input_iterator<T, is_iterator_v<T>> { };

template <typename T>
constexpr inline bool is_input_iterator_v = is_input_iterator<T>::value;

template <typename T>
struct is_output_iterator : detail::is_output_iterator<T, is_iterator_v<T>> { };

template <typename T>
constexpr inline bool is_output_iterator_v = is_output_iterator<T>::value;

template <typename T>
struct is_forward_iterator
: detail::is_forward_iterator<T, is_iterator_v<T>> { };

template <typename T>
constexpr inline bool is_forward_iterator_v = is_forward_iterator<T>::value;

template <typename T>
struct is_bidirectional_iterator
: detail::is_bidirectional_iterator<T, is_iterator_v<T>> { };

template <typename T>
constexpr inline bool is_bidirectional_iterator_v
    = is_bidirectional_iterator<T>::value;

template <typename T>
struct is_random_access_iterator
: detail::is_random_access_iterator<T, is_iterator_v<T>> { };

template <typename T>
constexpr inline bool is_random_access_iterator_v
    = is_random_access_iterator<T>::value;

} // namespace umigv

#endif
