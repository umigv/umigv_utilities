#include "umigv_utilities/invoke.hpp"

#include <string>

struct A {
    int i;
};

int main() {
    static_assert(umigv::is_invocable_v<decltype(&std::string::size), const std::string&>, "");
    static_assert(umigv::is_invocable_v<decltype(&std::string::size), const std::string*>, "");

    static_assert(!umigv::is_nothrow_invocable_v<decltype(&std::string::size), const std::string&>, "");
    static_assert(!umigv::is_nothrow_invocable_v<decltype(&std::string::size), const std::string*>, "");

    static_assert(umigv::is_nothrow_invocable_v<decltype(&A::i), const A&>, "");
    static_assert(umigv::is_nothrow_invocable_v<decltype(&A::i), const A*>, "");
}
