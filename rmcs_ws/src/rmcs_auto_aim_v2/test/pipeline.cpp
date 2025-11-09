#include "modules/pipeline/common.hpp"
#include <gtest/gtest.h>

using Int = std::int32_t;

struct Add {
    using I = Int;
    using O = Int;
    Int value;
    auto operator()(I i) const { return i + value; }
};
struct MultiplyByTwo {
    using I = Int;
    using O = Int;
    auto operator()(I i) const { return i * 2; }
};
struct Subtract {
    using I = Int;
    using O = Int;
    Int offset;
    auto operator()(I i) const { return i - offset; }
};

TEST(lazy_process, pipeline) {
    using namespace rmcs::pipe;

    auto add = Pipe<Add> { 2333 };
    auto mul = Pipe<MultiplyByTwo> {};
    auto sub = Pipe<Subtract> { 200 };

    auto pipeline = add | mul | sub;

    GTEST_ASSERT_EQ(pipeline(114514), ((114514 + 2333) * 2) - 200);
}
