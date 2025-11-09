#include "utility/duck_type.hpp"

#include <cstddef>
#include <print>
#include <string>

#include <gtest/gtest.h>

template <class T>
concept is_logger = requires(const T& t) {
    { t.info("") };
};
struct check_logger {
    template <class T>
    struct result {
        static constexpr auto v = is_logger<T>;
    };
};

TEST(DuckArrayTest, Access) {

    // Duck type
    struct LoggerType0 {
        std::string head { "[logger-type-0]" };
        auto info(const char* const msg) const { std::println("{}: {}", head, msg); }
        static auto unique_func() { return "LoggerType0::unique_func"; }
    };
    struct LoggerType1 {
        std::string head { "[logger-type-1]" };
        auto info(const std::string& msg) const { std::println("{}: {}", head, msg); }
    };
    struct LoggerType2 {
        static auto info(const std::string& msg) {
            std::string head { "[logger-type-2]" };
            std::println("{}: {}", head, msg);
        }
    };

    const auto duck_array = rmcs::duck_array {
        check_logger {},
        LoggerType0 {},
        LoggerType1 {},
        LoggerType2 {},
    };

    // Assert
    static_assert(duck_array.contains<LoggerType1>(), "Do not contain LoggerType1");
    static_assert(duck_array.size() == 3, "Array has wrong size");

    EXPECT_EQ(decltype(duck_array)::size(), 3);

    duck_array.foreach ([](const auto& element) { element.info("Hello World"); });

    duck_array.at<0>().info("Hello World By LoggerType0::at<index>");
    EXPECT_STREQ(duck_array.at<0>().unique_func(), "LoggerType0::unique_func");

    duck_array.at<1>().info("Hello World By LoggerType1::at<index>");
    duck_array.at<2>().info("Hello World By LoggerType2::at<index>");

    auto has_unique_count = std::size_t { 0 };
    auto use_element      = [&](const auto& element) {
        using T = std::decay_t<decltype(element)>;
        if constexpr (std::same_as<T, LoggerType0>) {
            has_unique_count++;
            std::println("{}", element.unique_func());
        } else std::println("Has no unique function");
    };
    static auto index = std::size_t { 0 };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    duck_array.at(index++, use_element);
    duck_array.at(index++, use_element);
    duck_array.at(index++, use_element);
#pragma GCC diagnostic pop

    EXPECT_EQ(has_unique_count, 1);
}
