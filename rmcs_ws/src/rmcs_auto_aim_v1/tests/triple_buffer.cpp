#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include "utils/mat_triple_buffer.hpp"   // 你的类定义头文件

using namespace world_exe::util::memory;

// 一个简单的时间戳 functor，用于测试
struct DummyClock {
    world_exe::data::TimeStamp operator()() const {
        return world_exe::data::TimeStamp::from_seconds(123456); // 假设 TimeStamp 可用整数初始化
    }
};

// 测试 fixture
class MatTripleBufferTest : public ::testing::Test {
protected:
    MatTripleBuffer<DummyClock> buffer{DummyClock{}};

    void SetUp() override {
        // 初始化 buffer 内部 Mat
        for (auto& slot : buffer.buffer) {
            slot.mat = cv::Mat::zeros(10, 10, CV_8UC1);
        }
    }
};

// 测试 set/get 基本功能
TEST_F(MatTripleBufferTest, SetAndGetWorks) {
    cv::Mat img = cv::Mat::ones(10, 10, CV_8UC1) * 42;
    buffer.set(img);

    auto result = buffer.get();
    ASSERT_TRUE(result.has_value());

    auto& stamped = result->get();
    EXPECT_EQ(cv::mean(stamped.mat)[0], 42);   // 均值应为 42
    EXPECT_EQ(stamped.stamp.to_seconds(), 123456);    // 时间戳应为 DummyClock 提供的值
}

// 测试 get 在没有数据时返回 nullopt
TEST_F(MatTripleBufferTest, GetWithoutSetReturnsNullopt) {
    auto result = buffer.get();
    EXPECT_FALSE(result.has_value());
}

// 测试多次 set/get
TEST_F(MatTripleBufferTest, MultipleSetAndGet) {
    for (int i = 0; i < 5; i++) {
        cv::Mat img = cv::Mat::ones(10, 10, CV_8UC1) * i;
        buffer.set(img);

        auto result = buffer.get();
        ASSERT_TRUE(result.has_value());
        auto& stamped = result->get();
        EXPECT_EQ(cv::mean(stamped.mat)[0], i);
    }
}
