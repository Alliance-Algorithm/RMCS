#include "data/sync_data.hpp"
#include "v1/sync/syncer.hpp"
#include <data/time_stamped.hpp>
#include <gtest/gtest.h>
#include <ctime>
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <iomanip>

namespace world_exe::tests::sync {

using ns_t = int64_t; // 纳秒

struct FrameB {
    data::CameraGimbalMuzzleSyncData data; // 纳秒
    FrameB(ns_t t) {
        data.camera_capture_begin_time_stamp = data::TimeStamp::from_nanosec(t);
    }
};

struct SampleA {
    long id;
    ns_t timestamp; // 纳秒
    const data::CameraGimbalMuzzleSyncData* matchedB = nullptr;
    SampleA(ns_t t) : timestamp(t) {}
};

static double ns_to_sec(ns_t ns) { return static_cast<double>(ns) / 1'000'000'000.0; }

// NOTE: changed return type to void so ASSERT_* (which may expand to return;) is valid here.

void sync_test_main() {
    // 配置（以秒为参考，然后换算成纳秒）
    const double frequency = 165.0;
    const ns_t NS_PER_SEC = 1'000'000'000LL;
    const ns_t period_ns = static_cast<ns_t>(NS_PER_SEC / frequency + 0.5);
    const double total_seconds = 2.0;
    const ns_t total_ns = static_cast<ns_t>(total_seconds * NS_PER_SEC + 0.5);
    const data::TimeStamp min_margin_to_next_b = data::TimeStamp::from_nanosec(500'000); // 0.0005s

    const unsigned seed = 12345;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> aCountDist(0, 3);
    std::uniform_real_distribution<double> unitDist(0.0, 1.0);

    v1::Syncer syncer(std::chrono::seconds(2), static_cast<long>(500000));
    std::vector<data::CameraGimbalMuzzleSyncData> generatedBs;
    std::vector<data::CameraGimbalMuzzleSyncData> generatedAs;

    // 生成 B 时间点（纳秒）
    std::vector<data::TimeStamp> bTimes;
    for (ns_t t = 0; t <= total_ns + period_ns; t += period_ns) bTimes.push_back(data::TimeStamp::from_nanosec(t));

    // 第一个 B
    data::CameraGimbalMuzzleSyncData firstB{};
    firstB.camera_capture_begin_time_stamp = bTimes.front();
    generatedBs.push_back(firstB);
    syncer.set_data(firstB);
    for (size_t i = 0; i + 1 < bTimes.size(); ++i) {
        auto tb = bTimes[i];
        auto tbNext = bTimes[i + 1];

        data::CameraGimbalMuzzleSyncData b{};
        b.camera_capture_begin_time_stamp = tbNext;
        ASSERT_EQ(b.camera_capture_begin_time_stamp, tbNext) << "tbNext should be equal to camera_capture_begin_time_stamp";
        generatedBs.push_back(b);
        syncer.set_data(b);

        int aCountThisInterval = aCountDist(rng);
        for (int k = 0; k < aCountThisInterval; ++k) {
            auto windowStart = tb + data::TimeStamp::from_nanosec(1);
            auto windowEnd = std::max(windowStart, tbNext - min_margin_to_next_b);
            if (windowEnd <= windowStart) continue;
            double u = unitDist(rng);
            data::TimeStamp tA = windowStart + ((windowEnd - windowStart) * u);

            data::CameraGimbalMuzzleSyncData a{};
            a.camera_capture_begin_time_stamp = tA;
            generatedAs.push_back(a);

            ASSERT_LT(tb, tA) << "ta should be larger then tb";
            auto [matched, ok] = syncer.get_data(tA);
            ASSERT_TRUE(ok) << "get_data should succeed for A after at least one B";
            ASSERT_LE(matched.camera_capture_begin_time_stamp, tA) << "matched B timestamp <= A timestamp";

            auto it = std::find_if(generatedBs.begin(), generatedBs.end(),
                                   [&](const data::CameraGimbalMuzzleSyncData& db) {
                                       return db.camera_capture_begin_time_stamp == matched.camera_capture_begin_time_stamp;
                                   });
            ASSERT_NE(it, generatedBs.end()) << "matched B must be found in generatedBs";
            size_t matchedIndex = std::distance(generatedBs.begin(), it);
            if (matchedIndex + 1 < generatedBs.size()) {
                const auto& nextB = generatedBs[matchedIndex + 1];
                ASSERT_LT(tA, nextB.camera_capture_begin_time_stamp) << "A must occur before next B";
                auto gapToNextB = nextB.camera_capture_begin_time_stamp - tA;
                ASSERT_GE(gapToNextB, min_margin_to_next_b) << "A too close to next B";
            }
        }
    }


    std::cout << std::fixed << std::setprecision(6);
    std::cout << "B count: " << generatedBs.size() << "\n";
    std::cout << "A count: " << generatedAs.size() << "\n";

    size_t limit = std::min<size_t>(generatedAs.size(), 10);
    for (size_t i = 0; i < limit; ++i) {
        const auto& a = generatedAs[i];
        auto [matched, ok] = syncer.get_data(a.camera_capture_begin_time_stamp);
        std::cout << "A#" << i
                  << " t(ns)=" << a.camera_capture_begin_time_stamp.to_nanosec() << " t(s)=" << a.camera_capture_begin_time_stamp.to_seconds()
                  << " -> matched B t(ns)=" << matched.camera_capture_begin_time_stamp.to_nanosec()
                  << " t(s)=" << matched.camera_capture_begin_time_stamp.to_seconds() << "\n";
    }

    std::cout << "All assertions passed.\n";
}

} // namespace world_exe::tests::sync
