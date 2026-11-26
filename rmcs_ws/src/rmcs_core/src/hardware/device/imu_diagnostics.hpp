#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rmcs_executor/component.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "hardware/device/bmi088_ekf.hpp"

namespace rmcs_core::hardware::device {

class ImuDiagnostics {
public:
    ImuDiagnostics(rmcs_executor::Component& component, std::string csv_path) {
        const auto placeholder = std::string{"<timestamp>"};
        const auto pos = csv_path.find(placeholder);
        if (pos != std::string::npos) {
            const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::ostringstream timestamp;
            timestamp << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S");
            csv_path.replace(pos, placeholder.size(), timestamp.str());
        }
        native_csv_.open(csv_path);
        if (!native_csv_)
            throw std::runtime_error("Failed to open top IMU diagnostics: " + csv_path);
        native_csv_ << std::setprecision(std::numeric_limits<double>::max_digits10);
        native_csv_ << "sequence,sensor,sensor_sequence,host_steady_ns,board_time_quarter_us,"
                       "board_timestamp_valid,raw_timestamp_quarter_us,x_counts,y_counts,z_counts,"
                       "snapshot_time_quarter_us,qw,qx,qy,qz,correction_time_quarter_us,"
                       "chi_square,correction_attempted,correction_applied,gyro_saturated,"
                       "time_gap_rejected,snapshot_updated,queue_dropped\n";
        native_csv_.flush();

        values_.fill(kNaN);
        for (std::size_t i = kAccelSequence; i < kSignalCount; ++i)
            if (i != kChiSquare && i != kCorrectionTime)
                values_[i] = 0.;
        for (std::size_t i = 0; i < kSignalCount; ++i)
            component.register_output(
                "/gimbal/imu/" + std::string{kSignalNames[i]}, outputs_[i], values_[i]);
    }

    ~ImuDiagnostics() {
        update();
        native_csv_.flush();
    }

    void record(
        bool gyroscope, std::int16_t x, std::int16_t y, std::int16_t z, std::uint32_t raw_timestamp,
        std::optional<Bmi088Ekf::TimePoint> sample_time,
        const std::optional<Bmi088Ekf::Snapshot>& snapshot,
        const Bmi088Ekf::Diagnostics& diagnostics = {}) {
        Record record;
        record.sequence = ++sequence_;
        record.gyroscope = gyroscope;
        record.sensor_sequence = ++sensor_sequences_[gyroscope];
        record.host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count();
        record.board_ticks = sample_time ? sample_time->time_since_epoch().count() : -1;
        record.timestamp_valid = sample_time.has_value();
        record.raw_timestamp = raw_timestamp;
        record.raw = {x, y, z};
        if (snapshot) {
            record.snapshot_ticks = snapshot->timestamp.time_since_epoch().count();
            const auto& q = snapshot->orientation;
            record.quaternion = {q.w(), q.x(), q.y(), q.z()};
        }
        record.diagnostics = diagnostics;
        if (!queue_.push_back(record))
            dropped_.fetch_add(1, std::memory_order_relaxed);
    }

    void update() {
        const auto records = queue_.readable_view();
        for (const auto& record : records) {
            const auto& diagnostics = record.diagnostics;
            const auto first = record.gyroscope ? kGyroX : kAccelX;
            for (std::size_t axis = 0; axis < 3; ++axis)
                values_[first + axis] = record.raw[axis];
            values_[record.gyroscope ? kGyroTime : kAccelTime] =
                record.timestamp_valid ? record.board_ticks * kBoardTickSeconds : kNaN;
            values_[record.gyroscope ? kGyroSequence : kAccelSequence] = record.sensor_sequence;
            if (record.snapshot_ticks >= 0) {
                for (std::size_t i = 0; i < 4; ++i)
                    values_[kQw + i] = record.quaternion[i];
                values_[kEkfTime] = record.snapshot_ticks * kBoardTickSeconds;
            }
            if (record.gyroscope) {
                values_[kSnapshotSequence] += record.snapshot_ticks >= 0;
                values_[kChiSquare] = diagnostics.chi_square;
                values_[kCorrectionTime] =
                    diagnostics.correction_time
                        ? std::chrono::duration<double>{diagnostics.correction_time
                                                            ->time_since_epoch()}
                              .count()
                        : kNaN;
                values_[kCorrectionAttempted] = diagnostics.correction_attempted;
                values_[kCorrectionApplied] = diagnostics.correction_applied;
                values_[kCorrectionAttempts] += diagnostics.correction_attempted;
                values_[kCorrections] += diagnostics.correction_applied;
                values_[kCorrectionRejections] +=
                    diagnostics.correction_attempted && !diagnostics.correction_applied;
                values_[kSaturations] += diagnostics.gyro_saturated;
                values_[kTimeGapRejections] += diagnostics.time_gap_rejected;
                values_[kSnapshotUpdated] = record.snapshot_ticks >= 0;
                values_[kTimestampValid] = record.timestamp_valid;
            }
            native_csv_ << record.sequence << ',' << (record.gyroscope ? "gyro" : "accel") << ','
                        << record.sensor_sequence << ',' << record.host_ns << ','
                        << record.board_ticks << ',' << record.timestamp_valid << ','
                        << record.raw_timestamp << ',' << record.raw[0] << ',' << record.raw[1]
                        << ',' << record.raw[2] << ',' << record.snapshot_ticks;
            for (const auto value : record.quaternion)
                native_csv_ << ',' << value;
            native_csv_ << ','
                        << (diagnostics.correction_time
                                ? diagnostics.correction_time->time_since_epoch().count()
                                : -1)
                        << ',' << diagnostics.chi_square << ',' << diagnostics.correction_attempted
                        << ',' << diagnostics.correction_applied << ','
                        << diagnostics.gyro_saturated << ',' << diagnostics.time_gap_rejected << ','
                        << (record.gyroscope && record.snapshot_ticks >= 0) << ','
                        << dropped_.load(std::memory_order_relaxed) << '\n';
            if (++native_rows_ % 1000 == 0)
                native_csv_.flush();
        }
        queue_.pop_front_n([](const Record&) noexcept {}, records.size());
        values_[kHostTime] =
            std::chrono::duration<double>{std::chrono::steady_clock::now().time_since_epoch()}
                .count();
        values_[kQueueDropped] = dropped_.load(std::memory_order_relaxed);
        values_[kIoFailed] = !native_csv_;
        for (std::size_t i = 0; i < kSignalCount; ++i)
            *outputs_[i] = values_[i];
    }

private:
    enum Signal {
        kHostTime,
        kAccelX,
        kAccelY,
        kAccelZ,
        kGyroX,
        kGyroY,
        kGyroZ,
        kAccelTime,
        kGyroTime,
        kQw,
        kQx,
        kQy,
        kQz,
        kEkfTime,
        kAccelSequence,
        kGyroSequence,
        kSnapshotSequence,
        kChiSquare,
        kCorrectionTime,
        kCorrectionAttempted,
        kCorrectionApplied,
        kCorrectionAttempts,
        kCorrections,
        kCorrectionRejections,
        kSaturations,
        kTimeGapRejections,
        kSnapshotUpdated,
        kTimestampValid,
        kQueueDropped,
        kIoFailed,
        kSignalCount
    };

    static constexpr std::array kSignalNames{
        "host_time_s",
        "raw/accel_x_counts",
        "raw/accel_y_counts",
        "raw/accel_z_counts",
        "raw/gyro_x_counts",
        "raw/gyro_y_counts",
        "raw/gyro_z_counts",
        "raw/accel_time_s",
        "raw/gyro_time_s",
        "ekf/qw",
        "ekf/qx",
        "ekf/qy",
        "ekf/qz",
        "ekf/time_s",
        "raw/accel_sequence",
        "raw/gyro_sequence",
        "ekf/sequence",
        "ekf/chi_square",
        "ekf/correction_time_s",
        "ekf/correction_attempted",
        "ekf/correction_applied",
        "ekf/correction_attempts",
        "ekf/corrections",
        "ekf/correction_rejections",
        "ekf/gyro_saturations",
        "ekf/time_gap_rejections",
        "ekf/snapshot_updated",
        "raw/gyro_timestamp_valid",
        "queue_dropped",
        "io_failed"};
    static_assert(kSignalNames.size() == kSignalCount);
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kBoardTickSeconds =
        std::chrono::duration<double>{rmcs_msgs::BoardClock::duration{1}}.count();

    struct Record {
        std::uint64_t sequence = 0, sensor_sequence = 0;
        std::int64_t host_ns = 0, board_ticks = -1, snapshot_ticks = -1;
        std::uint32_t raw_timestamp = 0;
        bool gyroscope = false, timestamp_valid = false;
        std::array<std::int16_t, 3> raw{};
        std::array<double, 4> quaternion{kNaN, kNaN, kNaN, kNaN};
        Bmi088Ekf::Diagnostics diagnostics;
    };

    std::array<rmcs_executor::Component::OutputInterface<double>, kSignalCount> outputs_;
    std::array<double, kSignalCount> values_;
    rmcs_utility::RingBuffer<Record> queue_{4096};
    std::atomic<std::uint64_t> dropped_{0};
    std::uint64_t sequence_ = 0, native_rows_ = 0;
    std::array<std::uint64_t, 2> sensor_sequences_{};
    std::ofstream native_csv_;
};

} // namespace rmcs_core::hardware::device
