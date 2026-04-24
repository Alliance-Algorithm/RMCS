#pragma once

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rmcs_utility {

class CsvWriter {
public:
    CsvWriter() = default;
    explicit CsvWriter(const std::filesystem::path& path) { open(path); }

    CsvWriter(const CsvWriter&) = delete;
    CsvWriter& operator=(const CsvWriter&) = delete;
    CsvWriter(CsvWriter&&) = delete;
    CsvWriter& operator=(CsvWriter&&) = delete;

    ~CsvWriter() { close(); }

    void open(const std::filesystem::path& path) {
        close();

        if (!path.parent_path().empty())
            std::filesystem::create_directories(path.parent_path());

        stream_.open(path, std::ios::out | std::ios::trunc);
        if (!stream_.is_open())
            throw std::runtime_error("Failed to open csv file: " + path.string());

        path_ = path;
    }

    [[nodiscard]] bool is_open() const { return stream_.is_open(); }

    const std::filesystem::path& path() const { return path_; }

    void flush() {
        if (stream_.is_open())
            stream_.flush();
    }

    void close() {
        if (stream_.is_open()) {
            stream_.flush();
            stream_.close();
        }
        path_.clear();
    }

    template <typename... Values>
    void write_row(const Values&... values) {
        ensure_open();

        bool first = true;
        (write_field(first, values), ...);
        stream_ << '\n';
    }

private:
    void ensure_open() const {
        if (!stream_.is_open())
            throw std::runtime_error("CsvWriter is not open");
    }

    template <typename T>
    void write_field(bool& first, const T& value) {
        if (!first)
            stream_ << ',';
        first = false;
        write_value(value);
    }

    void write_value(std::string_view value) {
        if (value.find_first_of(",\"\r\n") == std::string_view::npos) {
            stream_ << value;
            return;
        }

        stream_ << '"';
        for (const char ch : value) {
            if (ch == '"')
                stream_ << "\"\"";
            else
                stream_ << ch;
        }
        stream_ << '"';
    }

    void write_value(const std::string& value) { write_value(std::string_view{value}); }
    void write_value(const char* value) {
        write_value(value == nullptr ? std::string_view{} : std::string_view{value});
    }

    template <typename T>
    void write_value(const T& value) {
        stream_ << value;
    }

    std::ofstream stream_;
    std::filesystem::path path_;
};

} // namespace rmcs_utility