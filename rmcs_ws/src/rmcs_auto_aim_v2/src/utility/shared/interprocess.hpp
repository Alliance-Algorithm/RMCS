#pragma once
#include <atomic>
#include <concepts>
#include <fcntl.h>
#include <sys/mman.h>

namespace rmcs::shm {

namespace details {
    template <typename T>
    struct alignas(64) Context final {

        alignas(64) std::atomic<std::uint64_t> version;
        alignas(64) T data;
        std::byte padding[64 - sizeof(T) % 64];

        static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
    };
}

template <typename T>
struct Client {
    using Context = details::Context<T>;

    static constexpr auto context_len = sizeof(Context);
    static constexpr auto data_len    = sizeof(T);

    class Send final {
    public:
        ~Send() noexcept {
            if (context) {
                munmap(static_cast<void*>(context), context_len);
            }
            if (shm_fd != -1) {
                close(shm_fd);
            }
        }
        auto open(const char* id) noexcept -> bool {
            shm_fd = shm_open(id, O_CREAT | O_RDWR, 0666);
            if (shm_fd == -1) {
                return false;
            }
            if (ftruncate(shm_fd, context_len) == -1) {
                close(shm_fd);
                return false;
            }

            auto* shm_ptr =
                mmap(nullptr, context_len, PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
            if (shm_ptr == MAP_FAILED) {
                close(shm_fd);
                return false;
            }

            context = static_cast<Context*>(shm_ptr);
            return true;
        }
        auto opened() const noexcept { return context != nullptr; }
        auto send(const T& data) noexcept -> void {
            if (!context) return;
            context->version.fetch_add(1, std::memory_order::acq_rel);
            context->data = data;
            context->version.fetch_add(1, std::memory_order::acq_rel);
        }

        template <typename F>
        auto with_write(F&& fn) noexcept -> void
            requires std::invocable<F, T&>
        {
            if (!context) return;

            context->version.fetch_add(1, std::memory_order::acq_rel);
            fn(context->data);
            context->version.fetch_add(1, std::memory_order::acq_rel);
        }

    private:
        int shm_fd { -1 };
        Context* context { nullptr };
    };

    class Recv {
    public:
        ~Recv() noexcept {
            if (context) {
                munmap(static_cast<void*>(context), context_len);
            }
            if (shm_fd != -1) {
                close(shm_fd);
            }
        }

        auto open(const char* id) noexcept -> bool {
            shm_fd = shm_open(id, O_RDWR, 0666);
            if (shm_fd == -1) {
                return false;
            }

            auto* shm_ptr =
                mmap(nullptr, context_len, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
            if (shm_ptr == MAP_FAILED) {
                close(shm_fd);
                return false;
            }

            context = static_cast<Context*>(shm_ptr);
            return true;
        }
        auto opened() const noexcept { return context != nullptr; }
        auto recv(T& out_data) const noexcept -> void {
            if (!context) return;

            auto version1 = std::uint64_t {};
            auto version2 = std::uint64_t {};

            do {
                version1 = context->version.load(std::memory_order::acquire);
                out_data = context->data;
                version2 = context->version.load(std::memory_order::acquire);
            } while ((version1 != version2) || (version1 & 1));

            version = version2;
        }

        template <typename F>
        auto with_read(F&& fn) const noexcept -> void
            requires std::invocable<F, const T&>
        {
            if (!context) return;

            auto snapshot = T {};
            auto version1 = std::uint64_t {};
            auto version2 = std::uint64_t {};

            do {
                version1 = context->version.load(std::memory_order::acquire);
                snapshot = context->data;
                version2 = context->version.load(std::memory_order::acquire);
            } while ((version1 != version2) || (version1 & 1));

            version = version2;
            fn(snapshot);
        }

        auto is_updated() const noexcept -> bool {
            if (!context) return false;

            auto current = context->version.load(std::memory_order::acquire);
            return (current != version) && !(current & 1);
        }

    private:
        mutable std::uint64_t version { 0 };

        int shm_fd { -1 };
        Context* context { nullptr };
    };
};

}
