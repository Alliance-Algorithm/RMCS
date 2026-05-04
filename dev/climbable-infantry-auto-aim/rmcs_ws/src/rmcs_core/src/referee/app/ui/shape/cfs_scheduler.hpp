#pragma once

#include <cstdint>

#include <type_traits>

#include "referee/app/ui/shape/red_black_tree.hpp"

namespace rmcs_core::referee::app::ui {

template <typename T>
class CfsScheduler {
public:
    class __attribute__((packed, aligned(sizeof(void*)))) Entity
        : private RedBlackTree<Entity>::Node {
    public:
        friend class CfsScheduler;
        friend class RedBlackTree<Entity>;

        bool is_in_run_queue() requires(std::is_base_of_v<Entity, T>) {
            return !RedBlackTree<Entity>::Node::is_dangling();
        }

        void enter_run_queue(uint16_t priority) requires(std::is_base_of_v<Entity, T>) {
            if (this->priority_ != priority) {
                vruntime_ += this->priority_;
                vruntime_ -= priority;
                if (this->vruntime_ < min_vruntime_)
                    this->vruntime_ = min_vruntime_;

                this->priority_ = priority;

                if (is_in_run_queue())
                    run_queue_.erase(*this);
            } else {
                if (is_in_run_queue())
                    return;
            }

            run_queue_.insert(*this);
        }

        void leave_run_queue() requires(std::is_base_of_v<Entity, T>) {
            if (is_in_run_queue()) [[likely]]
                run_queue_.erase(*this);
        }

    private:
        bool operator<(const Entity& obj) const { return vruntime_ < obj.vruntime_; }
        uint64_t vruntime_ : 48 = 65536;
        uint16_t priority_      = 0;
    };

    // class T : public Entity {};

    class UpdateIterator {
    public:
        UpdateIterator()
            : current_(run_queue_.first())
            , ignored_(nullptr) {}
        UpdateIterator(const UpdateIterator&)            = delete;
        UpdateIterator& operator=(const UpdateIterator&) = delete;
        UpdateIterator(UpdateIterator&&)                 = default;
        UpdateIterator& operator=(UpdateIterator&&)      = default;

        T* get() const {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
            return static_cast<T*>(current_);
        }

        T& operator*() const { return *get(); }
        T* operator->() const { return get(); }
        explicit operator bool() const { return get(); }

        auto update() {
            min_vruntime_ = current_->vruntime_;
            int shift     = 65536 - current_->priority_;
            current_->vruntime_ += shift;

            run_queue_.erase(*current_);
            auto result = get()->update();
            current_    = ignored_ ? ignored_->next() : run_queue_.first();

            return result;
        }

        void ignore() {
            ignored_ = current_;
            current_ = ignored_->next();
        }

    private:
        Entity *current_, *ignored_;
    };

    static inline bool empty() { return run_queue_.empty(); }

    static inline UpdateIterator get_update_iterator()
        requires std::is_base_of_v<Entity, T> && requires(T t) { t.update(); } {
        return UpdateIterator{};
    }

private:
    static inline RedBlackTree<Entity> run_queue_;
    static inline uint64_t min_vruntime_ = 0;
};

} // namespace rmcs_core::referee::app::ui