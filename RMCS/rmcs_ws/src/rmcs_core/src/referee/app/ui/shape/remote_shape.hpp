#pragma once

#include <cstdint>

#include "referee/app/ui/shape/red_black_tree.hpp"

namespace rmcs_core::referee::app::ui {
template <typename T>
class RemoteShape {
public:
    class Descriptor : private RedBlackTree<Descriptor>::Node {
    public:
        friend class RemoteShape;
        friend class RedBlackTree<Descriptor>;

        Descriptor()                             = default;
        Descriptor(const Descriptor&)            = delete;
        Descriptor& operator=(const Descriptor&) = delete;
        Descriptor(Descriptor&&)                 = delete;
        Descriptor& operator=(Descriptor&& obj)  = delete;

        [[nodiscard]] bool has_id() const { return id_; }
        [[nodiscard]] bool try_assign_id()
            requires std::is_base_of_v<Descriptor, T> && requires(T t) { t.id_revoked(); } {
            if (has_id()) [[unlikely]]
                return false;

            if (Descriptor* first = swapping_queue_.first()) {
                // Optimization: Try to find a descriptor to avoid creating a new one.
                swapping_queue_.erase(*first);
                swap_id(*first);
                return true;
            }

            if (next_id_ > id_assignment_max) [[unlikely]]
                return false;
            else {
                assign_id();
                return true;
            }
        }
        [[nodiscard]] bool predict_try_assign_id(uint8_t& existence_confidence) const {
            if (has_id()) [[unlikely]]
                return false;

            if (Descriptor* first = swapping_queue_.first()) {
                existence_confidence = first->existence_confidence_;
                return true;
            }

            return next_id_ <= id_assignment_max;
        }

        [[nodiscard]] bool swapping_enabled() const {
            return !RedBlackTree<Descriptor>::Node::is_dangling();
        }
        void enable_swapping() {
            if (swapping_enabled())
                return;
            swapping_queue_.insert(*this);
        }
        void disable_swapping() {
            if (!swapping_enabled())
                return;
            swapping_queue_.erase(*this);
        }

        [[nodiscard]] uint8_t id() const { return id_; }
        [[nodiscard]] uint8_t existence_confidence() const { return existence_confidence_; }

        uint8_t increase_existence_confidence() {
            ++existence_confidence_;
            if (swapping_enabled()) {
                disable_swapping(), enable_swapping();
            }
            return existence_confidence_;
        }

    private:
        /* Swap requirement: !this->id_ && victim.id_ */
        void swap_id(Descriptor& victim) {
            id_                     = victim.id_;
            assigned_list_[id_ - 1] = this;
            existence_confidence_   = victim.existence_confidence_;

            victim.revoke_id();
        }

        /* Assign requirement: next_id_ <= id_assignment_max */
        void assign_id() {
            id_ = next_id_++;

            assigned_list_[id_ - 1] = this;
        }

        void revoke_id() {
            id_                   = 0;
            existence_confidence_ = 0;

            static_cast<T*>(this)->id_revoked();
        }

        bool operator<(const Descriptor& obj) const {
            return existence_confidence_ < obj.existence_confidence_;
        }

        uint8_t id_                   = 0;
        uint8_t existence_confidence_ = 0;
    };

    static inline void force_revoke_all_id() {
        for (int i = 0; i < next_id_ - 1; ++i) {
            assigned_list_[i]->revoke_id();
        }
        next_id_ = 1;
    }

private:
    static constexpr uint8_t id_assignment_max = 201;

    static inline uint8_t next_id_ = 1;
    static inline Descriptor* assigned_list_[id_assignment_max];

    static inline RedBlackTree<Descriptor> swapping_queue_;
};
} // namespace rmcs_core::referee::app::ui