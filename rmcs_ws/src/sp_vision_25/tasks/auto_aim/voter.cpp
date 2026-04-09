#include "voter.hpp"

namespace auto_aim
{
Voter::Voter() : count_(COLORS.size() * ARMOR_NAMES.size() * ARMOR_TYPES.size(), 0) {}

void Voter::vote(const Color color, const ArmorName name, const ArmorType type)
{
  count_[index(color, name, type)] += 1;
}

std::size_t Voter::count(const Color color, const ArmorName name, const ArmorType type)
{
  return count_[index(color, name, type)];
}

std::size_t Voter::index(const Color color, const ArmorName name, const ArmorType type) const
{
  return color * (ARMOR_NAMES.size() * ARMOR_TYPES.size()) + name * ARMOR_TYPES.size() + type;
}
}