#ifndef AUTO_AIM_MULTITHREAD__HPP
#define AUTO_AIM_MULTITHREAD__HPP

#include <optional>

#include "io/cboard.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/plotter.hpp"

namespace auto_aim
{
namespace multithread
{

class CommandGener
{
public:
  CommandGener(
    auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
    tools::Plotter & plotter, bool debug = false);

  ~CommandGener();

  void push(
    const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
    double bullet_speed, const Eigen::Vector3d & gimbal_pos);

private:
  struct Input
  {
    std::list<auto_aim::Target> targets_;
    std::chrono::steady_clock::time_point t;
    // std::function<void()> decide;
    double bullet_speed;
    Eigen::Vector3d gimbal_pos;
  };

  io::CBoard & cboard_;
  auto_aim::Shooter & shooter_;
  auto_aim::Aimer & aimer_;
  tools::Plotter & plotter_;

  std::optional<Input> latest_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread thread_;
  bool stop_, debug_;

  void generate_command();
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM_MULTITHREAD__HPP