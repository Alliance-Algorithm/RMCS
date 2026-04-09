#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace
{
struct PlannerConfig
{
  double decision_speed;
  double max_yaw_acc;
  double max_pitch_acc;
};

struct Scenario
{
  std::string name;
  std::string note;
  bool use_optional_api;
  double distance;
  double angular_speed;
  double radius;
  double height;
  double bullet_speed;
  int nullopt_period = 0;
};

struct ScalarSummary
{
  double mean = 0.0;
  double p95 = 0.0;
  double max = 0.0;
};

struct ScenarioSummary
{
  std::string name;
  std::string note;
  std::size_t measured_calls = 0;
  std::size_t target_present_calls = 0;
  std::size_t control_true_with_target = 0;
  std::size_t fire_true_with_target = 0;
  std::size_t nullopt_calls = 0;
  ScalarSummary call_latency_us;
  ScalarSummary solve_latency_us;
  ScalarSummary midpoint_error_mrad;
  ScalarSummary yaw_acc_ratio;
  ScalarSummary pitch_acc_ratio;
  bool control_contract_ok = true;
  bool bound_contract_ok = true;
};

double ratio(std::size_t count, std::size_t total)
{
  if (total == 0) return 0.0;
  return 100.0 * static_cast<double>(count) / static_cast<double>(total);
}

ScalarSummary summarize(std::vector<double> values)
{
  if (values.empty()) return {};

  std::sort(values.begin(), values.end());

  double sum = 0.0;
  for (double value : values) sum += value;

  auto p95_index = static_cast<std::size_t>(std::ceil(0.95 * values.size())) - 1;
  p95_index = std::min(p95_index, values.size() - 1);

  return {sum / static_cast<double>(values.size()), values[p95_index], values.back()};
}

auto_aim::Target make_target(const Scenario & scenario)
{
  auto_aim::Target target(
    scenario.distance, scenario.angular_speed, scenario.radius, scenario.height);
  target.name = auto_aim::ArmorName::one;
  target.armor_type = auto_aim::ArmorType::small;
  target.priority = auto_aim::ArmorPriority::first;
  target.jumped = false;
  target.last_id = 0;
  return target;
}

ScenarioSummary run_scenario(
  auto_aim::Planner & planner, const PlannerConfig & config, const Scenario & scenario, int warmup,
  int steps, int repeat)
{
  ScenarioSummary summary;
  summary.name = scenario.name;
  summary.note = scenario.note;

  std::vector<double> call_latency_us;
  std::vector<double> solve_latency_us;
  std::vector<double> midpoint_error_mrad;
  std::vector<double> yaw_acc_ratio;
  std::vector<double> pitch_acc_ratio;

  const double yaw_tol = std::max(1e-3, config.max_yaw_acc * 1e-3);
  const double pitch_tol = std::max(1e-3, config.max_pitch_acc * 1e-3);

  for (int rep = 0; rep < repeat; rep++) {
    auto target = make_target(scenario);

    for (int step = 0; step < warmup + steps; step++) {
      target.predict(auto_aim::DT);

      const bool has_target =
        !(scenario.use_optional_api && scenario.nullopt_period > 0 &&
          (step + 1) % scenario.nullopt_period == 0);

      auto t0 = std::chrono::steady_clock::now();

      auto_aim::Plan plan;
      if (scenario.use_optional_api) {
        std::optional<auto_aim::Target> maybe_target =
          has_target ? std::optional<auto_aim::Target>(target) : std::nullopt;
        plan = planner.plan(maybe_target, scenario.bullet_speed);
      } else {
        plan = planner.plan(target, scenario.bullet_speed);
      }

      auto t1 = std::chrono::steady_clock::now();
      double latency_us =
        std::chrono::duration<double, std::micro>(t1 - t0).count();

      if (step < warmup) continue;

      summary.measured_calls++;
      call_latency_us.push_back(latency_us);

      if (scenario.use_optional_api && !has_target) {
        summary.nullopt_calls++;
        if (plan.control) summary.control_contract_ok = false;
        continue;
      }

      summary.target_present_calls++;
      solve_latency_us.push_back(latency_us);

      if (plan.control) {
        summary.control_true_with_target++;
      } else {
        summary.control_contract_ok = false;
      }

      if (plan.fire) summary.fire_true_with_target++;

      double yaw_error = tools::limit_rad(plan.target_yaw - plan.yaw);
      double pitch_error = plan.target_pitch - plan.pitch;
      midpoint_error_mrad.push_back(std::hypot(yaw_error, pitch_error) * 1e3);

      double yaw_ratio = std::abs(plan.yaw_acc) / config.max_yaw_acc;
      double pitch_ratio = std::abs(plan.pitch_acc) / config.max_pitch_acc;
      yaw_acc_ratio.push_back(yaw_ratio);
      pitch_acc_ratio.push_back(pitch_ratio);

      if (std::abs(plan.yaw_acc) > config.max_yaw_acc + yaw_tol) summary.bound_contract_ok = false;
      if (std::abs(plan.pitch_acc) > config.max_pitch_acc + pitch_tol)
        summary.bound_contract_ok = false;
    }
  }

  summary.call_latency_us = summarize(call_latency_us);
  summary.solve_latency_us = summarize(solve_latency_us);
  summary.midpoint_error_mrad = summarize(midpoint_error_mrad);
  summary.yaw_acc_ratio = summarize(yaw_acc_ratio);
  summary.pitch_acc_ratio = summarize(pitch_acc_ratio);
  return summary;
}

void print_summary(const ScenarioSummary & summary)
{
  fmt::print("[{}]\n", summary.name);
  fmt::print("  note: {}\n", summary.note);
  fmt::print(
    "  workload: measured={} target_present={} nullopt={} active_control={:.1f}% fire={:.1f}%\n",
    summary.measured_calls, summary.target_present_calls, summary.nullopt_calls,
    ratio(summary.control_true_with_target, summary.target_present_calls),
    ratio(summary.fire_true_with_target, summary.target_present_calls));
  fmt::print(
    "  latency_us(all): mean={:.2f} p95={:.2f} max={:.2f} throughput={:.0f} Hz\n",
    summary.call_latency_us.mean, summary.call_latency_us.p95, summary.call_latency_us.max,
    summary.call_latency_us.mean > 0.0 ? 1e6 / summary.call_latency_us.mean : 0.0);
  fmt::print(
    "  latency_us(active): mean={:.2f} p95={:.2f} max={:.2f}\n", summary.solve_latency_us.mean,
    summary.solve_latency_us.p95, summary.solve_latency_us.max);
  fmt::print(
    "  midpoint_error_mrad: mean={:.3f} p95={:.3f} max={:.3f}\n",
    summary.midpoint_error_mrad.mean, summary.midpoint_error_mrad.p95,
    summary.midpoint_error_mrad.max);
  fmt::print(
    "  yaw_acc_ratio: mean={:.3f} p95={:.3f} max={:.3f}\n", summary.yaw_acc_ratio.mean,
    summary.yaw_acc_ratio.p95, summary.yaw_acc_ratio.max);
  fmt::print(
    "  pitch_acc_ratio: mean={:.3f} p95={:.3f} max={:.3f}\n", summary.pitch_acc_ratio.mean,
    summary.pitch_acc_ratio.p95, summary.pitch_acc_ratio.max);
  fmt::print(
    "  contract: control={} bounds={}\n\n", summary.control_contract_ok ? "PASS" : "FAIL",
    summary.bound_contract_ok ? "PASS" : "FAIL");
}

}  // namespace

int main(int argc, char * argv[])
{
  const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明        }"
    "{warmup         | 100  | 每个场景的预热轮数        }"
    "{steps          | 800  | 每个场景每轮的测量轮数    }"
    "{repeat         | 5    | 每个场景重复次数          }"
    "{@config-path   |      | yaml配置文件路径          }";

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  int warmup = cli.get<int>("warmup");
  int steps = cli.get<int>("steps");
  int repeat = cli.get<int>("repeat");

  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  auto yaml = tools::load(config_path);
  PlannerConfig config{
    tools::read<double>(yaml, "decision_speed"),
    tools::read<double>(yaml, "max_yaw_acc"),
    tools::read<double>(yaml, "max_pitch_acc"),
  };

  auto_aim::Planner planner(config_path);

  const double slow_speed = std::max(1.0, config.decision_speed * 0.6);
  const double threshold_speed = std::max(config.decision_speed + 1.0, config.decision_speed * 1.2);
  const double aggressive_speed = std::max(config.decision_speed + 3.0, config.decision_speed * 1.8);

  std::vector<Scenario> scenarios{
    {
      "direct_low_speed",
      "直接走 Planner::plan(Target, bullet_speed)，只测 solver 主路径。",
      false,
      3.0,
      slow_speed,
      0.20,
      0.10,
      22.0,
    },
    {
      "optional_low_speed",
      "低于 decision_speed，覆盖 optional<Target> 的低速延迟分支。",
      true,
      3.0,
      slow_speed,
      0.20,
      0.10,
      22.0,
    },
    {
      "optional_high_speed",
      "高于 decision_speed，覆盖 docs 里的高速延迟补偿路径。",
      true,
      3.0,
      threshold_speed,
      0.22,
      0.12,
      22.0,
    },
    {
      "optional_aggressive_close_range",
      "近距离 + 高频旋转，观察加速度约束接近饱和时的耗时与误差。",
      true,
      2.0,
      aggressive_speed,
      0.28,
      0.15,
      22.0,
    },
    {
      "optional_invalid_bullet_speed",
      "弹速传 0，验证 planner 的公共 fallback 路径仍可稳定输出。",
      true,
      3.0,
      slow_speed,
      0.20,
      0.10,
      0.0,
    },
    {
      "optional_intermittent_target_loss",
      "每 8 帧喂一次 nullopt，评估无目标快路径和恢复后的 planner 开销。",
      true,
      3.0,
      threshold_speed,
      0.20,
      0.10,
      22.0,
      8,
    },
  };

  fmt::print(
    "planner benchmark\n  config={}\n  warmup={} steps={} repeat={}\n  decision_speed={:.2f}"
    " max_yaw_acc={:.2f} max_pitch_acc={:.2f}\n\n",
    config_path, warmup, steps, repeat, config.decision_speed, config.max_yaw_acc,
    config.max_pitch_acc);

  bool overall_ok = true;
  for (const auto & scenario : scenarios) {
    auto summary = run_scenario(planner, config, scenario, warmup, steps, repeat);
    print_summary(summary);
    overall_ok = overall_ok && summary.control_contract_ok && summary.bound_contract_ok;
  }

  fmt::print("overall contract: {}\n", overall_ok ? "PASS" : "FAIL");
  return overall_ok ? 0 : 1;
}
