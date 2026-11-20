#include "private_config.hpp"
#include "../include/ff/ConfigJson.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <tuple>
#include <unordered_set>

namespace ff {
namespace {
std::string readFile(const std::string &p) {
  std::ifstream f(p);
  return {std::istreambuf_iterator<char>(f), {}};
}
std::string channelName(ChannelId id) {
  return std::string("CH") + char('1' + static_cast<int>(id));
}

double opticalPosition(const RodConfig &config, double u) {
  return config.p0 + config.p1 * u;
}

bool hasMonotonicOpticalPosition(const RodConfig &config) {
  double direction = 0.0;
  for (int sample = 0; sample <= 32; ++sample) {
    const double u =
        config.softMin + (config.softMax - config.softMin) * sample / 32.0;
    const double derivative = config.p1;
    if (std::abs(derivative) < 1e-12)
      return false;
    if (direction == 0.0)
      direction = derivative;
    else if (direction * derivative < 0.0)
      return false;
  }
  return true;
}

const RodConfig *findRodConfig(const FFChannelConfig &channel, RodId id) {
  const auto it = std::find_if(channel.rods.begin(), channel.rods.end(),
                               [id](const auto &rod) { return rod.id == id; });
  return it == channel.rods.end() ? nullptr : &*it;
}

const LayerMembership *findLayerMember(const FFChannelConfig &channel,
                                       int layer, int rod) {
  const auto it =
      std::find_if(channel.layerMembers.begin(), channel.layerMembers.end(),
                   [layer, rod](const auto &member) {
                     return member.layer == layer && int(member.rod) == rod;
                   });
  return it == channel.layerMembers.end() ? nullptr : &*it;
}

Result<void> validateLayerTopology(const FFChannelConfig &channel) {
  std::set<std::pair<int, int>> seen;
  for (const auto &member : channel.layerMembers) {
    if (!findRodConfig(channel, member.rod))
      return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                   "Rod "+std::to_string(member.rod)+" layer_members.rod_id references an unknown Rod");
    if (!seen.insert({member.layer, int(member.rod)}).second)
      return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                   "Rod "+std::to_string(member.rod)+" layer_members is duplicated");
    const bool hasLeft = member.leftNeighbor >= 0;
    const bool hasRight = member.rightNeighbor >= 0;
    if (member.role < 0 || member.role > 3 ||
        (member.role == 0 && (!hasLeft || !hasRight)) ||
        (member.role == 1 && (hasLeft || !hasRight)) ||
        (member.role == 2 && (!hasLeft || hasRight)) ||
        (member.role == 3 && (hasLeft || hasRight)))
      return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                   "Rod "+std::to_string(member.rod)+" layer_members.role/neighbor is inconsistent");
    if (hasLeft) {
      const auto *left =
          findLayerMember(channel, member.layer, member.leftNeighbor);
      if (!left || left->rightNeighbor != int(member.rod))
        return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                     "Rod "+std::to_string(member.rod)+" layer_members.left_neighbor is not reciprocal");
    }
    if (hasRight) {
      const auto *right =
          findLayerMember(channel, member.layer, member.rightNeighbor);
      if (!right || right->leftNeighbor != int(member.rod))
        return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                     "Rod "+std::to_string(member.rod)+" layer_members.right_neighbor is not reciprocal");
    }
  }
  return Result<void>::success();
}

double clearanceFor(const FFChannelConfig &channel, int layer, RodId first,
                    RodId second, bool &found) {
  for (const auto &rule : channel.clearanceRules) {
    if (rule.layer == layer &&
        ((rule.first == first && rule.second == second) ||
         (rule.first == second && rule.second == first))) {
      found = true;
      return rule.minimumClearanceMm;
    }
  }
  found = false;
  return 0.0;
}

Result<void>
validateLayerClearance(const FFChannelConfig &channel,
                       const ChannelStatus &status,
                       const std::unordered_map<RodId, int32_t> &overrides,
                       const std::unordered_set<RodId> *checkedRods = nullptr) {
  const auto topology = validateLayerTopology(channel);
  if (!topology)
    return topology;
  auto positionFor = [&](RodId id, int32_t &position) {
    if (const auto it = overrides.find(id); it != overrides.end()) {
      position = it->second;
      return true;
    }
    const auto configIt =
        std::find_if(channel.rods.begin(), channel.rods.end(),
                     [id](const auto &rod) { return rod.id == id; });
    if (configIt == channel.rods.end())
      return false;
    const auto index = static_cast<size_t>(configIt - channel.rods.begin());
    if (index >= status.rods.size() || !status.rods[index].positionValid)
      return false;
    position = status.rods[index].actualPosition;
    return true;
  };
  for (const auto &member : channel.layerMembers) {
    if (member.rightNeighbor < 0)
      continue;
    const auto first = member.rod;
    const auto second = static_cast<RodId>(member.rightNeighbor);
    // A partial command must not be rejected by position noise from an
    // unrelated stationary pair.  The commanded Rod still checks both of its
    // direct neighbors, so this never relaxes its own collision boundary.
    if (checkedRods && !checkedRods->count(first) &&
        !checkedRods->count(second))
      continue;
    bool hasClearanceRule = false;
    const auto clearance =
        clearanceFor(channel, member.layer, first, second, hasClearanceRule);
    if (!hasClearanceRule)
      return Result<void>::failure(
          ErrorCode::InvalidConfiguration,
          "Every neighboring Rod pair requires a clearance rule");
    int32_t firstPosition{}, secondPosition{};
    if (!positionFor(first, firstPosition) ||
        !positionFor(second, secondPosition))
      return Result<void>::failure(ErrorCode::SafetyViolation,
                                   "Layer position feedback is unavailable");
    const auto *firstConfig = findRodConfig(channel, first);
    const auto *secondConfig = findRodConfig(channel, second);
    const double firstMm = opticalPosition(*firstConfig, firstPosition);
    const double secondMm = opticalPosition(*secondConfig, secondPosition);
    // member.rightNeighbor is ordered topology: the right Rod must remain to
    // the right of this (left) Rod by at least the configured separation.
    // Do not use abs() here: it would discard that order and would make a
    // negative separation rule meaningless.
    if ((secondMm - firstMm) + kLayerClearanceToleranceMm < clearance)
      return Result<void>::failure(ErrorCode::SafetyViolation,
                                   "Layer neighbor order or clearance is violated");
  }
  return Result<void>::success();
}
} // namespace

Result<void> FFConfigIO::validate(const FFConfig &config) {
  return config_json::validate(config,findRodConfig);
}

Result<FFConfig> FFConfigIO::load(const std::string &fileName) {
  auto text = readFile(fileName);
  if (text.empty())
    return Result<FFConfig>::failure(ErrorCode::InvalidConfiguration,
                                     "Unable to read configuration file");
  try {
    auto config=config_json::decode(text);
    auto valid=validate(config);
    if(!valid) return Result<FFConfig>::failure(valid.error,valid.message);
    return Result<FFConfig>::success(std::move(config));
  } catch(const std::exception& error) {
    return Result<FFConfig>::failure(ErrorCode::InvalidConfiguration,error.what());
  }

}

Result<void> FFConfigIO::save(const std::string &fileName, const FFConfig &c) {
  auto valid = validate(c);
  if (!valid)
    return valid;
  const RodConfig *shared = nullptr;
  bool sharedMotion = true;
  for (const auto &channel : c.channels) {
    for (const auto &rod : channel.rods) {
      if (!shared) { shared = &rod; continue; }
      if (rod.maxVelocity != shared->maxVelocity ||
          rod.maxAcceleration != shared->maxAcceleration ||
          rod.maxDeceleration != shared->maxDeceleration ||
          rod.positionTolerance != shared->positionTolerance ||
          rod.stableTime != shared->stableTime ||
          rod.moveTimeout != shared->moveTimeout)
        sharedMotion = false;
    }
  }
  sharedMotion = sharedMotion && shared;
  std::ofstream f(fileName);
  if (!f)
    return Result<void>::failure(ErrorCode::BackendFailure,
                                 "Unable to write configuration");
  f << "{\n  \"schema\": \"ff-config\",\n  \"schema_version\": 2,\n  "
       "\"machine_id\": \""
    << config_json::escape(c.machineId) << "\",\n  \"config_version\": \"" << config_json::escape(c.configVersion)
    << "\",\n  \"max_active_rod_count\": " << c.maxActiveRodCount
    << ",\n  \"status_expiration_ms\": " << c.statusExpiration.count()
    << ",\n  \"coordinate_units\": {\"encoder_position\": \"count\", \"optical_position\": \"mm\", \"motor_speed\": \"raw_0_1023\", \"motor_acceleration\": \"raw_0_255\"},\n";
  if (sharedMotion) {
    f << "  \"motor_parameters\": {\"speed_limit_raw\": " << shared->maxVelocity
      << ", \"acceleration_raw\": " << shared->maxAcceleration
      << ", \"deceleration_raw\": " << shared->maxDeceleration << "},\n";
    f << "  \"rod_motion\": {\"coordinate_system\": \"encoder_counts\", \"position_tolerance\": "
      << shared->positionTolerance << ", \"stable_time_ms\": " << shared->stableTime.count()
      << ", \"move_timeout_ms\": " << shared->moveTimeout.count() << "},\n";
  }
  f << "  \"channels\": [\n";
  bool firstChannel=true;
  for (int i = 0; i < 3; i++) {
    const auto &ch = c.channels[i];
    if (ch.rods.empty()) continue;
    if (!firstChannel) f << ",\n";
    firstChannel=false;
    f << "    {\n"
      << "      \"id\": \"" << channelName(ch.id) << "\",\n"
      << "      \"controller\": {\"name\": \"" << config_json::escape(ch.controller.name)
      << "\", \"serial\": {\"port\": \"" << config_json::escape(ch.controller.serial.portName)
      << "\", \"baud_rate\": " << ch.controller.serial.baudRate
      << ", \"timeout_ms\": " << ch.controller.serial.timeout.count()
      << "}, \"poll_period_ms\": " << ch.controller.pollPeriod.count()
      << ", \"retry_count\": " << ch.controller.retryCount << "},\n"
      << "      \"training_rod_id\": "
      << (ch.trainingRod ? std::to_string(int(*ch.trainingRod))
                         : std::string("null"))
      << ",\n"
      << "      \"layer_members\": [\n";
    for (size_t j = 0; j < ch.layerMembers.size(); ++j) {
      const auto &member = ch.layerMembers[j];
      f << "        {\"rod_id\": " << int(member.rod)
        << ", \"layer\": " << member.layer << ", \"role\": " << member.role
        << ", \"left_neighbor\": " << member.leftNeighbor
        << ", \"right_neighbor\": " << member.rightNeighbor << "}"
        << (j + 1 == ch.layerMembers.size() ? "\n" : ",\n");
    }
    f << "      ],\n"
      << "      \"clearance_rules\": [\n";
    for (size_t j = 0; j < ch.clearanceRules.size(); ++j) {
      const auto &rule = ch.clearanceRules[j];
      f << "        {\"layer\": " << rule.layer
        << ", \"rod_a\": " << int(rule.first)
        << ", \"rod_b\": " << int(rule.second)
        << ", \"minimum_clearance_mm\": " << rule.minimumClearanceMm << "}"
        << (j + 1 == ch.clearanceRules.size() ? "\n" : ",\n");
    }
    f << "      ],\n"
      << "      \"rods\": [\n";
    for (size_t j = 0; j < ch.rods.size(); j++) {
      auto &r = ch.rods[j];
      f << "        {\"id\": " << int(r.id)
        << ", \"name\": \"" << config_json::escape(r.name) << "\""
        << ", \"hard_min\": " << r.hardMin
        << ", \"hard_max\": " << r.hardMax
        << ", \"soft_min\": " << r.softMin
        << ", \"soft_max\": " << r.softMax
        << ", \"zero_position\": " << r.zeroPosition
        << ", \"safe_position\": " << r.safePosition
        << ", \"safe_position_valid\": "
        << (r.safePositionValid ? "true" : "false")
        << ", \"P0\": " << r.p0
        << ", \"P1\": " << r.p1
        << ", \"inward_direction\": " << r.inwardDirection;
      if (!sharedMotion)
        f << ", \"max_velocity\": " << r.maxVelocity
        << ", \"max_acceleration\": " << r.maxAcceleration
        << ", \"max_deceleration\": " << r.maxDeceleration
        << ", \"position_tolerance\": " << r.positionTolerance
        << ", \"stable_time_ms\": " << r.stableTime.count()
        << ", \"move_timeout_ms\": " << r.moveTimeout.count();
      f << "}" << (j + 1 == ch.rods.size() ? "\n" : ",\n");
    }
    f << "      ]\n    }";
  }
  f << "\n  ]\n}\n";
  return f ? Result<void>::success()
           : Result<void>::failure(ErrorCode::BackendFailure,
                                   "Configuration write failed");
}
} // namespace ff
