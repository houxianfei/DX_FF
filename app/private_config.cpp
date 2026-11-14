#include "private_config.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
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
std::string strField(const std::string &s, const char *k,
                     const std::string &d = {}) {
  std::regex r("\\\"" + std::string(k) + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch m;
  return std::regex_search(s, m, r) ? m[1].str() : d;
}
long long intField(const std::string &s, const char *k, long long d = 0) {
  std::regex r("\\\"" + std::string(k) + "\\\"\\s*:\\s*(-?[0-9]+)");
  std::smatch m;
  return std::regex_search(s, m, r) ? std::stoll(m[1].str()) : d;
}
double dblField(const std::string &s, const char *k, double d = 0) {
  std::regex r("\\\"" + std::string(k) +
               "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
  std::smatch m;
  return std::regex_search(s, m, r) ? std::stod(m[1].str()) : d;
}
bool boolField(const std::string &s, const char *k, bool d = false) {
  std::regex r("\\\"" + std::string(k) + "\\\"\\s*:\\s*(true|false|1|0)",
               std::regex::icase);
  std::smatch match;
  if (!std::regex_search(s, match, r)) return d;
  const auto value = match[1].str();
  return value == "true" || value == "TRUE" || value == "1";
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
                                   "Layer member references an unknown Rod");
    if (!seen.insert({member.layer, int(member.rod)}).second)
      return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                   "Duplicate Rod membership in a layer");
    const bool hasLeft = member.leftNeighbor >= 0;
    const bool hasRight = member.rightNeighbor >= 0;
    if (member.role < 0 || member.role > 3 ||
        (member.role == 0 && (!hasLeft || !hasRight)) ||
        (member.role == 1 && (hasLeft || !hasRight)) ||
        (member.role == 2 && (!hasLeft || hasRight)) ||
        (member.role == 3 && (hasLeft || hasRight)))
      return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                   "Layer role and neighbors are inconsistent");
    if (hasLeft) {
      const auto *left =
          findLayerMember(channel, member.layer, member.leftNeighbor);
      if (!left || left->rightNeighbor != int(member.rod))
        return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                     "Layer left neighbor is not reciprocal");
    }
    if (hasRight) {
      const auto *right =
          findLayerMember(channel, member.layer, member.rightNeighbor);
      if (!right || right->leftNeighbor != int(member.rod))
        return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                     "Layer right neighbor is not reciprocal");
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
  if (config.machineId.empty() || config.configVersion.empty())
    return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                 "Machine ID and configuration version are required");
  std::size_t rodCount = 0;
  for (int i = 0; i < 3; ++i) {
    const auto &channel = config.channels[i];
    if (static_cast<int>(channel.id) != i)
      return Result<void>::failure(ErrorCode::MissingChannel,
                                   "Configuration must contain CH1, CH2 and CH3");
    std::unordered_set<int> ids;
    for (const auto &rod : channel.rods) {
      ++rodCount;
      if (!ids.insert(static_cast<int>(rod.id)).second)
        return Result<void>::failure(ErrorCode::DuplicateId, "Duplicate Rod ID");
      if (rod.hardMin > rod.hardMax || rod.softMin < rod.hardMin ||
          rod.softMax > rod.hardMax || rod.softMin > rod.softMax ||
          (rod.safePositionValid && (rod.safePosition < rod.softMin ||
                                     rod.safePosition > rod.softMax)) ||
          std::abs(rod.p1) < 1e-12 || rod.maxVelocity < 0 ||
          rod.maxAcceleration < 0 || rod.maxDeceleration < 0 ||
          rod.positionTolerance < 0 || rod.stableTime.count() < 0 ||
          rod.moveTimeout.count() < 0)
        return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                     "Invalid Rod limits, calibration, or motion parameter");
      for (const auto &range : rod.absoluteSafeRanges)
        if (range.first > range.second || range.first < rod.hardMin || range.second > rod.hardMax)
          return Result<void>::failure(ErrorCode::InvalidConfiguration, "Invalid absolute safe range");
      for (const auto &range : rod.forbiddenRanges)
        if (range.first > range.second || range.first < rod.hardMin || range.second > rod.hardMax)
          return Result<void>::failure(ErrorCode::InvalidConfiguration, "Invalid forbidden range");
    }
    if (channel.trainingRod) {
      if (!findRodConfig(channel, *channel.trainingRod))
        return Result<void>::failure(
            ErrorCode::InvalidConfiguration,
            "Training Rod references an unknown Rod");
      const bool isLayer2 = std::any_of(
          channel.layerMembers.begin(), channel.layerMembers.end(),
          [&](const LayerMembership &member) {
            return member.layer == 2 && member.rod == *channel.trainingRod;
          });
      if (!isLayer2)
        return Result<void>::failure(
            ErrorCode::InvalidConfiguration,
            "Training Rod must be a Layer 2 Rod");
    }
    if (auto topology = validateLayerTopology(channel); !topology)
      return topology;
    std::set<std::tuple<int, int, int>> rules;
    for (const auto &rule : channel.clearanceRules) {
      if (!findRodConfig(channel, rule.first) || !findRodConfig(channel, rule.second) ||
          rule.first == rule.second || !std::isfinite(rule.minimumClearanceMm) ||
          !rules.insert({rule.layer, std::min(int(rule.first), int(rule.second)),
                         std::max(int(rule.first), int(rule.second))}).second)
        return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                     "Invalid or duplicate clearance rule");
    }
  }
  if (config.maxActiveRodCount == 0 || config.maxActiveRodCount < rodCount ||
      config.statusExpiration.count() < 0)
    return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                 "Invalid system limits");
  return Result<void>::success();
}

Result<FFConfig> FFConfigIO::load(const std::string &fileName) {
  auto text = readFile(fileName);
  if (text.empty())
    return Result<FFConfig>::failure(ErrorCode::InvalidConfiguration,
                                     "Unable to read configuration file");
  std::replace(text.begin(), text.end(), '\n', ' ');
  std::replace(text.begin(), text.end(), '\r', ' ');
  FFConfig c;
  c.machineId = strField(text, "machine_id", "FF-001");
  c.configVersion = strField(text, "config_version", "FF-CONFIG-1.0");
  c.maxActiveRodCount =
      static_cast<size_t>(intField(text, "max_active_rod_count", 8));
  c.statusExpiration =
      std::chrono::milliseconds(intField(text, "status_expiration_ms", 200));
  std::regex cr("\\{\\s*\\\"id\\\"\\s*:\\s*\\\"(CH[123])\\\"([\\s\\S]*?)"
                "\\\"rods\\\"\\s*:\\s*\\[([\\s\\S]*?)\\]",
                std::regex::icase);
  std::sregex_iterator it(text.begin(), text.end(), cr), end;
  std::set<std::string> seen;
  for (; it != end; ++it) {
    auto block = (*it)[0].str();
    auto cid = (*it)[1].str();
    int idx = cid == "CH1" ? 0 : cid == "CH2" ? 1 : 2;
    c.channels[idx].id = static_cast<ChannelId>(idx);
    c.channels[idx].controller.name = strField(block, "name", "CTRL-" + cid);
    c.channels[idx].controller.serial.portName = strField(block, "port");
    c.channels[idx].controller.serial.baudRate =
        static_cast<uint32_t>(intField(block, "baud_rate", 115200));
    c.channels[idx].controller.serial.timeout =
        std::chrono::milliseconds(intField(block, "timeout_ms", 500));
    c.channels[idx].controller.pollPeriod =
        std::chrono::milliseconds(intField(block, "poll_period_ms", 20));
    c.channels[idx].controller.retryCount =
        static_cast<uint32_t>(intField(block, "retry_count", 3));
    const int configuredTrainingRod =
        static_cast<int>(intField(block, "training_rod_id", -1));
    if (configuredTrainingRod >= 0 && configuredTrainingRod <= 255)
      c.channels[idx].trainingRod =
          static_cast<RodId>(configuredTrainingRod);
    // layer_members is intentionally written before rods so it is included in
    // this channel block. A Rod may appear once for each ordinary layer.
    std::regex layerArray("\\\"layer_members\\\"\\s*:\\s*\\[([\\s\\S]*?)\\]");
    std::smatch layerMatch;
    if (std::regex_search(block, layerMatch, layerArray)) {
      std::regex member("\\{(.*?)\\}");
      const auto layerText = layerMatch[1].str();
      for (std::sregex_iterator li(layerText.begin(), layerText.end(), member);
           li != end; ++li) {
        const auto item = (*li)[1].str();
        LayerMembership membership;
        membership.rod = static_cast<RodId>(intField(item, "rod_id"));
        membership.layer = static_cast<int>(intField(item, "layer"));
        membership.role = static_cast<int>(intField(item, "role"));
        membership.leftNeighbor =
            static_cast<int>(intField(item, "left_neighbor", -1));
        membership.rightNeighbor =
            static_cast<int>(intField(item, "right_neighbor", -1));
        c.channels[idx].layerMembers.push_back(membership);
      }
    }
    std::regex clearanceArray(
        "\\\"clearance_rules\\\"\\s*:\\s*\\[([\\s\\S]*?)\\]");
    std::smatch clearanceMatch;
    if (std::regex_search(block, clearanceMatch, clearanceArray)) {
      std::regex rule("\\{(.*?)\\}");
      const auto clearanceText = clearanceMatch[1].str();
      for (std::sregex_iterator ci(clearanceText.begin(), clearanceText.end(),
                                   rule);
           ci != end; ++ci) {
        const auto item = (*ci)[1].str();
        PairClearanceRule clearance;
        clearance.layer = static_cast<int>(intField(item, "layer"));
        clearance.first = static_cast<RodId>(intField(item, "rod_a"));
        clearance.second = static_cast<RodId>(intField(item, "rod_b"));
        clearance.minimumClearanceMm = dblField(item, "minimum_clearance_mm");
        c.channels[idx].clearanceRules.push_back(clearance);
      }
    }
    std::string rods = (*it)[3].str();
    std::regex rr("\\{(.*?)\\}");
    for (std::sregex_iterator ri(rods.begin(), rods.end(), rr); ri != end;
         ++ri) {
      auto rs = (*ri)[1].str();
      RodConfig r;
      r.id = static_cast<RodId>(intField(rs, "id"));
      r.name = strField(rs, "name", cid + "-R" + std::to_string(r.id));
      r.hardMin = intField(rs, "hard_min");
      r.hardMax = intField(rs, "hard_max", 4095);
      r.softMin = intField(rs, "soft_min", r.hardMin);
      r.softMax = intField(rs, "soft_max", r.hardMax);
      r.zeroPosition = intField(rs, "zero_position");
      r.safePosition = intField(rs, "safe_position");
      r.safePositionValid = boolField(rs, "safe_position_valid");
      // optical_position = P0 + P1*u.
      // Legacy linear fields remain readable as P0/P1.
      r.p0 = dblField(rs, "P0", dblField(rs, "stroke_bias"));
      r.p1 = dblField(rs, "P1", dblField(rs, "stroke_slope", 1));
      r.inwardDirection = intField(rs, "inward_direction", 1);
      r.maxVelocity = dblField(rs, "max_velocity", 300);
      r.maxAcceleration = dblField(rs, "max_acceleration", 100);
      r.maxDeceleration = dblField(rs, "max_deceleration", 100);
      r.positionTolerance = intField(rs, "position_tolerance", 5);
      r.stableTime =
          std::chrono::milliseconds(intField(rs, "stable_time_ms", 100));
      r.moveTimeout =
          std::chrono::milliseconds(intField(rs, "move_timeout_ms", 5000));
      c.channels[idx].rods.push_back(r);
    }
    seen.insert(cid);
  }
  if (seen.size() != 3 || seen.count("CH1") == 0 || seen.count("CH2") == 0 ||
      seen.count("CH3") == 0)
    return Result<FFConfig>::failure(
        ErrorCode::MissingChannel,
        "Configuration must contain CH1, CH2 and CH3");
  auto valid = validate(c);
  if (!valid)
    return Result<FFConfig>::failure(valid.error, valid.message);
  return Result<FFConfig>::success(std::move(c));
}

Result<void> FFConfigIO::save(const std::string &fileName, const FFConfig &c) {
  auto valid = validate(c);
  if (!valid)
    return valid;
  std::ofstream f(fileName);
  if (!f)
    return Result<void>::failure(ErrorCode::BackendFailure,
                                 "Unable to write configuration");
  f << "{\n  \"schema\": \"ff-config\",\n  \"schema_version\": 2,\n  "
       "\"machine_id\": \""
    << c.machineId << "\",\n  \"config_version\": \"" << c.configVersion
    << "\",\n  \"max_active_rod_count\": " << c.maxActiveRodCount
    << ",\n  \"status_expiration_ms\": " << c.statusExpiration.count()
    << ",\n  \"channels\": [\n";
  for (int i = 0; i < 3; i++) {
    const auto &ch = c.channels[i];
    f << "    {\n"
      << "      \"id\": \"" << channelName(ch.id) << "\",\n"
      << "      \"controller\": {\n"
      << "        \"name\": \"" << ch.controller.name << "\",\n"
      << "        \"serial\": {\n"
      << "          \"port\": \"" << ch.controller.serial.portName << "\",\n"
      << "          \"baud_rate\": " << ch.controller.serial.baudRate << ",\n"
      << "          \"timeout_ms\": " << ch.controller.serial.timeout.count()
      << "\n        },\n"
      << "        \"poll_period_ms\": " << ch.controller.pollPeriod.count()
      << ",\n"
      << "        \"retry_count\": " << ch.controller.retryCount
      << "\n      },\n"
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
      f << "        {\n"
        << "          \"id\": " << int(r.id) << ",\n"
        << "          \"name\": \"" << r.name << "\",\n"
        << "          \"hard_min\": " << r.hardMin << ",\n"
        << "          \"hard_max\": " << r.hardMax << ",\n"
        << "          \"soft_min\": " << r.softMin << ",\n"
        << "          \"soft_max\": " << r.softMax << ",\n"
        << "          \"zero_position\": " << r.zeroPosition << ",\n"
        << "          \"safe_position\": " << r.safePosition << ",\n"
        << "          \"safe_position_valid\": "
        << (r.safePositionValid ? "true" : "false") << ",\n"
        << "          \"P0\": " << r.p0 << ",\n"
        << "          \"P1\": " << r.p1 << ",\n"
        << "          \"inward_direction\": " << r.inwardDirection << ",\n"
        << "          \"max_velocity\": " << r.maxVelocity << ",\n"
        << "          \"max_acceleration\": " << r.maxAcceleration << ",\n"
        << "          \"max_deceleration\": " << r.maxDeceleration << ",\n"
        << "          \"position_tolerance\": " << r.positionTolerance << ",\n"
        << "          \"stable_time_ms\": " << r.stableTime.count() << ",\n"
        << "          \"move_timeout_ms\": " << r.moveTimeout.count() << "\n"
        << "        }" << (j + 1 == ch.rods.size() ? "\n" : ",\n");
    }
    f << "      ]\n    }" << (i == 2 ? "\n" : ",\n");
  }
  f << "  ]\n}\n";
  return f ? Result<void>::success()
           : Result<void>::failure(ErrorCode::BackendFailure,
                                   "Configuration write failed");
}
} // namespace ff
