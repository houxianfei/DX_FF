#include "ff/FF.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <tuple>
#include <unordered_set>
#include "Training.cpp"

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
  return intField(s, k, d ? 1 : 0) != 0 || (strField(s, k) == "true");
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
                       const std::unordered_map<RodId, int32_t> &overrides) {
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
    if (secondMm - firstMm < clearance)
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
  std::unordered_set<std::string> ports;
  std::size_t rodCount = 0;
  for (int i = 0; i < 3; ++i) {
    const auto &channel = config.channels[i];
    if (static_cast<int>(channel.id) != i)
      return Result<void>::failure(ErrorCode::MissingChannel,
                                   "Configuration must contain CH1, CH2 and CH3");
    if (channel.controller.serial.portName.empty() ||
        !ports.insert(channel.controller.serial.portName).second)
      return Result<void>::failure(ErrorCode::DuplicatePort,
                                   "Controller serial ports must be non-empty and unique");
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

Rod::Rod(RodAddress a, RodConfig c) : address_(a), config_(std::move(c)) {}
RodAddress Rod::address() const noexcept { return address_; }
const RodConfig &Rod::config() const noexcept { return config_; }
RodStatus Rod::status() const {
  std::lock_guard l(mutex_);
  return status_;
}
bool Rod::isPositionLegal(int32_t p) const noexcept {
  return p >= config_.softMin && p <= config_.softMax && p >= config_.hardMin &&
         p <= config_.hardMax;
}
bool Rod::isAtTarget() const noexcept {
  auto s = status();
  return s.positionValid && std::abs(s.actualPosition - s.targetPosition) <=
                                config_.positionTolerance;
}
double Rod::encoderToOpticalPosition(int32_t position) const noexcept {
  return opticalPosition(config_, position);
}
std::optional<int32_t>
Rod::opticalPositionToEncoder(double position) const noexcept {
  if (!hasMonotonicOpticalPosition(config_))
    return std::nullopt;
  double low = config_.softMin;
  double high = config_.softMax;
  const bool increasing =
      opticalPosition(config_, high) > opticalPosition(config_, low);
  const double minimum =
      std::min(opticalPosition(config_, low), opticalPosition(config_, high));
  const double maximum =
      std::max(opticalPosition(config_, low), opticalPosition(config_, high));
  if (position < minimum || position > maximum)
    return std::nullopt;
  for (int iteration = 0; iteration < 48; ++iteration) {
    const double middle = (low + high) * 0.5;
    const double optical = opticalPosition(config_, middle);
    if ((increasing && optical < position) ||
        (!increasing && optical > position))
      low = middle;
    else
      high = middle;
  }
  return static_cast<int32_t>(std::lround((low + high) * 0.5));
}

Controller::Controller(ChannelId c, ControllerConfig cfg,
                       std::unique_ptr<IControllerBackend> b)
    : channel_(c), config_(std::move(cfg)), backend_(std::move(b)) {}
Controller::~Controller() { disconnect(); }
ChannelId Controller::channel() const noexcept { return channel_; }
Result<void> Controller::addRod(RodConfig c) {
  if (findRod(c.id))
    return Result<void>::failure(ErrorCode::DuplicateId, "Duplicate Rod ID");
  rodsById_[c.id] = nullptr;
  rods_.push_back(
      std::make_unique<Rod>(RodAddress{channel_, c.id}, std::move(c)));
  rodsById_[rods_.back()->address().rod] = rods_.back().get();
  return Result<void>::success();
}
Rod *Controller::findRod(RodId id) noexcept {
  auto i = rodsById_.find(id);
  return i == rodsById_.end() ? nullptr : i->second;
}
const Rod *Controller::findRod(RodId id) const noexcept {
  auto i = rodsById_.find(id);
  return i == rodsById_.end() ? nullptr : i->second;
}
const std::vector<std::unique_ptr<Rod>> &Controller::rods() const noexcept {
  return rods_;
}
Result<void> Controller::connect() {
  std::lock_guard l(ioMutex_);
  std::vector<RodConfig> configs;
  configs.reserve(rods_.size());
  for (const auto &rod : rods_)
    configs.push_back(rod->config());
  auto configured = backend_->configureRods(configs);
  if (!configured)
    return configured;
  return backend_->open(config_.serial);
}
Result<void> Controller::disconnect() {
  std::lock_guard l(ioMutex_);
  return backend_->close();
}
bool Controller::isConnected() const noexcept { return backend_->isOpen(); }
Result<void> Controller::refreshStatus() {
  std::lock_guard l(ioMutex_);
  for (auto &r : rods_) {
    auto x = backend_->readStatus(r->address().rod);
    if (!x)
      return Result<void>::failure(x.error, x.message);
    std::lock_guard rl(r->mutex_);
    r->status_ = x.value;
  }
  return Result<void>::success();
}
Result<void> Controller::enable(RodId id, bool e) {
  auto *r = findRod(id);
  if (!r)
    return Result<void>::failure(ErrorCode::RodNotFound);
  std::lock_guard l(ioMutex_);
  return backend_->enable(id, e);
}
Result<void> Controller::home(RodId id) {
  auto *r = findRod(id);
  if (!r)
    return Result<void>::failure(ErrorCode::RodNotFound);
  std::lock_guard l(ioMutex_);
  return backend_->home(id);
}
Result<void> Controller::submit(RodTarget t) {
  auto *r = findRod(t.address.rod);
  if (!r)
    return Result<void>::failure(ErrorCode::RodNotFound);
  if (!r->isPositionLegal(t.position))
    return Result<void>::failure(ErrorCode::LimitViolation);
  std::lock_guard l(ioMutex_);
  auto x = backend_->setGoal(t.address.rod, t.position, t.velocity);
  if (x) {
    std::lock_guard rl(r->mutex_);
    r->status_.targetPosition = t.position;
    r->status_.state = RodState::Moving;
  }
  return x;
}
Result<void> Controller::submitSynchronous(const std::vector<RodTarget> &ts) {
  std::lock_guard l(ioMutex_);
  return backend_->setGoalsSynchronous(ts);
}
Result<void> Controller::stop(RodId id) {
  std::lock_guard l(ioMutex_);
  return backend_->stop(id);
}
Result<void> Controller::stopAll() {
  std::lock_guard l(ioMutex_);
  return backend_->stopAll();
}
Result<void> Controller::resetFaults() {
  std::lock_guard l(ioMutex_);
  for (auto &r : rods_) {
    auto x = backend_->resetAlarm(r->address().rod);
    if (!x)
      return x;
  }
  return Result<void>::success();
}
ChannelStatus Controller::status() const {
  ChannelStatus s;
  s.id = channel_;
  s.connected = isConnected();
  s.state = s.connected ? ChannelState::Ready : ChannelState::Offline;
  for (auto &r : rods_)
    s.rods.push_back(r->status());
  return s;
}
ChannelSafetyStatus Controller::evaluateSafety() const {
  ChannelSafetyStatus s;
  s.channel = channel_;
  s.state = SafetyState::Safe;
  for (auto &r : rods_) {
    auto x = r->status();
    if (!x.positionValid)
      s.violations.push_back(SafetyViolation::PositionUnknown);
    if (x.alarm)
      s.violations.push_back(SafetyViolation::DeviceAlarm);
    if (!r->isPositionLegal(x.actualPosition))
      s.violations.push_back(SafetyViolation::SoftLimitViolation);
    if (!isConnected())
      s.violations.push_back(SafetyViolation::CommunicationLost);
  }
  if (!s.violations.empty())
    s.state = SafetyState::Unsafe;
  s.checkedAt = std::chrono::steady_clock::now();
  return s;
}
Result<ChannelSafetyStatus> Controller::refreshAndEvaluateSafety() {
  auto x = refreshStatus();
  if (!x)
    return Result<ChannelSafetyStatus>::failure(x.error, x.message);
  return Result<ChannelSafetyStatus>::success(evaluateSafety());
}
Result<void> Controller::setExternalSafetyOverride(std::optional<bool>,
                                                   std::string) {
  return Result<void>::success();
}
bool Controller::isSafe() const noexcept {
  return evaluateSafety().state == SafetyState::Safe;
}

FFChannel::FFChannel(FFChannelConfig c, std::unique_ptr<IControllerBackend> b)
    : id_(c.id), controller_(std::make_unique<Controller>(c.id, c.controller,
                                                          std::move(b))) {
  for (auto &r : c.rods)
    controller_->addRod(r);
}
FFChannel::~FFChannel() = default;
FFChannel::FFChannel(FFChannel &&) noexcept = default;
FFChannel &FFChannel::operator=(FFChannel &&) noexcept = default;
ChannelId FFChannel::id() const noexcept { return id_; }
Controller &FFChannel::controller() noexcept { return *controller_; }
const Controller &FFChannel::controller() const noexcept {
  return *controller_;
}
ChannelStatus FFChannel::status() const { return controller_->status(); }
Result<void> FFChannel::moveAllToSafePositions() {
  if (!controller_->isSafe())
    return Result<void>::failure(ErrorCode::SafetyViolation,
                                 "Channel is not safe");
  std::vector<RodTarget> t;
  for (auto &r : controller_->rods()) {
    if (!r->config().safePositionValid)
      return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                   "Safe position is not configured");
    t.push_back(
        {r->address(), r->config().safePosition, r->config().maxVelocity});
  }
  return controller_->submitSynchronous(t);
}
Result<void> FFChannel::moveAllToPositions(const std::vector<RodTarget> &t) {
  if (!controller_->isSafe())
    return Result<void>::failure(ErrorCode::SafetyViolation);
  for (auto &x : t)
    if (x.address.channel != id_ || !controller_->findRod(x.address.rod) ||
        !controller_->findRod(x.address.rod)->isPositionLegal(x.position))
      return Result<void>::failure(ErrorCode::InvalidTarget);
  return controller_->submitSynchronous(t);
}

FF::FF(FFConfig c, std::array<std::unique_ptr<IControllerBackend>, 3> b)
    : config_(std::move(c)) {
  for (int i = 0; i < 3; i++)
    channels_[i] =
        std::make_unique<FFChannel>(config_.channels[i], std::move(b[i]));
}
FF::~FF() { disconnect(); }
Result<void> FF::initialize() {
  auto x = validateConfiguration();
  if (!x) {
    lastError_ = x.message;
    state_ = FFState::Fault;
    return x;
  }
  state_ = FFState::Ready;
  return Result<void>::success();
}
Result<FFConfig> FF::loadConfig(const std::string &p) {
  return FFConfigIO::load(p);
}
Result<void> FF::saveConfig(const std::string &p) const {
  return FFConfigIO::save(p, config_);
}
Result<void> FF::applyConfig(FFConfig c) {
  if (state_ != FFState::Offline)
    return Result<void>::failure(ErrorCode::Busy);
  config_ = std::move(c);
  return initialize();
}
const FFConfig &FF::config() const noexcept { return config_; }
Result<void> FF::connect() {
  for (auto &c : channels_) {
    auto x = c->controller().connect();
    if (!x) {
      state_ = FFState::Degraded;
      lastError_ = x.message;
      return x;
    }
  }
  state_ = FFState::Ready;
  auto refreshed = refreshStatus();
  if (!refreshed)
    return refreshed;
  const auto current = status();
  for (const auto &safety : current.safety) {
    if (safety.state != SafetyState::Safe) {
      disconnect();
      return Result<void>::failure(ErrorCode::SafetyViolation,
                                   "Initial motor positions are unsafe");
    }
  }
  return Result<void>::success();
}
Result<void> FF::disconnect() {
  for (auto &c : channels_)
    c->controller().disconnect();
  state_ = FFState::Offline;
  return Result<void>::success();
}
Result<void> FF::refreshStatus() {
  for (auto &c : channels_) {
    auto x = c->controller().refreshStatus();
    if (!x) {
      state_ = FFState::Degraded;
      lastError_ = x.message;
      return x;
    }
  }
  return Result<void>::success();
}
Result<ChannelSafetyStatus> FF::refreshChannelSafety(ChannelId id) {
  auto *c = channel(id);
  if (!c)
    return Result<ChannelSafetyStatus>::failure(ErrorCode::RodNotFound);
  return c->controller().refreshAndEvaluateSafety();
}
ChannelSafetyStatus FF::channelSafety(ChannelId id) const {
  return channel(id) ? channel(id)->controller().evaluateSafety()
                     : ChannelSafetyStatus{};
}
Result<MotionReceipt> FF::submitMotion(const MotionRequest &r) {
  // Safety boundaries depend on the actual positions of neighboring Rods.
  // Refresh every configured Rod before validating so callers cannot make a
  // decision from stale cached feedback.
  auto refreshed = refreshStatus();
  if (!refreshed)
    return Result<MotionReceipt>::failure(refreshed.error, refreshed.message);
  auto v = validateMotion(r);
  if (!v)
    return Result<MotionReceipt>::failure(v.error, v.message);
  MotionReceipt m;
  for (int i = 0; i < 3; i++) {
    std::vector<RodTarget> x;
    for (auto &t : r.targets)
      if (t.address.channel == static_cast<ChannelId>(i))
        x.push_back(t);
    if (!x.empty()) {
      auto z = channels_[i]->controller().submitSynchronous(x);
      m.channels.push_back(
          {static_cast<ChannelId>(i), z, std::chrono::steady_clock::now()});
    }
  }
  state_ = FFState::Moving;
  return Result<MotionReceipt>::success(m);
}
Result<void> FF::moveChannelToSafePositions(ChannelId id) {
  auto *c = channel(id);
  return c ? c->moveAllToSafePositions()
           : Result<void>::failure(ErrorCode::RodNotFound);
}
Result<void> FF::moveChannelToPositions(ChannelId id,
                                        const std::vector<RodTarget> &t) {
  auto *c = channel(id);
  return c ? c->moveAllToPositions(t)
           : Result<void>::failure(ErrorCode::RodNotFound);
}
Result<void> FF::stopAll() {
  for (auto &c : channels_)
    c->controller().stopAll();
  state_ = FFState::Ready;
  return Result<void>::success();
}
Result<void> FF::emergencyStop() {
  state_ = FFState::EmergencyStopped;
  for (auto &c : channels_)
    c->controller().stopAll();
  return Result<void>::success();
}
Result<void> FF::resetFaults() {
  for (auto &c : channels_)
    c->controller().resetFaults();
  if (state_ == FFState::EmergencyStopped)
    state_ = FFState::Ready;
  return Result<void>::success();
}
FFState FF::state() const noexcept { return state_; }
FFStatus FF::status() const {
  FFStatus s;
  s.state = state_;
  for (int i = 0; i < 3; i++) {
    s.channels[i] = channels_[i]->status();
    s.safety[i] = channels_[i]->controller().evaluateSafety();
    auto layerSafety =
        validateLayerClearance(config_.channels[i], s.channels[i], {});
    if (!layerSafety) {
      s.safety[i].violations.push_back(layerSafety.error ==
                                               ErrorCode::InvalidConfiguration
                                           ? SafetyViolation::InvalidTopology
                                           : SafetyViolation::RodInterference);
      s.safety[i].state = SafetyState::Unsafe;
    }
  }
  return s;
}
FFChannel *FF::channel(ChannelId id) noexcept {
  return channels_[static_cast<int>(id)].get();
}
const FFChannel *FF::channel(ChannelId id) const noexcept {
  return channels_[static_cast<int>(id)].get();
}
Rod *FF::findRod(RodAddress a) noexcept {
  return channel(a.channel) ? channel(a.channel)->controller().findRod(a.rod)
                            : nullptr;
}
const Rod *FF::findRod(RodAddress a) const noexcept {
  return channel(a.channel) ? channel(a.channel)->controller().findRod(a.rod)
                            : nullptr;
}
Result<void> FF::validateConfiguration() const {
  std::unordered_set<std::string> ports;
  for (auto &c : config_.channels) {
    if (c.controller.serial.portName.empty())
      return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                   "Serial port is empty");
    if (!ports.insert(c.controller.serial.portName).second)
      return Result<void>::failure(ErrorCode::DuplicatePort,
                                   "Duplicate serial port");
    for (auto &r : c.rods)
      if (r.hardMin > r.hardMax || r.softMin < r.hardMin ||
          r.softMax > r.hardMax || r.softMin > r.softMax)
        return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                     "Invalid limit configuration");
    auto topology = validateLayerTopology(c);
    if (!topology)
      return topology;
  }
  return Result<void>::success();
}
std::string FF::lastError() const { return lastError_; }
Result<void> FF::validateMotion(const MotionRequest &r) const {
  std::unordered_set<RodAddress, RodAddressHash> seen;
  for (auto &t : r.targets) {
    if (!seen.insert(t.address).second)
      return Result<void>::failure(ErrorCode::DuplicateTarget);
    auto *x = findRod(t.address);
    if (!x)
      return Result<void>::failure(ErrorCode::RodNotFound);
    if (!x->isPositionLegal(t.position))
      return Result<void>::failure(ErrorCode::LimitViolation);
  }
  const auto current = status();
  for (int i = 0; i < 3; ++i) {
    std::unordered_map<RodId, int32_t> positions;
    for (const auto &target : r.targets)
      if (static_cast<int>(target.address.channel) == i)
        positions[target.address.rod] = target.position;
    if (positions.empty())
      continue;
    auto safety = validateLayerClearance(config_.channels[i],
                                         current.channels[i], positions);
    if (!safety)
      return safety;
  }
  return Result<void>::success();
}

Result<MotionPlan> MotionPlanner::plan(const FFConfig &c, const FFStatus &s,
                                       const MotionRequest &r) {
  MotionPlan p;
  p.configVersion = c.configVersion;
  p.id = static_cast<uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  for (auto &t : r.targets) {
    int ci = static_cast<int>(t.address.channel);
    if (s.safety[ci].state != SafetyState::Safe)
      return Result<MotionPlan>::failure(ErrorCode::SafetyViolation,
                                         "Current state is not safe");
  }
  for (auto &t : r.targets) {
    const auto &ch = c.channels[static_cast<int>(t.address.channel)];
    auto it = std::find_if(ch.rods.begin(), ch.rods.end(),
                           [&](auto &x) { return x.id == t.address.rod; });
    if (it == ch.rods.end() || !it->safePositionValid)
      return Result<MotionPlan>::failure(ErrorCode::InvalidConfiguration,
                                         "Target rod has no safe position");
  }
  // Check the current state and every resulting serial phase. A Rod that is
  // present in several layer records is therefore checked in every layer.
  std::array<std::unordered_map<RodId, int32_t>, 3> phasePositions;
  for (int i = 0; i < 3; ++i) {
    const bool used = std::any_of(
        r.targets.begin(), r.targets.end(), [i](const auto &target) {
          return static_cast<int>(target.address.channel) == i;
        });
    if (!used)
      continue;
    auto safety =
        validateLayerClearance(c.channels[i], s.channels[i], phasePositions[i]);
    if (!safety)
      return Result<MotionPlan>::failure(safety.error, safety.message);
  }
  for (const auto &target : r.targets) {
    const int channelIndex = static_cast<int>(target.address.channel);
    const auto *rod =
        findRodConfig(c.channels[channelIndex], target.address.rod);
    phasePositions[channelIndex][target.address.rod] = rod->safePosition;
    auto safety = validateLayerClearance(c.channels[channelIndex],
                                         s.channels[channelIndex],
                                         phasePositions[channelIndex]);
    if (!safety)
      return Result<MotionPlan>::failure(safety.error, safety.message);
  }
  for (const auto &target : r.targets) {
    const int channelIndex = static_cast<int>(target.address.channel);
    phasePositions[channelIndex][target.address.rod] = target.position;
    auto safety = validateLayerClearance(c.channels[channelIndex],
                                         s.channels[channelIndex],
                                         phasePositions[channelIndex]);
    if (!safety)
      return Result<MotionPlan>::failure(safety.error, safety.message);
  }
  for (auto &t : r.targets) {
    const auto &ch = c.channels[static_cast<int>(t.address.channel)];
    const auto it =
        std::find_if(ch.rods.begin(), ch.rods.end(),
                     [&](const auto &rod) { return rod.id == t.address.rod; });
    p.steps.push_back({PlanStepKind::MoveToSafe, t.address, it->safePosition,
                       it->maxVelocity, "Move to safe position"});
    p.steps.push_back({PlanStepKind::MoveToTarget, t.address, t.position,
                       t.velocity, "Move to target position"});
  }
  p.valid = true;
  p.message = "Plan valid (serial within each channel)";
  return Result<MotionPlan>::success(std::move(p));
}
} // namespace ff
