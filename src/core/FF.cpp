#include "ff/FF.hpp"
#include "ff/ConfigJson.hpp"
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

Rod::Rod(RodAddress a, RodConfig c) : address_(a), config_(std::move(c)) {}
RodAddress Rod::address() const noexcept { return address_; }
const RodConfig &Rod::config() const noexcept { return config_; }
RodStatus Rod::status() const {
  std::lock_guard l(mutex_);
  return status_;
}
bool Rod::isPositionLegal(int32_t p) const noexcept {
  if (p < config_.softMin || p > config_.softMax ||
      p < config_.hardMin || p > config_.hardMax)
    return false;
  for (const auto &range : config_.forbiddenRanges)
    if (p >= range.first && p <= range.second)
      return false;
  return true;
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
Result<void> Controller::refreshStatus(RodId id) {
  auto *rod = findRod(id);
  if (!rod)
    return Result<void>::failure(ErrorCode::RodNotFound);
  std::lock_guard l(ioMutex_);
  auto status = backend_->readStatus(id);
  if (!status)
    return Result<void>::failure(status.error, status.message);
  std::lock_guard rl(rod->mutex_);
  rod->status_ = status.value;
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
  auto x = backend_->setGoal(t.address.rod, t.position);
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
    t.push_back({r->address(), r->config().safePosition});
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

FF::FF(FFConfig c, std::array<std::unique_ptr<IControllerBackend>, 3> b,
       std::array<bool,3> enabled)
    : config_(std::move(c)), enabled_(enabled) {
  for (int i = 0; i < 3; i++)
    if (enabled_[i]) channels_[i] =
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
  for (int i=0;i<3;++i) {
    if (!enabled_[i]) continue;
    auto &c=channels_[i];
    auto x = c->controller().connect();
    if (!x) {
      for (int j=0;j<3;++j)
        if (enabled_[j]) channels_[j]->controller().disconnect();
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
  for (int i=0;i<3;++i) {
    if (!enabled_[i]) continue;
    const auto &safety=current.safety[i];
    if (safety.state != SafetyState::Safe) {
      disconnect();
      return Result<void>::failure(ErrorCode::SafetyViolation,
                                   "Initial motor positions are unsafe");
    }
  }
  return Result<void>::success();
}
Result<void> FF::disconnect() {
  for (int i=0;i<3;++i)
    if (enabled_[i]) channels_[i]->controller().disconnect();
  state_ = FFState::Offline;
  return Result<void>::success();
}
Result<void> FF::refreshStatus() {
  bool anyMoving = false;
  for (int i=0;i<3;++i) {
    if (!enabled_[i]) continue;
    auto &c=channels_[i];
    auto x = c->controller().refreshStatus();
    if (!x) {
      state_ = FFState::Degraded;
      lastError_ = x.message;
      return x;
    }
    for (const auto& rod : c->controller().status().rods)
      anyMoving = anyMoving || rod.state == RodState::Moving || rod.state == RodState::Homing;
  }
  if (state_ == FFState::Moving && !anyMoving) state_ = FFState::Ready;
  return Result<void>::success();
}
Result<void> FF::refreshRodStatus(RodAddress address) {
  auto *selectedChannel = channel(address.channel);
  if (!selectedChannel || !enabled_[static_cast<int>(address.channel)])
    return Result<void>::failure(ErrorCode::RodNotFound);
  auto refreshed = selectedChannel->controller().refreshStatus(address.rod);
  if (!refreshed) {
    state_ = FFState::Degraded;
    lastError_ = refreshed.message;
  } else if (state_ == FFState::Moving) {
    bool anyMoving = false;
    for (int i=0;i<3;++i) {
      if (!enabled_[i]) continue;
      for (const auto &rod : channels_[i]->controller().status().rods)
        anyMoving = anyMoving || rod.state == RodState::Moving || rod.state == RodState::Homing;
    }
    if (!anyMoving) state_ = FFState::Ready;
  }
  return refreshed;
}
Result<ChannelSafetyStatus> FF::refreshChannelSafety(ChannelId id) {
  auto *c = channel(id);
  if (!c || !enabled_[static_cast<int>(id)])
    return Result<ChannelSafetyStatus>::failure(ErrorCode::RodNotFound);
  return c->controller().refreshAndEvaluateSafety();
}
ChannelSafetyStatus FF::channelSafety(ChannelId id) const {
  return enabled_[static_cast<int>(id)] ? channel(id)->controller().evaluateSafety()
                     : ChannelSafetyStatus{};
}
Result<MotionReceipt> FF::submitMotion(const MotionRequest &r) {
  for (const auto& target:r.targets) {
    const int ch=static_cast<int>(target.address.channel);
    if (ch<0 || ch>=3 || !enabled_[ch])
      return Result<MotionReceipt>::failure(ErrorCode::InvalidTarget,"Channel is disabled");
  }
  if (r.targets.size() > 1) {
    auto fresh = refreshStatus();
    if (!fresh)
      return Result<MotionReceipt>::failure(fresh.error, fresh.message);
  }
  // A partial command validates only the commanded Rods and their direct
  // layer neighbors. Refresh that same scope so unrelated stationary Rod I/O
  // cannot dominate a Training scan step or introduce feedback-noise stops.
  std::array<std::unordered_set<RodId>, 3> refreshIds;
  for (const auto &target : r.targets) {
    const auto channelIndex = static_cast<int>(target.address.channel);
    refreshIds[channelIndex].insert(target.address.rod);
    for (const auto &member : config_.channels[channelIndex].layerMembers) {
      if (member.rod != target.address.rod)
        continue;
      if (member.leftNeighbor >= 0)
        refreshIds[channelIndex].insert(
            static_cast<RodId>(member.leftNeighbor));
      if (member.rightNeighbor >= 0)
        refreshIds[channelIndex].insert(
            static_cast<RodId>(member.rightNeighbor));
    }
  }
  for (int channelIndex = 0; channelIndex < 3; ++channelIndex) {
    for (const auto rod : refreshIds[channelIndex]) {
      auto refreshed = channels_[channelIndex]->controller().refreshStatus(rod);
      if (!refreshed)
        return Result<MotionReceipt>::failure(refreshed.error,
                                              refreshed.message);
      const auto *checked = channels_[channelIndex]->controller().findRod(rod);
      if (!checked)
        return Result<MotionReceipt>::failure(ErrorCode::RodNotFound);
      const auto state = checked->status();
      if (!state.positionValid || state.alarm || state.state == RodState::Fault)
        return Result<MotionReceipt>::failure(ErrorCode::SafetyViolation,
                                              "Moving Rod or neighbor feedback is unsafe");
    }
  }
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
      if (!z) {
        stopAll();
        return Result<MotionReceipt>::failure(z.error, z.message);
      }
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
  Result<void> firstFailure;
  for (int i=0;i<3;++i) {
    if (!enabled_[i]) continue;
    auto r=channels_[i]->controller().stopAll();
    if(!r && firstFailure) firstFailure=r;
  }
  state_ = FFState::Ready;
  return firstFailure;
}
Result<void> FF::emergencyStop() {
  state_ = FFState::EmergencyStopped;
  Result<void> firstFailure;
  for (int i=0;i<3;++i) {
    if (!enabled_[i]) continue;
    auto r=channels_[i]->controller().stopAll();
    if(!r && firstFailure) firstFailure=r;
  }
  return firstFailure;
}
Result<void> FF::resetFaults() {
  Result<void> firstFailure;
  for (int i=0;i<3;++i) {
    if (!enabled_[i]) continue;
    auto r=channels_[i]->controller().resetFaults();
    if(!r && firstFailure) firstFailure=r;
  }
  if (firstFailure && state_ == FFState::EmergencyStopped)
    state_ = FFState::Ready;
  return firstFailure;
}
FFState FF::state() const noexcept { return state_; }
FFStatus FF::status() const {
  FFStatus s;
  s.state = state_;
  for (int i = 0; i < 3; i++) {
    if (!enabled_[i]) continue;
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
  const int index=static_cast<int>(id);
  return index>=0 && index<3 ? channels_[index].get() : nullptr;
}
const FFChannel *FF::channel(ChannelId id) const noexcept {
  const int index=static_cast<int>(id);
  return index>=0 && index<3 ? channels_[index].get() : nullptr;
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
  for (int i=0;i<3;++i) {
    if (!enabled_[i]) continue;
    const auto &c=config_.channels[i];
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
      return Result<void>::failure(topology.error,
          "CH"+std::to_string(i+1)+" "+topology.message);
    if (c.trainingRod &&
        std::none_of(c.layerMembers.begin(),c.layerMembers.end(),
            [&](const LayerMembership& member) {
              return member.layer==2 && member.rod==*c.trainingRod;
            }))
      return Result<void>::failure(ErrorCode::InvalidConfiguration,
          "CH"+std::to_string(i+1)+" Rod "+std::to_string(*c.trainingRod)+
          " training_rod_id must be in Layer 2");
  }
  return Result<void>::success();
}
std::string FF::lastError() const { return lastError_; }
Result<void> FF::validateMotion(const MotionRequest &r) const {
  std::unordered_set<RodAddress, RodAddressHash> seen;
  for (auto &t : r.targets) {
    const int ch=static_cast<int>(t.address.channel);
    if (ch<0 || ch>=3 || !enabled_[ch])
      return Result<void>::failure(ErrorCode::InvalidTarget,"Channel is disabled");
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
    std::unordered_set<RodId> checkedRods;
    for (const auto &target : r.targets)
      if (static_cast<int>(target.address.channel) == i) {
        positions[target.address.rod] = target.position;
        checkedRods.insert(target.address.rod);
      }
    if (positions.empty())
      continue;
    if (r.targets.size() > 1 &&
        current.safety[i].state != SafetyState::Safe)
      return Result<void>::failure(ErrorCode::SafetyViolation,
                                   "Current channel state is not safe");
    auto safety = validateLayerClearance(config_.channels[i],
                                         current.channels[i], positions,
                                         &checkedRods);
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
  if (r.targets.empty())
    return Result<MotionPlan>::failure(ErrorCode::InvalidTarget,
                                       "Empty motion target set");
  std::unordered_set<RodAddress, RodAddressHash> seen;
  for (auto &t : r.targets) {
    int ci = static_cast<int>(t.address.channel);
    if (ci < 0 || ci >= 3 || !seen.insert(t.address).second)
      return Result<MotionPlan>::failure(ErrorCode::InvalidTarget,
                                         "Invalid or duplicate Rod address");
    if (s.safety[ci].state != SafetyState::Safe)
      return Result<MotionPlan>::failure(ErrorCode::SafetyViolation,
                                         "Current state is not safe");
    const auto &ch = c.channels[ci];
    const auto rod = std::find_if(ch.rods.begin(), ch.rods.end(),
                                  [&](const auto &x) { return x.id == t.address.rod; });
    if (rod == ch.rods.end() || t.position < rod->softMin ||
        t.position > rod->softMax || t.position < rod->hardMin ||
        t.position > rod->hardMax ||
        std::any_of(rod->forbiddenRanges.begin(), rod->forbiddenRanges.end(),
                    [&](const auto &range) {
                      return t.position >= range.first &&
                             t.position <= range.second;
                    }))
      return Result<MotionPlan>::failure(ErrorCode::InvalidTarget,
                                         "Invalid Rod target");
  }
  // The selected multi-Rod policy checks the complete starting and ending
  // layouts. Each channel receives its target positions as one batch.
  std::array<std::unordered_map<RodId, int32_t>, 3> finalPositions;
  for (int i = 0; i < 3; ++i) {
    const bool used = std::any_of(
        r.targets.begin(), r.targets.end(), [i](const auto &target) {
          return static_cast<int>(target.address.channel) == i;
        });
    if (!used)
      continue;
    auto safety =
        validateLayerClearance(c.channels[i], s.channels[i], finalPositions[i]);
    if (!safety)
      return Result<MotionPlan>::failure(safety.error, safety.message);
  }
  for (const auto &target : r.targets) {
    const int channelIndex = static_cast<int>(target.address.channel);
    finalPositions[channelIndex][target.address.rod] = target.position;
  }
  for (int i = 0; i < 3; ++i) {
    if (finalPositions[i].empty()) continue;
    auto safety = validateLayerClearance(c.channels[i], s.channels[i],
                                         finalPositions[i]);
    if (!safety)
      return Result<MotionPlan>::failure(safety.error, safety.message);
  }
  for (auto &t : r.targets)
    p.steps.push_back({t.address, t.position, "Grouped target move"});
  p.valid = true;
  p.message = "Plan valid (grouped within each channel)";
  return Result<MotionPlan>::success(std::move(p));
}
} // namespace ff
