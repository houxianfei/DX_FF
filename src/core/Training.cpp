#include "ff/Training.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <set>
#include <sstream>

namespace ff {
namespace {
bool inside(const std::vector<std::pair<std::int32_t, std::int32_t>> &ranges,
            std::int32_t value) {
  return std::any_of(ranges.begin(), ranges.end(), [value](const auto &range) {
    return value >= range.first && value <= range.second;
  });
}
double optical(const RodConfig &rod, std::int32_t position) {
  return rod.p0 + rod.p1 * static_cast<double>(position);
}
const RodConfig *rodById(const FFChannelConfig &channel, RodId id) {
  const auto it = std::find_if(channel.rods.begin(), channel.rods.end(),
                               [id](const RodConfig &r) { return r.id == id; });
  return it == channel.rods.end() ? nullptr : &*it;
}
int indexById(const FFChannelConfig &channel, RodId id) {
  for (size_t i = 0; i < channel.rods.size(); ++i)
    if (channel.rods[i].id == id) return static_cast<int>(i);
  return -1;
}
bool legalEncoder(const RodConfig &rod, std::int32_t encoder) {
  return encoder >= rod.softMin && encoder <= rod.softMax &&
         !inside(rod.forbiddenRanges, encoder);
}
bool isLayer2TrainingRod(const FFChannelConfig &channel, RodId id) {
  bool inLayer2 = false;
  bool inOtherLayer = false;
  for (const auto &member : channel.layerMembers) {
    if (member.rod != id) continue;
    inLayer2 = inLayer2 || member.layer == 2;
    inOtherLayer = inOtherLayer || member.layer != 2;
  }
  return inLayer2 && !inOtherLayer;
}
// For the usual Layer 2 chain, an increasing sequence of obstruction
// positions can only be assigned to Rods in increasing topology order.
// Exploiting that fact avoids factorial permutation searches during GUI
// editing.  Return no order for unusual/ambiguous topology so those machines
// keep the original exhaustive algorithm.
std::optional<std::vector<int>> orderedLayer2Ranks(
    const FFChannelConfig &channel) {
  std::vector<const LayerMembership *> members;
  for (const auto &member : channel.layerMembers)
    if (member.layer == 2) members.push_back(&member);
  if (members.empty()) return std::nullopt;

  const LayerMembership *first = nullptr;
  for (const auto *member : members) {
    if (member->leftNeighbor >= 0) continue;
    if (first) return std::nullopt;
    first = member;
  }
  if (!first) return std::nullopt;

  std::vector<int> ranksByRodIndex(channel.rods.size(), -1);
  std::set<int> visited;
  const LayerMembership *member = first;
  int rank = 0;
  while (member) {
    if (!visited.insert(int(member->rod)).second) return std::nullopt;
    const int rodIndex = indexById(channel, member->rod);
    if (rodIndex < 0) return std::nullopt;
    ranksByRodIndex[static_cast<size_t>(rodIndex)] = rank++;
    if (member->rightNeighbor < 0) break;
    const auto clearance = std::find_if(
        channel.clearanceRules.begin(), channel.clearanceRules.end(),
        [&](const PairClearanceRule &rule) {
          return rule.layer == 2 &&
                 ((rule.first == member->rod &&
                   rule.second == static_cast<RodId>(member->rightNeighbor)) ||
                  (rule.second == member->rod &&
                   rule.first == static_cast<RodId>(member->rightNeighbor)));
        });
    if (clearance == channel.clearanceRules.end() ||
        clearance->minimumClearanceMm < 0.0)
      return std::nullopt;
    const auto next = std::find_if(
        members.begin(), members.end(), [&](const LayerMembership *entry) {
          return entry->rod == static_cast<RodId>(member->rightNeighbor) &&
                 entry->leftNeighbor == int(member->rod);
        });
    if (next == members.end()) return std::nullopt;
    member = *next;
  }
  if (visited.size() != members.size()) return std::nullopt;
  return ranksByRodIndex;
}
bool safeTopology(const FFChannelConfig &channel, const ChannelStatus &status,
                  const std::vector<std::pair<RodId, std::int32_t>> &override) {
  auto position = [&](RodId id, std::int32_t &value) {
    for (const auto &entry : override)
      if (entry.first == id) { value = entry.second; return true; }
    const int index = indexById(channel, id);
    if (index < 0 || static_cast<size_t>(index) >= status.rods.size() ||
        !status.rods[index].positionValid) return false;
    value = status.rods[index].actualPosition;
    return true;
  };
  for (const auto &member : channel.layerMembers) {
    if (member.rightNeighbor < 0) continue;
    const RodId right = static_cast<RodId>(member.rightNeighbor);
    const auto rule = std::find_if(channel.clearanceRules.begin(), channel.clearanceRules.end(),
      [&](const PairClearanceRule &r) {
        return r.layer == member.layer &&
          ((r.first == member.rod && r.second == right) ||
           (r.second == member.rod && r.first == right));
      });
    if (rule == channel.clearanceRules.end()) return false;
    std::int32_t leftPosition{}, rightPosition{};
    const auto *leftRod = rodById(channel, member.rod);
    const auto *rightRod = rodById(channel, right);
    if (!leftRod || !rightRod || !position(member.rod, leftPosition) ||
        !position(right, rightPosition) ||
        optical(*rightRod, rightPosition) - optical(*leftRod, leftPosition) +
                kLayerClearanceToleranceMm <
            rule->minimumClearanceMm) return false;
  }
  return true;
}
std::string escapeJson(const std::string &value) {
  std::string out;
  for (char c : value) { if (c == '\\' || c == '"') out.push_back('\\'); out.push_back(c); }
  return out;
}
std::string stringField(const std::string &text, const char *key) {
  std::smatch match;
  const std::regex pattern("\"" + std::string(key) +
                           "\"\\s*:\\s*\"((?:\\\\.|[^\"\\\\])*)\"");
  if (!std::regex_search(text, match, pattern)) return {};
  std::string value;
  const auto encoded=match[1].str();
  for (size_t i=0;i<encoded.size();++i) {
    if (encoded[i]=='\\' && i+1<encoded.size()) ++i;
    value.push_back(encoded[i]);
  }
  return value;
}
double numberField(const std::string &text, const char *key, double fallback = 0) {
  std::smatch match;
  const std::regex pattern("\"" + std::string(key) + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
  try { return std::regex_search(text, match, pattern) ? std::stod(match[1].str()) : fallback; }
  catch (...) { return fallback; }
}
bool unsignedField(const std::string& text,const char* key,std::uint64_t& value) {
  std::smatch match;
  const std::regex pattern("\""+std::string(key)+"\"\\s*:\\s*([0-9]+)(?=\\s*[,}])");
  if (!std::regex_search(text,match,pattern)) return false;
  try { value=std::stoull(match[1].str()); return true; }
  catch (...) { return false; }
}
bool trainingBoolField(const std::string &text, const char *key) {
  std::smatch match;
  const std::regex pattern("\"" + std::string(key) + "\"\\s*:\\s*(true|false)");
  return std::regex_search(text, match, pattern) && match[1].str() == "true";
}
std::string arrayBody(const std::string &text, const char *key) {
  const auto pos = text.find(std::string("\"") + key + "\"");
  if (pos == std::string::npos) return {};
  const auto begin = text.find('[', pos);
  if (begin == std::string::npos) return {};
  int depth = 0;
  bool quoted=false, escaped=false;
  for (size_t i = begin; i < text.size(); ++i) {
    if (quoted) {
      if (escaped) escaped=false;
      else if (text[i]=='\\') escaped=true;
      else if (text[i]=='"') quoted=false;
      continue;
    }
    if (text[i]=='"') { quoted=true; continue; }
    if (text[i] == '[') ++depth;
    else if (text[i] == ']' && --depth == 0) return text.substr(begin + 1, i - begin - 1);
  }
  return {};
}
std::vector<std::string> objects(const std::string &array) {
  std::vector<std::string> result; int depth = 0; size_t start = 0;
  bool quoted=false, escaped=false;
  for (size_t i = 0; i < array.size(); ++i) {
    if (quoted) {
      if (escaped) escaped=false;
      else if (array[i]=='\\') escaped=true;
      else if (array[i]=='"') quoted=false;
      continue;
    }
    if (array[i]=='"') { quoted=true; continue; }
    if (array[i] == '{') { if (depth++ == 0) start = i; }
    else if (array[i] == '}' && --depth == 0) result.push_back(array.substr(start, i - start + 1));
  }
  return result;
}
std::string decimal(double value) {
  const double rounded=std::round(value*1000.0)/1000.0;
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << rounded;
  auto text=out.str();
  while (text.find('.')!=std::string::npos && text.back()=='0') text.pop_back();
  if (text.back()=='.') text.pop_back();
  return text=="-0" ? "0" : text;
}
bool validNumbers(const TrainingSession& s) {
  if (!std::isfinite(s.startMm) || !std::isfinite(s.endMm) ||
      !std::isfinite(s.stepMm) || s.stepMm<=0 ||
      std::round(s.stepMm*1000.0)<=0 ||
      !std::isfinite(s.valleySettings.minimumProminenceFraction) ||
      !std::isfinite(s.valleySettings.minimumDistanceMm)) return false;
  for (const auto& sample:s.samples) if (!std::isfinite(sample.positionMm)) return false;
  for (double candidate:s.candidatesMm) if (!std::isfinite(candidate)) return false;
  for (const auto& aim:s.aims)
    if (!std::isfinite(aim.requestedMm) || !std::isfinite(aim.quantizedMm)) return false;
  return true;
}
void writeSessionFields(std::ostream& out,const TrainingSession& s,const std::string& pad) {
  out << pad << "\"source\": \"" << trainingSourceName(s.source) << "\",\n"
      << pad << "\"training_rod\": " << int(s.trainingRod) << ",\n"
      << pad << "\"occlusion_summary\": \"" << escapeJson(s.occlusionSummary) << "\",\n"
      << pad << "\"start_mm\": " << decimal(s.startMm)
      << ", \"end_mm\": " << decimal(s.endMm)
      << ", \"step_mm\": " << decimal(s.stepMm) << ",\n"
      << pad << "\"complete\": " << (s.complete?"true":"false") << ",\n"
      << pad << "\"valley\": {\"smoothing\": " << s.valleySettings.smoothingPoints
      << ", \"prominence\": " << decimal(s.valleySettings.minimumProminenceFraction)
      << ", \"distance_mm\": " << decimal(s.valleySettings.minimumDistanceMm)
      << ", \"maximum\": " << s.valleySettings.maximumCount << "},\n";
  out << pad << "\"samples\": [";
  for (size_t i=0;i<s.samples.size();++i)
    out << (i?",":"") << '\n' << pad << "  {\"position_mm\": "
        << decimal(s.samples[i].positionMm) << ", \"energy\": " << s.samples[i].energy << "}";
  if (!s.samples.empty()) out << '\n' << pad;
  out << "],\n" << pad << "\"candidates_mm\": [";
  for (size_t i=0;i<s.candidatesMm.size();++i)
    out << (i?",":"") << '\n' << pad << "  " << decimal(s.candidatesMm[i]);
  if (!s.candidatesMm.empty()) out << '\n' << pad;
  out << "],\n" << pad << "\"aims\": [";
  for (size_t i=0;i<s.aims.size();++i) {
    const auto& a=s.aims[i];
    out << (i?",":"") << '\n' << pad << "  {\"rod\": " << int(a.address.rod)
        << ", \"requested_mm\": " << decimal(a.requestedMm)
        << ", \"quantized_mm\": " << decimal(a.quantizedMm)
        << ", \"encoder_aim\": " << a.encoderAim
        << ", \"actual_encoder\": " << a.actualEncoder
        << ", \"assigned\": " << (a.assigned?"true":"false") << "}";
  }
  if (!s.aims.empty()) out << '\n' << pad;
  out << "],\n" << pad << "\"rod_states\": [";
  for (size_t i=0;i<s.rodStates.size();++i) {
    const auto& r=s.rodStates[i];
    out << (i?",":"") << '\n' << pad << "  {\"channel\": " << int(r.address.channel)
        << ", \"rod\": " << int(r.address.rod)
        << ", \"actual_encoder\": " << r.actualEncoder
        << ", \"target_encoder\": " << r.targetEncoder << "}";
  }
  if (!s.rodStates.empty()) out << '\n' << pad;
  out << "],\n" << pad << "\"execution_targets\": [";
  for (size_t i=0;i<s.executionTargets.size();++i) {
    const auto& t=s.executionTargets[i];
    out << (i?",":"") << '\n' << pad << "  {\"channel\": " << int(t.address.channel)
        << ", \"rod\": " << int(t.address.rod)
        << ", \"position\": " << t.position << "}";
  }
  if (!s.executionTargets.empty()) out << '\n' << pad;
  out << ']';
}
} // namespace

const char *trainingSourceName(TrainingSource source) noexcept {
  switch (source) {
  case TrainingSource::Sim1: return "Sim1";
  case TrainingSource::Sim2: return "Sim2";
  case TrainingSource::Real: return "Real";
  case TrainingSource::Loaded: return "Loaded";
  }
  return "Unknown";
}

bool ItekEnergySource::available() const noexcept { return false; }
Result<std::uint64_t> ItekEnergySource::captureEnergy() {
  return Result<std::uint64_t>::failure(
      ErrorCode::NotReady,
      "Itek camera support requires the vendor runtime, hardware, and a validated acquisition configuration.");
}
std::string ItekEnergySource::diagnostic() const {
  return "Itek camera runtime/hardware is not available in this build.";
}

std::uint64_t TrainingAnalysis::simulatedEnergy(TrainingSource source, double p,
                                                  double start, double end) {
  const double span = std::max(1e-6, std::abs(end - start));
  const double t = (p - std::min(start, end)) / span;
  const auto dip = [&](double centre, double width, double depth) {
    const double x = (t - centre) / width;
    return depth * std::exp(-0.5 * x * x);
  };
  double energy = 1000000.0;
  if (source == TrainingSource::Sim1)
    energy -= dip(.25, .035, 260000) + dip(.50, .045, 420000) + dip(.75, .030, 310000);
  else
    energy -= dip(.20, .050, 180000) + dip(.40, .080, 330000) +
              dip(.64, .055, 250000) + dip(.82, .040, 150000);
  energy += (source == TrainingSource::Sim2 ? 18000.0 : 5000.0) *
            (std::sin(79.0 * t) + .4 * std::sin(151.0 * t));
  return static_cast<std::uint64_t>(std::max(0.0, std::round(energy)));
}

double TrainingAnalysis::interpolateEnergy(const std::vector<TrainingSample> &samples,
                                            double position) {
  if (samples.empty()) return 0.0;
  auto ordered = samples;
  std::sort(ordered.begin(), ordered.end(), [](const auto &a, const auto &b) {
    return a.positionMm < b.positionMm;
  });
  if (position <= ordered.front().positionMm) return static_cast<double>(ordered.front().energy);
  if (position >= ordered.back().positionMm) return static_cast<double>(ordered.back().energy);
  for (size_t i = 1; i < ordered.size(); ++i) if (position <= ordered[i].positionMm) {
    const auto &a = ordered[i - 1]; const auto &b = ordered[i];
    const double ratio = (position - a.positionMm) / (b.positionMm - a.positionMm);
    return a.energy + ratio * (static_cast<double>(b.energy) - a.energy);
  }
  return 0.0;
}

std::vector<double> TrainingAnalysis::findValleys(const std::vector<TrainingSample> &samples,
                                                    const ValleyDetectionSettings &settings) {
  if (samples.size() < 3) return {};
  std::vector<TrainingSample> ordered = samples;
  std::sort(ordered.begin(), ordered.end(), [](const auto &a, const auto &b) { return a.positionMm < b.positionMm; });
  const int radius = std::max(0, settings.smoothingPoints / 2);
  std::vector<double> smooth(ordered.size());
  for (size_t i = 0; i < ordered.size(); ++i) {
    double sum = 0; int count = 0;
    for (int j = std::max(0, int(i) - radius); j <= std::min(int(ordered.size()) - 1, int(i) + radius); ++j) { sum += ordered[j].energy; ++count; }
    smooth[i] = sum / count;
  }
  const auto minmax = std::minmax_element(smooth.begin(), smooth.end());
  const double threshold = (*minmax.second - *minmax.first) * std::max(0.0, settings.minimumProminenceFraction);
  struct Peak { double p, prominence; };
  std::vector<Peak> found;
  for (size_t i = 1; i + 1 < smooth.size(); ++i) {
    if (!(smooth[i] <= smooth[i-1] && smooth[i] < smooth[i+1])) continue;
    double left = smooth[i], right = smooth[i];
    for (size_t j = 0; j < i; ++j) left = std::max(left, smooth[j]);
    for (size_t j = i + 1; j < smooth.size(); ++j) right = std::max(right, smooth[j]);
    const double prominence = std::min(left, right) - smooth[i];
    if (prominence >= threshold) found.push_back({ordered[i].positionMm, prominence});
  }
  std::sort(found.begin(), found.end(), [](const Peak &a, const Peak &b) { return a.prominence > b.prominence; });
  std::vector<double> chosen;
  for (const auto &peak : found) {
    if (settings.maximumCount > 0 && static_cast<int>(chosen.size()) >= settings.maximumCount) break;
    if (std::none_of(chosen.begin(), chosen.end(), [&](double p) { return std::abs(p - peak.p) < settings.minimumDistanceMm; })) chosen.push_back(peak.p);
  }
  std::sort(chosen.begin(), chosen.end());
  return chosen;
}

Result<RodAimAssignment> TrainingAnalysis::assign(const FFConfig &config, const FFStatus &status,
                                                    ChannelId channelId, const std::vector<double> &candidates,
                                                    std::optional<RodId> excludedRod) {
  const int ci = static_cast<int>(channelId);
  if (ci < 0 || ci >= 3) return Result<RodAimAssignment>::failure(ErrorCode::InvalidTarget, "Invalid training channel");
  const auto &channel = config.channels[ci]; const auto &state = status.channels[ci];
  const auto trainingRodCount = static_cast<size_t>(std::count_if(
      channel.rods.begin(), channel.rods.end(),
      [&](const RodConfig &rod) { return isLayer2TrainingRod(channel, rod.id) && (!excludedRod || rod.id != *excludedRod); }));
  if (candidates.size() > trainingRodCount) return Result<RodAimAssignment>::failure(ErrorCode::InvalidTarget, "More candidates than available Layer 2 training Rods");
  if (candidates.empty()) return Result<RodAimAssignment>::success({{}, 0.0, "No candidate positions"});

  if (const auto ranksByRod = orderedLayer2Ranks(channel)) {
    std::vector<int> chain;
    for (size_t index = 0; index < ranksByRod->size(); ++index)
      if ((*ranksByRod)[index] >= 0) chain.push_back(static_cast<int>(index));
    std::sort(chain.begin(), chain.end(), [&](int a, int b) {
      return (*ranksByRod)[a] < (*ranksByRod)[b];
    });
    std::vector<int> candidateOrder(candidates.size());
    for (size_t index = 0; index < candidateOrder.size(); ++index)
      candidateOrder[index] = static_cast<int>(index);
    std::sort(candidateOrder.begin(), candidateOrder.end(),
              [&](int a, int b) { return candidates[a] < candidates[b]; });

    double bestCost = std::numeric_limits<double>::infinity();
    std::vector<int> bestEncoders, bestCandidateAtRank, chosenRanks;
    const auto clearanceBetween = [&](int leftIndex, int rightIndex,
                                      double &clearance) {
      const auto &left = channel.rods[leftIndex];
      const auto &right = channel.rods[rightIndex];
      const auto rule = std::find_if(
          channel.clearanceRules.begin(), channel.clearanceRules.end(),
          [&](const PairClearanceRule &entry) {
            return entry.layer == 2 &&
                   ((entry.first == left.id && entry.second == right.id) ||
                    (entry.second == left.id && entry.first == right.id));
          });
      if (rule == channel.clearanceRules.end()) return false;
      clearance = rule->minimumClearanceMm - kLayerClearanceToleranceMm;
      return true;
    };
    const auto tryPlan = [&]() {
      std::vector<int> candidateAtRank(chain.size(), -1);
      for (size_t index = 0; index < chosenRanks.size(); ++index)
        candidateAtRank[chosenRanks[index]] = candidateOrder[index];
      std::vector<int> encoders(chain.size());
      std::vector<double> actualMms(chain.size()), lowMms(chain.size());
      std::vector<bool> movableRanks(chain.size());
      double previousMm = 0.0;
      for (size_t rank = 0; rank < chain.size(); ++rank) {
        const int rodIndex = chain[rank];
        const auto &rod = channel.rods[rodIndex];
        if (rodIndex >= static_cast<int>(state.rods.size()) ||
            !state.rods[rodIndex].positionValid || std::abs(rod.p1) < 1e-12)
          return;
        const double low = std::min(optical(rod, rod.softMin), optical(rod, rod.softMax));
        const double high = std::max(optical(rod, rod.softMin), optical(rod, rod.softMax));
        lowMms[rank] = low;
        double required = low;
        if (rank > 0) {
          double clearance{};
          if (!clearanceBetween(chain[rank - 1], rodIndex, clearance)) return;
          required = std::max(required, previousMm + clearance);
        }
        const bool assigned = candidateAtRank[rank] >= 0;
        // Every member of this ordered Layer 2 chain may need to participate
        // in the safe layout. Candidate selection still excludes Rods that
        // also belong to another layer, but leaving such an endpoint fixed
        // can make an otherwise valid chain impossible to place.
        const bool movable = true;
        movableRanks[rank] = movable;
        const double currentMm = optical(rod, state.rods[rodIndex].actualPosition);
        double requestedMm = assigned ? candidates[candidateAtRank[rank]] : currentMm;
        if (!assigned && movable) requestedMm = std::max(requestedMm, required);
        if (!assigned && !movable && currentMm + 1e-9 < required) return;
        // A movable right-end Rod may reach its limit before its predecessor.
        // Keep it at the legal limit and let the backward pass move preceding
        // non-candidate Rods left to satisfy the complete chain.
        if (!assigned) requestedMm = std::min(requestedMm, high);
        int encoder = static_cast<int>(std::lround((requestedMm - rod.p0) / rod.p1));
        const int step = rod.p1 > 0.0 ? 1 : -1;
        while (legalEncoder(rod, encoder) && optical(rod, encoder) + 1e-9 < required)
          encoder += step;
        if (!legalEncoder(rod, encoder)) return;
        const double actualMm = optical(rod, encoder);
        if (actualMm + 1e-9 < required) return;
        encoders[rank] = encoder;
        previousMm = actualMm;
        actualMms[rank] = actualMm;
      }
      // Respect fixed right-end Rods too.  A training Rod at its previous
      // maximum must be allowed to retreat just enough for its right neighbor.
      for (int rank = static_cast<int>(chain.size()) - 2; rank >= 0; --rank) {
        double clearance{};
        if (!clearanceBetween(chain[rank], chain[rank + 1], clearance)) return;
        const double maximum = actualMms[rank + 1] - clearance;
        if (actualMms[rank] <= maximum + 1e-9) continue;
        if (candidateAtRank[rank] >= 0 || !movableRanks[rank]) return;
        const auto &rod = channel.rods[chain[rank]];
        int encoder = static_cast<int>(std::lround((maximum - rod.p0) / rod.p1));
        const int step = rod.p1 > 0.0 ? -1 : 1;
        while (legalEncoder(rod, encoder) && optical(rod, encoder) > maximum + 1e-9)
          encoder += step;
        if (!legalEncoder(rod, encoder)) return;
        const double actualMm = optical(rod, encoder);
        if (actualMm + 1e-9 < lowMms[rank]) return;
        encoders[rank] = encoder;
        actualMms[rank] = actualMm;
      }
      std::vector<std::pair<RodId, std::int32_t>> overrides;
      double cost = 0.0;
      for (size_t rank = 0; rank < chain.size(); ++rank) {
        if (rank > 0) {
          double clearance{};
          if (!clearanceBetween(chain[rank - 1], chain[rank], clearance) ||
              actualMms[rank] + 1e-9 < actualMms[rank - 1] + clearance)
            return;
        }
        const int rodIndex = chain[rank];
        overrides.push_back({channel.rods[rodIndex].id, encoders[rank]});
        cost += std::abs(double(encoders[rank] - state.rods[rodIndex].actualPosition));
      }
      if (!safeTopology(channel, state, overrides) || cost >= bestCost) return;
      bestCost = cost;
      bestEncoders = std::move(encoders);
      bestCandidateAtRank = std::move(candidateAtRank);
    };
    std::function<void(size_t, int)> chooseRanks = [&](size_t depth, int begin) {
      if (depth == candidateOrder.size()) { tryPlan(); return; }
      for (int rank = begin;
           rank <= static_cast<int>(chain.size() - (candidateOrder.size() - depth));
           ++rank) {
        const auto &rod = channel.rods[chain[rank]];
        if (!isLayer2TrainingRod(channel, rod.id) ||
            (excludedRod && rod.id == *excludedRod))
          continue;
        chosenRanks.push_back(rank);
        chooseRanks(depth + 1, rank + 1);
        chosenRanks.pop_back();
      }
    };
    chooseRanks(0, 0);
    if (std::isfinite(bestCost)) {
      RodAimAssignment result;
      result.totalEncoderTravel = bestCost;
      result.message = "Minimum-travel assignment with Layer 2 evasion is valid";
      for (size_t rank = 0; rank < chain.size(); ++rank) {
        const int rodIndex = chain[rank];
        const auto &rod = channel.rods[rodIndex];
        const int candidateIndex = bestCandidateAtRank[rank];
        const bool assigned = candidateIndex >= 0;
        result.aims.push_back({{channelId, rod.id},
                               assigned ? candidates[candidateIndex]
                                        : optical(rod, state.rods[rodIndex].actualPosition),
                               optical(rod, bestEncoders[rank]), bestEncoders[rank],
                               state.rods[rodIndex].actualPosition, assigned});
      }
      return Result<RodAimAssignment>::success(std::move(result));
    }
  }
  struct Option { int rod; std::int32_t encoder; double cost; };
  std::vector<std::vector<Option>> options(candidates.size());
  for (size_t c = 0; c < candidates.size(); ++c) for (size_t r = 0; r < channel.rods.size(); ++r) {
    const auto &rod = channel.rods[r]; if (!isLayer2TrainingRod(channel, rod.id) || (excludedRod && rod.id == *excludedRod) || std::abs(rod.p1) < 1e-12 || r >= state.rods.size() || !state.rods[r].positionValid) continue;
    const auto encoder = static_cast<std::int32_t>(std::lround((candidates[c] - rod.p0) / rod.p1));
    if (legalEncoder(rod, encoder)) options[c].push_back({int(r), encoder, std::abs(double(encoder - state.rods[r].actualPosition))});
  }
  for (const auto &list : options) if (list.empty()) return Result<RodAimAssignment>::failure(ErrorCode::LimitViolation, "A candidate is outside every Rod's legal range");
  std::vector<int> order(candidates.size()); for (size_t i = 0; i < order.size(); ++i) order[i] = int(i);
  const auto orderedRanks = orderedLayer2Ranks(channel);
  if (orderedRanks) {
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return candidates[a] < candidates[b];
    });
    for (auto &list : options)
      std::sort(list.begin(), list.end(), [](const Option &a, const Option &b) {
        return a.cost < b.cost;
      });
  } else {
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return options[a].size() < options[b].size();
    });
  }
  double best = std::numeric_limits<double>::infinity(); std::vector<Option> bestOptions, selected(candidates.size()); std::vector<bool> used(channel.rods.size());
  std::function<void(size_t, double, int)> search = [&](size_t depth, double cost,
                                                          int previousRank) {
    if (cost >= best) return;
    if (depth == order.size()) {
      std::vector<std::pair<RodId, std::int32_t>> overrides;
      for (size_t c = 0; c < selected.size(); ++c) overrides.push_back({channel.rods[selected[c].rod].id, selected[c].encoder});
      if (safeTopology(channel, state, overrides)) { best = cost; bestOptions = selected; }
      return;
    }
    const int candidate = order[depth];
    for (const auto &option : options[candidate]) if (!used[option.rod]) {
      const int rank = orderedRanks ? (*orderedRanks)[option.rod] : -1;
      if (orderedRanks && (rank < 0 || rank <= previousRank)) continue;
      used[option.rod] = true; selected[candidate] = option;
      search(depth + 1, cost + option.cost, orderedRanks ? rank : previousRank);
      used[option.rod] = false;
    }
  };
  search(0, 0.0, -1);
  if (!std::isfinite(best)) return Result<RodAimAssignment>::failure(ErrorCode::SafetyViolation, "No one-to-one assignment satisfies layer clearance");
  RodAimAssignment result; result.totalEncoderTravel = best; result.message = "Minimum-travel assignment is valid";
  for (size_t c = 0; c < candidates.size(); ++c) { const auto &option = bestOptions[c]; const auto &rod = channel.rods[option.rod]; result.aims.push_back({{channelId, rod.id}, candidates[c], optical(rod, option.encoder), option.encoder, state.rods[option.rod].actualPosition, true}); }
  std::sort(result.aims.begin(), result.aims.end(), [](const TrainingAim &a, const TrainingAim &b) { return a.address.rod < b.address.rod; });
  return Result<RodAimAssignment>::success(std::move(result));
}

Result<void> TrainingAnalysis::validateAssignment(
    const FFConfig &config, const FFStatus &status, ChannelId channelId,
    const std::vector<double> &candidates, std::optional<RodId> excludedRod) {
  const auto assignment = assign(config, status, channelId, candidates,
                                 excludedRod);
  if (!assignment)
    return Result<void>::failure(assignment.error, assignment.message);
  return Result<void>::success();
}

TrainingDragInterval TrainingAnalysis::dragInterval(
    const std::vector<double> &candidates, std::size_t movingIndex,
    double anchorMm, double scanStartMm, double scanEndMm,
    double windowWidthMm) {
  TrainingDragInterval interval;
  if (movingIndex >= candidates.size() || !std::isfinite(anchorMm) ||
      !std::isfinite(scanStartMm) || !std::isfinite(scanEndMm) ||
      !std::isfinite(windowWidthMm) || windowWidthMm <= 0.0)
    return interval;
  const double halfWidth = windowWidthMm / 2.0;
  interval.minimumMm = std::min(scanStartMm, scanEndMm) + halfWidth;
  interval.maximumMm = std::max(scanStartMm, scanEndMm) - halfWidth;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (index == movingIndex) continue;
    if (candidates[index] < anchorMm)
      interval.minimumMm = std::max(interval.minimumMm,
                                    candidates[index] + windowWidthMm);
    else if (candidates[index] > anchorMm)
      interval.maximumMm = std::min(interval.maximumMm,
                                    candidates[index] - windowWidthMm);
  }
  interval.valid = interval.minimumMm <= interval.maximumMm;
  return interval;
}

Result<void> TrainingAnalysis::saveSession(const std::string &path, const TrainingSession &session) {
  if (!validNumbers(session))
    return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                 "Training decimals are invalid at 0.001 precision");
  std::ofstream out(path,std::ios::trunc);
  if (!out) return Result<void>::failure(ErrorCode::BackendFailure,"Could not write training session");
  out << "{\n  \"schema\": \"" << TrainingSession::Schema << "\",\n"
      << "  \"machine_id\": \"" << escapeJson(session.machineId) << "\",\n"
      << "  \"config_version\": \"" << escapeJson(session.configVersion) << "\",\n"
      << "  \"created_unix_seconds\": " << session.createdUnixSeconds << ",\n"
      << "  \"channel\": " << int(session.channel) << ",\n";
  writeSessionFields(out,session,"  ");
  out << "\n}\n";
  out.flush();
  return out ? Result<void>::success() :
               Result<void>::failure(ErrorCode::BackendFailure,"Could not finish training session");
}

static Result<TrainingSession> parseSessionText(const std::string& text) {
  if (stringField(text, "schema") != TrainingSession::Schema) return Result<TrainingSession>::failure(ErrorCode::InvalidConfiguration, "Unsupported training-session schema");
  TrainingSession s; const auto source=stringField(text,"source"); s.source=source=="Sim2"?TrainingSource::Sim2:source=="Real"?TrainingSource::Real:source=="Loaded"?TrainingSource::Loaded:TrainingSource::Sim1;
  s.machineId=stringField(text,"machine_id"); s.configVersion=stringField(text,"config_version"); s.occlusionSummary=stringField(text,"occlusion_summary"); s.createdUnixSeconds=static_cast<std::int64_t>(numberField(text,"created_unix_seconds")); s.channel=static_cast<ChannelId>(int(numberField(text,"channel"))); s.trainingRod=static_cast<RodId>(int(numberField(text,"training_rod"))); s.startMm=numberField(text,"start_mm"); s.endMm=numberField(text,"end_mm"); s.stepMm=numberField(text,"step_mm"); s.complete=trainingBoolField(text,"complete");
  const auto valleyPos=text.find("\"valley\""); const auto valley=valleyPos==std::string::npos?std::string{}:text.substr(valleyPos, text.find('}', valleyPos)-valleyPos+1); s.valleySettings.smoothingPoints=int(numberField(valley,"smoothing",5)); s.valleySettings.minimumProminenceFraction=numberField(valley,"prominence",.1); s.valleySettings.minimumDistanceMm=numberField(valley,"distance_mm",.5); s.valleySettings.maximumCount=int(numberField(valley,"maximum"));
  for (const auto &item:objects(arrayBody(text,"samples"))) { const double p=numberField(item,"position_mm",std::numeric_limits<double>::quiet_NaN()); std::uint64_t energy{}; if(!std::isfinite(p)||!unsignedField(item,"energy",energy)) return Result<TrainingSession>::failure(ErrorCode::InvalidConfiguration,"Invalid training sample"); s.samples.push_back({p,energy}); }
  const std::regex number("-?[0-9]+(?:\\.[0-9]+)?"); const auto candidateBody=arrayBody(text,"candidates_mm"); for(std::sregex_iterator it(candidateBody.begin(),candidateBody.end(),number),end;it!=end;++it){ const double value=std::stod(it->str()); if(!std::isfinite(value)) return Result<TrainingSession>::failure(ErrorCode::InvalidConfiguration,"Invalid candidate"); s.candidatesMm.push_back(value); }
  std::set<int> seen; for(const auto &item:objects(arrayBody(text,"aims"))) { const int rod=int(numberField(item,"rod",-1)); if(rod<0||rod>255||!seen.insert(rod).second) return Result<TrainingSession>::failure(ErrorCode::InvalidConfiguration,"Invalid or duplicate aim Rod"); s.aims.push_back({{s.channel,static_cast<RodId>(rod)},numberField(item,"requested_mm"),numberField(item,"quantized_mm"),int(numberField(item,"encoder_aim")),int(numberField(item,"actual_encoder")),trainingBoolField(item,"assigned")}); }
  std::set<std::pair<int, int>> stateAddresses;
  for (const auto &item : objects(arrayBody(text, "rod_states"))) {
    const int channel = int(numberField(item, "channel", -1));
    const int rod = int(numberField(item, "rod", -1));
    const double actual = numberField(item, "actual_encoder",
                                      std::numeric_limits<double>::quiet_NaN());
    const double target = numberField(item, "target_encoder",
                                      std::numeric_limits<double>::quiet_NaN());
    if (channel < 0 || channel > 2 || rod < 0 || rod > 255 ||
        !std::isfinite(actual) || !std::isfinite(target) ||
        actual < std::numeric_limits<std::int32_t>::min() ||
        actual > std::numeric_limits<std::int32_t>::max() ||
        target < std::numeric_limits<std::int32_t>::min() ||
        target > std::numeric_limits<std::int32_t>::max() ||
        !stateAddresses.insert({channel, rod}).second)
      return Result<TrainingSession>::failure(
          ErrorCode::InvalidConfiguration,
          "Invalid or duplicate saved Rod state");
    s.rodStates.push_back(
        {{static_cast<ChannelId>(channel), static_cast<RodId>(rod)},
         static_cast<std::int32_t>(actual), static_cast<std::int32_t>(target)});
  }
  std::set<std::pair<int, int>> targetAddresses;
  for (const auto &item : objects(arrayBody(text, "execution_targets"))) {
    const int channel = int(numberField(item, "channel", -1));
    const int rod = int(numberField(item, "rod", -1));
    const double position = numberField(item, "position",
                                        std::numeric_limits<double>::quiet_NaN());
    if (channel < 0 || channel > 2 || rod < 0 || rod > 255 ||
        !std::isfinite(position) || position < std::numeric_limits<std::int32_t>::min() ||
        position > std::numeric_limits<std::int32_t>::max() ||
        !targetAddresses.insert({channel, rod}).second)
      return Result<TrainingSession>::failure(
          ErrorCode::InvalidConfiguration,
          "Invalid or duplicate executable Rod target");
    s.executionTargets.push_back(
        {{static_cast<ChannelId>(channel), static_cast<RodId>(rod)},
         static_cast<std::int32_t>(position)});
  }
  if (s.stepMm <= 0 || !std::isfinite(s.startMm) || !std::isfinite(s.endMm)) return Result<TrainingSession>::failure(ErrorCode::InvalidConfiguration,"Invalid scan settings");
  return Result<TrainingSession>::success(std::move(s));
}
Result<TrainingSession> TrainingAnalysis::loadSession(const std::string &path) {
  std::ifstream in(path);
  if (!in) return Result<TrainingSession>::failure(ErrorCode::BackendFailure,
                                                   "Could not read training session");
  const std::string text((std::istreambuf_iterator<char>(in)), {});
  return parseSessionText(text);
}
Result<void> TrainingAnalysis::saveArchive(const std::string& path,
                                            const TrainingArchive& archive) {
  if (archive.selectedChannel<0 || archive.selectedChannel>2 ||
      archive.machineId.empty() || archive.configVersion.empty())
    return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                 "Invalid training archive header");
  for (int ch=0;ch<3;++ch)
    if (archive.channels[ch] &&
        (int(archive.channels[ch]->channel)!=ch || !validNumbers(*archive.channels[ch])))
      return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                   "Invalid channel data at 0.001 precision");
  std::ofstream out(path,std::ios::trunc);
  if (!out) return Result<void>::failure(ErrorCode::BackendFailure,
                                         "Could not write training archive");
  out << "{\n  \"schema\": \"FF-TRAIN-2\",\n"
      << "  \"machine_id\": \"" << escapeJson(archive.machineId) << "\",\n"
      << "  \"config_version\": \"" << escapeJson(archive.configVersion) << "\",\n"
      << "  \"created_unix_seconds\": " << archive.createdUnixSeconds << ",\n"
      << "  \"selected_channel\": " << archive.selectedChannel << ",\n"
      << "  \"channels\": [\n";
  for (int ch=0;ch<3;++ch) {
    out << "    {\"channel\": " << ch << ", \"training\": ";
    if (archive.channels[ch]) {
      out << "{\n";
      writeSessionFields(out,*archive.channels[ch],"      ");
      out << "\n    }";
    } else out << "{}";
    out << "}" << (ch<2?",":"") << '\n';
  }
  out << "  ]\n}\n";
  out.flush();
  return out ? Result<void>::success() :
               Result<void>::failure(ErrorCode::BackendFailure,
                                     "Could not finish training archive");
}
Result<TrainingArchive> TrainingAnalysis::loadArchive(const std::string& path) {
  std::ifstream in(path);
  if (!in) return Result<TrainingArchive>::failure(ErrorCode::BackendFailure,
                                                   "Could not read training archive");
  const std::string text((std::istreambuf_iterator<char>(in)),{});
  const auto schema=stringField(text,"schema");
  TrainingArchive archive;
  if (schema==TrainingSession::Schema) {
    auto single=parseSessionText(text);
    if (!single) return Result<TrainingArchive>::failure(single.error,single.message);
    const int ch=int(single.value.channel);
    if (ch<0 || ch>2)
      return Result<TrainingArchive>::failure(ErrorCode::InvalidConfiguration,
                                              "Invalid legacy training channel");
    archive.machineId=single.value.machineId;
    archive.configVersion=single.value.configVersion;
    archive.createdUnixSeconds=single.value.createdUnixSeconds;
    archive.selectedChannel=ch;
    archive.channels[ch]=std::move(single.value);
    return Result<TrainingArchive>::success(std::move(archive));
  }
  if (schema!="FF-TRAIN-2")
    return Result<TrainingArchive>::failure(ErrorCode::InvalidConfiguration,
                                            "Unsupported training archive schema");
  archive.machineId=stringField(text,"machine_id");
  archive.configVersion=stringField(text,"config_version");
  archive.createdUnixSeconds=static_cast<std::int64_t>(numberField(text,"created_unix_seconds"));
  archive.selectedChannel=int(numberField(text,"selected_channel",-1));
  if (archive.machineId.empty() || archive.configVersion.empty() ||
      archive.selectedChannel<0 || archive.selectedChannel>2)
    return Result<TrainingArchive>::failure(ErrorCode::InvalidConfiguration,
                                            "Invalid training archive header");
  const auto channelObjects=objects(arrayBody(text,"channels"));
  if (channelObjects.size()!=3)
    return Result<TrainingArchive>::failure(ErrorCode::InvalidConfiguration,
                                            "Training archive must list three channels");
  std::set<int> seen;
  const std::regex emptyTraining("\"training\"\\s*:\\s*\\{\\s*\\}");
  for (const auto& entry:channelObjects) {
    const int ch=int(numberField(entry,"channel",-1));
    if (ch<0 || ch>2 || !seen.insert(ch).second ||
        entry.find("\"training\"")==std::string::npos)
      return Result<TrainingArchive>::failure(ErrorCode::InvalidConfiguration,
                                              "Invalid training channel entry");
    if (std::regex_search(entry,emptyTraining)) continue;
    if (stringField(entry,"source").empty())
      return Result<TrainingArchive>::failure(ErrorCode::InvalidConfiguration,
                                              "Incomplete training channel entry");
    std::ostringstream wrapped;
    wrapped << "{\"schema\":\"FF-TRAIN-1\",\"machine_id\":\""
            << escapeJson(archive.machineId) << "\",\"config_version\":\""
            << escapeJson(archive.configVersion) << "\",\"created_unix_seconds\":"
            << archive.createdUnixSeconds << ',' << entry.substr(1);
    auto parsed=parseSessionText(wrapped.str());
    if (!parsed) return Result<TrainingArchive>::failure(parsed.error,parsed.message);
    if (int(parsed.value.channel)!=ch)
      return Result<TrainingArchive>::failure(ErrorCode::InvalidConfiguration,
                                              "Training channel identity mismatch");
    archive.channels[ch]=std::move(parsed.value);
  }
  return Result<TrainingArchive>::success(std::move(archive));
}
} // namespace ff
