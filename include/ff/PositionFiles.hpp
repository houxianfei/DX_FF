#pragma once
#include "ff/FF.hpp"

namespace ff {
struct PositionFileData {
  std::string machineId, configVersion, summary;
  std::vector<RodTarget> targets;
};
class PositionFiles {
public:
  static Result<PositionFileData> load(const std::string& path, const FFConfig& current);
  static Result<void> save(const std::string& path, const PositionFileData& file);
  static Result<void> validate(const PositionFileData& file, const FFConfig& current,
                               int activeChannelCount);
  static Result<void> execute(FFApplicationService& service, const PositionFileData& file,
                              bool waitForStable);
};
} // namespace ff
