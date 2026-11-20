#include "ff_gui.h"
#include <cstdlib>
#include "ui_helpers.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <set>
#include <sstream>
namespace fs = std::filesystem;
namespace ff_gui {
MainUi::MainUi() {
  auto root =
      fs::path(std::getenv("PROGRAMDATA") ? std::getenv("PROGRAMDATA") : ".") /
      "RSIC" / "FF";
  fs::create_directories(root);
  configPath_ = (root / "ff_config.json").string();
  iniPath_ = (root / "ff_gui.ini").string();
  presetDirectory_ = (root / "position_presets").string();
  fs::create_directories(presetDirectory_);
  trainingDirectory_ = (root / "training_sessions").string();
  fs::create_directories(trainingDirectory_);
  for (int channel = 0; channel < 3; ++channel)
    trainingSessions_[channel].channel = static_cast<ff::ChannelId>(channel);
  training_ = trainingSessions_[0];
  refreshPresetSlots();
  loadUiSettings();
  scanSerialPorts();
  ensureConfig();
}

void MainUi::refreshPresetSlots() {
  presetSlots_.clear();
  // JSON is the canonical slot format.  Continue discovering legacy CSV
  // files so existing installations can load and migrate them by saving the
  // same slot again.
  const std::regex fileNamePattern(R"(^slot_([0-9]+)\.(?:json|csv)$)",
                                   std::regex::icase);
  std::error_code error;
  for (const auto &entry : fs::directory_iterator(presetDirectory_, error)) {
    if (error || !entry.is_regular_file())
      continue;
    std::smatch match;
    const auto name = entry.path().filename().string();
    if (!std::regex_match(name, match, fileNamePattern))
      continue;
    try {
      presetSlots_.push_back(std::stoi(match[1].str()));
    } catch (const std::exception &) {
      // Ignore file names outside the supported slot-number range.
    }
  }
  std::sort(presetSlots_.begin(), presetSlots_.end());
  presetSlots_.erase(std::unique(presetSlots_.begin(), presetSlots_.end()),
                     presetSlots_.end());
}

bool MainUi::hasPresetSlot(int slot) const {
  return std::find(presetSlots_.begin(), presetSlots_.end(), slot) !=
         presetSlots_.end();
}

std::string MainUi::presetPath(int slot) const {
  return (fs::path(presetDirectory_) / ("slot_" + std::to_string(slot) +
                                        ".json"))
      .string();
}

bool MainUi::savePreset(int slot,
                        const std::vector<ff::RodTarget> &targets) {
  const auto config = service_.config();
  if (!config) return false;
  ff::PositionFileData file{config->machineId, config->configVersion,
      slot == 0 ? "Main UI Slot Initial" : "Main UI Slot" + std::to_string(slot),
      targets};
  return static_cast<bool>(ff::GuiPositionFiles::save(service_, presetPath(slot), file));
}

ff::Result<ff::PositionFileData> MainUi::loadPreset(int slot) {
  const auto config = service_.config();
  if (!config) return ff::Result<ff::PositionFileData>::failure(ff::ErrorCode::InvalidConfiguration,"Configuration unavailable");
  const auto path = fs::exists(presetPath(slot)) ? presetPath(slot) :
      (fs::path(presetDirectory_) / ("slot_" + std::to_string(slot) + ".csv")).string();
  auto file = ff::GuiPositionFiles::load(service_, path, *config);
  if (!file) return file;
  auto valid=ff::GuiPositionFiles::validate(file.value,*config,service_.enabledChannels());
  if (!valid) return ff::Result<ff::PositionFileData>::failure(valid.error,valid.message);
  return file;
}

void MainUi::loadUiSettings() {
  std::ifstream input(iniPath_);
  std::string line;
  bool perChannelTrainingSettings = false;
  bool hasExplicitMask = false;
  while (std::getline(input, line)) {
    const auto split = line.find('=');
    if (split == std::string::npos)
      continue;
    const auto key = line.substr(0, split);
    const auto value = line.substr(split + 1);
    if (key == "mode")
      simulation_ = value != "real";
    else if (key == "config_path" && !value.empty() && fs::exists(value))
      configPath_ = value;
    else if (key == "enabled_channels") {
      try {
        enabledChannelCount_ = std::clamp(std::stoi(value), 1, 3);
        enabledChannels_={enabledChannelCount_>=1,enabledChannelCount_>=2,enabledChannelCount_>=3};
      } catch (const std::exception &) {
        // Ignore malformed user settings and preserve the safe default.
      }
    } else if (key == "enabled_mask" && value.size()==3 &&
               value.find_first_not_of("01")==std::string::npos && value!="000") {
      for (int i=0;i<3;++i) enabledChannels_[i]=value[i]=='1';
      enabledChannelCount_=static_cast<int>(std::count(enabledChannels_.begin(),enabledChannels_.end(),true));
      hasExplicitMask=true;
    } else if (key.rfind("serial_ch", 0) == 0) {
      try {
        const int channel = std::stoi(key.substr(9));
        if (channel >= 0 && channel < 3)
          serialPorts_[channel] = value;
      } catch (const std::exception &) {
        // Ignore malformed user settings and preserve the safe defaults.
      }
    } else if (key.rfind("training_ch", 0) == 0 && key.size() > 13 &&
               key[11] >= '0' && key[11] <= '2' && key[12] == '_') {
      auto &session = trainingSessions_[key[11] - '0'];
      const auto field = key.substr(13);
      perChannelTrainingSettings = true;
      try {
        if (field == "source") {
          const int source = std::clamp(std::stoi(value), 0, 3);
          session.source = source == static_cast<int>(ff::TrainingSource::Loaded)
                               ? ff::TrainingSource::Sim1
                               : static_cast<ff::TrainingSource>(source);
        } else if (field == "start_mm") session.startMm = std::stod(value);
        else if (field == "end_mm") session.endMm = std::stod(value);
        else if (field == "step_mm") session.stepMm = std::stod(value);
        else if (field == "smoothing") session.valleySettings.smoothingPoints = std::stoi(value);
        else if (field == "prominence") session.valleySettings.minimumProminenceFraction = std::stod(value);
        else if (field == "distance_mm") session.valleySettings.minimumDistanceMm = std::stod(value);
        else if (field == "max_valleys") session.valleySettings.maximumCount = std::stoi(value);
      } catch (const std::exception &) {}
    } else if (key == "training_source") {
      try {
        const int source = std::clamp(std::stoi(value), 0, 3);
        // A loaded session has no durable path in the INI.  Start from Sim1
        // instead and let the user explicitly load that session again.
        training_.source = source == static_cast<int>(ff::TrainingSource::Loaded)
                               ? ff::TrainingSource::Sim1
                               : static_cast<ff::TrainingSource>(source);
      } catch (const std::exception &) {}
    } else if (key == "training_channel") {
      try { training_.channel = static_cast<ff::ChannelId>(std::clamp(std::stoi(value), 0, 2)); }
      catch (const std::exception &) {}
    } else if (key == "training_start_mm") {
      try { training_.startMm = std::stod(value); } catch (const std::exception &) {}
    } else if (key == "training_end_mm") {
      try { training_.endMm = std::stod(value); } catch (const std::exception &) {}
    } else if (key == "training_step_mm") {
      try { training_.stepMm = std::stod(value); } catch (const std::exception &) {}
    } else if (key == "training_settle_ms") {
      try { trainingSettleMs_ = std::clamp(std::stoi(value), 0, 500); }
      catch (const std::exception &) {}
    } else if (key == "training_sample_count") {
      try { trainingSampleCount_ = std::clamp(std::stoi(value), 2, 100000); }
      catch (const std::exception &) {}
    } else if (key == "training_smoothing") {
      try { training_.valleySettings.smoothingPoints = std::stoi(value); } catch (const std::exception &) {}
    } else if (key == "training_prominence") {
      try { training_.valleySettings.minimumProminenceFraction = std::stod(value); } catch (const std::exception &) {}
    } else if (key == "training_distance_mm") {
      try { training_.valleySettings.minimumDistanceMm = std::stod(value); } catch (const std::exception &) {}
    } else if (key == "training_max_valleys") {
      try { training_.valleySettings.maximumCount = std::stoi(value); } catch (const std::exception &) {}
    }
  }
  const auto normalizeTraining = [](ff::TrainingSession &session) {
    if (!std::isfinite(session.startMm)) session.startMm = 0.0;
    if (!std::isfinite(session.endMm)) session.endMm = 0.0;
    if (!std::isfinite(session.stepMm) || session.stepMm <= 0) session.stepMm = 0.1;
    session.valleySettings.smoothingPoints = std::max(1, session.valleySettings.smoothingPoints);
    if (!std::isfinite(session.valleySettings.minimumProminenceFraction) ||
        session.valleySettings.minimumProminenceFraction < 0)
      session.valleySettings.minimumProminenceFraction = 0.10;
    if (!std::isfinite(session.valleySettings.minimumDistanceMm) ||
        session.valleySettings.minimumDistanceMm < 0)
      session.valleySettings.minimumDistanceMm = 0.5;
    session.valleySettings.maximumCount = std::max(0, session.valleySettings.maximumCount);
  };
  if (!perChannelTrainingSettings) {
    normalizeTraining(training_);
    const int selected = std::clamp(static_cast<int>(training_.channel), 0, 2);
    for (int channel = 0; channel < 3; ++channel) {
      trainingSessions_[channel] = training_;
      trainingSessions_[channel].channel = static_cast<ff::ChannelId>(channel);
    }
    training_ = trainingSessions_[selected];
  } else {
    for (int channel = 0; channel < 3; ++channel) {
      trainingSessions_[channel].channel = static_cast<ff::ChannelId>(channel);
      normalizeTraining(trainingSessions_[channel]);
    }
    training_ = trainingSessions_[std::clamp(static_cast<int>(training_.channel), 0, 2)];
  }
  if (!hasExplicitMask) {
    // Legacy installations stored only a channel count. Their old default of
    // three must not silently enable channels with missing mechanical rules.
    enabledChannels_={true,false,false};
    enabledChannelCount_=1;
    activeChannel_=0;
    saveUiSettings();
  }
}

void MainUi::saveUiSettings() const {
  std::ofstream output(iniPath_, std::ios::trunc);
  if (!output)
    return;
  output << "mode=" << (simulation_ ? "simulation" : "real") << "\n";
  output << "enabled_channels=" << enabledChannelCount_ << "\n";
  output << "enabled_mask=" << (enabledChannels_[0]?'1':'0')
         << (enabledChannels_[1]?'1':'0') << (enabledChannels_[2]?'1':'0') << "\n";
  output << "config_path=" << configPath_ << "\n";
  for (int i = 0; i < 3; ++i)
    output << "serial_ch" << i << "=" << serialPorts_[i] << "\n";
  output << std::setprecision(17);
  auto sessions = trainingSessions_;
  sessions[std::clamp(static_cast<int>(training_.channel), 0, 2)] = training_;
  output << "training_source=" << static_cast<int>(training_.source) << "\n";
  output << "training_channel=" << static_cast<int>(training_.channel) << "\n";
  output << "training_start_mm=" << training_.startMm << "\n";
  output << "training_end_mm=" << training_.endMm << "\n";
  output << "training_step_mm=" << training_.stepMm << "\n";
  output << "training_settle_ms=" << trainingSettleMs_ << "\n";
  output << "training_sample_count=" << trainingSampleCount_ << "\n";
  output << "training_smoothing=" << training_.valleySettings.smoothingPoints << "\n";
  output << "training_prominence="
         << training_.valleySettings.minimumProminenceFraction << "\n";
  output << "training_distance_mm="
         << training_.valleySettings.minimumDistanceMm << "\n";
  output << "training_max_valleys="
         << training_.valleySettings.maximumCount << "\n";
  for (int channel = 0; channel < 3; ++channel) {
    const auto &session = sessions[channel];
    const std::string key = "training_ch" + std::to_string(channel) + "_";
    output << key << "source=" << static_cast<int>(session.source) << "\n";
    output << key << "start_mm=" << session.startMm << "\n";
    output << key << "end_mm=" << session.endMm << "\n";
    output << key << "step_mm=" << session.stepMm << "\n";
    output << key << "smoothing=" << session.valleySettings.smoothingPoints << "\n";
    output << key << "prominence=" << session.valleySettings.minimumProminenceFraction << "\n";
    output << key << "distance_mm=" << session.valleySettings.minimumDistanceMm << "\n";
    output << key << "max_valleys=" << session.valleySettings.maximumCount << "\n";
  }
}

void MainUi::scanSerialPorts() {
  availablePorts_.clear();
  wchar_t deviceName[256]{};
  for (int port = 1; port <= 256; ++port) {
    const auto portName = L"COM" + std::to_wstring(port);
    const auto size =
        static_cast<DWORD>(sizeof(deviceName) / sizeof(deviceName[0]));
    if (QueryDosDeviceW(portName.c_str(), deviceName, size) != 0)
      availablePorts_.push_back("COM" + std::to_string(port));
  }
}

void MainUi::ensureConfig() {
  if (!fs::exists(configPath_)) {
    wchar_t executable[MAX_PATH]{};
    const auto length=GetModuleFileNameW(nullptr,executable,MAX_PATH);
    if (!length || length>=MAX_PATH) {
      configLoaded_=false;
      notice_="Could not locate the bundled FF configuration.";
      return;
    }
    const auto executableDirectory=fs::path(executable).parent_path();
    const std::array<fs::path,2> sources{
        executableDirectory/"ff_config.default.json",
        executableDirectory.parent_path().parent_path()/"config"/"defaults"/"ff_config.json"};
    std::string error="Bundled FF configuration is missing or invalid.";
    for (const auto& source:sources) {
      if (!fs::is_regular_file(source)) continue;
      auto checked=ff::FFConfigIO::load(source.string());
      if (!checked) { error="Bundled FF configuration rejected: "+checked.message; continue; }
      std::error_code copyError;
      fs::create_directories(fs::path(configPath_).parent_path(),copyError);
      if (!copyError) fs::copy_file(source,configPath_,fs::copy_options::none,copyError);
      if (!copyError) break;
      error="Could not install FF configuration: "+copyError.message();
    }
    if (!fs::is_regular_file(configPath_)) {
      configLoaded_=false;
      notice_=error;
      return;
    }
  }
  auto x = service_.loadConfig(configPath_, simulation_, serialPorts_,
                               enabledChannels_);
  configLoaded_ = static_cast<bool>(x);
  targetsInitialized_ = false;
  if (!x)
    notice_ = x.message;
}

} // namespace ff_gui

