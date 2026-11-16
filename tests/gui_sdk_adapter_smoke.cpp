#include "gui_driver.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  ff::GuiDriver gui;
  auto loaded=gui.loadConfig(argv[1],true,{},3);
  if (!loaded) { std::cerr << loaded.message; return 3; }
  auto connected=gui.connect();
  if (!connected) { std::cerr << connected.message; return 4; }
  auto info=gui.sdk().machine_info();
  if (info.activeChannels!=3 || info.channels[0].rods.empty()) return 5;
  auto current=gui.sdk().m_current_positions.front();
  if (!gui.refreshRod({ff::ChannelId::CH1,static_cast<ff::RodId>(current.address.rod)})) return 6;
  if (!gui.setRodTarget({{ff::ChannelId::CH1,static_cast<ff::RodId>(current.address.rod)},
                         current.encoder})) return 7;
  auto invalid=gui.sdk().send_rod({0,current.address.rod},-100000);
  if (invalid || invalid.code!=FF::ErrorCode::InvalidTarget) return 8;
  auto presetLoad=gui.sdk().load_positions(argv[2]);
  if (!presetLoad) { std::cerr << presetLoad.result.message; return 9; }
  auto init=gui.sdk().move_rods(presetLoad.value.targets);
  if (!init) { std::cerr << init.message; return 9; }
  auto start=gui.sdk().m_current_positions.front();
  auto sent=gui.sdk().send_rod(start.address,start.encoder-1);
  if (!sent || !gui.sdk().refresh_rod(start.address)) return 16;
  if (gui.sdk().m_current_positions.front().encoder!=start.encoder-1) return 17;
  if (!gui.sdk().stop()) return 18;
  if (!gui.sdk().send_rod(start.address,start.encoder)) return 19;
  if (!gui.sdk().refresh_rod(start.address)) return 20;
  const auto path=std::filesystem::temp_directory_path()/"ff_gui_sdk_adapter_slot.json";
  auto saved=gui.sdk().record_positions(path.string(),"adapter smoke");
  if (!saved) { std::cerr << saved.message; return 10; }
  auto preset=ff::GuiPositionFiles::load(gui,path.string(),*gui.config());
  if (!preset || preset.value.summary!="adapter smoke") return 11;
  auto savedTargets = preset.value;
  auto& firstTarget = savedTargets.targets.front();
  const auto firstRod = gui.rodConfig(firstTarget.address);
  if (!firstRod) return 25;
  firstTarget.position += firstTarget.position < firstRod->softMax ? 1 : -1;
  const auto presetPath = std::filesystem::temp_directory_path()/"ff_gui_explicit_targets.json";
  const auto beforeSave = gui.sdk().m_current_positions.front().encoder;
  if (!ff::GuiPositionFiles::save(gui,presetPath.string(),savedTargets)) return 26;
  { std::ifstream input(presetPath);
    const std::string text((std::istreambuf_iterator<char>(input)),{});
    if (text.find("\"schema\": \"FF-POS-1\"")==std::string::npos ||
        text.find("\"samples\"")!=std::string::npos ||
        text.find("\"channel\": 2")==std::string::npos) return 30;
  }
  auto restoredTargets = ff::GuiPositionFiles::load(gui,presetPath.string(),*gui.config());
  if (!restoredTargets || restoredTargets.value.targets.front().position != firstTarget.position)
    return 27;
  if (!gui.refresh()) return 29;
  if (gui.sdk().m_current_positions.front().encoder != beforeSave) return 28;
  if (!ff::GuiPositionFiles::execute(gui,preset.value,false)) return 12;
  FF::TrainingSession session;
  session.channel=0; session.trainingRod=current.address.rod;
  session.machineId=gui.sdk().m_config.machineId;
  session.configVersion=gui.sdk().m_config.configVersion;
  session.samples={{0,100},{1,50},{2,100}};
  session.aims.push_back({{0,current.address.rod},1,1,current.encoder,current.encoder,true});
  const auto sessionPath=std::filesystem::temp_directory_path()/"ff_gui_sdk_adapter_session.json";
  if (!gui.sdk().save_training_session(sessionPath.string(),session)) return 13;
  auto restored=gui.sdk().load_training_session(sessionPath.string());
  if (!restored || restored.value.samples.size()!=3 ||
      restored.value.aims.front().actualEncoder!=current.encoder) return 14;
  ff::TrainingSession guiSession;
  guiSession.channel=ff::ChannelId::CH1;
  guiSession.trainingRod=static_cast<ff::RodId>(current.address.rod);
  guiSession.machineId=gui.sdk().m_config.machineId;
  guiSession.configVersion=gui.sdk().m_config.configVersion;
  guiSession.samples={{0,100},{1,10},{2,100}};
  const auto directory=std::filesystem::temp_directory_path()/"ff_gui_training_sessions_smoke";
  std::filesystem::create_directories(directory);
  const auto savedPath=directory/"Training_CH1_smoke.json";
  if (!ff::GuiTrainingAnalysis::saveSession(gui,savedPath.string(),guiSession) ||
      !std::filesystem::is_regular_file(savedPath)) return 31;
  auto loadedGuiSession=ff::GuiTrainingAnalysis::loadSession(gui,savedPath.string());
  if (!loadedGuiSession || loadedGuiSession.value.samples.size()!=3) return 32;
  ff::GuiTrainingArchive archive;
  archive.machineId=guiSession.machineId;
  archive.configVersion=guiSession.configVersion;
  archive.createdUnixSeconds=1794832729;
  archive.selectedChannel=0;
  archive.channels[0]=guiSession;
  auto secondSession=guiSession;
  secondSession.channel=ff::ChannelId::CH2;
  archive.channels[1]=secondSession;
  const auto archivePath=directory/"Training_all_channels_smoke.json";
  if (!ff::GuiTrainingAnalysis::saveArchive(gui,archivePath.string(),archive)) return 33;
  { std::ifstream input(archivePath);
    const std::string text((std::istreambuf_iterator<char>(input)),{});
    if (text.find("\"schema\": \"FF-TRAIN-2\"")==std::string::npos ||
        text.find("\"channel\": 2, \"training\": {}") == std::string::npos ||
        text.find("\"samples\": [\n") == std::string::npos) return 35;
  }
  auto loadedArchive=ff::GuiTrainingAnalysis::loadArchive(gui,archivePath.string());
  if (!loadedArchive || !loadedArchive.value.channels[0] ||
      !loadedArchive.value.channels[1] || loadedArchive.value.channels[2]) return 34;
  if (!gui.disconnect()) return 15;
  auto editable=ff::FFConfigIO::load(argv[1]);
  if (!editable || !ff::FFConfigIO::validate(editable.value)) return 21;
  editable.value.machineId="GUI-PRIVATE-CONFIG-TEST";
  const auto configPath=std::filesystem::temp_directory_path()/"ff_gui_private_config.json";
  if (!ff::FFConfigIO::save(configPath.string(),editable.value)) return 22;
  auto reopened=ff::FFConfigIO::load(configPath.string());
  if (!reopened || reopened.value.machineId!=editable.value.machineId) return 23;
  reopened.value.channels[0].rods[0].softMin=1000;
  if (ff::FFConfigIO::validate(reopened.value)) return 24;
  std::cout << "GUI SDK adapter smoke passed\n";
  return 0;
}
