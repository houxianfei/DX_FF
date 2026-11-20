#include <DX_FF.h>
#include <cassert>
#include <cmath>
#include <crtdbg.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <type_traits>

static_assert(!std::is_copy_constructible_v<FF::ff_driver>);
static_assert(!std::is_assignable_v<decltype(std::declval<FF::ff_driver&>().m_current_positions),
                                    std::vector<FF::RodPosition>>);

static void variant(const std::string& path, std::string text,
                    const std::string& from, const std::string& to) {
  auto i=text.find(from); assert(i!=std::string::npos);
  text.replace(i,from.size(),to);
  std::ofstream(path,std::ios::binary)<<text;
}

int main() {
  _set_abort_behavior(0,_WRITE_ABORT_MSG | _CALL_REPORTFAULT);
  _CrtSetReportMode(_CRT_ASSERT,_CRTDBG_MODE_FILE);
  _CrtSetReportFile(_CRT_ASSERT,_CRTDBG_FILE_STDERR);
  FF::ff_driver driver;
  int callbacks=0;
  driver.set_status_callback([&](const FF::DeviceStatus& s) {
    ++callbacks; assert(driver.m_status.state==s.state);
  });
  assert(!driver.open("missing.json",FF::Mode::Simulation));
  assert(driver.open("FF_SDK/examples/ff_config.sim.json",FF::Mode::Simulation));
  assert(driver.refresh() && callbacks>0);
  assert(driver.m_current_positions.size()==3);
  auto before=driver.m_current_positions;
  auto loaded=driver.load_positions("FF_SDK/examples/slot_0.json");
  if (!loaded) std::cerr << "Slot0 load failed: " << loaded.result.message << '\n';
  assert(loaded && loaded.value.targets.size()==3);
  FF::TrainingArchive archive;
  archive.machineId="SDK-SIM"; archive.configVersion="1";
  archive.selectedChannel=0;
  for (int ch=0;ch<3;++ch) {
    FF::TrainingSession channel;
    channel.channel=ch;
    channel.trainingRod=0;
    channel.stepMm=0.1;
    channel.executionTargets={{{ch,0},50}};
    archive.channels[ch]=channel;
  }
  assert(driver.save_training_archive("FF_SDK/examples/three_channel_training.json",archive));
  auto reloadedArchive=driver.load_training_archive("FF_SDK/examples/three_channel_training.json");
  assert(reloadedArchive && reloadedArchive.value.channels[0] &&
         reloadedArchive.value.channels[1] && reloadedArchive.value.channels[2]);
  auto positionFromTraining=driver.load_positions("FF_SDK/examples/three_channel_training.json");
  assert(positionFromTraining && positionFromTraining.value.targets.size()==3);
  { std::ofstream old("FF_SDK/examples/legacy_position.json");
    old << "{\"schema\":\"FF-TRAIN-1\",\"source\":\"Loaded\","
        << "\"machine_id\":\"SDK-SIM\",\"config_version\":\"1\","
        << "\"occlusion_summary\":\"old\",\"step_mm\":0.1,"
        << "\"execution_targets\":[{\"channel\":0,\"rod\":0,\"position\":50,\"velocity\":100},"
        << "{\"channel\":1,\"rod\":0,\"position\":50},"
        << "{\"channel\":2,\"rod\":0,\"position\":50}]}"; }
  auto legacy=driver.load_positions("FF_SDK/examples/legacy_position.json");
  assert(legacy && legacy.value.summary=="old" && legacy.value.targets.size()==3);
  assert(driver.m_current_positions.front().encoder==before.front().encoder);
  assert(!driver.load_positions("missing.json"));
  std::ifstream source("FF_SDK/examples/slot_0.json",std::ios::binary);
  const std::string json((std::istreambuf_iterator<char>(source)),{});
  variant("FF_SDK/examples/mismatch.json",json,
          "\"config_version\": \"1\"","\"config_version\": \"old\"");
  assert(!driver.load_positions("FF_SDK/examples/mismatch.json"));
  assert(!driver.initialize("FF_SDK/examples/mismatch.json"));
  assert(!driver.m_status.initialized);
  variant("FF_SDK/examples/duplicate.json",json,
          "\"channel\": 2","\"channel\": 1");
  assert(!driver.load_positions("FF_SDK/examples/duplicate.json"));
  auto incomplete=json;
  auto third=incomplete.find("\"channel\": 2"); assert(third!=std::string::npos);
  auto target=incomplete.find("{\"rod\": 0, \"position\": 50}",third);
  assert(target!=std::string::npos);
  incomplete.erase(target,std::string("{\"rod\": 0, \"position\": 50}").size());
  std::ofstream("FF_SDK/examples/incomplete.json",std::ios::binary)<<incomplete;
  auto partial=driver.load_positions("FF_SDK/examples/incomplete.json");
  assert(partial && partial.value.targets.size()==2);
  variant("FF_SDK/examples/limit.json",json,
          "\"position\": 50","\"position\": 999");
  assert(!driver.load_positions("FF_SDK/examples/limit.json"));
  assert(!driver.move_rods({loaded.value.targets.front()}));
  auto duplicate=loaded.value.targets; duplicate[1]=duplicate[0];
  assert(!driver.send_rods(duplicate));
  auto beyond=loaded.value.targets; beyond[0].position=999;
  assert(!driver.send_rods(beyond));
  assert(!driver.send_rod({0,0},999));
  assert(driver.move_rods(loaded.value.targets));
  assert(driver.m_status.state=="Ready");
  int repeated=0,previous=-1;
  for (int sample=0;sample<300;++sample) {
    const int forward=static_cast<int>(std::lround(10.0+80.0*sample/299));
    const int reverse=static_cast<int>(std::lround(90.0-80.0*sample/299));
    const int shorter=static_cast<int>(std::lround(20.0+60.0*sample/299));
    assert(driver.validate_rods({{{0,0},forward},{{1,0},reverse},{{2,0},shorter}}));
    if (sample && forward==previous) ++repeated;
    previous=forward;
  }
  assert(repeated>0);
  assert(driver.send_partial_rods({{{0,0},53}}));
  assert(!driver.send_partial_rods({{{0,0},53},{{0,0},54}}));
  assert(!driver.send_partial_rods({{{3,0},53}}));
  assert(driver.send_rods(loaded.value.targets));
  assert(driver.refresh());
  assert(driver.move_rod({0,0},55));
  assert(driver.m_current_positions.front().encoder==55);
  assert(driver.send_rod({0,0},54));
  assert(driver.refresh_rod({0,0}));
  assert(driver.m_current_positions.front().encoder==54);
  assert(driver.record_positions("FF_SDK/examples/smoke_record.json","smoke"));
  { std::ifstream file("FF_SDK/examples/smoke_record.json");
    const std::string text((std::istreambuf_iterator<char>(file)),{});
    assert(text.find("\"schema\": \"FF-POS-2\"")!=std::string::npos);
    assert(text.find("\"velocity\"")==std::string::npos);
    assert(text.find("\"samples\"")==std::string::npos);
    assert(text.find("\"channel\": 0")!=std::string::npos);
    assert(text.find("\"channel\": 1")!=std::string::npos);
    assert(text.find("\"channel\": 2")!=std::string::npos); }
  auto recorded=driver.load_positions("FF_SDK/examples/smoke_record.json");
  assert(recorded && recorded.value.summary=="smoke");
  { std::ofstream csv("FF_SDK/examples/smoke.csv");
    csv<<"channel,rod,position,velocity\n0,0,54,10\n1,0,50,10\n2,0,50,10\n"; }
  assert(driver.load_positions("FF_SDK/examples/smoke.csv"));
  assert(driver.initialize("FF_SDK/examples/smoke.csv"));
  assert(driver.m_status.initialized);
  std::vector<FF::Sample> samples{{0,100},{1,10},{2,100}};
  auto valleys=driver.find_valleys(samples,{});
  auto assignment=driver.assign_training(0,valleys);
  assert(assignment);
  auto trainingTargets=recorded.value.targets;
  for (const auto& aim:assignment.value.aims)
    if (aim.assigned)
      for (auto& target:trainingTargets)
        if (target.address.channel==aim.address.channel &&
            target.address.rod==aim.address.rod) target.position=aim.encoderAim;
  assert(driver.move_rods(trainingTargets));
  assert(driver.emergency_stop());
  assert(!driver.record_positions("FF_SDK/examples/unsafe_record.json"));
  assert(driver.stop() && driver.reset_faults());
  assert(driver.close());
  assert(driver.m_current_positions.empty());
  assert(!driver.refresh());
  assert(std::string(driver.version())=="1.0.1");
  FF::ff_driver oneChannel;
  assert(oneChannel.open("FF_SDK/examples/ff_config.sim.json",FF::Mode::Simulation,{},1));
  FF::PositionSet onlyFirst;
  onlyFirst.targets={{{0,0},50}};
  assert(oneChannel.save_positions("FF_SDK/examples/one_channel.json",onlyFirst));
  std::ifstream oneFile("FF_SDK/examples/one_channel.json");
  const std::string oneText((std::istreambuf_iterator<char>(oneFile)),{});
  assert(oneText.find("\"schema\": \"FF-POS-2\"")!=std::string::npos);
  assert(oneText.find("\"channel\": 1")==std::string::npos);
  assert(oneText.find("\"channel\": 2")==std::string::npos);
  assert(oneChannel.load_positions("FF_SDK/examples/one_channel.json"));
  assert(oneChannel.close());
  {
    std::ifstream input("FF_SDK/examples/ff_config.sim.json",std::ios::binary);
    std::string twoRodConfig((std::istreambuf_iterator<char>(input)),{});
    const auto members=twoRodConfig.find("\"layer_members\":[");
    assert(members!=std::string::npos);
    const auto membersEnd=twoRodConfig.find(']',members);
    assert(membersEnd!=std::string::npos);
    twoRodConfig.insert(membersEnd,
        ",{\"rod_id\":1,\"layer\":2,\"role\":3,\"left_neighbor\":-1,\"right_neighbor\":-1}");
    const auto rods=twoRodConfig.find("\"rods\":[",membersEnd);
    assert(rods!=std::string::npos);
    const auto rodBegin=twoRodConfig.find("{\"id\":0",rods);
    const auto rodEnd=twoRodConfig.find('}',rodBegin);
    assert(rodBegin!=std::string::npos && rodEnd!=std::string::npos);
    auto secondRod=twoRodConfig.substr(rodBegin,rodEnd-rodBegin+1);
    secondRod.replace(secondRod.find("\"id\":0"),6,"\"id\":1");
    twoRodConfig.insert(rodEnd+1,","+secondRod);
    const std::string path="FF_SDK/examples/two_rod_config.json";
    std::ofstream(path,std::ios::binary)<<twoRodConfig;
    FF::ff_driver twoRods;
    assert(twoRods.open(path,FF::Mode::Simulation,{},1));
    assert(!twoRods.load_positions("FF_SDK/examples/slot_0.json"));
    assert(!twoRods.initialize("FF_SDK/examples/slot_0.json"));
    assert(!twoRods.m_status.initialized);
    assert(twoRods.close());
  }
  for (int bits=1;bits<8;++bits) {
    FF::ChannelSelection enabled{bool(bits&1),bool(bits&2),bool(bits&4)};
    FF::ff_driver selected;
    assert(selected.open("FF_SDK/examples/ff_config.sim.json",FF::Mode::Simulation,{},enabled));
    const auto info=selected.machine_info();
    assert(info.enabledChannels==enabled);
    assert(info.activeChannels==int(enabled[0])+int(enabled[1])+int(enabled[2]));
    for (int ch=0;ch<3;++ch)
      assert(selected.m_status.channels[ch].connected==enabled[ch]);
    for (int ch=0;ch<3;++ch) {
      const std::vector<FF::Target> probe{{{ch,0},50}};
      assert(static_cast<bool>(selected.validate_rods(probe))==enabled[ch]);
      if (enabled[ch]) assert(!selected.validate_rods({{{ch,0},999}}));
    }
    if (bits==2) {
      auto sparse=selected.load_positions("FF_SDK/examples/smoke_record.json");
      assert(sparse && sparse.value.targets.size()==1 &&
             sparse.value.targets.front().address.channel==1 &&
             sparse.value.ignoredChannels.size()==2);
    }
    if(bits!=1) assert(!selected.initialize("FF_SDK/examples/one_channel.json"));
    assert(selected.initialize("FF_SDK/examples/slot_0.json"));
    assert(selected.m_status.initialized);
    assert(selected.close());
    assert(!selected.m_status.initialized);
  }
  std::cout<<"public SDK smoke passed\n";
}
