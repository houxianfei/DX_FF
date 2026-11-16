#include <DX_FF.h>
#include <cassert>
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
  variant("FF_SDK/examples/duplicate.json",json,
          "\"channel\": 2","\"channel\": 1");
  assert(!driver.load_positions("FF_SDK/examples/duplicate.json"));
  auto incomplete=json;
  auto third=incomplete.find("\"channel\": 2"); assert(third!=std::string::npos);
  auto target=incomplete.find("{\"rod\": 0, \"position\": 50}",third);
  assert(target!=std::string::npos);
  incomplete.erase(target,std::string("{\"rod\": 0, \"position\": 50}").size());
  std::ofstream("FF_SDK/examples/incomplete.json",std::ios::binary)<<incomplete;
  assert(!driver.load_positions("FF_SDK/examples/incomplete.json"));
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
    assert(text.find("\"schema\": \"FF-POS-1\"")!=std::string::npos);
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
  assert(oneText.find("\"channel\": 1, \"execution_targets\": []")!=std::string::npos);
  assert(oneText.find("\"channel\": 2, \"execution_targets\": []")!=std::string::npos);
  assert(oneChannel.load_positions("FF_SDK/examples/one_channel.json"));
  assert(oneChannel.close());
  std::cout<<"public SDK smoke passed\n";
}
