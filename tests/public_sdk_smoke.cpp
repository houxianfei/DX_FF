#include <DX_FF.h>
#include <cassert>
#include <fstream>
#include <iostream>
#include <cmath>
#include <sstream>
#include <type_traits>

static_assert(!std::is_assignable_v<decltype(std::declval<FF::ff_driver&>().m_config), FF::ConfigSnapshot>);
static_assert(!std::is_assignable_v<decltype(std::declval<FF::ff_driver&>().m_status), FF::DeviceStatus>);
static_assert(!std::is_assignable_v<decltype(std::declval<FF::ff_driver&>().m_last_error), FF::Result>);
static_assert(!std::is_copy_constructible_v<FF::ff_driver>);
static_assert(!std::is_assignable_v<decltype(std::declval<FF::ff_driver&>().m_current_positions), std::vector<FF::RodPosition>>);
static_assert(!std::is_assignable_v<decltype(std::declval<FF::ff_driver&>().m_target_positions), std::vector<FF::TargetPosition>>);

static void write_variant(const std::string& name,std::string text,
                          const std::string& from,const std::string& to) {
  const auto index=text.find(from); assert(index!=std::string::npos);
  text.replace(index,from.size(),to);
  std::ofstream(name,std::ios::binary)<<text;
}

int main() {
  FF::ff_driver driver;
  int callbacks=0;
  driver.set_status_callback([&](const FF::DeviceStatus& s){
    ++callbacks; assert(driver.m_status.state==s.state);
  });
  assert(!driver.open("missing.json",FF::Mode::Simulation));
  assert(driver.m_last_error.code==FF::ErrorCode::InvalidConfiguration);
  assert(driver.open("FF_SDK/examples/ff_config.sim.json",FF::Mode::Simulation));
  assert(driver.m_config.machineId=="SDK-SIM");
  assert(driver.m_last_error.code==FF::ErrorCode::None);
  assert(driver.m_status.state=="Ready");
  assert(driver.refresh()); assert(callbacks>0);
  assert(driver.m_current_positions.size()==3);
  assert(driver.m_target_positions.size()==3);
  for (size_t i=0;i<3;++i) {
    assert(driver.m_current_positions[i].valid);
    assert(driver.m_target_positions[i].encoder==driver.m_current_positions[i].encoder);
    assert(std::abs(driver.m_current_positions[i].opticalMm-
                    driver.m_current_positions[i].encoder)<1e-9);
  }
  auto before=driver.m_target_positions;
  assert(!driver.load_positions("missing.json"));
  assert(driver.m_target_positions.front().encoder==before.front().encoder);
  assert(driver.load_positions("FF_SDK/examples/slot_0.json"));
  assert(driver.m_target_positions.size()==3);
  std::ifstream source("FF_SDK/examples/slot_0.json",std::ios::binary);
  const std::string json((std::istreambuf_iterator<char>(source)),{});
  const auto previous=driver.m_target_positions[0].encoder;
  write_variant("FF_SDK/examples/mismatch.json",json,"\"config_version\":\"1\"","\"config_version\":\"old\"");
  assert(!driver.load_positions("FF_SDK/examples/mismatch.json"));
  assert(driver.m_target_positions[0].encoder==previous);
  auto incomplete=json;
  auto third=incomplete.find("{\"channel\":2"); assert(third!=std::string::npos);
  auto lastComma=incomplete.rfind(',',third); assert(lastComma!=std::string::npos);
  auto end=incomplete.find('}',third); assert(end!=std::string::npos);
  incomplete.erase(lastComma,end-lastComma+1);
  std::ofstream("FF_SDK/examples/incomplete.json",std::ios::binary)<<incomplete;
  assert(!driver.load_positions("FF_SDK/examples/incomplete.json"));
  write_variant("FF_SDK/examples/duplicate.json",json,
                "\"channel\":2,\"rod\":0","\"channel\":1,\"rod\":0");
  assert(!driver.load_positions("FF_SDK/examples/duplicate.json"));
  write_variant("FF_SDK/examples/limit.json",json,"\"position\":50","\"position\":999");
  assert(!driver.load_positions("FF_SDK/examples/limit.json"));
  assert(driver.m_target_positions[0].encoder==previous);
  assert(driver.m_current_positions[0].encoder==before[0].encoder);
  assert(!driver.set_target_position({0,0},999,10));
  assert(driver.m_last_error.code==FF::ErrorCode::InvalidTarget);
  auto targets=std::vector<FF::Target>{{{0,0},50,10},{{1,0},50,10},{{2,0},50,10}};
  assert(driver.set_target_positions(targets));
  assert(driver.m_target_positions[0].encoder==50);
  assert(!driver.set_target_positions({targets[0],targets[0],targets[2]}));
  assert(driver.m_target_positions[0].encoder==50);
  assert(driver.plan_positions());
  assert(driver.initialize("FF_SDK/examples/slot_0.json"));
  assert(driver.m_status.state=="Ready");
  assert(driver.record_positions("FF_SDK/examples/smoke_record.json"));
  assert(driver.load_positions("FF_SDK/examples/smoke_record.json"));
  assert(driver.execute_positions());
  {std::ofstream csv("FF_SDK/examples/smoke.csv");
   csv<<"channel,rod,position,velocity\n0,0,50,10\n1,0,50,10\n2,0,50,10\n";}
  assert(driver.load_positions("FF_SDK/examples/smoke.csv"));
  assert(driver.move_rod({0,0},55,10));
  assert(driver.m_target_positions[0].encoder==55);
  assert(driver.m_current_positions[0].encoder==55);
  assert(driver.m_target_positions[1].encoder==50);
  assert(driver.emergency_stop());
  assert(!driver.record_positions("FF_SDK/examples/unsafe_record.json"));
  assert(driver.m_last_error.code==FF::ErrorCode::SafetyViolation);
  assert(driver.stop()); assert(driver.reset_faults());
  std::vector<FF::Sample> samples{{0,100},{1,10},{2,100}};
  (void)driver.find_valleys(samples,{});
  assert(driver.close());
  assert(driver.m_config.machineId.empty());
  assert(driver.m_status.state=="Offline");
  assert(driver.m_current_positions.empty());
  assert(driver.m_target_positions.empty());
  assert(driver.m_last_error.code==FF::ErrorCode::None);
  assert(!driver.refresh());
  assert(driver.m_last_error.code==FF::ErrorCode::NotOpen);
  assert(std::string(driver.version())=="1.0.1");
  std::ifstream configSource("FF_SDK/examples/ff_config.sim.json",std::ios::binary);
  const std::string configJson((std::istreambuf_iterator<char>(configSource)),{});
  write_variant("FF_SDK/examples/calibrated.json",configJson,
                "\"position_tolerance\":1",
                "\"P0\":5,\"P1\":0.5,\"position_tolerance\":1");
  FF::ff_driver calibrated;
  assert(calibrated.open("FF_SDK/examples/calibrated.json",FF::Mode::Simulation));
  assert(std::abs(calibrated.m_current_positions[0].opticalMm-
                  (5.0+0.5*calibrated.m_current_positions[0].encoder))<1e-9);
  assert(calibrated.set_target_position({0,0},60,10));
  assert(std::abs(calibrated.m_target_positions[0].opticalMm-35.0)<1e-9);
  assert(calibrated.close());
  std::cout<<"public SDK smoke passed\n";
}
