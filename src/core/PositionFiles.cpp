#include "ff/PositionFiles.hpp"
#include "ff/Training.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace ff {
namespace {
bool csvPath(const std::string& path) {
  if(path.size()<4) return false;
  auto ext=path.substr(path.size()-4);
  std::transform(ext.begin(),ext.end(),ext.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
  return ext==".csv";
}
}
Result<PositionFileData> PositionFiles::load(const std::string& path,const FFConfig& current) {
  PositionFileData file;
  if(csvPath(path)) {
    std::ifstream input(path);
    if(!input) return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,"Cannot read CSV");
    std::string row; std::getline(input,row);
    while(std::getline(input,row)) {
      if(row.empty()) continue;
      std::istringstream line(row); std::string field[4];
      for(auto& item:field) if(!std::getline(line,item,','))
        return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,"Invalid CSV row");
      try {
        size_t end[4]{};
        const int ch=std::stoi(field[0],&end[0]), rod=std::stoi(field[1],&end[1]);
        const long long pos=std::stoll(field[2],&end[2]);
        const double velocity=std::stod(field[3],&end[3]);
        for(int i=0;i<4;++i) if(end[i]!=field[i].size())
          return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,"Invalid CSV number");
        if(ch<0 || ch>2 || rod<0 || rod>255 ||
           pos<std::numeric_limits<int32_t>::min() || pos>std::numeric_limits<int32_t>::max() ||
           !std::isfinite(velocity) || velocity<0)
          return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,"Invalid CSV target");
        file.targets.push_back({{static_cast<ChannelId>(ch),static_cast<RodId>(rod)},
                                static_cast<int32_t>(pos),velocity});
      } catch(...) { return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,"Invalid CSV number"); }
    }
    file.machineId=current.machineId; file.configVersion=current.configVersion;
    file.summary="Legacy CSV";
  } else {
    auto session=TrainingAnalysis::loadSession(path);
    if(!session) return Result<PositionFileData>::failure(session.error,session.message);
    file.machineId=session.value.machineId;
    file.configVersion=session.value.configVersion;
    file.summary=session.value.occlusionSummary;
    file.targets=session.value.executionTargets;
  }
  return Result<PositionFileData>::success(std::move(file));
}
Result<void> PositionFiles::save(const std::string& path,const PositionFileData& file) {
  TrainingSession session; session.source=TrainingSource::Loaded;
  session.machineId=file.machineId; session.configVersion=file.configVersion;
  session.occlusionSummary=file.summary; session.executionTargets=file.targets;
  session.complete=true;
  session.createdUnixSeconds=std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const auto temporary=path+".tmp";
  auto saved=TrainingAnalysis::saveSession(temporary,session);
  if(!saved) { std::remove(temporary.c_str()); return saved; }
  if(!MoveFileExA(temporary.c_str(),path.c_str(),
                  MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
    std::remove(temporary.c_str());
    return Result<void>::failure(ErrorCode::BackendFailure,"Cannot replace position file");
  }
  return Result<void>::success();
}
Result<void> PositionFiles::validate(const PositionFileData& file,const FFConfig& current,int count) {
  if(file.machineId!=current.machineId || file.configVersion!=current.configVersion)
    return Result<void>::failure(ErrorCode::InvalidConfiguration,"Machine or configuration version mismatch");
  if(file.targets.empty()) return Result<void>::failure(ErrorCode::InvalidTarget,"No position targets");
  std::array<std::set<int>,3> seen; std::array<bool,3> used{};
  for(const auto& target:file.targets) {
    const int ch=static_cast<int>(target.address.channel);
    if(ch<0 || ch>=count || target.velocity<0 || !std::isfinite(target.velocity) ||
       !seen[ch].insert(target.address.rod).second)
      return Result<void>::failure(ErrorCode::InvalidTarget,"Invalid or duplicate target");
    used[ch]=true;
  }
  for(int ch=0;ch<count;++ch) if(used[ch]) {
    if(seen[ch].size()!=current.channels[ch].rods.size())
      return Result<void>::failure(ErrorCode::InvalidTarget,"Incomplete channel target set");
    for(const auto& rod:current.channels[ch].rods) if(!seen[ch].count(rod.id))
      return Result<void>::failure(ErrorCode::InvalidTarget,"Incomplete channel target set");
  }
  return Result<void>::success();
}
Result<void> PositionFiles::execute(FFApplicationService& service,const PositionFileData& file,
                                    bool waitForStable) {
  auto config=service.config();
  if(!config) return Result<void>::failure(ErrorCode::NotReady);
  auto valid=validate(file,*config,service.activeChannelCount()); if(!valid) return valid;
  valid=service.validateRodTargets(file.targets); if(!valid) return valid;
  if(!waitForStable) return service.setRodTargets(file.targets);
  auto plan=service.previewPlan({file.targets,false});
  if(!plan) return Result<void>::failure(plan.error,plan.message);
  return service.executePlan(plan.value);
}
} // namespace ff
