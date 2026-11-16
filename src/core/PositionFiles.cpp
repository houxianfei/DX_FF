#include "ff/PositionFiles.hpp"
#include "ff/Training.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <regex>
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
std::string escapeJson(const std::string& value) {
  std::string out;
  for (char c:value) {
    if (c=='\\' || c=='"') out.push_back('\\');
    if (c=='\n') { out += "\\n"; continue; }
    if (c=='\r') { out += "\\r"; continue; }
    out.push_back(c);
  }
  return out;
}
bool stringField(const std::string& text,const std::string& key,std::string& value) {
  std::smatch match;
  const std::regex pattern("\""+key+"\"\\s*:\\s*\"((?:\\\\.|[^\"\\\\])*)\"");
  if (!std::regex_search(text,match,pattern)) return false;
  value.clear();
  const auto encoded=match[1].str();
  for (size_t i=0;i<encoded.size();++i) {
    if (encoded[i]=='\\' && ++i<encoded.size()) {
      value.push_back(encoded[i]=='n'?'\n':encoded[i]=='r'?'\r':encoded[i]);
    } else value.push_back(encoded[i]);
  }
  return true;
}
bool integerField(const std::string& text,const std::string& key,long long& value) {
  std::smatch match;
  const std::regex pattern("\""+key+"\"\\s*:\\s*(-?[0-9]+)(?=\\s*[,}])");
  if (!std::regex_search(text,match,pattern)) return false;
  try { value=std::stoll(match[1].str()); return true; }
  catch (...) { return false; }
}
bool objectsField(const std::string& text,const std::string& key,
                  std::vector<std::string>& objects) {
  const auto name=text.find("\""+key+"\"");
  if (name==std::string::npos) return false;
  const auto begin=text.find('[',name);
  if (begin==std::string::npos) return false;
  size_t objectBegin=std::string::npos;
  int arrayDepth=0, objectDepth=0;
  bool quoted=false, escaped=false;
  for (size_t i=begin;i<text.size();++i) {
    const char c=text[i];
    if (quoted) {
      if (escaped) escaped=false;
      else if (c=='\\') escaped=true;
      else if (c=='"') quoted=false;
      continue;
    }
    if (c=='"') { quoted=true; continue; }
    if (c=='[') ++arrayDepth;
    else if (c==']') {
      if (--arrayDepth==0) return objectDepth==0;
    } else if (c=='{') {
      if (arrayDepth==1 && objectDepth==0) objectBegin=i;
      ++objectDepth;
    } else if (c=='}') {
      if (objectDepth==0) return false;
      if (--objectDepth==0 && arrayDepth==1)
        objects.push_back(text.substr(objectBegin,i-objectBegin+1));
    }
  }
  return false;
}
Result<PositionFileData> loadPositionJson(const std::string& text) {
  PositionFileData file;
  std::string schema;
  if (!stringField(text,"schema",schema) || schema!="FF-POS-1" ||
      !stringField(text,"machine_id",file.machineId) ||
      !stringField(text,"config_version",file.configVersion) ||
      !stringField(text,"summary",file.summary))
    return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,
                                              "Invalid position file header");
  std::vector<std::string> channels;
  if (!objectsField(text,"channels",channels) || channels.size()!=3)
    return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,
                                              "Position file must list CH1, CH2 and CH3");
  std::set<int> seenChannels;
  for (const auto& channel:channels) {
    long long id{};
    std::vector<std::string> targets;
    if (!integerField(channel,"channel",id) || id<0 || id>2 ||
        !seenChannels.insert(static_cast<int>(id)).second ||
        !objectsField(channel,"execution_targets",targets))
      return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,
                                                "Invalid position channel");
    for (const auto& target:targets) {
      long long rod{},position{};
      if (!integerField(target,"rod",rod) || rod<0 || rod>255 ||
          !integerField(target,"position",position) ||
          position<std::numeric_limits<std::int32_t>::min() ||
          position>std::numeric_limits<std::int32_t>::max())
        return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,
                                                  "Invalid position target");
      file.targets.push_back({{static_cast<ChannelId>(id),static_cast<RodId>(rod)},
                              static_cast<std::int32_t>(position)});
    }
  }
  return Result<PositionFileData>::success(std::move(file));
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
      std::istringstream line(row); std::string field[3];
      for(auto& item:field) if(!std::getline(line,item,','))
        return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,"Invalid CSV row");
      try {
        size_t end[3]{};
        const int ch=std::stoi(field[0],&end[0]), rod=std::stoi(field[1],&end[1]);
        const long long pos=std::stoll(field[2],&end[2]);
        for(int i=0;i<3;++i) if(end[i]!=field[i].size())
          return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,"Invalid CSV number");
        if(ch<0 || ch>2 || rod<0 || rod>255 ||
           pos<std::numeric_limits<int32_t>::min() || pos>std::numeric_limits<int32_t>::max())
          return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,"Invalid CSV target");
        file.targets.push_back({{static_cast<ChannelId>(ch),static_cast<RodId>(rod)},
                                static_cast<int32_t>(pos)});
      } catch(...) { return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,"Invalid CSV number"); }
    }
    file.machineId=current.machineId; file.configVersion=current.configVersion;
    file.summary="Legacy CSV";
  } else {
    std::ifstream input(path,std::ios::binary);
    if (!input) return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,
                                                          "Cannot read position file");
    const std::string text((std::istreambuf_iterator<char>(input)),{});
    std::string schema;
    if (stringField(text,"schema",schema) && schema=="FF-POS-1")
      return loadPositionJson(text);
    if (schema=="FF-TRAIN-2") {
      auto archive=TrainingAnalysis::loadArchive(path);
      if (!archive) return Result<PositionFileData>::failure(archive.error,archive.message);
      file.machineId=archive.value.machineId;
      file.configVersion=archive.value.configVersion;
      for (const auto& session:archive.value.channels)
        if (session) {
          if (file.summary.empty()) file.summary=session->occlusionSummary;
          file.targets.insert(file.targets.end(),session->executionTargets.begin(),
                              session->executionTargets.end());
        }
      return Result<PositionFileData>::success(std::move(file));
    }
    if (schema!="FF-TRAIN-1")
      return Result<PositionFileData>::failure(ErrorCode::InvalidConfiguration,
                                                "Unsupported position file schema");
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
  const auto temporary=path+".tmp";
  {
    std::ofstream out(temporary,std::ios::binary|std::ios::trunc);
    if (!out) return Result<void>::failure(ErrorCode::BackendFailure,
                                           "Cannot write position file");
    out << "{\n  \"schema\": \"FF-POS-1\",\n"
        << "  \"machine_id\": \"" << escapeJson(file.machineId) << "\",\n"
        << "  \"config_version\": \"" << escapeJson(file.configVersion) << "\",\n"
        << "  \"summary\": \"" << escapeJson(file.summary) << "\",\n"
        << "  \"channels\": [\n";
    for (int ch=0;ch<3;++ch) {
      const bool hasTargets=std::any_of(file.targets.begin(),file.targets.end(),
        [ch](const auto& target){return static_cast<int>(target.address.channel)==ch;});
      if (!hasTargets) {
        out << "    {\"channel\": " << ch << ", \"execution_targets\": []}"
            << (ch<2?",":"") << '\n';
        continue;
      }
      out << "    {\"channel\": " << ch << ", \"execution_targets\": [\n";
      bool first=true;
      for (const auto& target:file.targets) {
        if (static_cast<int>(target.address.channel)!=ch) continue;
        if (!first) out << ",\n";
        out << "      {\"rod\": " << int(target.address.rod)
            << ", \"position\": " << target.position << "}";
        first=false;
      }
      if (!first) out << '\n';
      out << "    ]}" << (ch<2?",":"") << '\n';
    }
    out << "  ]\n}\n";
    out.flush();
    if (!out) { out.close(); std::remove(temporary.c_str());
      return Result<void>::failure(ErrorCode::BackendFailure,
                                    "Cannot finish position file"); }
  }
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
    if(ch<0 || ch>=count ||
       !seen[ch].insert(target.address.rod).second)
      return Result<void>::failure(ErrorCode::InvalidTarget,"Invalid or duplicate target");
    used[ch]=true;
  }
  for(int ch=0;ch<count;++ch) {
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
