#pragma once
#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_set>

namespace ff::config_json {
inline std::string escape(const std::string& value) {
  std::string out;
  const char* digits="0123456789abcdef";
  for (unsigned char c:value) {
    switch(c) {
    case '"': out+="\\\"";break;
    case '\\': out+="\\\\";break;
    case '\n': out+="\\n";break;
    case '\r': out+="\\r";break;
    case '\t': out+="\\t";break;
    default:
      if(c<0x20) {out+="\\u00";out+=digits[c>>4];out+=digits[c&15];}
      else out+=char(c);
    }
  }
  return out;
}
struct Value {
  enum Kind { Null, Boolean, Number, String, Array, Object } kind{Null};
  bool boolean{};
  double number{};
  std::string str;
  std::vector<Value> array;
  std::map<std::string,Value> object;
  const Value* get(const char* key) const {
    if (kind!=Object) throw std::runtime_error("Expected JSON object");
    const auto it=object.find(key);
    return it==object.end()?nullptr:&it->second;
  }
};
class Parser {
  const std::string& input_;
  size_t at_{};
  void whitespace() { while (at_<input_.size() &&
      (input_[at_]==' ' || input_[at_]=='\t' || input_[at_]=='\n' || input_[at_]=='\r')) ++at_; }
  bool take(char c) { whitespace(); if (at_<input_.size() && input_[at_]==c) {++at_; return true;} return false; }
  void need(char c) { if (!take(c)) throw std::runtime_error("Malformed JSON at byte "+std::to_string(at_)); }
  static int hex(char c) { if(c>='0'&&c<='9') return c-'0'; if(c>='a'&&c<='f') return c-'a'+10; if(c>='A'&&c<='F') return c-'A'+10; throw std::runtime_error("Invalid JSON escape"); }
  static void utf8(std::string& out,unsigned code) {
    if(code<0x80) out+=char(code);
    else if(code<0x800) {out+=char(0xc0|(code>>6));out+=char(0x80|(code&63));}
    else {out+=char(0xe0|(code>>12));out+=char(0x80|((code>>6)&63));out+=char(0x80|(code&63));}
  }
  std::string string() {
    need('"'); std::string out;
    while(at_<input_.size()) {
      const unsigned char c=static_cast<unsigned char>(input_[at_++]);
      if(c=='"') return out;
      if(c<0x20) throw std::runtime_error("Control character in JSON string");
      if(c!='\\') {out+=char(c);continue;}
      if(at_>=input_.size()) throw std::runtime_error("Invalid JSON escape");
      const char e=input_[at_++];
      switch(e) {
      case '"': out+='"';break; case '\\':out+='\\';break; case '/':out+='/';break;
      case 'b':out+='\b';break; case 'f':out+='\f';break; case 'n':out+='\n';break;
      case 'r':out+='\r';break; case 't':out+='\t';break;
      case 'u': {
        if(at_+4>input_.size()) throw std::runtime_error("Short Unicode escape");
        unsigned code=0;for(int i=0;i<4;++i) code=(code<<4)|hex(input_[at_++]);
        if(code>=0xd800 && code<=0xdfff) throw std::runtime_error("Unsupported Unicode surrogate");
        utf8(out,code);break;
      }
      default: throw std::runtime_error("Invalid JSON escape");
      }
    }
    throw std::runtime_error("Unterminated JSON string");
  }
  Value value() {
    whitespace(); if(at_>=input_.size()) throw std::runtime_error("Unexpected end of JSON");
    Value out;
    if(input_[at_]=='{') {
      out.kind=Value::Object; ++at_;
      if(take('}')) return out;
      do {const auto key=string();need(':');auto child=value();
          if(!out.object.emplace(key,std::move(child)).second)
            throw std::runtime_error("Duplicate JSON field: "+key);
          if(take('}')) return out;need(',');} while(true);
    }
    if(input_[at_]=='[') {
      out.kind=Value::Array;++at_;
      if(take(']')) return out;
      do {out.array.push_back(value());if(take(']')) return out;need(',');} while(true);
    }
    if(input_[at_]=='"') {out.kind=Value::String;out.str=string();return out;}
    if(input_.compare(at_,4,"true")==0) {at_+=4;out.kind=Value::Boolean;out.boolean=true;return out;}
    if(input_.compare(at_,5,"false")==0) {at_+=5;out.kind=Value::Boolean;return out;}
    if(input_.compare(at_,4,"null")==0) {at_+=4;return out;}
    const size_t begin=at_;
    if(input_[at_]=='-') ++at_;
    if(at_>=input_.size()) throw std::runtime_error("Invalid JSON number");
    if(input_[at_]=='0') ++at_;
    else {if(input_[at_]<'1'||input_[at_]>'9') throw std::runtime_error("Invalid JSON number");
      while(at_<input_.size() && input_[at_]>='0' && input_[at_]<='9') ++at_;}
    if(at_<input_.size() && input_[at_]=='.') {
      ++at_;const size_t digits=at_;
      while(at_<input_.size() && input_[at_]>='0' && input_[at_]<='9') ++at_;
      if(digits==at_) throw std::runtime_error("Invalid JSON fraction");
    }
    if(at_<input_.size() && (input_[at_]=='e'||input_[at_]=='E')) {
      ++at_;if(at_<input_.size() && (input_[at_]=='+'||input_[at_]=='-')) ++at_;
      const size_t digits=at_;
      while(at_<input_.size() && input_[at_]>='0' && input_[at_]<='9') ++at_;
      if(digits==at_) throw std::runtime_error("Invalid JSON exponent");
    }
    out.kind=Value::Number;
    try {out.number=std::stod(input_.substr(begin,at_-begin));}
    catch(...) {throw std::runtime_error("JSON number out of range");}
    if(!std::isfinite(out.number)) throw std::runtime_error("Non-finite JSON number");
    return out;
  }
public:
  explicit Parser(const std::string& text):input_(text) {}
  Value parse() {auto out=value();whitespace();if(at_!=input_.size()) throw std::runtime_error("Trailing JSON data");return out;}
};
inline const Value& required(const Value& object,const char* key) {
  auto field=object.get(key);if(!field) throw std::runtime_error(std::string("Missing field: ")+key);return *field;
}
inline std::string string(const Value& v,const std::string& field) {
  if(v.kind!=Value::String) throw std::runtime_error(field+" must be a string");return v.str;
}
inline double number(const Value& v,const std::string& field) {
  if(v.kind!=Value::Number) throw std::runtime_error(field+" must be a number");return v.number;
}
inline long long integer(const Value& v,const std::string& field,long long minimum,long long maximum) {
  const auto n=number(v,field);
  if(n<std::max(double(minimum),-9007199254740991.0) || n>std::min(double(maximum),9007199254740991.0) || std::floor(n)!=n)
    throw std::runtime_error(field+" is outside its integer range");
  return static_cast<long long>(n);
}
inline bool boolean(const Value& v,const std::string& field) {
  if(v.kind!=Value::Boolean) throw std::runtime_error(field+" must be true or false");return v.boolean;
}
inline const Value& array(const Value& v,const std::string& field) {
  if(v.kind!=Value::Array) throw std::runtime_error(field+" must be an array");return v;
}
inline const Value* optional(const Value& object,const char* key) {return object.get(key);}
inline long long integerOr(const Value& object,const char* key,long long fallback,long long minimum,long long maximum,const std::string& path) {
  const auto* v=optional(object,key);return v?integer(*v,path+"."+key,minimum,maximum):fallback;
}
inline double numberOr(const Value& object,const char* key,double fallback,const std::string& path) {
  const auto* v=optional(object,key);return v?number(*v,path+"."+key):fallback;
}
inline std::string stringOr(const Value& object,const char* key,std::string fallback,const std::string& path) {
  const auto* v=optional(object,key);return v?string(*v,path+"."+key):fallback;
}
inline const Value& objectOrEmpty(const Value& object,const char* key) {
  static const Value empty=[] {Value v;v.kind=Value::Object;return v;}();
  const auto* v=optional(object,key);if(!v) return empty;
  if(v->kind!=Value::Object) throw std::runtime_error(std::string(key)+" must be an object");return *v;
}
inline FFConfig decode(const std::string& text) {
  const auto root=Parser(text).parse();
  if(root.kind!=Value::Object) throw std::runtime_error("Configuration root must be an object");
  FFConfig config;
  for(int ch=0;ch<3;++ch) config.channels[ch].id=static_cast<ChannelId>(ch);
  config.machineId=string(required(root,"machine_id"),"machine_id");
  config.configVersion=string(required(root,"config_version"),"config_version");
  config.maxActiveRodCount=static_cast<size_t>(integerOr(root,"max_active_rod_count",8,1,768,"config"));
  config.statusExpiration=std::chrono::milliseconds(integerOr(root,"status_expiration_ms",200,0,600000,"config"));
  const auto& motor=objectOrEmpty(root,"motor_parameters");
  const auto& motion=objectOrEmpty(root,"rod_motion");
  const auto rejectSharedConflict=[&](const char* motorKey,const char* motionKey) {
    const auto* a=optional(motor,motorKey);
    const auto* b=optional(motion,motionKey);
    if(a && b && number(*a,std::string("motor_parameters.")+motorKey)!=
                  number(*b,std::string("rod_motion.")+motionKey))
      throw std::runtime_error(std::string("motor_parameters.")+motorKey+
                               " conflicts with rod_motion."+motionKey);
  };
  rejectSharedConflict("speed_limit_raw","max_velocity");
  rejectSharedConflict("acceleration_raw","max_acceleration");
  rejectSharedConflict("deceleration_raw","max_deceleration");
  const auto& channels=array(required(root,"channels"),"channels").array;
  if(channels.empty() || channels.size()>3) throw std::runtime_error("channels must contain 1..3 entries");
  std::array<bool,3> seen{};
  for(const auto& entry:channels) {
    const auto id=string(required(entry,"id"),"channels.id");
    if(id.size()!=3 || id.substr(0,2)!="CH" || id[2]<'1'||id[2]>'3')
      throw std::runtime_error("Invalid channel ID: "+id);
    const int index=id[2]-'1';
    if(seen[index]) throw std::runtime_error("Duplicate channel ID: "+id);
    seen[index]=true;
    auto& channel=config.channels[index];
    const auto& controller=objectOrEmpty(entry,"controller");
    const auto& serial=objectOrEmpty(controller,"serial");
    channel.controller.name=stringOr(controller,"name","CTRL-"+id,id+".controller");
    channel.controller.serial.portName=stringOr(serial,"port","",id+".serial");
    channel.controller.serial.baudRate=static_cast<uint32_t>(integerOr(serial,"baud_rate",115200,1,10000000,id+".serial"));
    channel.controller.serial.timeout=std::chrono::milliseconds(integerOr(serial,"timeout_ms",500,1,600000,id+".serial"));
    channel.controller.pollPeriod=std::chrono::milliseconds(integerOr(controller,"poll_period_ms",20,1,600000,id+".controller"));
    channel.controller.retryCount=static_cast<uint32_t>(integerOr(controller,"retry_count",3,0,100,id+".controller"));
    if(const auto* training=optional(entry,"training_rod_id")) {
      if(training->kind!=Value::Null)
        channel.trainingRod=static_cast<RodId>(integer(*training,id+".training_rod_id",0,255));
    }
    if(const auto* members=optional(entry,"layer_members")) {
      for(const auto& item:array(*members,id+".layer_members").array) {
        LayerMembership m;
        m.rod=static_cast<RodId>(integer(required(item,"rod_id"),id+".layer_members.rod_id",0,255));
        m.layer=static_cast<int>(integer(required(item,"layer"),id+".layer_members.layer",0,255));
        m.role=static_cast<int>(integer(required(item,"role"),id+".layer_members.role",0,3));
        m.leftNeighbor=static_cast<int>(integerOr(item,"left_neighbor",-1,-1,255,id+".layer_members"));
        m.rightNeighbor=static_cast<int>(integerOr(item,"right_neighbor",-1,-1,255,id+".layer_members"));
        channel.layerMembers.push_back(m);
      }
    }
    if(const auto* rules=optional(entry,"clearance_rules")) {
      for(const auto& item:array(*rules,id+".clearance_rules").array) {
        PairClearanceRule rule;
        rule.layer=static_cast<int>(integer(required(item,"layer"),id+".clearance_rules.layer",0,255));
        rule.first=static_cast<RodId>(integer(required(item,"rod_a"),id+".clearance_rules.rod_a",0,255));
        rule.second=static_cast<RodId>(integer(required(item,"rod_b"),id+".clearance_rules.rod_b",0,255));
        rule.minimumClearanceMm=number(required(item,"minimum_clearance_mm"),id+".clearance_rules.minimum_clearance_mm");
        if(rule.minimumClearanceMm<0) throw std::runtime_error(id+" clearance must be nonnegative");
        channel.clearanceRules.push_back(rule);
      }
    }
    for(const auto& item:array(required(entry,"rods"),id+".rods").array) {
      RodConfig rod;
      rod.id=static_cast<RodId>(integer(required(item,"id"),id+".rod.id",0,255));
      const auto path=id+".Rod"+std::to_string(rod.id);
      rod.name=stringOr(item,"name",path,path);
      rod.hardMin=static_cast<int32_t>(integerOr(item,"hard_min",0,0,4095,path));
      rod.hardMax=static_cast<int32_t>(integerOr(item,"hard_max",4095,0,4095,path));
      rod.softMin=static_cast<int32_t>(integerOr(item,"soft_min",rod.hardMin,0,4095,path));
      rod.softMax=static_cast<int32_t>(integerOr(item,"soft_max",rod.hardMax,0,4095,path));
      rod.zeroPosition=static_cast<int32_t>(integerOr(item,"zero_position",0,0,4095,path));
      rod.safePosition=static_cast<int32_t>(integerOr(item,"safe_position",0,0,4095,path));
      if(const auto* safe=optional(item,"safe_position_valid")) rod.safePositionValid=boolean(*safe,path+".safe_position_valid");
      rod.p0=numberOr(item,"P0",numberOr(item,"stroke_bias",0,path),path);
      rod.p1=numberOr(item,"P1",numberOr(item,"stroke_slope",1,path),path);
      rod.inwardDirection=static_cast<int>(integerOr(item,"inward_direction",1,-1,1,path));
      if(rod.inwardDirection==0) throw std::runtime_error(path+" inward_direction must be -1 or 1");
      const auto speed=numberOr(item,"max_velocity",numberOr(motion,"max_velocity",300,"rod_motion"),path);
      const auto acceleration=numberOr(item,"max_acceleration",numberOr(motion,"max_acceleration",100,"rod_motion"),path);
      const auto deceleration=numberOr(item,"max_deceleration",numberOr(motion,"max_deceleration",100,"rod_motion"),path);
      const auto rejectConflict=[&](const char* rodKey,const Value& sharedObject,
                                    const char* sharedKey) {
        const auto* local=optional(item,rodKey);
        const auto* global=optional(sharedObject,sharedKey);
        if(local && global && number(*local,path+"."+rodKey)!=
                              number(*global,std::string(sharedKey)))
          throw std::runtime_error(path+"."+rodKey+" conflicts with shared "+sharedKey);
      };
      rejectConflict("max_velocity",motor,"speed_limit_raw");
      rejectConflict("max_acceleration",motor,"acceleration_raw");
      rejectConflict("max_deceleration",motor,"deceleration_raw");
      rejectConflict("max_velocity",motion,"max_velocity");
      rejectConflict("max_acceleration",motion,"max_acceleration");
      rejectConflict("max_deceleration",motion,"max_deceleration");
      rejectConflict("position_tolerance",motion,"position_tolerance");
      rejectConflict("stable_time_ms",motion,"stable_time_ms");
      rejectConflict("move_timeout_ms",motion,"move_timeout_ms");
      rod.maxVelocity=numberOr(motor,"speed_limit_raw",speed,"motor_parameters");
      rod.maxAcceleration=numberOr(motor,"acceleration_raw",acceleration,"motor_parameters");
      rod.maxDeceleration=numberOr(motor,"deceleration_raw",deceleration,"motor_parameters");
      rod.positionTolerance=static_cast<int32_t>(integerOr(item,"position_tolerance",
        integerOr(motion,"position_tolerance",5,0,4095,"rod_motion"),0,4095,path));
      rod.stableTime=std::chrono::milliseconds(integerOr(item,"stable_time_ms",
        integerOr(motion,"stable_time_ms",100,0,600000,"rod_motion"),0,600000,path));
      rod.moveTimeout=std::chrono::milliseconds(integerOr(item,"move_timeout_ms",
        integerOr(motion,"move_timeout_ms",5000,1,600000,"rod_motion"),1,600000,path));
      if(rod.stableTime>=rod.moveTimeout) throw std::runtime_error(path+" stable_time_ms must be less than move_timeout_ms");
      channel.rods.push_back(rod);
    }
  }
  const RodConfig* common=nullptr;
  for(const auto& channel:config.channels)
    for(const auto& rod:channel.rods) {
      if(!common) {common=&rod;continue;}
      if(rod.maxVelocity!=common->maxVelocity ||
         rod.maxAcceleration!=common->maxAcceleration ||
         rod.maxDeceleration!=common->maxDeceleration ||
         rod.positionTolerance!=common->positionTolerance ||
         rod.stableTime!=common->stableTime ||
         rod.moveTimeout!=common->moveTimeout)
        throw std::runtime_error("Motor motion and completion parameters must be identical for every Rod");
    }
  return config;
}
template<class FindRod>
Result<void> validate(const FFConfig& config, FindRod findRod) {
  const auto invalid=[](const std::string& field) {
    return Result<void>::failure(ErrorCode::InvalidConfiguration,field+" is invalid");
  };
  if(config.machineId.empty()) return invalid("machine_id");
  if(config.configVersion.empty()) return invalid("config_version");
  if(config.maxActiveRodCount==0 || config.maxActiveRodCount>768)
    return invalid("max_active_rod_count");
  if(config.statusExpiration.count()<0 || config.statusExpiration.count()>600000)
    return invalid("status_expiration_ms");
  std::size_t rodCount=0;
  const RodConfig* shared=nullptr;
  for(int ch=0;ch<3;++ch) {
    const auto& channel=config.channels[ch];
    const std::string channelPath="CH"+std::to_string(ch+1);
    if(static_cast<int>(channel.id)!=ch) return invalid(channelPath+" id");
    if(channel.controller.serial.baudRate==0 ||
       channel.controller.serial.baudRate>10000000)
      return invalid(channelPath+" baud_rate");
    if(channel.controller.serial.timeout.count()<=0 ||
       channel.controller.serial.timeout.count()>600000)
      return invalid(channelPath+" timeout_ms");
    if(channel.controller.pollPeriod.count()<=0 ||
       channel.controller.pollPeriod.count()>600000)
      return invalid(channelPath+" poll_period_ms");
    if(channel.controller.retryCount>100) return invalid(channelPath+" retry_count");
    std::unordered_set<int> ids;
    for(const auto& rod:channel.rods) {
      ++rodCount;
      const std::string path=channelPath+" Rod "+std::to_string(rod.id)+" ";
      if(!ids.insert(rod.id).second)
        return Result<void>::failure(ErrorCode::DuplicateId,path+"id is duplicated");
      if(rod.hardMin<0 || rod.hardMin>4095) return invalid(path+"hard_min");
      if(rod.hardMax<0 || rod.hardMax>4095 || rod.hardMax<rod.hardMin)
        return invalid(path+"hard_max");
      if(rod.softMin<rod.hardMin || rod.softMin>rod.hardMax)
        return invalid(path+"soft_min");
      if(rod.softMax<rod.softMin || rod.softMax>rod.hardMax)
        return invalid(path+"soft_max");
      if(rod.zeroPosition<rod.hardMin || rod.zeroPosition>rod.hardMax)
        return invalid(path+"zero_position");
      if(rod.safePositionValid &&
         (rod.safePosition<rod.softMin || rod.safePosition>rod.softMax))
        return invalid(path+"safe_position");
      if(!std::isfinite(rod.p0)) return invalid(path+"P0");
      if(!std::isfinite(rod.p1) || std::abs(rod.p1)<1e-12)
        return invalid(path+"P1");
      if(rod.inwardDirection!=-1 && rod.inwardDirection!=1)
        return invalid(path+"inward_direction");
      if(!std::isfinite(rod.maxVelocity) || rod.maxVelocity<1 ||
         rod.maxVelocity>1023 || std::floor(rod.maxVelocity)!=rod.maxVelocity)
        return invalid(path+"max_velocity");
      if(!std::isfinite(rod.maxAcceleration) || rod.maxAcceleration<0 ||
         rod.maxAcceleration>255 || std::floor(rod.maxAcceleration)!=rod.maxAcceleration)
        return invalid(path+"max_acceleration");
      if(!std::isfinite(rod.maxDeceleration) || rod.maxDeceleration<0 ||
         rod.maxDeceleration>255 || std::floor(rod.maxDeceleration)!=rod.maxDeceleration)
        return invalid(path+"max_deceleration");
      if(rod.positionTolerance<0 || rod.positionTolerance>4095)
        return invalid(path+"position_tolerance");
      if(rod.stableTime.count()<0 || rod.stableTime.count()>600000)
        return invalid(path+"stable_time_ms");
      if(rod.moveTimeout.count()<=0 || rod.moveTimeout.count()>600000 ||
         rod.stableTime>=rod.moveTimeout)
        return invalid(path+"move_timeout_ms");
      for(const auto& range:rod.absoluteSafeRanges)
        if(range.first>range.second || range.first<rod.hardMin ||
           range.second>rod.hardMax) return invalid(path+"absolute_safe_ranges");
      for(const auto& range:rod.forbiddenRanges)
        if(range.first>range.second || range.first<rod.hardMin ||
           range.second>rod.hardMax) return invalid(path+"forbidden_ranges");
      if(shared) {
        if(rod.maxVelocity!=shared->maxVelocity) return invalid(path+"max_velocity (shared)");
        if(rod.maxAcceleration!=shared->maxAcceleration) return invalid(path+"max_acceleration (shared)");
        if(rod.maxDeceleration!=shared->maxDeceleration) return invalid(path+"max_deceleration (shared)");
        if(rod.positionTolerance!=shared->positionTolerance) return invalid(path+"position_tolerance (shared)");
        if(rod.stableTime!=shared->stableTime) return invalid(path+"stable_time_ms (shared)");
        if(rod.moveTimeout!=shared->moveTimeout) return invalid(path+"move_timeout_ms (shared)");
      } else shared=&rod;
    }
    if(channel.trainingRod) {
      const std::string path=channelPath+" Rod "+std::to_string(*channel.trainingRod);
      if(!findRod(channel,*channel.trainingRod)) return invalid(path+" training_rod_id");
    }
    std::set<std::pair<int,int>> memberships;
    for(const auto& member:channel.layerMembers) {
      const std::string path=channelPath+" Rod "+std::to_string(member.rod)+" layer_members";
      if(!findRod(channel,member.rod) ||
         !memberships.insert({member.layer,int(member.rod)}).second)
        return invalid(path);
    }
    std::set<std::tuple<int,int,int>> rules;
    for(const auto& rule:channel.clearanceRules) {
      const std::string path=channelPath+" Rod "+std::to_string(rule.first)+
          "/"+std::to_string(rule.second)+" clearance_rules";
      if(!findRod(channel,rule.first) || !findRod(channel,rule.second) ||
         rule.first==rule.second || !std::isfinite(rule.minimumClearanceMm) ||
         rule.minimumClearanceMm<0 ||
         !rules.insert({rule.layer,std::min(int(rule.first),int(rule.second)),
                        std::max(int(rule.first),int(rule.second))}).second)
        return invalid(path);
    }
  }
  if(rodCount==0) return invalid("channels.rods");
  return Result<void>::success();
}
}
