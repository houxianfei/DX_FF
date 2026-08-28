#include "ff/FF.hpp"
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#ifdef _WIN32
#include <windows.h>
#include "CppDllMightyZap_Ver2010.h"
#endif

namespace ff {
class SimulatedControllerBackend final : public IControllerBackend {
    bool open_{false}; std::mutex mutex_; std::unordered_map<RodId,RodStatus> rods_;
public:
    Result<void> open(const SerialConfig&) override { open_=true; return Result<void>::success(); }
    Result<void> close() override { open_=false; return Result<void>::success(); }
    bool isOpen() const noexcept override { return open_; }
    Result<void> setGoal(RodId id,int32_t p,double) override { if(!open_)return Result<void>::failure(ErrorCode::NotConnected); auto&s=rods_[id];s.targetPosition=p;s.actualPosition=p;s.positionValid=true;s.state=RodState::Idle;s.enabled=true;s.updateTime=std::chrono::steady_clock::now();return Result<void>::success(); }
    Result<void> setGoalsSynchronous(const std::vector<RodTarget>& ts) override { for(auto&t:ts){auto x=setGoal(t.address.rod,t.position,t.velocity);if(!x)return x;}return Result<void>::success(); }
    Result<void> stop(RodId id) override { rods_[id].state=RodState::Idle; return Result<void>::success(); }
    Result<void> stopAll() override { for(auto&x:rods_)x.second.state=RodState::Idle; return Result<void>::success(); }
    Result<void> enable(RodId id,bool e) override { rods_[id].enabled=e;return Result<void>::success(); }
    Result<void> home(RodId id) override { return setGoal(id,0,0); }
    Result<RodStatus> readStatus(RodId id) override { if(!open_)return Result<RodStatus>::failure(ErrorCode::NotConnected); auto&s=rods_[id];if(!s.positionValid){s.actualPosition=80;s.targetPosition=80;s.enabled=true;s.state=RodState::Idle;}s.positionValid=true;s.updateTime=std::chrono::steady_clock::now();return Result<RodStatus>::success(s); }
    Result<void> resetAlarm(RodId id) override { rods_[id].alarm=false;rods_[id].alarmCode=0;return Result<void>::success(); }
};

#ifdef _WIN32
class MightyZapControllerBackend final : public IControllerBackend {
    MightyZap zap_; bool open_{false}; std::mutex mutex_; std::unordered_set<RodId> ids_;
public:
    Result<void> open(const SerialConfig& c) override { std::wstring w(c.portName.begin(),c.portName.end()); if(zap_.OpenMightyZap(w.c_str(),c.baudRate)){open_=true;zap_.SerialTimeout(1,0,1,1,1);return Result<void>::success();}return Result<void>::failure(ErrorCode::Communication,"MightyZap 打开串口失败"); }
    Result<void> close() override { if(open_)zap_.CloseMightyZap();open_=false;return Result<void>::success(); }
    bool isOpen() const noexcept override{return open_;}
    Result<void> setGoal(RodId id,int32_t p,double) override {if(!open_)return Result<void>::failure(ErrorCode::NotConnected);ids_.insert(id);zap_.GoalPosition(id,static_cast<short>(p));return Result<void>::success();}
    Result<void> setGoalsSynchronous(const std::vector<RodTarget>& ts) override {for(auto&t:ts){auto x=setGoal(t.address.rod,t.position,t.velocity);if(!x)return x;}return Result<void>::success();}
    Result<void> stop(RodId id) override {if(!open_)return Result<void>::failure(ErrorCode::NotConnected);zap_.ForceEnable(id,false);return Result<void>::success();}
    Result<void> stopAll() override{for(auto id:ids_)zap_.ForceEnable(id,false);return Result<void>::success();}
    Result<void> enable(RodId id,bool e) override{if(!open_)return Result<void>::failure(ErrorCode::NotConnected);ids_.insert(id);zap_.ForceEnable(id,e);return Result<void>::success();}
    Result<void> home(RodId id) override{return setGoal(id,0,0);}
    Result<RodStatus> readStatus(RodId id) override{if(!open_)return Result<RodStatus>::failure(ErrorCode::NotConnected);ids_.insert(id);short p=0;unsigned char a=0;zap_.PresentPosition(id,p);zap_.ReadError(id,a);RodStatus s;s.actualPosition=p;s.targetPosition=p;s.positionValid=true;s.enabled=true;s.alarm=a!=0;s.alarmCode=a;s.state=s.alarm?RodState::Fault:RodState::Idle;s.updateTime=std::chrono::steady_clock::now();return Result<RodStatus>::success(s);}
    Result<void> resetAlarm(RodId id) override{if(!open_)return Result<void>::failure(ErrorCode::NotConnected);zap_.reset_write(id,0);return Result<void>::success();}
};
#endif

std::unique_ptr<IControllerBackend> makeSimulationBackend(){return std::make_unique<SimulatedControllerBackend>();}
std::unique_ptr<IControllerBackend> makeMightyZapBackend(){
#ifdef _WIN32
    return std::make_unique<MightyZapControllerBackend>();
#else
    return nullptr;
#endif
}
}
