#include "ff/FF.hpp"
#include <cassert>
#include <iostream>
using namespace ff;
static FFConfig cfg(){FFConfig c;c.machineId="test";c.configVersion="v1";for(int i=0;i<3;i++){auto&ch=c.channels[i];ch.id=static_cast<ChannelId>(i);ch.controller.serial.portName="COM"+std::to_string(i+1);RodConfig r;r.id=0;r.hardMin=0;r.hardMax=100;r.softMin=10;r.softMax=90;r.safePosition=80;r.safePositionValid=true;r.maxVelocity=10;ch.rods.push_back(r);}return c;}
int main(){auto c=cfg();std::array<std::unique_ptr<IControllerBackend>,3>b;for(auto&i:b)i=makeSimulationBackend();FF f(c,std::move(b));assert(f.initialize());assert(f.connect());assert(f.refreshStatus());auto s=f.status();assert(s.channels[0].connected);MotionRequest req{{{{ChannelId::CH1,0},50,5}},false};auto p=MotionPlanner::plan(c,s,req);assert(p);assert(p.value.steps.size()==2);MotionRequest badReq{{{{ChannelId::CH1,0},101,1}},false};auto bad=f.submitMotion(badReq);assert(!bad);assert(f.emergencyStop());std::cout<<"FF tests passed\n";}
