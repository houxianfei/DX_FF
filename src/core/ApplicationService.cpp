#include "ff/FF.hpp"
#include <mutex>

namespace ff {
struct FFApplicationService::Impl {
    mutable std::mutex mutex;
    std::unique_ptr<FF> machine;
    bool simulation{true};
    bool connected{false};
    std::string error;
    FFApplicationService::StatusCallback callback;
};

FFApplicationService::FFApplicationService():impl_(std::make_unique<Impl>()){}
FFApplicationService::~FFApplicationService(){if(impl_->machine)impl_->machine->disconnect();}
Result<void> FFApplicationService::loadConfig(const std::string& file,bool sim){
    auto loaded=FFConfigIO::load(file); if(!loaded)return Result<void>::failure(loaded.error,loaded.message);
    std::array<std::unique_ptr<IControllerBackend>,3> b; for(auto&i:b)i=sim?makeSimulationBackend():makeMightyZapBackend();
    std::lock_guard l(impl_->mutex); impl_->simulation=sim; impl_->machine=std::make_unique<FF>(std::move(loaded.value),std::move(b)); auto x=impl_->machine->initialize(); if(!x){impl_->error=x.message;return x;} return Result<void>::success();
}
Result<void> FFApplicationService::connect(){std::lock_guard l(impl_->mutex);if(!impl_->machine)return Result<void>::failure(ErrorCode::NotReady,"尚未加载配置");auto x=impl_->machine->connect();impl_->connected=static_cast<bool>(x);if(!x)impl_->error=x.message;return x;}
Result<void> FFApplicationService::disconnect(){std::lock_guard l(impl_->mutex);if(impl_->machine)impl_->machine->disconnect();impl_->connected=false;return Result<void>::success();}
Result<void> FFApplicationService::refresh(){std::lock_guard l(impl_->mutex);if(!impl_->machine)return Result<void>::failure(ErrorCode::NotReady);auto x=impl_->machine->refreshStatus();if(impl_->callback)impl_->callback(impl_->machine->status());return x;}
Result<MotionPlan> FFApplicationService::previewPlan(const MotionRequest&r){std::lock_guard l(impl_->mutex);if(!impl_->machine)return Result<MotionPlan>::failure(ErrorCode::NotReady);return MotionPlanner::plan(impl_->machine->config(),impl_->machine->status(),r);}
Result<void> FFApplicationService::executePlan(const MotionPlan&p){std::lock_guard l(impl_->mutex);if(!impl_->machine||!p.valid)return Result<void>::failure(ErrorCode::InvalidTarget,"无效规划");if(p.configVersion!=impl_->machine->config().configVersion)return Result<void>::failure(ErrorCode::Busy,"规划已过期");for(auto&s:p.steps){auto x=impl_->machine->submitMotion({{{s.address,s.position,s.velocity}},false});if(!x)return Result<void>::failure(x.error,x.message);}return Result<void>::success();}
Result<void> FFApplicationService::cancelMotion(){std::lock_guard l(impl_->mutex);return impl_->machine?impl_->machine->stopAll():Result<void>::success();}
Result<void> FFApplicationService::emergencyStop(){std::lock_guard l(impl_->mutex);return impl_->machine?impl_->machine->emergencyStop():Result<void>::success();}
FFStatus FFApplicationService::status()const{std::lock_guard l(impl_->mutex);return impl_->machine?impl_->machine->status():FFStatus{};} bool FFApplicationService::connected()const noexcept{return impl_->connected;} bool FFApplicationService::simulation()const noexcept{return impl_->simulation;} std::string FFApplicationService::lastError()const{return impl_->error;} void FFApplicationService::setStatusCallback(StatusCallback c){std::lock_guard l(impl_->mutex);impl_->callback=std::move(c);}
}
