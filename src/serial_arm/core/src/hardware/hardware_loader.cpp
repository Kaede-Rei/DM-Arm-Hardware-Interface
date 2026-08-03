#include "serial_arm/hardware/hardware_loader.hpp"

#include <dlfcn.h>

#include <filesystem>
#include <memory>
#include <utility>
#include <vector>

namespace serial_arm {

namespace {

using CreateMotorBusFn = MotorBus* (*)();
using DestroyMotorBusFn = void (*)(MotorBus*);

class LoadedMotorBus final : public MotorBus {
public:
    LoadedMotorBus(void* handle, MotorBus* bus, DestroyMotorBusFn destroy) noexcept
        : handle_(handle),
          bus_(bus),
          destroy_(destroy) {}

    ~LoadedMotorBus() override {
        if(bus_ && destroy_) {
            destroy_(std::exchange(bus_, nullptr));
        }
        if(handle_) {
            dlclose(std::exchange(handle_, nullptr));
        }
    }

    LoadedMotorBus(const LoadedMotorBus&) = delete;
    LoadedMotorBus& operator=(const LoadedMotorBus&) = delete;
    LoadedMotorBus(LoadedMotorBus&&) = delete;
    LoadedMotorBus& operator=(LoadedMotorBus&&) = delete;

    tl::expected<void, MotorBusErr> configure(const std::string& config_path) override {
        return bus_->configure(config_path);
    }

    tl::expected<void, MotorBusErr> connect() override {
        return bus_->connect();
    }

    tl::expected<ActuatorState, MotorBusErr> read() override {
        return bus_->read();
    }

    tl::expected<void, MotorBusErr> activate() override {
        return bus_->activate();
    }

    tl::expected<void, MotorBusErr> write(const ActuatorCtrlCmd& cmd) override {
        return bus_->write(cmd);
    }

    tl::expected<void, MotorBusErr> stop() override {
        return bus_->stop();
    }

    tl::expected<void, MotorBusErr> deactivate() override {
        return bus_->deactivate();
    }

    tl::expected<void, MotorBusErr> recover() override {
        return bus_->recover();
    }

    const HardwareCapabilities& capabilities() const noexcept override {
        return bus_->capabilities();
    }

    void cleanup() noexcept override {
        bus_->cleanup();
    }

    std::size_t size() const noexcept override {
        return bus_->size();
    }

private:
    void* handle_{ nullptr };
    MotorBus* bus_{ nullptr };
    DestroyMotorBusFn destroy_{ nullptr };
};

void close_handle(void* handle) noexcept {
    if(handle) dlclose(handle);
}

} // namespace

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

HardwareLoader::~HardwareLoader() = default;

HardwareLoader::HardwareLoader(HardwareLoader&& other) noexcept = default;

HardwareLoader& HardwareLoader::operator=(HardwareLoader&& other) noexcept = default;

tl::expected<std::unique_ptr<MotorBus>, HardwareLoaderErr> HardwareLoader::load(const std::string& plugin, const std::string& config_path) {
    std::vector<std::string> candidates;
    candidates.push_back(plugin);
    if(plugin.find('/') == std::string::npos) {
        candidates.push_back("lib" + plugin + ".so");
    }

    void* handle{ nullptr };
    for(const auto& candidate : candidates) {
        handle = dlopen(candidate.c_str(), RTLD_NOW | RTLD_LOCAL);
        if(handle) break;
    }
    if(!handle) return tl::make_unexpected(HardwareLoaderErr::OPEN_FAILED);

    auto create = reinterpret_cast<CreateMotorBusFn>(dlsym(handle, "create_motor_bus"));
    auto destroy = reinterpret_cast<DestroyMotorBusFn>(dlsym(handle, "destroy_motor_bus"));
    if(!create || !destroy) {
        close_handle(handle);
        return tl::make_unexpected(HardwareLoaderErr::SYMBOL_FAILED);
    }

    std::unique_ptr<MotorBus, DestroyFn> raw(create(), destroy);
    if(!raw) {
        close_handle(handle);
        return tl::make_unexpected(HardwareLoaderErr::CREATE_FAILED);
    }

    const auto configured = raw->configure(config_path);
    if(!configured) {
        raw.reset();
        close_handle(handle);
        return tl::make_unexpected(HardwareLoaderErr::CONFIGURE_FAILED);
    }

    return std::unique_ptr<MotorBus>(std::make_unique<LoadedMotorBus>(handle, raw.release(), destroy));
}

} // namespace serial_arm
