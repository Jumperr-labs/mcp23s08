#include "ModelingFramework.h"
#include <cassert>
#include <utility>
#include <unordered_map>
#include "iJemuInterface.h"
#include <iostream>
#include <memory>

iJemuInterface *jemu_interface_ = nullptr;

std::unordered_map<int, callback_t> timers_callbacks_map_;

DLL_EXPORT void InitJemuInterface(iJemuInterface *jemu_interface) {
    jemu_interface_ = jemu_interface;
}

int GetPinNumber(const std::string &pin_name) {
    assert(jemu_interface_);
    return jemu_interface_->GetPinNumber(pin_name.c_str());
}

iSpiSlave* CreateSpiSlave(SpiSlaveConfig &spi_config) {
    assert(jemu_interface_);
    return jemu_interface_->CreateSpiSlave(spi_config);
}

iI2cSlave* CreateI2cSlave() {
    assert(jemu_interface_);
    return jemu_interface_->CreateI2cSlave();
}

bool GetPinLevel(int pin_number) {
    assert(jemu_interface_);
    return jemu_interface_->GetPinLevel(pin_number);
}

bool GetPinLevel(const std::string &pin_name) {
    assert(jemu_interface_);
    const auto pin_number = jemu_interface_->GetPinNumber(pin_name.c_str());
    return jemu_interface_->GetPinLevel(pin_number);    
}

void SetPinLevel(int pin_number, bool pin_level) {
    assert(jemu_interface_);
    jemu_interface_->SetPinLevel(pin_number, pin_level);
}

void SetPinLevel(const std::string &pin_name, bool pin_level) {
    assert(jemu_interface_);
    const auto pin_number = jemu_interface_->GetPinNumber(pin_name.c_str());
    jemu_interface_->SetPinLevel(pin_number, pin_level);
}

int32_t GetNextInt32FromDataGenerator(std::string name) {
    assert(jemu_interface_);
    return jemu_interface_->GetNextInt32FromDataGenerator(name.c_str());
}

double GetNextDoubleFromDataGenerator(std::string name) {
    assert(jemu_interface_);
    return jemu_interface_->GetNextDoubleFromDataGenerator(name.c_str());
}

int16_t GetNextInt16FromDataGenerator(std::string name) {
    assert(jemu_interface_);
    return jemu_interface_->GetNextInt16FromDataGenerator(name.c_str());
}

uint8_t GetNextUInt8FromDataGenerator(std::string name) {
    assert(jemu_interface_);
    return jemu_interface_->GetNextUInt8FromDataGenerator(name.c_str());
}

double GetCachedValueFromDataGenerator(std::string name) {
    assert(jemu_interface_);
    return jemu_interface_->GetCachedValueFromDataGenerator(name.c_str());
}

void TimedCallback (int id) {
    assert(timers_callbacks_map_.find(id) != timers_callbacks_map_.end());
    timers_callbacks_map_[id]();
}

int AddTimedCallback(uint64_t ns, const callback_t &callback, bool run_once) {
    assert(jemu_interface_);
    int id = jemu_interface_->AddTimedCallback(ns, &TimedCallback, run_once);
    timers_callbacks_map_[id] = callback;
    return id;
}

void CancelTimedCallback(int id) {
    assert(jemu_interface_);
    jemu_interface_->CancelTimedCallback(id);
}

void UpdateTimedCallback(int id, uint64_t ns) {
    assert(jemu_interface_);
    jemu_interface_->UpdateTimedCallback(id,ns);
}

void SetMultiplePinsLevel(const pin_level_changes_t& pin_changes) {
    assert(jemu_interface_);

    if (pin_changes.empty()) 
        return;

    std::unique_ptr<WireChange> arr(new WireChange[pin_changes.size()]);
    std::copy(pin_changes.begin(),pin_changes.end(), arr.get() );
    jemu_interface_->SetMultiplePinsLevel(arr.get(), pin_changes.size());
}

void SetPinDirection(uint32_t pin_id, pin_direction_t direction, bool pullup) {
    assert(jemu_interface_);
    jemu_interface_->SetPinDirection(pin_id, direction, pullup);
}

class PinChangeLevelEventCallback : public iPinChangeLevelEventCallback {
  public:
    explicit PinChangeLevelEventCallback(pin_change_level_callback_t pin_change_level_callback) :
            pin_change_level_callback_(std::move(pin_change_level_callback)) {}
    void Call(WireLogicLevelEvent* arr, size_t size) override {
        WireLogicLevelEventVector_t vec;
        std::copy(arr, arr + size, std::back_inserter(vec));
        pin_change_level_callback_(vec);
    }
  private:
    pin_change_level_callback_t pin_change_level_callback_;
};

int SetPinChangeLevelEventCallback(uint32_t pin_id, const pin_change_level_callback_t& callback) {
    assert(jemu_interface_);
    static int id = 0;
    auto pin_change_level_callback = new PinChangeLevelEventCallback(callback);
    jemu_interface_->SetPinChangeLevelEventCallback(pin_id, pin_change_level_callback);
    return id++;
}
