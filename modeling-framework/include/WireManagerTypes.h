#pragma once

#include <cstdint>
#include <utility>
#include <vector>
#include <functional>

typedef enum {
    FALLING = 0,
    RISING = 1
} transition_type_t;

typedef enum {
    DIRECTION_IN = 0,
    DIRECTION_OUT = 1,
    DIRECTION_ALTERNATE_FUNCTION = 2,
    UNDEFINED = 3
} pin_direction_t;

typedef enum {
    DISABLED = 0,
    PULL_UP = 1,
    PULL_DOWN = 2,
} pin_pullup_t;

/*
 * PinChangeEvent used to send event by PinManager and should
 * be changed only if GPIO tests are changing accordingly
 * */

typedef struct {
    transition_type_t type;
    uint32_t wire_number;
    uint32_t pin_number;
    double voltage;
} WireLogicLevelEvent;

typedef struct {
    uint32_t length;
    uint8_t *data;
    uint32_t wire_number;
    uint32_t pin_number;
} WireDataEvent;

typedef struct {
    bool is_level_high;
    double voltage;
    uint32_t wire_number;
} ConstWireState;


struct WireChange {
    WireChange() = default;
    WireChange(uint32_t wire_number, bool level_is_high, uint32_t pin_number) :
        wire_number_(wire_number),
        level_is_high_(level_is_high),
        pin_number_(pin_number) {}

  uint32_t wire_number_{0};
  bool level_is_high_{false};
  uint32_t pin_number_{0};
};

typedef std::vector<WireChange> pin_level_changes_t;
typedef std::vector<WireLogicLevelEvent> WireLogicLevelEventVector_t;
typedef std::function<void(WireLogicLevelEventVector_t& notifications )> pin_change_level_callback_t;

class iPinChangeLevelEventCallback {
  public:
    virtual void Call(WireLogicLevelEvent* arr, size_t size) = 0;
};
