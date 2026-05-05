#ifndef Neza74HC165_h
#define Neza74HC165_h

#include <Arduino.h>
#include <string.h>
#include <stddef.h>

// ============================================================================
// Neza74HC165 Library
//
// GPIO bit-banged 74HC165 chain reader.
// Includes HC165_Button helper for debounce, press, release, hold and double.
// ============================================================================


// ======================= Clock Delay Configuration ===========================
//
// You can override before including this file:
//
// #define NEZA_74HC165_DELAY_HIGH 0
// #define NEZA_74HC165_DELAY_LOW  0
//
// Default is safe and stable for most boards.
// ============================================================================

#ifndef NEZA_74HC165_DELAY_HIGH
#define NEZA_74HC165_DELAY_HIGH 5
#endif

#ifndef NEZA_74HC165_DELAY_LOW
#define NEZA_74HC165_DELAY_LOW 5
#endif

#ifndef NEZA_74HC165_LATCH_PULSE_US
#define NEZA_74HC165_LATCH_PULSE_US 1
#endif


// ======================= Optional Fast GPIO for ESP32 ========================
//
// To enable:
//
// #define NEZA_USE_ESP32_FAST_GPIO
// #include "Neza74HC165.h"
// ============================================================================

#if defined(ARDUINO_ARCH_ESP32) && defined(NEZA_USE_ESP32_FAST_GPIO)

  #include "driver/gpio.h"

  inline void neza_gpio_set_level(int pin, int level) {
    gpio_set_level((gpio_num_t)pin, level);
  }

  inline int neza_gpio_get_level(int pin) {
    return gpio_get_level((gpio_num_t)pin);
  }

  inline void neza_gpio_init_out(int pin, int initial) {
    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)pin, initial);
  }

  inline void neza_gpio_init_in(int pin) {
    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
  }

#else

  inline void neza_gpio_set_level(int pin, int level) {
    digitalWrite(pin, level);
  }

  inline int neza_gpio_get_level(int pin) {
    return digitalRead(pin);
  }

  inline void neza_gpio_init_out(int pin, int initial) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, initial);
  }

  inline void neza_gpio_init_in(int pin) {
    pinMode(pin, INPUT);
  }

#endif


// ============================ HC165_Flags ====================================
//
// Small safe flag helper.
// ============================================================================

template <typename T>
class HC165_Flags {
private:
  T flags_ = 0;

  static constexpr uint8_t bitCount_() {
    return static_cast<uint8_t>(sizeof(T) * 8u);
  }

public:
  HC165_Flags() {
    reset();
  }

  bool read(uint8_t bit) const {
    if (bit >= bitCount_()) return false;
    return ((flags_ >> bit) & static_cast<T>(1)) != 0;
  }

  T get() const {
    return flags_;
  }

  void write(uint8_t bit, bool value) {
    if (bit >= bitCount_()) return;

    if (value) {
      on(bit);
    } else {
      off(bit);
    }
  }

  void on(uint8_t bit) {
    if (bit >= bitCount_()) return;
    flags_ |= static_cast<T>(static_cast<T>(1) << bit);
  }

  void off(uint8_t bit) {
    if (bit >= bitCount_()) return;
    flags_ &= static_cast<T>(~static_cast<T>(static_cast<T>(1) << bit));
  }

  bool toggle(uint8_t bit) {
    if (bit >= bitCount_()) return false;

    flags_ ^= static_cast<T>(static_cast<T>(1) << bit);
    return read(bit);
  }

  bool consumeIfSet(uint8_t bit) {
    if (read(bit)) {
      off(bit);
      return true;
    }

    return false;
  }

  // Backward-compatible alias.
  bool toggleIfTrue(uint8_t bit) {
    return consumeIfSet(bit);
  }

  void reset() {
    flags_ = 0;
  }

  void set(T val) {
    flags_ = val;
  }
};


// ============================ Neza74HC165 ====================================
//
// Template argument:
//   _muxCount = number of physical 74HC165 chips.
//   1 chip = 8 inputs.
// ============================================================================

template <size_t _muxCount>
class Neza74HC165 {
  static_assert(_muxCount > 0, "_muxCount must be greater than 0");
  static_assert(_muxCount <= 255, "_muxCount too large");

public:
  enum class BitOrder : uint8_t {
    MSB_FIRST = 0,
    LSB_FIRST = 1
  };

  enum class Logic : uint8_t {
    Normal = 0,
    Inverted = 1
  };

private:
  uint8_t clkPin_ = 0;
  uint8_t loadPin_ = 0;
  uint8_t dataPin_ = 0;

  int8_t inhibitPin_ = -1;
  bool useInhibitPin_ = false;

  BitOrder bitOrder_ = BitOrder::MSB_FIRST;
  Logic logic_ = Logic::Normal;

  bool initialized_ = false;

  uint8_t states_[_muxCount] = {};

private:
  bool pinsAreUnique_(uint8_t data, uint8_t load, uint8_t clk) const {
    return !(data == load || data == clk || load == clk);
  }

  bool pinsAreUnique_(uint8_t data, uint8_t load, uint8_t clk, int8_t inhibit) const {
    if (!pinsAreUnique_(data, load, clk)) return false;

    if (inhibit < 0) return true;

    return !(inhibit == data || inhibit == load || inhibit == clk);
  }

public:
  Neza74HC165() = default;

  bool begin(uint8_t t_data,
             uint8_t t_load,
             uint8_t t_clk,
             BitOrder order = BitOrder::MSB_FIRST,
             Logic logic = Logic::Normal)
  {
    if (!pinsAreUnique_(t_data, t_load, t_clk)) {
      initialized_ = false;
      return false;
    }

    dataPin_ = t_data;
    loadPin_ = t_load;
    clkPin_ = t_clk;

    inhibitPin_ = -1;
    useInhibitPin_ = false;

    bitOrder_ = order;
    logic_ = logic;

    neza_gpio_init_out(clkPin_, LOW);
    neza_gpio_init_out(loadPin_, HIGH);
    neza_gpio_init_in(dataPin_);

    memset(states_, 0, sizeof(states_));

    initialized_ = true;

    update();

    return true;
  }

  // Optional clock inhibit pin version.
  // If used, inhibit pin is driven LOW to allow clocking.
  bool begin(uint8_t t_data,
             uint8_t t_load,
             uint8_t t_clk,
             int8_t t_inhibit,
             BitOrder order = BitOrder::MSB_FIRST,
             Logic logic = Logic::Normal)
  {
    if (!pinsAreUnique_(t_data, t_load, t_clk, t_inhibit)) {
      initialized_ = false;
      return false;
    }

    dataPin_ = t_data;
    loadPin_ = t_load;
    clkPin_ = t_clk;

    inhibitPin_ = t_inhibit;
    useInhibitPin_ = t_inhibit >= 0;

    bitOrder_ = order;
    logic_ = logic;

    neza_gpio_init_out(clkPin_, LOW);
    neza_gpio_init_out(loadPin_, HIGH);
    neza_gpio_init_in(dataPin_);

    if (useInhibitPin_) {
      neza_gpio_init_out(inhibitPin_, LOW);
    }

    memset(states_, 0, sizeof(states_));

    initialized_ = true;

    update();

    return true;
  }

  void setBitOrder(BitOrder order) {
    bitOrder_ = order;
  }

  void setLogic(Logic logic) {
    logic_ = logic;
  }

  bool isInitialized() const {
    return initialized_;
  }

  size_t chipCount() const {
    return _muxCount;
  }

  uint16_t getLength() const {
    return static_cast<uint16_t>(_muxCount * 8u);
  }

  void update() {
    if (!initialized_) return;

    if (useInhibitPin_) {
      neza_gpio_set_level(inhibitPin_, LOW);
    }

    // Parallel load.
    neza_gpio_set_level(loadPin_, LOW);

#if NEZA_74HC165_LATCH_PULSE_US > 0
    delayMicroseconds(NEZA_74HC165_LATCH_PULSE_US);
#endif

    neza_gpio_set_level(loadPin_, HIGH);

    for (size_t mux = 0; mux < _muxCount; mux++) {
      uint8_t value = 0;

      for (int8_t i = 7; i >= 0; i--) {
        uint8_t bit = static_cast<uint8_t>(neza_gpio_get_level(dataPin_)) & 0x01;

        if (logic_ == Logic::Inverted) {
          bit ^= 1;
        }

        const uint8_t targetBit =
          bitOrder_ == BitOrder::MSB_FIRST
          ? static_cast<uint8_t>(i)
          : static_cast<uint8_t>(7 - i);

        if (bit) {
          value |= static_cast<uint8_t>(1u << targetBit);
        }

        neza_gpio_set_level(clkPin_, HIGH);

#if NEZA_74HC165_DELAY_HIGH > 0
        delayMicroseconds(NEZA_74HC165_DELAY_HIGH);
#endif

        neza_gpio_set_level(clkPin_, LOW);

#if NEZA_74HC165_DELAY_LOW > 0
        delayMicroseconds(NEZA_74HC165_DELAY_LOW);
#endif
      }

      states_[mux] = value;
    }
  }

  bool read(uint16_t n) const {
    if (n >= getLength()) return false;

    return ((states_[n >> 3] >> (n & 0x07)) & 0x01) != 0;
  }

  bool readPin(uint16_t n) const {
    return read(n);
  }

  uint8_t readByte(size_t chip) const {
    if (chip >= _muxCount) return 0;
    return states_[chip];
  }

  uint8_t readByteReversed(size_t chip) const {
    if (chip >= _muxCount) return 0;

    uint8_t v = states_[chip];

    v = static_cast<uint8_t>(((v & 0xF0) >> 4) | ((v & 0x0F) << 4));
    v = static_cast<uint8_t>(((v & 0xCC) >> 2) | ((v & 0x33) << 2));
    v = static_cast<uint8_t>(((v & 0xAA) >> 1) | ((v & 0x55) << 1));

    return v;
  }

  uint32_t readUInt32() const {
    uint32_t value = 0;
    const size_t maxBytes = _muxCount > 4 ? 4 : _muxCount;

    for (size_t i = 0; i < maxBytes; i++) {
      value |= static_cast<uint32_t>(states_[i]) << (i * 8u);
    }

    return value;
  }

  const uint8_t* data() const {
    return states_;
  }
};


// Each HC165_165<N> manages N*2 physical 74HC165 chips.
// Example:
// HC165_165<1> = 2 chips = 16 inputs.
// HC165_165<2> = 4 chips = 32 inputs.

template <size_t _muxinCount>
class HC165_165 : public Neza74HC165<_muxinCount * 2u> {
  static_assert(_muxinCount > 0, "_muxinCount must be greater than 0");
  static_assert((_muxinCount * 2u) <= 255, "_muxinCount too large");
};


// ============================ HC165_Button ===================================

#ifndef NEZA_BTN_HOLD_THRESH
#define NEZA_BTN_HOLD_THRESH 500
#endif

#ifndef NEZA_BTN_DBL_DEFAULT
#define NEZA_BTN_DBL_DEFAULT 350
#endif


#define NEZA_NONE      0
#define NEZA_RELEASED  1
#define NEZA_PRESSED   2
#define NEZA_HELD      3
#define NEZA_DOUBLE    4

// Alias because with double-click enabled, NEZA_PRESSED means confirmed click.
#define NEZA_CLICK     NEZA_PRESSED


enum class HC165_ButtonEvent : uint8_t {
  None = NEZA_NONE,
  Released = NEZA_RELEASED,
  Pressed = NEZA_PRESSED,
  Click = NEZA_CLICK,
  Held = NEZA_HELD,
  Double = NEZA_DOUBLE
};


enum class ActiveLevel : uint8_t {
  Low = 0,
  High = 1
};


#define BTN_FLAG_STATE      0
#define BTN_FLAG_CHANGED    1
#define BTN_FLAG_HOLD_ARMED 2
#define BTN_FLAG_HOLD_FIRED 3


class HC165_Button {
public:
  HC165_Button() = default;

  void begin() {
    flags_.reset();

    clickState_ = ClickState::IDLE;

    rawChangeTs_ = millis();
    holdTs_ = 0;
    firstReleaseTs_ = 0;

    clearEvents_();

    initialized_ = true;
    rawSeeded_ = false;
    lastRawPressed_ = false;
  }

  bool isInitialized() const {
    return initialized_;
  }

  void setOnUpdate(void (*fptr)(uint8_t type)) {
    callback_ = fptr;
  }

  void setDoublePressThreshold(uint16_t ms) {
    doublePressTime_ = ms;
  }

  uint16_t getDoublePressThreshold() const {
    return doublePressTime_;
  }

  bool update(bool rawState,
              uint16_t debounceMs = 50,
              ActiveLevel logic = ActiveLevel::Low)
  {
    if (!initialized_) return false;

    flags_.off(BTN_FLAG_CHANGED);

    const uint32_t now = millis();
    const bool pressed = logic == ActiveLevel::Low ? !rawState : rawState;

    // First sample only seeds raw state.
    if (!rawSeeded_) {
      rawSeeded_ = true;
      lastRawPressed_ = pressed;
      rawChangeTs_ = now;
      return false;
    }

    // Raw changed, restart debounce timer.
    if (pressed != lastRawPressed_) {
      lastRawPressed_ = pressed;
      rawChangeTs_ = now;
      return false;
    }

    // Not stable long enough.
    if ((uint32_t)(now - rawChangeTs_) < (uint32_t)debounceMs) {
      return false;
    }

    // No debounced state change.
    if (pressed == isPressed_()) {
      return false;
    }

    // If a single click was waiting and timeout expired, confirm it now.
    if (doublePressTime_ > 0 &&
        clickState_ == ClickState::WAIT_SECOND &&
        (uint32_t)(now - firstReleaseTs_) > (uint32_t)doublePressTime_) {
      queueEvent_(NEZA_PRESSED);
      clickState_ = ClickState::IDLE;
    }

    flags_.write(BTN_FLAG_STATE, pressed);
    flags_.write(BTN_FLAG_CHANGED, true);

    if (pressed) {
      onPressedEdge_(now);
    } else {
      onReleasedEdge_(now);
    }

    return true;
  }

  uint8_t updateWithCallback(bool rawState,
                             uint16_t debounceMs = 50,
                             ActiveLevel logic = ActiveLevel::Low,
                             uint16_t holdThresh = NEZA_BTN_HOLD_THRESH,
                             bool ignoreAfterHold = false)
  {
    update(rawState, debounceMs, logic);

    const uint8_t ev = read(holdThresh, ignoreAfterHold);

    if (callback_ && ev != NEZA_NONE) {
      callback_(ev);
    }

    return ev;
  }

  uint8_t read(uint16_t holdThresh = NEZA_BTN_HOLD_THRESH,
               bool ignoreAfterHold = false)
  {
    if (!initialized_) return NEZA_NONE;

    const uint32_t now = millis();

    checkHold_(now, holdThresh);
    checkSingleClickTimeout_(now);

    uint8_t ev = popEvent_();

    if (ev == NEZA_NONE) {
      return NEZA_NONE;
    }

    // Optional: ignore release after hold.
    if (ignoreAfterHold &&
        ev == NEZA_RELEASED &&
        flags_.read(BTN_FLAG_HOLD_FIRED)) {
      flags_.off(BTN_FLAG_HOLD_FIRED);
      return NEZA_NONE;
    }

    if (ev == NEZA_RELEASED) {
      flags_.off(BTN_FLAG_HOLD_FIRED);
    }

    return ev;
  }

  HC165_ButtonEvent readEvent(uint16_t holdThresh = NEZA_BTN_HOLD_THRESH,
                              bool ignoreAfterHold = false)
  {
    return static_cast<HC165_ButtonEvent>(read(holdThresh, ignoreAfterHold));
  }

  bool getCurrentState() const {
    return isPressed_();
  }

  bool latched() const {
    return flags_.read(BTN_FLAG_CHANGED) && isPressed_();
  }

  bool unlatched() const {
    return flags_.read(BTN_FLAG_CHANGED) && !isPressed_();
  }

  bool hasPendingEvent() const {
    return eventHead_ != eventTail_;
  }

private:
  enum class ClickState : uint8_t {
    IDLE,
    FIRST_DOWN,
    WAIT_SECOND,
    SECOND_DOWN
  };

  static constexpr uint8_t EVENT_QUEUE_SIZE = 4;

private:
  bool isPressed_() const {
    return flags_.read(BTN_FLAG_STATE);
  }

  void onPressedEdge_(uint32_t now) {
    holdTs_ = now;

    flags_.on(BTN_FLAG_HOLD_ARMED);
    flags_.off(BTN_FLAG_HOLD_FIRED);

    if (doublePressTime_ == 0) {
      queueEvent_(NEZA_PRESSED);
      clickState_ = ClickState::IDLE;
      return;
    }

    if (clickState_ == ClickState::WAIT_SECOND &&
        (uint32_t)(now - firstReleaseTs_) <= (uint32_t)doublePressTime_) {
      clickState_ = ClickState::SECOND_DOWN;
    } else {
      clickState_ = ClickState::FIRST_DOWN;
    }
  }

  void onReleasedEdge_(uint32_t now) {
    flags_.off(BTN_FLAG_HOLD_ARMED);

    if (flags_.read(BTN_FLAG_HOLD_FIRED)) {
      queueEvent_(NEZA_RELEASED);
      clickState_ = ClickState::IDLE;
      return;
    }

    if (doublePressTime_ == 0) {
      queueEvent_(NEZA_RELEASED);
      clickState_ = ClickState::IDLE;
      return;
    }

    if (clickState_ == ClickState::FIRST_DOWN) {
      clickState_ = ClickState::WAIT_SECOND;
      firstReleaseTs_ = now;
      return;
    }

    if (clickState_ == ClickState::SECOND_DOWN &&
        (uint32_t)(now - firstReleaseTs_) <= (uint32_t)doublePressTime_) {
      queueEvent_(NEZA_DOUBLE);
      clickState_ = ClickState::IDLE;
      return;
    }

    clickState_ = ClickState::IDLE;
  }

  void checkHold_(uint32_t now, uint16_t holdThresh) {
    if (isPressed_() &&
        flags_.read(BTN_FLAG_HOLD_ARMED) &&
        !flags_.read(BTN_FLAG_HOLD_FIRED)) {
      if ((uint32_t)(now - holdTs_) >= (uint32_t)holdThresh) {
        flags_.on(BTN_FLAG_HOLD_FIRED);
        flags_.off(BTN_FLAG_HOLD_ARMED);

        clickState_ = ClickState::IDLE;

        queueEvent_(NEZA_HELD);
      }
    }
  }

  void checkSingleClickTimeout_(uint32_t now) {
    if (doublePressTime_ > 0 &&
        clickState_ == ClickState::WAIT_SECOND &&
        (uint32_t)(now - firstReleaseTs_) > (uint32_t)doublePressTime_) {
      queueEvent_(NEZA_PRESSED);
      clickState_ = ClickState::IDLE;
    }
  }

  void clearEvents_() {
    for (uint8_t i = 0; i < EVENT_QUEUE_SIZE; i++) {
      eventQueue_[i] = NEZA_NONE;
    }

    eventHead_ = 0;
    eventTail_ = 0;
  }

  void queueEvent_(uint8_t ev) {
    if (ev == NEZA_NONE) return;

    const uint8_t next = static_cast<uint8_t>((eventHead_ + 1u) % EVENT_QUEUE_SIZE);

    // Queue full: drop newest event to avoid overwriting unread event.
    if (next == eventTail_) {
      return;
    }

    eventQueue_[eventHead_] = ev;
    eventHead_ = next;
  }

  uint8_t popEvent_() {
    if (eventHead_ == eventTail_) {
      return NEZA_NONE;
    }

    const uint8_t ev = eventQueue_[eventTail_];
    eventQueue_[eventTail_] = NEZA_NONE;

    eventTail_ = static_cast<uint8_t>((eventTail_ + 1u) % EVENT_QUEUE_SIZE);

    return ev;
  }

private:
  HC165_Flags<uint8_t> flags_;

  ClickState clickState_ = ClickState::IDLE;

  uint32_t rawChangeTs_ = 0;
  uint32_t holdTs_ = 0;
  uint32_t firstReleaseTs_ = 0;

  uint16_t doublePressTime_ = NEZA_BTN_DBL_DEFAULT;

  uint8_t eventQueue_[EVENT_QUEUE_SIZE] = {};
  uint8_t eventHead_ = 0;
  uint8_t eventTail_ = 0;

  bool initialized_ = false;
  bool rawSeeded_ = false;
  bool lastRawPressed_ = false;

  void (*callback_)(uint8_t type) = nullptr;
};

#endif // Neza74HC165_h
