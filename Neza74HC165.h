

#ifndef Neza74HC165_h
#define Neza74HC165_h

#include <Arduino.h>
#include <cstring>  // memset

// ======================= Clock Delay Configuration =======================
// Dual-phase delays to control bit-banged clock frequency.
// If you want ~100 kHz, use 5 + 5 µs (10 µs period).
// If you want ~200 kHz, use ~2 + 3 µs (≈5 µs period).
// If you want ~300 kHz, use ~1 + 2 µs (≈3 µs period).
#ifndef NEZA_74HC165_DELAY_HIGH
#define NEZA_74HC165_DELAY_HIGH 5  // microseconds while CLK is HIGH
#endif
#ifndef NEZA_74HC165_DELAY_LOW
#define NEZA_74HC165_DELAY_LOW  5  // microseconds while CLK is LOW
#endif

// Optional: small latch pulse delay during /PL (parallel load).
#ifndef NEZA_74HC165_LATCH_PULSE_US
#define NEZA_74HC165_LATCH_PULSE_US 1  // microseconds
#endif

// ======================= Optional Fast GPIO for ESP32 =====================
// Enable by defining NEZA_USE_ESP32_FAST_GPIO.
// Uses ESP-IDF driver/gpio for lower overhead/jitter.
#if defined(ARDUINO_ARCH_ESP32)
  #ifdef NEZA_USE_ESP32_FAST_GPIO
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
    inline void neza_gpio_set_level(int pin, int level) { digitalWrite(pin, level); }
    inline int  neza_gpio_get_level(int pin) { return digitalRead(pin); }
    inline void neza_gpio_init_out(int pin, int initial) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, initial);
    }
    inline void neza_gpio_init_in(int pin) {
      pinMode(pin, INPUT);
    }
  #endif
#else
  inline void neza_gpio_set_level(int pin, int level) { digitalWrite(pin, level); }
  inline int  neza_gpio_get_level(int pin) { return digitalRead(pin); }
  inline void neza_gpio_init_out(int pin, int initial) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, initial);
  }
  inline void neza_gpio_init_in(int pin) {
    pinMode(pin, INPUT);
  }
#endif


// ============================ HC165_Flags ============================
template <typename T>
class HC165_Flags {
private:
  T flags;
public:
  HC165_Flags() { reset(); }

  bool read(uint8_t bit) const { return bitRead(flags, bit); }
  T get() const { return flags; }

  // mutators
  void write(uint8_t bit, bool value) { bitWrite(flags, bit, value); }
  void on(uint8_t bit) { bitWrite(flags, bit, 1); }
  void off(uint8_t bit) { bitWrite(flags, bit, 0); }

  bool toggle(uint8_t bit) {
    flags ^= static_cast<T>(1UL << bit);
    return bitRead(flags, bit);
  }

  bool toggleIfTrue(uint8_t bit) {
    if (bitRead(flags, bit)) {
      bitClear(flags, bit);
      return true;
    }
    return false;
  }

  void reset() { flags = 0; }
  void set(T t_flags) { flags = t_flags; }
};


// ============================ Neza74HC165 ============================
// Bit-banged (GPIO) 74HC165 reader with tunable clock delays.
//

template <uint8_t _muxCount>
class Neza74HC165 {
public:
  enum class BitOrder : uint8_t { MSB_FIRST = 0, LSB_FIRST = 1 };
  enum class Logic    : uint8_t { Normal = 0, Inverted = 1 };

private:
  uint8_t clkPin  = 0;
  uint8_t loadPin = 0; // /PL (active low)
  uint8_t dataPin = 0; // Q7

  BitOrder bitOrder_ = BitOrder::MSB_FIRST;
  Logic    logic_    = Logic::Normal;

  uint8_t states[_muxCount]; // one byte per chip

public:
  Neza74HC165() {}

  // Order: begin(dataPin (Q7), loadPin (/PL), clkPin (CP))
  bool begin(uint8_t t_data, uint8_t t_load, uint8_t t_clk,
             BitOrder order, Logic logic)
  {
    // Validate pin uniqueness
    if (t_clk == t_load || t_clk == t_data || t_load == t_data) {
      return false;
    }

    clkPin  = t_clk;
    loadPin = t_load;
    dataPin = t_data;
    bitOrder_ = order;
    logic_    = logic;

    // Init pins
    neza_gpio_init_out(clkPin, LOW);
    neza_gpio_init_out(loadPin, HIGH);
    neza_gpio_init_in(dataPin);

    std::memset(states, 0, sizeof(states));

    // Read initial state immediately so first reads are real
    update();
    return true;
  }

  bool begin(uint8_t t_data, uint8_t t_load, uint8_t t_clk) {
    return begin(t_data, t_load, t_clk, BitOrder::MSB_FIRST, Logic::Normal);
  }

  // Runtime configuration helpers (optional)
  void setBitOrder(BitOrder order) { bitOrder_ = order; }
  void setLogic(Logic logic)       { logic_ = logic; }

  uint16_t getLength() const { return (uint16_t)_muxCount * 8; }

  // Latch + shift out all bytes with dual-phase delays
  void update() {
    // Parallel load: /PL low → latch → high
    neza_gpio_set_level(loadPin, LOW);
    delayMicroseconds(NEZA_74HC165_LATCH_PULSE_US);
    neza_gpio_set_level(loadPin, HIGH);

    for (uint8_t mux = 0; mux < _muxCount; mux++) {
      // Read 8 bits from Q7; sample BEFORE clock edge (common 165 pattern)
      for (int i = 7; i >= 0; i--) {
        uint8_t bit = (uint8_t)neza_gpio_get_level(dataPin) & 0x01;

        if (logic_ == Logic::Inverted) bit ^= 1;

        uint8_t targetBit = (bitOrder_ == BitOrder::MSB_FIRST)
                              ? (uint8_t)i
                              : (uint8_t)(7 - i);

        bitWrite(states[mux], targetBit, bit);

        // Clock HIGH phase
        neza_gpio_set_level(clkPin, HIGH);
        delayMicroseconds(NEZA_74HC165_DELAY_HIGH);

        // Clock LOW phase
        neza_gpio_set_level(clkPin, LOW);
        delayMicroseconds(NEZA_74HC165_DELAY_LOW);
      }
    }
  }

  bool read(uint16_t n) const {
    if (n >= ((uint16_t)_muxCount * 8)) {
      return false;
    }
    return bitRead(states[(n >> 3)], (n & 0x07));
  }

  bool readPin(uint16_t n) const { return read(n); }

  // Convenience: raw byte access
  uint8_t readByte(uint8_t chip) const {
    return (chip < _muxCount) ? states[chip] : 0;
  }
  const uint8_t* data() const { return states; }
};

// Preserve alias template (2x chips)
template <uint8_t _muxinCount>
class HC165_165 : public Neza74HC165<_muxinCount * 2> {};


// ============================ HC165_Button ============================
// Button/PIR state machine with:
//  - default doublePressTime
//  - public callback setter
//  - fewer millis() calls
#define NEZA_BTN_STATE                 0
#define NEZA_BTN_STATE_CHANGED         1
#define NEZA_BTN_STATE_HELD            2
#define NEZA_BTN_STATE_HOLD_TRIGGERED  3
#define NEZA_BTN_STATE_DBL_ACTIVE      4
#define NEZA_BTN_STATE_DBL_TRIGGERED   5
#define NEZA_BTN_STATE_DBL_IGNORE_REL  6

#ifndef NEZA_BTN_HOLD_THRESH
#define NEZA_BTN_HOLD_THRESH   500
#endif

#ifndef NEZA_BTN_DBL_DEFAULT
#define NEZA_BTN_DBL_DEFAULT   350
#endif

#define NEZA_NONE      0
#define NEZA_RELEASED  1
#define NEZA_PRESSED   2
#define NEZA_HELD      3
#define NEZA_DOUBLE    4

class HC165_Button {
private:
  uint32_t debounce = 0;
  uint32_t holdDebounce = 0;
  uint32_t dblDebounce = 0;
  uint16_t doublePressTime = NEZA_BTN_DBL_DEFAULT;

  HC165_Flags<uint8_t> flags;
  void (*onUpdateCallback)(uint8_t type) = nullptr;

  bool isPressed_() const { return flags.read(NEZA_BTN_STATE); }
  bool stateChanged_() const { return flags.read(NEZA_BTN_STATE_CHANGED); }
  bool btnHeld_() const { return flags.read(NEZA_BTN_STATE_HELD); }

  bool setDblPress_(uint32_t now) {
    if (flags.toggleIfTrue(NEZA_BTN_STATE_DBL_ACTIVE)) {
      if ((uint32_t)(now - dblDebounce) <= (uint32_t)doublePressTime) {
        flags.on(NEZA_BTN_STATE_DBL_TRIGGERED);
        return true;
      }
    }
    return false;
  }

  void setDblRelease_(uint32_t now) {
    if (!flags.read(NEZA_BTN_STATE_DBL_ACTIVE)) {
      flags.on(NEZA_BTN_STATE_DBL_ACTIVE);
      dblDebounce = now;
    }
  }

public:
  HC165_Button() {
    flags.reset();
  }

  void begin() {
    debounce = millis();
  }

  //  public callback setter
  void setOnUpdate(void (*fptr)(uint8_t type)) {
    onUpdateCallback = fptr;
  }

  void setDoublePressThreshold(uint16_t t_thres) {
    doublePressTime = t_thres;
  }

  bool update(bool state, uint16_t t_debounce = 50, bool t_logic = LOW) {
    flags.off(NEZA_BTN_STATE_CHANGED);

    const uint32_t now = millis();
    bool pressed = (t_logic == LOW) ? !state : state;

    if ((uint32_t)(now - debounce) >= (uint32_t)t_debounce) {
      debounce = now;
      if (pressed != flags.read(NEZA_BTN_STATE)) {
        holdDebounce = now;
        flags.write(NEZA_BTN_STATE, pressed);
        flags.write(NEZA_BTN_STATE_CHANGED, true);
        flags.write(NEZA_BTN_STATE_HELD, pressed);
        return true;
      }
    }
    return false;
  }

  void updateWithCallback(bool state,
                          uint16_t t_debounce = 50,
                          bool t_logic = LOW,
                          uint16_t t_hold = NEZA_BTN_HOLD_THRESH,
                          bool ignoreAfterHold = false)
  {
    if (update(state, t_debounce, t_logic) && onUpdateCallback) {
      read(t_hold, ignoreAfterHold);
    }
  }

  uint8_t read(uint16_t t_hold = NEZA_BTN_HOLD_THRESH, bool ignoreAfterHold = false) {
    if (stateChanged_()) {
      if (held(t_hold)) {
        if (onUpdateCallback) onUpdateCallback(NEZA_HELD);
        return NEZA_HELD;
      } else {
        if (doublePressed()) {
          if (onUpdateCallback) onUpdateCallback(NEZA_DOUBLE);
          return NEZA_DOUBLE;
        } else if (isPressed_()) {
          if (onUpdateCallback) onUpdateCallback(NEZA_PRESSED);
          return NEZA_PRESSED;
        } else if (released(ignoreAfterHold)) {
          if (onUpdateCallback) onUpdateCallback(NEZA_RELEASED);
          return NEZA_RELEASED;
        }
      }
    }
    return NEZA_NONE;
  }

  bool doublePressed(bool allowRelease = false) {
    bool state = flags.toggleIfTrue(NEZA_BTN_STATE_DBL_TRIGGERED);
    if (state && !allowRelease) {
      flags.on(NEZA_BTN_STATE_DBL_IGNORE_REL);
    }
    return state;
  }

  bool pressed() {
    const uint32_t now = millis();
    bool state = stateChanged_() && isPressed_();
    if (state) {
      if (setDblPress_(now)) return false;
    }
    return state;
  }

  bool released(bool ignoreAfterHold = false) {
    const uint32_t now = millis();
    bool state = stateChanged_() && !isPressed_();

    bool holdTriggered = flags.read(NEZA_BTN_STATE_HOLD_TRIGGERED);
    bool dblTriggered  = flags.read(NEZA_BTN_STATE_DBL_IGNORE_REL);

    if (state) {
      flags.off(NEZA_BTN_STATE_HOLD_TRIGGERED);
      flags.off(NEZA_BTN_STATE_DBL_IGNORE_REL);
    }

    if (state && ignoreAfterHold && holdTriggered) {
      return false;
    }

    if (state) {
      setDblRelease_(now);
      if (dblTriggered) return false;
    }
    return state;
  }

  bool held(uint16_t holdTime = NEZA_BTN_HOLD_THRESH) {
    const uint32_t now = millis();
    bool state = isPressed_() && btnHeld_() && ((uint32_t)(now - holdDebounce) >= (uint32_t)holdTime);

    if (state && !flags.read(NEZA_BTN_STATE_HOLD_TRIGGERED)) {
      flags.on(NEZA_BTN_STATE_HOLD_TRIGGERED);
      flags.off(NEZA_BTN_STATE_DBL_ACTIVE);
      return true;
    }
    return false;
  }

  bool latched()   { return stateChanged_() &&  isPressed_(); }
  bool unlatched() { return stateChanged_() && !isPressed_(); }
  bool getCurrentState() { return isPressed_(); }
};

#endif // Neza74HC165_h
