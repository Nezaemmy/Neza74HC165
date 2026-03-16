#ifndef Neza74HC165_h
#define Neza74HC165_h

#include <Arduino.h>
#include <cstring>  // memset

// ======================= Clock Delay Configuration =======================
// Dual-phase delays to control bit-banged clock frequency.
// ~100 kHz: 5 + 5 µs | ~200 kHz: 2 + 3 µs | ~300 kHz: 1 + 2 µs
#ifndef NEZA_74HC165_DELAY_HIGH
#define NEZA_74HC165_DELAY_HIGH 5   // µs while CLK is HIGH
#endif
#ifndef NEZA_74HC165_DELAY_LOW
#define NEZA_74HC165_DELAY_LOW  5   // µs while CLK is LOW
#endif

// Small latch pulse during /PL (parallel load)
#ifndef NEZA_74HC165_LATCH_PULSE_US
#define NEZA_74HC165_LATCH_PULSE_US 1
#endif

// ======================= Optional Fast GPIO for ESP32 ====================
// Enable with: #define NEZA_USE_ESP32_FAST_GPIO
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
  inline void neza_gpio_set_level(int pin, int level) { digitalWrite(pin, level); }
  inline int  neza_gpio_get_level(int pin)            { return digitalRead(pin); }
  inline void neza_gpio_init_out(int pin, int initial) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, initial);
  }
  inline void neza_gpio_init_in(int pin) {
    pinMode(pin, INPUT);
  }
#endif


// ============================ HC165_Flags ====================================
// Generic bit-flag wrapper around any unsigned integer type.
// ============================================================================
template <typename T>
class HC165_Flags {
private:
  T flags = 0;

public:
  HC165_Flags() { reset(); }

  // ── Readers ──────────────────────────────────────────────────────────────
  bool read(uint8_t bit) const { return bitRead(flags, bit); }
  T    get()             const { return flags; }

  // ── Mutators ─────────────────────────────────────────────────────────────
  void write(uint8_t bit, bool value) { bitWrite(flags, bit, value); }
  void on(uint8_t bit)                { bitWrite(flags, bit, 1); }
  void off(uint8_t bit)               { bitWrite(flags, bit, 0); }

  bool toggle(uint8_t bit) {
    flags ^= static_cast<T>(1UL << bit);
    return bitRead(flags, bit);
  }

  bool consumeIfSet(uint8_t bit) {
    if (bitRead(flags, bit)) {
      bitClear(flags, bit);
      return true;
    }
    return false;
  }
  // Backward-compat alias
  bool toggleIfTrue(uint8_t bit) { return consumeIfSet(bit); }

  void reset()    { flags = 0; }
  void set(T val) { flags = val; }
};


// ============================ Neza74HC165 ====================================
// Bit-banged (GPIO) 74HC165 chain reader.
// _muxCount = number of daisy-chained ICs (each contributes 8 bits).
// ============================================================================
template <uint8_t _muxCount>
class Neza74HC165 {
  static_assert(_muxCount > 0, "_muxCount must be > 0");

public:
  enum class BitOrder : uint8_t { MSB_FIRST = 0, LSB_FIRST = 1 };
  enum class Logic    : uint8_t { Normal = 0, Inverted = 1 };

private:
  uint8_t clkPin  = 0;
  uint8_t loadPin = 0;   // /PL — active low
  uint8_t dataPin = 0;   // Q7 serial output

  BitOrder bitOrder_   = BitOrder::MSB_FIRST;
  Logic    logic_      = Logic::Normal;
  bool     initialized_ = false;

  uint8_t states[_muxCount] = {};   // one byte per IC

public:
  Neza74HC165() {}

  // begin(dataPin/Q7, loadPin//PL, clkPin/CP, bitOrder, logic)
  // Returns false if any two pins share the same number.
  bool begin(uint8_t t_data, uint8_t t_load, uint8_t t_clk,
             BitOrder order = BitOrder::MSB_FIRST,
             Logic    logic = Logic::Normal)
  {
    if (t_clk == t_load || t_clk == t_data || t_load == t_data) {
      return false;  // pins must be unique
    }

    clkPin    = t_clk;
    loadPin   = t_load;
    dataPin   = t_data;
    bitOrder_ = order;
    logic_    = logic;

    neza_gpio_init_out(clkPin,  LOW);
    neza_gpio_init_out(loadPin, HIGH);
    neza_gpio_init_in(dataPin);

    std::memset(states, 0, sizeof(states));
    initialized_ = true;

    // Capture real initial state so first reads are meaningful
    update();
    return true;
  }

  // Convenience overload (MSB-first, normal logic)
  bool begin(uint8_t t_data, uint8_t t_load, uint8_t t_clk) {
    return begin(t_data, t_load, t_clk, BitOrder::MSB_FIRST, Logic::Normal);
  }

  // ── Runtime configuration ─────────────────────────────────────────────────
  void setBitOrder(BitOrder order) { bitOrder_ = order; }
  void setLogic(Logic logic)       { logic_    = logic; }

  uint16_t getLength() const { return static_cast<uint16_t>(_muxCount) * 8u; }

  // ── Latch + shift all bytes with dual-phase clock delays ──────────────────
  // Call once per main loop iteration.
  void update() {
    if (!initialized_) return;

    // Parallel load: drive /PL low briefly, then high to latch inputs
    neza_gpio_set_level(loadPin, LOW);
    delayMicroseconds(NEZA_74HC165_LATCH_PULSE_US);
    neza_gpio_set_level(loadPin, HIGH);

    // Read each IC in chain order (leftmost/first IC = states[0])
    for (uint8_t mux = 0; mux < _muxCount; mux++) {
      // Sample BEFORE clock edge — standard 74HC165 timing
      for (int i = 7; i >= 0; i--) {
        uint8_t bit = static_cast<uint8_t>(neza_gpio_get_level(dataPin)) & 0x01;

        if (logic_ == Logic::Inverted) bit ^= 1;

        const uint8_t targetBit = (bitOrder_ == BitOrder::MSB_FIRST)
                                    ? static_cast<uint8_t>(i)
                                    : static_cast<uint8_t>(7 - i);

        bitWrite(states[mux], targetBit, bit);

        neza_gpio_set_level(clkPin, HIGH);
        delayMicroseconds(NEZA_74HC165_DELAY_HIGH);
        neza_gpio_set_level(clkPin, LOW);
        delayMicroseconds(NEZA_74HC165_DELAY_LOW);
      }
    }
  }

  // ── Bit access ────────────────────────────────────────────────────────────

  // Read bit n across the entire chain (0 = first bit of first IC).
  bool read(uint16_t n) const {
    if (n >= getLength()) return false;
    return bitRead(states[n >> 3], n & 0x07);
  }

  bool readPin(uint16_t n) const { return read(n); }

  // Raw byte for IC index `chip` (0-based).
  uint8_t readByte(uint8_t chip) const {
    return (chip < _muxCount) ? states[chip] : 0u;
  }

  // Pointer to the raw state buffer (use with care — volatile between update calls).
  const uint8_t* data() const { return states; }
};


// Each HC165_165<N> manages N*2 physical 74HC165 chips = N*16 input bits.
template <uint8_t _muxinCount>
class HC165_165 : public Neza74HC165<_muxinCount * 2> {};


// ============================ HC165_Button ===================================


// ── Thresholds ───────────────────────────────────────────────────────────────
#ifndef NEZA_BTN_HOLD_THRESH
#define NEZA_BTN_HOLD_THRESH    500   // ms until press becomes "held"
#endif
#ifndef NEZA_BTN_DBL_DEFAULT
#define NEZA_BTN_DBL_DEFAULT    350   // ms window to catch second press
#endif

// ── Event return codes ────────────────────────────────────────────────────────
#define NEZA_NONE      0
#define NEZA_RELEASED  1
#define NEZA_PRESSED   2
#define NEZA_HELD      3
#define NEZA_DOUBLE    4

// ── Active-level enum 
enum class ActiveLevel : uint8_t {
  Low  = 0,   // button grounds the line when pressed  (most common)
  High = 1,   // button drives the line high when pressed
};

// ── Internal flag indices ─────────────────────────────────────────────────────
#define BTN_FLAG_STATE      0   // current debounced state (1 = pressed)
#define BTN_FLAG_CHANGED    1   // state changed this update cycle
#define BTN_FLAG_HOLD_ARMED 2   // waiting for hold threshold
#define BTN_FLAG_HOLD_FIRED 3   // hold event has already fired this press

class HC165_Button {
public:
  HC165_Button() = default;

  void begin() {
    flags_.reset();

    clickState_       = ClickState::IDLE;
    rawChangeTs_      = millis();
    holdTs_           = 0;
    firstReleaseTs_   = 0;
    pendingEvent_     = NEZA_NONE;

    initialized_      = true;
    rawSeeded_        = false;
    lastRawPressed_   = false;
  }

  void setOnUpdate(void (*fptr)(uint8_t type)) { callback_ = fptr; }

  // Double-click window in ms.
  // If > 0, single click (NEZA_PRESSED) is delayed until this window expires.
  void setDoublePressThreshold(uint16_t ms) { doublePressTime_ = ms; }

  // Debounce + state update
  // Returns true only when the debounced physical state changes
  bool update(bool rawState,
              uint16_t debounceMs = 50,
              ActiveLevel logic   = ActiveLevel::Low)
  {
    if (!initialized_) return false;

    flags_.off(BTN_FLAG_CHANGED);

    const uint32_t now = millis();
    const bool pressed = (logic == ActiveLevel::Low) ? !rawState : rawState;

    // First sample: seed raw state, don't generate fake events
    if (!rawSeeded_) {
      rawSeeded_      = true;
      lastRawPressed_ = pressed;
      rawChangeTs_    = now;
      return false;
    }

    // Raw changed -> restart debounce timer
    if (pressed != lastRawPressed_) {
      lastRawPressed_ = pressed;
      rawChangeTs_    = now;
      return false;
    }

    // Not stable long enough yet
    if ((uint32_t)(now - rawChangeTs_) < (uint32_t)debounceMs) {
      return false;
    }

    // No debounced state change
    if (pressed == isPressed_()) {
      return false;
    }

    // If a previous single click was waiting and the window already expired,
    // confirm it now before starting a new click sequence.
    if (doublePressTime_ > 0 &&
        clickState_ == ClickState::WAIT_SECOND &&
        (uint32_t)(now - firstReleaseTs_) > (uint32_t)doublePressTime_) {
      queueEvent_(NEZA_PRESSED);   // confirmed single click
      clickState_ = ClickState::IDLE;
    }

    // ---------------- Genuine debounced edge ----------------
    flags_.write(BTN_FLAG_STATE,   pressed);
    flags_.write(BTN_FLAG_CHANGED, true);

    if (pressed) {
      holdTs_ = now;
      flags_.on(BTN_FLAG_HOLD_ARMED);
      flags_.off(BTN_FLAG_HOLD_FIRED);

      if (doublePressTime_ == 0) {
        // Immediate edge mode (double-click disabled)
        queueEvent_(NEZA_PRESSED);
        clickState_ = ClickState::IDLE;
      } else {
        // Double-click enabled
        if (clickState_ == ClickState::WAIT_SECOND &&
            (uint32_t)(now - firstReleaseTs_) <= (uint32_t)doublePressTime_) {
          // Second press started within double-click window
          clickState_ = ClickState::SECOND_DOWN;
        } else {
          // Start first click
          clickState_ = ClickState::FIRST_DOWN;
        }
      }
    } else {
      // Release edge
      flags_.off(BTN_FLAG_HOLD_ARMED);

      // Release after a hold
      if (flags_.read(BTN_FLAG_HOLD_FIRED)) {
        queueEvent_(NEZA_RELEASED);
        clickState_ = ClickState::IDLE;
        return true;
      }

      if (doublePressTime_ == 0) {
        // Immediate edge mode (double-click disabled)
        queueEvent_(NEZA_RELEASED);
        clickState_ = ClickState::IDLE;
      } else {
        // Double-click enabled
        if (clickState_ == ClickState::FIRST_DOWN) {
          // First click finished -> wait for second click
          clickState_     = ClickState::WAIT_SECOND;
          firstReleaseTs_ = now;
          // No event yet: must wait to know single vs double
        }
        else if (clickState_ == ClickState::SECOND_DOWN &&
                 (uint32_t)(now - firstReleaseTs_) <= (uint32_t)doublePressTime_) {
          // Valid second click completed in time
          queueEvent_(NEZA_DOUBLE);
          clickState_ = ClickState::IDLE;
        }
        else {
          // Fallback/reset
          clickState_ = ClickState::IDLE;
        }
      }
    }

    return true;
  }

  uint8_t updateWithCallback(bool rawState,
                             uint16_t debounceMs  = 50,
                             ActiveLevel logic    = ActiveLevel::Low,
                             uint16_t holdThresh  = NEZA_BTN_HOLD_THRESH,
                             bool ignoreAfterHold = false)
  {
    update(rawState, debounceMs, logic);
    const uint8_t ev = read(holdThresh, ignoreAfterHold);

    if (callback_ && ev != NEZA_NONE) {
      callback_(ev);
    }

    return ev;
  }

  // Returns one-shot event:
  //   NEZA_NONE / NEZA_PRESSED / NEZA_RELEASED / NEZA_HELD / NEZA_DOUBLE
  uint8_t read(uint16_t holdThresh  = NEZA_BTN_HOLD_THRESH,
               bool ignoreAfterHold = false)
  {
    if (!initialized_) return NEZA_NONE;

    const uint32_t now = millis();

    // ---------------- Hold detection ----------------
    if (isPressed_() &&
        flags_.read(BTN_FLAG_HOLD_ARMED) &&
        !flags_.read(BTN_FLAG_HOLD_FIRED))
    {
      if ((uint32_t)(now - holdTs_) >= (uint32_t)holdThresh) {
        flags_.on(BTN_FLAG_HOLD_FIRED);
        flags_.off(BTN_FLAG_HOLD_ARMED);

        // Cancel any click/double waiting state
        clickState_ = ClickState::IDLE;

        queueEvent_(NEZA_HELD);
      }
    }

    // ---------------- Single-click confirmation timeout ----------------
    if (doublePressTime_ > 0 &&
        clickState_ == ClickState::WAIT_SECOND &&
        (uint32_t)(now - firstReleaseTs_) > (uint32_t)doublePressTime_) {
      queueEvent_(NEZA_PRESSED);   // confirmed single click
      clickState_ = ClickState::IDLE;
    }

    if (pendingEvent_ == NEZA_NONE) {
      return NEZA_NONE;
    }

    const uint8_t ev = pendingEvent_;
    pendingEvent_ = NEZA_NONE;

    // Optional: swallow the release after a hold
    if (ignoreAfterHold &&
        ev == NEZA_RELEASED &&
        flags_.read(BTN_FLAG_HOLD_FIRED))
    {
      flags_.off(BTN_FLAG_HOLD_FIRED);
      return NEZA_NONE;
    }

    // Clear hold flag once release is consumed
    if (ev == NEZA_RELEASED) {
      flags_.off(BTN_FLAG_HOLD_FIRED);
    }

    return ev;
  }

  // Current debounced physical state (true = physically pressed)
  bool getCurrentState() const { return isPressed_(); }

  // Physical edge helpers (valid after update, before next update)
  bool latched() const   { return flags_.read(BTN_FLAG_CHANGED) &&  isPressed_(); }
  bool unlatched() const { return flags_.read(BTN_FLAG_CHANGED) && !isPressed_(); }

private:
  enum class ClickState : uint8_t {
    IDLE,
    FIRST_DOWN,   // first press is currently down
    WAIT_SECOND,  // first click finished; waiting for second click
    SECOND_DOWN   // second press is currently down
  };

  bool isPressed_() const { return flags_.read(BTN_FLAG_STATE); }

  void queueEvent_(uint8_t ev) {
    // Keep first pending event until user consumes it with read()
    if (pendingEvent_ == NEZA_NONE) {
      pendingEvent_ = ev;
    }
  }

private:
  HC165_Flags<uint8_t> flags_;

  ClickState clickState_      = ClickState::IDLE;

  uint32_t rawChangeTs_       = 0;  // raw input changed at
  uint32_t holdTs_            = 0;  // current press started at
  uint32_t firstReleaseTs_    = 0;  // first click release time

  uint16_t doublePressTime_   = NEZA_BTN_DBL_DEFAULT;

  uint8_t pendingEvent_       = NEZA_NONE;

  bool initialized_           = false;
  bool rawSeeded_             = false;
  bool lastRawPressed_        = false;

  void (*callback_)(uint8_t type) = nullptr;
};

#endif // Neza74HC165_h
