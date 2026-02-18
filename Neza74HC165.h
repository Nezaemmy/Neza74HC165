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

  // ── Lifecycle ──────────────────────────────────────────────────────────────
  HC165_Button() = default;

  void begin() {
    flags_.reset();
    dblState_      = DblState::IDLE;
    debounceTs_    = millis();
    holdTs_        = 0;
    dblWindowTs_   = 0;
    lastEvent_     = NEZA_NONE;
    initialized_   = true;
  }

  // ── Configuration ──────────────────────────────────────────────────────────
  void setOnUpdate(void (*fptr)(uint8_t type)) { callback_ = fptr; }
  void setDoublePressThreshold(uint16_t ms)    { doublePressTime_ = ms; }

  // ── update() ───────────────────────────────────────────────────────────────
  // Call every loop with the raw pin state.
  // Returns true if the debounced state changed this cycle.
  
  bool update(bool rawState,
              uint16_t debounceMs = 50,
              ActiveLevel logic   = ActiveLevel::Low)
  {
    if (!initialized_) return false;

    flags_.off(BTN_FLAG_CHANGED);

    const uint32_t now     = millis();
    const bool     pressed = (logic == ActiveLevel::Low) ? !rawState : rawState;

    // Reject changes within debounce window
    if ((uint32_t)(now - debounceTs_) < (uint32_t)debounceMs) return false;
    debounceTs_ = now;

    if (pressed == flags_.read(BTN_FLAG_STATE)) return false;

    // Genuine state change ────────────────────────────────────────────────────
    flags_.write(BTN_FLAG_STATE,      pressed);
    flags_.write(BTN_FLAG_CHANGED,    true);
    flags_.write(BTN_FLAG_HOLD_ARMED, pressed);

    if (pressed) {
      holdTs_ = now;
      advanceDblOnPress_(now);
    } else {
      advanceDblOnRelease_(now);
    }

    return true;
  }

  // ── updateWithCallback() ───────────────────────────────────────────────────
  
  uint8_t updateWithCallback(bool rawState,
                             uint16_t debounceMs  = 50,
                             ActiveLevel logic    = ActiveLevel::Low,
                             uint16_t holdThresh  = NEZA_BTN_HOLD_THRESH,
                             bool ignoreAfterHold = false)
  {
    update(rawState, debounceMs, logic);
    const uint8_t ev = read(holdThresh, ignoreAfterHold);
    if (callback_ && ev != NEZA_NONE) callback_(ev);
    return ev;
  }

  // ── read() ─────────────────────────────────────────────────────────────────
  // Returns the current event for this update cycle.
  // Double-firing when both held() and read() were called.
  uint8_t read(uint16_t holdThresh  = NEZA_BTN_HOLD_THRESH,
               bool ignoreAfterHold = false)
  {
    if (!initialized_) return NEZA_NONE;

    // ── Hold check (fires once per press, while button is down) ──────────────
    if (isPressed_() && flags_.read(BTN_FLAG_HOLD_ARMED)) {
      const uint32_t now = millis();
      if ((uint32_t)(now - holdTs_) >= (uint32_t)holdThresh) {
        if (!flags_.read(BTN_FLAG_HOLD_FIRED)) {
          flags_.on(BTN_FLAG_HOLD_FIRED);
          flags_.off(BTN_FLAG_HOLD_ARMED);
          dblState_  = DblState::IDLE;   // cancel pending double
          lastEvent_ = NEZA_HELD;
          if (callback_) callback_(NEZA_HELD);
        }
        return lastEvent_;
      }
    }

    // ── Edge events (only on state change) ───────────────────────────────────
    if (!flags_.read(BTN_FLAG_CHANGED)) return lastEvent_;

    // Double-press confirmed
    if (dblState_ == DblState::CONFIRMED) {
      dblState_  = DblState::IDLE;
      lastEvent_ = NEZA_DOUBLE;
      return lastEvent_;
    }

    if (isPressed_()) {
      lastEvent_ = NEZA_PRESSED;
      return lastEvent_;
    }

    // Released
    if (ignoreAfterHold && flags_.read(BTN_FLAG_HOLD_FIRED)) {
      // Swallow the release that follows a hold event
      flags_.off(BTN_FLAG_HOLD_FIRED);
      lastEvent_ = NEZA_NONE;
      return NEZA_NONE;
    }
    flags_.off(BTN_FLAG_HOLD_FIRED);
    lastEvent_ = NEZA_RELEASED;
    return lastEvent_;
  }

  // ── Convenience helpers ───────────────────────────────────────────────────
  bool getCurrentState() const { return isPressed_(); }

  bool latched()   { return flags_.read(BTN_FLAG_CHANGED) &&  isPressed_(); }
  bool unlatched() { return flags_.read(BTN_FLAG_CHANGED) && !isPressed_(); }

private:

  // ── Double-press state machine ─────────────────────────────────────────────
  enum class DblState : uint8_t {
    IDLE,
    FIRST_PRESS,
    WAITING_FOR_SECOND,
    CONFIRMED,
  };

  // Called on every debounced press edge.
  void advanceDblOnPress_(uint32_t now) {
    switch (dblState_) {
      case DblState::IDLE:
        dblState_ = DblState::FIRST_PRESS;
        break;

      case DblState::WAITING_FOR_SECOND:
        if ((uint32_t)(now - dblWindowTs_) <= (uint32_t)doublePressTime_) {
          dblState_ = DblState::CONFIRMED;
        } else {
          // Window expired — treat this press as a fresh start
          dblState_ = DblState::FIRST_PRESS;
        }
        break;

      case DblState::FIRST_PRESS:
      case DblState::CONFIRMED:
        // Shouldn't normally reach here; reset to be safe
        dblState_ = DblState::FIRST_PRESS;
        break;
    }
  }

  // Called on every debounced release edge.
  void advanceDblOnRelease_(uint32_t now) {
    if (dblState_ == DblState::FIRST_PRESS) {
      dblState_    = DblState::WAITING_FOR_SECOND;
      dblWindowTs_ = now;   // start timing the window from release
    }
  }

  // ── Private helpers ───────────────────────────────────────────────────────
  bool isPressed_()  const { return flags_.read(BTN_FLAG_STATE);   }

  // ── Members ───────────────────────────────────────────────────────────────
  HC165_Flags<uint8_t> flags_;

  DblState dblState_        = DblState::IDLE;
  uint32_t debounceTs_      = 0;
  uint32_t holdTs_          = 0;
  uint32_t dblWindowTs_     = 0;
  uint16_t doublePressTime_ = NEZA_BTN_DBL_DEFAULT;
  uint8_t  lastEvent_       = NEZA_NONE;
  bool     initialized_     = false;

  void (*callback_)(uint8_t type) = nullptr;
};

#endif // Neza74HC165_h
