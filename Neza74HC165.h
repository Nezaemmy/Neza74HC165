// Neza74HC165.h
#ifndef Neza74HC165_h
#define Neza74HC165_h

#include <Arduino.h>

// HC165_Flags Template Class
template <typename T>
class HC165_Flags {
private:
    T flags;
public:
    HC165_Flags() {
        reset();
    }
    bool read(uint8_t bit) {
        return bitRead(flags, bit);
    }
    void write(uint8_t bit, bool value) {
        bitWrite(flags, bit, value);
    }
    void on(uint8_t bit) {
        bitWrite(flags, bit, 1);
    }
    void off(uint8_t bit) {
        bitWrite(flags, bit, 0);
    }
    bool toggle(uint8_t bit) {
        flags ^= 1UL << bit;
        return bitRead(flags, bit);
    }
    bool toggleIfTrue(uint8_t bit) {
        if (bitRead(flags, bit)) {
            bitClear(flags, bit);
            return true;
        }
        return false;
    }
    void reset() {
        flags = 0;
    }
    void set(T t_flags) {
        flags = t_flags;
    }
    T get() {
        return flags;
    }
};

// Neza74HC165 Constants and Classes
#define NEZA_74HC165_DELAY 1

template <uint8_t _muxCount>
class Neza74HC165 {
private:
    uint8_t clkPin = 0;
    uint8_t loadPin = 0;
    uint8_t dataPin = 0;
    uint8_t states[_muxCount];
public:
    Neza74HC165() {}
    void begin(uint8_t t_data, uint8_t t_load, uint8_t t_clk) {
        if (t_clk == t_load || t_clk == t_data || t_load == t_data) {
            Serial.println("invalid 74HC165 pins used");
            while (1);
        }
        clkPin = t_clk;
        loadPin = t_load;
        dataPin = t_data;
        pinMode(clkPin, OUTPUT);
        pinMode(loadPin, OUTPUT);
        pinMode(dataPin, INPUT);
        digitalWrite(clkPin, LOW);
        digitalWrite(loadPin, HIGH);
        memset(states, 0, sizeof(states[0]) * _muxCount);
    }
    uint16_t getLength() {
        return _muxCount * 8;
    }
    void update() {
        digitalWrite(loadPin, LOW);
        delayMicroseconds(NEZA_74HC165_DELAY);
        digitalWrite(loadPin, HIGH);
        for (uint8_t mux = 0; mux < _muxCount; mux++) {
            for (int i = 7; i >= 0; i--) {
                uint8_t bit = digitalRead(dataPin);
                bitWrite(states[mux], i, bit);
                digitalWrite(clkPin, HIGH);
                delayMicroseconds(NEZA_74HC165_DELAY);
                digitalWrite(clkPin, LOW);
            }
        }
    }
    bool read(uint16_t n) {
        if (n >= (_muxCount * 8)) {
            return HIGH;
        }
        return bitRead(states[(n >> 3)], (n & 0x07));
    }
    bool readPin(uint16_t n) {
        return read(n);
    }
};

template <uint8_t _muxinCount>
class HC165_165 : public Neza74HC165<_muxinCount * 2> {};

// HC165_Button Constants and Class
#define NEZA_BTN_STATE                 0
#define NEZA_BTN_STATE_CHANGED         1
#define NEZA_BTN_STATE_HELD            2
#define NEZA_BTN_STATE_HOLD_TRIGGERED  3
#define NEZA_BTN_STATE_DBL_ACTIVE      4
#define NEZA_BTN_STATE_DBL_TRIGGERED   5
#define NEZA_BTN_STATE_DBL_IGNORE_REL  6

#define NEZA_BTN_HOLD_THRESH   500

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
    uint16_t doublePressTime = 0;
    HC165_Flags<uint8_t> flags;
    void (*onUpdateCallback)(uint8_t type);
    void onUpdate(void (*fptr)(uint8_t type)) {
        onUpdateCallback = fptr;
    }
    bool isPressed() {
        return flags.read(NEZA_BTN_STATE);
    }
    bool stateChanged() {
        return flags.read(NEZA_BTN_STATE_CHANGED);
    }
    bool btnHeld() {
        return flags.read(NEZA_BTN_STATE_HELD);
    }
    bool setDblPress() {
        if (flags.toggleIfTrue(NEZA_BTN_STATE_DBL_ACTIVE)) {
            if (((uint32_t)millis() - dblDebounce) <= doublePressTime) {
                flags.on(NEZA_BTN_STATE_DBL_TRIGGERED);
                return true;
            }
        }
        return false;
    }
    void setDblRelease() {
        if (!flags.read(NEZA_BTN_STATE_DBL_ACTIVE)) {
            flags.on(NEZA_BTN_STATE_DBL_ACTIVE);
            dblDebounce = millis();
        }
    }
public:
    HC165_Button() {
        onUpdateCallback = 0;
        debounce = 0;
        holdDebounce = 0;
        flags.reset();
    }
    void begin() {
        debounce = millis();
    }
    bool update(bool state, uint16_t t_debounce = 50, bool t_logic = LOW) {
        flags.off(NEZA_BTN_STATE_CHANGED);
        bool pressed = (t_logic == LOW) ? !state : state;
        if (((uint32_t)millis() - debounce) >= t_debounce) {
            debounce = millis();
            if (pressed != flags.read(NEZA_BTN_STATE)) {
                holdDebounce = millis();
                flags.write(NEZA_BTN_STATE, pressed);
                flags.write(NEZA_BTN_STATE_CHANGED, true);
                flags.write(NEZA_BTN_STATE_HELD, pressed);
                if (onUpdateCallback) {
                    read(NEZA_BTN_HOLD_THRESH);
                }
                return true;
            }
        }
        return false;
    }
    void updateWithCallback(bool state, uint16_t t_debounce = 50, bool t_logic = LOW, uint16_t t_hold = NEZA_BTN_HOLD_THRESH, bool ignoreAfterHold = false) {
        if (update(state, t_debounce, t_logic) && onUpdateCallback) {
            read(t_hold, ignoreAfterHold);
        }
    }
    uint8_t read(uint16_t t_hold = NEZA_BTN_HOLD_THRESH, bool ignoreAfterHold = false) {
        if (stateChanged()) {
            if (held(t_hold)) {
                if (onUpdateCallback) {
                    onUpdateCallback(NEZA_HELD);
                }
                return NEZA_HELD;
            } else {
                if (doublePressed()) {
                    if (onUpdateCallback) {
                        onUpdateCallback(NEZA_DOUBLE);
                    }
                    return NEZA_DOUBLE;
                } else if (isPressed()) {
                    if (onUpdateCallback) {
                        onUpdateCallback(NEZA_PRESSED);
                    }
                    return NEZA_PRESSED;
                } else if (released(ignoreAfterHold)) {
                    if (onUpdateCallback) {
                        onUpdateCallback(NEZA_RELEASED);
                    }
                    return NEZA_RELEASED;
                }
            }
        }
        return NEZA_NONE;
    }
    void setDoublePressThreshold(uint16_t t_thres) {
        doublePressTime = t_thres;
    }
    bool doublePressed(bool allowRelease = false) {
        bool state = flags.toggleIfTrue(NEZA_BTN_STATE_DBL_TRIGGERED);
        if (state && !allowRelease) {
            flags.on(NEZA_BTN_STATE_DBL_IGNORE_REL);
        }
        return state;
    }
    bool pressed() {
        bool state = stateChanged() && isPressed();
        if (state) {
            if (setDblPress()) {
                return false;
            }
        }
        return state;
    }
    bool released(bool ignoreAfterHold = false) {
        bool state = stateChanged() && !isPressed();
        bool holdTriggered = flags.read(NEZA_BTN_STATE_HOLD_TRIGGERED);
        bool dblTriggered = flags.read(NEZA_BTN_STATE_DBL_IGNORE_REL);
        if (state) {
            flags.off(NEZA_BTN_STATE_HOLD_TRIGGERED);
            flags.off(NEZA_BTN_STATE_DBL_IGNORE_REL);
        }
        if (state && ignoreAfterHold && holdTriggered) {
            return false;
        }
        if (state) {
            setDblRelease();
            if (dblTriggered) {
                return false;
            }
        }
        return state;
    }
    bool held(uint16_t holdTime = NEZA_BTN_HOLD_THRESH) {
        bool state = isPressed() && btnHeld() && ((uint32_t)(millis() - holdDebounce) >= holdTime);
        if (state && !flags.read(NEZA_BTN_STATE_HOLD_TRIGGERED)) {
            flags.on(NEZA_BTN_STATE_HOLD_TRIGGERED);
            flags.off(NEZA_BTN_STATE_DBL_ACTIVE);
            return true;
        }
        return false;
    }
    bool latched() {
        return stateChanged() && isPressed();
    }
    bool unlatched() {
        return stateChanged() && !isPressed();
    }
    bool getCurrentState() {
        return isPressed();
    }
};

#endif