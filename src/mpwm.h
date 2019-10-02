#pragma once

#include <Arduino.h>
#include <stdint.h>

#ifndef __SAMD51__
#error "The Mpwm library only supports the ATSAMD51 boards!"
#endif

namespace MPwm {
    enum class GClk {GCLK0, GCLK1, GCLK2};

    class MatchPwm {
        public:
            MatchPwm(uint32_t pin, GClk clk);
            explicit MatchPwm(uint32_t pin) : MatchPwm(pin, GClk::GCLK0) {};
            MatchPwm(const MatchPwm & pwm);
            void enable();
            void disable();
            void setFrequency(float freq);
            float getFrequency();
            void setDutyRate(float rate);
            float getDutyRate();
            void write(float duty);
            void write(float duty, uint32_t freq);

            // bool supported; // this flag is set if the pin is unsupported
        private:
            void _setFrequency(float freq, bool update_duty);

            const uint32_t _pin;
            const PinDescription _pinDesc;
            const GClk _clk;
            uint32_t _tcNum;
            uint8_t _tcChannel;
            bool _hasTc;
    };
}
