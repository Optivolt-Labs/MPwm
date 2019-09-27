#include "mpwm.h"
#include "wiring_private.h"
#include "math.h"

using namespace MPwm;

// these values were pulled from arduino core
#define _MPWM_ARDUINO_WRITE_RES 12
#define _MPWM_ARDUINO_WRITE_MAX (1 << _MPWM_ARDUINO_WRITE_RES)

/* this variable is used to track which tc's are already
 * setup to avoid reseting a timer that is already set up
 * this part is not in sync with arduino since the equivalent
 * variable is a static var in the impl. of analogWrite()
 */
static GClk _tcConfigured[TCC_INST_NUM+TC_INST_NUM];

static inline void enable_tc(const uint32_t &tcNum, Tc* &TCx) {
    if (TCx->COUNT32.CTRLA.bit.ENABLE)
        return;

    TCx->COUNT32.CTRLA.bit.ENABLE = true;
    while(TCx->COUNT32.SYNCBUSY.bit.ENABLE);
}

static inline void disable_tc(const uint32_t &tcNum, Tc* &TCx) {
    if (!TCx->COUNT32.CTRLA.bit.ENABLE)
        return;

    TCx->COUNT32.CTRLA.bit.ENABLE = false;
    while(TCx->COUNT32.SYNCBUSY.bit.ENABLE);
}

// configures the TC module, some PWM pins use TCC check before calling
static void configure_tc(const uint32_t &tcNum, Tc* &TCx, const GClk &clk) {
    if (_tcConfigured[tcNum] == clk)
        return; // pinDesc is already configured to clk

    // connect the requested clock source to the TC module
    switch (clk) {
        case GClk::GCLK0:
            GCLK->PCHCTRL[GCLK_CLKCTRL_IDs[tcNum]].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
            break;
        case GClk::GCLK1:
            GCLK->PCHCTRL[GCLK_CLKCTRL_IDs[tcNum]].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
            break;
        case GClk::GCLK2:
            GCLK->PCHCTRL[GCLK_CLKCTRL_IDs[tcNum]].reg = GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
            break;
        default:
            GCLK->PCHCTRL[GCLK_CLKCTRL_IDs[tcNum]].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
            break;
    }

    // reset all bits in TC and disable it
    TCx->COUNT32.CTRLA.bit.SWRST = 1;
    while (TCx->COUNT32.SYNCBUSY.bit.SWRST);

    // disabling like this might be unnessescary
    TCx->COUNT32.CTRLA.bit.ENABLE = 0;
    while (TCx->COUNT32.SYNCBUSY.bit.ENABLE);

    TCx->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV1;
    TCx->COUNT32.WAVE.reg = TC_WAVE_WAVEGEN_MPWM;

    _tcConfigured[tcNum] = clk; // we're done configuring it!
}

MatchPwm::MatchPwm(uint32_t pin, GClk clk) : _pin(pin), _pinDesc(g_APinDescription[pin]), _clk(clk) {
    // verify that the pin is a valid pwm pin and throw an error if not
    const uint32_t attr = _pinDesc.ulPinAttribute;
    _tcNum = 0;
    _tcChannel = 0;
    _hasTc = false;

    bool pwm_supported;

    if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG) {
        if (pin == PIN_A0 || pin == PIN_A1) {
            pwm_supported = false;
        }
    }
    if ((attr & (PIN_ATTR_PWM_E|PIN_ATTR_PWM_F|PIN_ATTR_PWM_G)) == 0) {
        pwm_supported = false;
    }

    if (pwm_supported) {
        _tcNum = GetTCNumber(_pinDesc.ulPWMChannel);
        _tcChannel = GetTCChannelNumber(_pinDesc.ulPWMChannel);

        // set to timer peripheral instead of default
        if(attr & PIN_ATTR_PWM_E)
            pinPeripheral(pin, PIO_TIMER);
        else if(attr & PIN_ATTR_PWM_F)
            pinPeripheral(pin, PIO_TIMER_ALT);
        else if(attr & PIN_ATTR_PWM_G)
            pinPeripheral(pin, PIO_TCC_PDEC);

        // configure the TC if it hasn't been already
        if (_tcNum >= TCC_INST_NUM) {
            _hasTc = true;
            Tc* TCx = (Tc*) GetTC(_pinDesc.ulPWMChannel);
            configure_tc(_tcNum, TCx, _clk);
        }
    }
}

MatchPwm::MatchPwm(const MatchPwm & pwm) : _pin(pwm._pin), _pinDesc(pwm._pinDesc), _clk(pwm._clk)  {
    _tcNum = pwm._tcNum;
    _tcChannel = pwm._tcChannel;
    _hasTc = pwm._hasTc;
}

// enable pwm output (different from setting pinMode to output)
// can be used to toggle the PWM output without changing anything else
void MatchPwm::enable() {
    pinMode(_pin, OUTPUT);
}

// disable pwm output (different from setting pinMode to output)
// can be used to toggle the PWM output without changing anything else
void MatchPwm::disable() {
    pinMode(_pin, INPUT);
}

void MatchPwm::_setFrequency(float freq, bool update_duty) {
    if(_hasTc) {
        // set cc0 to the correct value
        // calculate what the period should ideally be to meet freq requested
        Tc* TCx = (Tc*) GetTC(_pinDesc.ulPWMChannel);
        float goal_period;

        switch (_clk) {
            case GClk::GCLK0:
                goal_period = VARIANT_GCLK0_FREQ / freq;
                break;
            case GClk::GCLK1:
                goal_period = VARIANT_GCLK1_FREQ / freq;
                break;
            case GClk::GCLK2:
                goal_period = VARIANT_GCLK2_FREQ / freq;
                break;
            default:
                return;
        }

        if (update_duty) {
            // scale up the frequency of the thing
            float duty_rate = this->getDutyRate();
            float goal_trigger_per = goal_period * duty_rate;

            TCx->COUNT32.CC[0].reg = (uint32_t) round(goal_period);
            TCx->COUNT32.CC[1].reg = (uint32_t) round(goal_trigger_per);
            while (TCx->COUNT32.SYNCBUSY.bit.CC0);
            while (TCx->COUNT32.SYNCBUSY.bit.CC1);
        } else {
            // this else block is implemented instead of just placing the code
            // outside of the condition block to save a few cycles when we're
            // waiting for the registers to sync
            TCx->COUNT32.CC[0].reg = (uint32_t) round(goal_period);
            while (TCx->COUNT32.SYNCBUSY.bit.CC0);
        }
    }
    // ignore if there is no Tc support
}

void MatchPwm::setFrequency(const float freq) {
    this->_setFrequency(freq, true);
}

// returns the frequency based on the reading the mmap'd registers
float MatchPwm::getFrequency() {
    if(_hasTc) {
        // get the state of cc0 to get the frequency
        Tc* TCx = (Tc*) GetTC(_pinDesc.ulPWMChannel);

        switch (_clk) {
            case GClk::GCLK0:
                return (float) VARIANT_GCLK0_FREQ / TCx->COUNT32.CC[0].reg;
            case GClk::GCLK1:
                return (float) VARIANT_GCLK1_FREQ / TCx->COUNT32.CC[0].reg;
            case GClk::GCLK2:
                return (float) VARIANT_GCLK2_FREQ / TCx->COUNT32.CC[0].reg;
        }
    }
    return -1; // it's possible to return valid responses but it's too annoying
}

void MatchPwm::setDutyRate(float duty) {
    if (duty > 100.0) {
        duty = 100.0;
    } else if (duty < 0.0) {
        duty = 0.0;
    }

    if(_hasTc) {
        Tc* TCx = (Tc*) GetTC(_pinDesc.ulPWMChannel);
        TCx->COUNT32.CC[1].reg = (uint32_t) (TCx->COUNT32.CC[0].reg * duty);
        while (TCx->COUNT32.SYNCBUSY.bit.CC1);

        enable_tc(_tcNum, TCx);
    }
    uint32_t val = ((uint32_t) (duty * _MPWM_ARDUINO_WRITE_MAX)) / 100;
    analogWrite(_pin, val);
}

// returns the frequency based on the reading the mmap'd registers
float MatchPwm::getDutyRate() {
    if(_hasTc) {
        // calculate based on prescaler mode and cc0
        Tc* TCx = (Tc*) GetTC(_pinDesc.ulPWMChannel);
        return TCx->COUNT32.CC[1].reg / this->getFrequency();
    }
    return -1; // it's possible to return valid responses but it's too annoying
}

// sets the InvertEnable register for the PWM output
// setting this allows you to effectively pass the PWM output through
// a NOT gate, this is useful if you need an inverted output without
// a phase shift
void MatchPwm::setInvertEnable(bool inverted) {
    if(_hasTc) {
        Tc* TCx = (Tc*) GetTC(_pinDesc.ulPWMChannel);
        // MPWM only works on W[1] anyway so we just need to invert that one
        TCx->COUNT32.DRVCTRL.bit.INVEN1 = inverted;
    }
}

const bool MatchPwm::isInverted() {
    if (_hasTc) {
        Tc* TCx = (Tc*) GetTC(_pinDesc.ulPWMChannel);
        return TCx->COUNT32.DRVCTRL.bit.INVEN1;
    }
    // digital pin invert is simply ignored so it's always false
    return false;
}

void MatchPwm::write(float duty) {
    this->setDutyRate(duty);
}

void MatchPwm::write(float duty, uint32_t freq) {
    // set the frequency and the duty at the same time
    this->_setFrequency(freq, false);
    this->setDutyRate(duty);
}
