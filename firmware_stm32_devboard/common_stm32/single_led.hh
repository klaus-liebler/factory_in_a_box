#pragma once

#include "gpio.hh"
#include <cstdio>
#include <cstdint>
#include <cstring>

#include <array>
#define TAG "SINGLE_LED"


namespace single_led
{
    class AnimationPattern
    {
    public:
        virtual void Reset(uint32_t now) = 0;
        virtual bool Animate(uint32_t now) = 0;
    };

    class BlinkPattern : public AnimationPattern
    {
    private:
        uint32_t nextChange{0};
        bool state{false};
        uint32_t timeOn;
        uint32_t timeOff;

    public:
        void Reset(uint32_t now) override
        {
            nextChange = now+timeOn;//TODO Problem nach ca 50 Tagen!
            state = true;
        }
        bool Animate(uint32_t now) override
        {
            if(now < nextChange) return state;
            if (state)
            {
                state = false;
                nextChange = now+timeOff;
            }
            else
            {
                state = true;
                nextChange = now+timeOn;
            }
            return state;
        }
        BlinkPattern(uint32_t timeOn, uint32_t timeOff) : timeOn(timeOn), timeOff(timeOff) {}
    };

    class :public AnimationPattern{
        void Reset(uint32_t now){}
        bool Animate(uint32_t now){return false;}
    }CONST_OFF;

    class :public AnimationPattern{
        void Reset(uint32_t now){}
        bool Animate(uint32_t now){return true;}
    }CONST_ON;

    template<bool ON_LEVEL= false>
    class M
    {
    private:
    	
        gpio::Pin pin{gpio::Pin::NO_PIN};
        AnimationPattern* pattern;
        uint32_t timeToAutoOff=UINT32_MAX;//time is absolute!
        AnimationPattern* standbyPattern;
    public:
        M(gpio::Pin  pin, AnimationPattern* standbyPattern=&CONST_OFF):pin(pin), standbyPattern(standbyPattern) {}

        void AnimatePixel(uint32_t now, AnimationPattern *pattern, uint32_t timeToAutoOff=0)//time is relative, "0" means: no auto off
        {
            if(pattern==nullptr) this->pattern=standbyPattern;
            
            if(timeToAutoOff==0){
                this->timeToAutoOff=UINT32_MAX;
            }else{
                this->timeToAutoOff=now+timeToAutoOff;
            }
            pattern->Reset(now);
            this->pattern = pattern;
        }

        void Loop(uint32_t now)
        {
            if(now>=timeToAutoOff){
                this->pattern=standbyPattern;
            } 
            bool on = this->pattern->Animate(now);
            
            gpio::Gpio::Set(pin, (on==ON_LEVEL));
            
        }

        void Force(bool onoff){
            gpio::Gpio::Set(pin, (onoff==ON_LEVEL));
        }

        void Begin(uint32_t now, AnimationPattern *pattern=&CONST_OFF, uint32_t timeToAutoOff=0)
        {
            gpio::Gpio::ConfigureGPIOOutput(pin, !(ON_LEVEL));
            this->AnimatePixel(now, pattern, timeToAutoOff);
        }
    };
}
#undef TAG