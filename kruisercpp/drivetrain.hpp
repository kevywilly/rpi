#ifndef DRIVETRAIN_HPP_
#define DRIVETRAIN_HPP_

#include "pigpio.h"
#include "utils.hpp"
#include <iostream>
#include "math.h"

using namespace std;
namespace kruiser {
    
class Motor {
    public:
        Motor(int pin1, int pin2, int pwm) {
            pin1_ = pin1;
            pin2_ = pin2;
            pwm_ = pwm;
            speed_ = 0;
            pins_ = new int[3];
            pins_[0] = pin1_;
            pins_[1] = pin2_;
            pins_[2] = pwm_;
            
            for(int i=0; i < 3; i++) {
                gpioSetMode(pins_[i], PI_OUTPUT);
            }
        }
        
        void brake() {
            for(int i=0; i < 3; i++) {
                gpioWrite(pins_[i], 0);
            }
        }
        
        void stop() {
            gpioHardwarePWM(pwm_, 50, 0);
        }
        
        void setSpeed(float speed) {
            if(speed > 0) {
                gpioWrite(pin1_, 1);
                gpioWrite(pin2_, 0);
            } else {
                gpioWrite(pin1_, 0);
                gpioWrite(pin2_, 1);
            }
        
            speed_ = speed;
            gpioHardwarePWM(pwm_, 50, int(abs(speed)*1000000));
        }
        
    private:
        int pin1_, pin2_, pwm_, speed_;
        int * pins_;
    
};
class Drivetrain {
    
    public:
        
        Drivetrain(int rear1, int rear2,int rearPWM, int front1, int front2, int frontPWM) : motorR_(rear1, rear2, rearPWM), motorF_(front1, front2, frontPWM) {}
        
        void brake() {
            motorR_.brake();
            motorF_.brake();
        }
        
        void stop(int pitch) {
            motorR_.stop();
            motorF_.stop();
        }
        
        void turn(float pct) { turn(pct, 0); }
        void turn(float pct, int duration) {
            motorF_.setSpeed(pct/100.0);
            delay(duration);
        }
        
        void forward(float pct) { forward(pct, 0); }
        
        void forward(float pct, int duration) {
            motorR_.setSpeed(pct/100.0);
            delay(duration);
        }
        
        void reverse(float pct) { forward(pct, 0);}
        void reverse(float pct, int duration) { forward(pct, duration);}
        void drive(float speed) { drive(speed, 0); }
        void drive(float speed, float turnPct) { drive(speed, turnPct, 0); }
        void drive(float speed, float turnPct, int duration) {
            turn(turnPct);
            forward(speed);
            delay(duration);
        }
        
    private:
        Motor motorR_, motorF_;
        
    };

}
#endif