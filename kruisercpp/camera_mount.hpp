#ifndef CAMERA_MOUNT_HPP_
#define CAMERA_MOUNT_HPP_

#include "pigpio.h"
#include "utils.hpp"
#include <iostream>

namespace kruiser {
class CameraMount {
    
    public:
        
        CameraMount(int pitch_pin, int yaw_pin, int min_pulse, int max_pulse) {
            p_pitch_ = pitch_pin;
            p_yaw_ = yaw_pin;
            min_pulse_ = min_pulse;
            max_pulse_ = max_pulse;
        }
        
        void move_to(int pitch, int yaw) {
            move_to_yaw(yaw); move_to_pitch(pitch);
        }
        
        void move_to_pitch(int pitch) {
             gpioServo(p_pitch_, angle_to_pulse(pitch));
        }
        
        void move_to_yaw(int yaw) {
            gpioServo(p_yaw_, angle_to_pulse(yaw));
        }
        
    private:
        int p_pitch_;
        int p_yaw_;
        int min_pulse_;
        int max_pulse_;
        
        int angle_to_pulse(int angle) {
            if(angle < -90)
                angle = -90;
            if(angle > 90)
                angle = 90;
            return map(angle, -90, 90, min_pulse_, max_pulse_);;
        }
    };

}
#endif