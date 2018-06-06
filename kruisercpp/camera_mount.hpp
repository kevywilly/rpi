#ifndef CAMERA_MOUNT_HPP_
#define CAMERA_MOUNT_HPP_

#include "pigpio.h"
#include "utils.h"
#include <iostream>

namespace kruiser {
class CameraMount {
    
    public:
        
        CameraMount(uint8_t pitch_pin, uint8_t yaw_pin, int min_pulse, int max_pulse) {
            p_pitch_ = pitch_pin;
            p_yaw_ = yaw_pin;
            min_pulse_ = min_pulse;
            max_pulse_ = max_pulse;
        }
        
        void move_to(uint8_t pitch, int8_t yaw) {
            move_to_yaw(yaw); move_to_pitch(pitch);
        }
        
        void move_to_pitch(int8_t pitch) {
             gpioServo(p_pitch_, angle_to_pulse(pitch));
        }
        
        void move_to_yaw(int8_t yaw) {
            gpioServo(p_yaw_, angle_to_pulse(yaw));
        }
        
    private:
        uint8_t p_pitch_;
        uint8_t p_yaw_;
        int min_pulse_;
        int max_pulse_;
        
        int angle_to_pulse(int8_t angle) {
            if(angle < -90)
                angle = -90;
            if(angle > 90)
                angle = 90;
            return map(angle, -90, 90, min_pulse_, max_pulse_);;
        }
    };

}
#endif