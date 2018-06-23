#ifndef SONAR_SENSOR_H
#define SONAR_SENSOR_H

#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>

#define MICROSECONDS_PER_CM  (1000000/34321)
#define MAX_SONAR_DISTANCE 400
#define MIN_SONAR_DISTANCE 5

class SonarSensor {
    private:
        uint8_t trigger_;
        uint8_t echo_;
        bool triggered_ = false;
        volatile uint32_t startTick_;
        volatile uint32_t firstTick_;
        
    public:
    
        // last distance reading
        volatile uint32_t distance;
        volatile uint32_t lastReading;
        
        // constructor
        SonarSensor(uint8_t trigger_pin, uint8_t echo_pin) {
            trigger_ = trigger_pin;
            echo_ = echo_pin;
            init();
        }
        
        // deconstructor
        virtual ~SonarSensor() {
            
        }
        
       
        void init() {
            
            // initialize pins
            gpioSetMode(trigger_, PI_OUTPUT);
            gpioWrite  (trigger_, PI_OFF);
            gpioSetMode(echo_,    PI_INPUT);
            
            // set distance to 0
            distance = lastReading = MAX_SONAR_DISTANCE;
            
            // set ticks to 0
            firstTick_ = startTick_ = 0;
        }
        
        
        // trigger a pulse
        void sonarTrigger(void)
        {
            // return if already triggered
            if(triggered_)
                return;
                
            startTick_, firstTick_ = 0;
            gpioTrigger(trigger_, 10, PI_ON);
            triggered_ = true;
           
        }
        
        void sonarEcho(int gpio, int level, uint32_t tick) {
           
           int diffTick;
           uint32_t newDistance;
        
           if (!firstTick_) firstTick_ = tick;
        
           if (level == PI_ON)
           {
              startTick_ = tick;
           }
           else if (level == PI_OFF)
           {
              diffTick = ((tick >> 0) - (startTick_ >> 0)) / 2 / MICROSECONDS_PER_CM;
           
              firstTick_ = firstTick_ = 0;
              
              newDistance = (diffTick > MAX_SONAR_DISTANCE || diffTick <= MIN_SONAR_DISTANCE) ? MAX_SONAR_DISTANCE : diffTick;   
           
              // debounce reading of 400 (must happen twice in a row)
              //if((newDistance < 400) || (newDistance == lastReading)) {
                  distance = newDistance;
              //}
              
              // store this reading as last reading
              lastReading = newDistance;
              
              triggered_ = false;
           }
        }
    
};


#endif
