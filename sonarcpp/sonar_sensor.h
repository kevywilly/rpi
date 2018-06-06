#ifndef SONAR_SENSOR_H
#define SONAR_SENSOR_H

#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>

#define MICROSECONDS_PER_CM  (1000000/34321)

class SonarSensor {
    private:
        uint8_t trigger_;
        uint8_t echo_;
        uint8_t timer_;
        bool pinged_ = true;
        bool triggered_ = false;
        volatile uint32_t startTick_;
        volatile uint32_t firstTick_;
        
    public:
    
        // last distance reading
        volatile uint32_t distance;
        
        // callback for echo
        void(*fnEcho) (int,int,uint32_t);
        
        // trigger function
        void(*fnTrigger) (void);
        
        // constructor
        SonarSensor(uint8_t trigger_pin, uint8_t echo_pin, uint8_t timer, void(*triggerFn) (void), void(*echoFn) (int,int,uint32_t)) {
            trigger_ = trigger_pin;
            echo_ = echo_pin;
            timer_ = timer;
            fnEcho = echoFn;
            fnTrigger = triggerFn;
            init();
        }
        
        // deconstructor
        virtual ~SonarSensor() {
            stop();
        }
        
       
        void init() {
            
            // initialize pins
            gpioSetMode(trigger_, PI_OUTPUT);
            gpioWrite  (trigger_, PI_OFF);
            gpioSetMode(echo_,    PI_INPUT);
            
            // set distance to 0
            distance = 0;
            
            // set ticks to 0
            firstTick_ = startTick_ = 0;
        }
        
        // start pinging
        void start() {
            
            // update sonar 20 times a second */
            gpioSetTimerFunc(timer_, 50, fnTrigger); /* every 50ms */

            /* monitor sonar echos */
            gpioSetAlertFunc(echo_, fnEcho);
        }
        
        // stop pinging
        void stop() {
            // stop triggers
            gpioSetTimerFunc(timer_, 50, NULL); 

            // stop monitoring sonar echos
            gpioSetAlertFunc(echo_, NULL);
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
            pinged_ = false;
           
        }
        
        void sonarEcho(int gpio, int level, uint32_t tick) {
           
           int diffTick;
        
           if (!firstTick_) firstTick_ = tick;
        
           if (level == PI_ON)
           {
              startTick_ = tick;
           }
           else if (level == PI_OFF)
           {
              diffTick = ((tick >> 0) - (startTick_ >> 0)) / 2 / MICROSECONDS_PER_CM;
           
              firstTick_ = firstTick_ = 0;
              
              distance = (diffTick > 400 || diffTick <= 5) ? 0 : diffTick;   
           
              pinged_ = true;
              triggered_ = false;
              
           }
        }
    
};


#endif