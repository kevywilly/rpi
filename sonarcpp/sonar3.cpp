#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>


void cbf(int gpio, int level, uint32_t tick);
void fnTrigger();

uint8_t trigger_ = 19;
uint8_t echo_ = 26;
uint32_t high_ = 0;
uint32_t time_ = 0;

bool triggered_ = false;
bool ping_ = false;
bool _inited = true;

int main(int argc, char *argv[])
{
   // initialize gpio  
    if (gpioInitialise()<0) return 1;
   
   // initialize pins
    gpioSetMode(trigger_, PI_OUTPUT);
    gpioWrite  (trigger_, PI_OFF);
    gpioSetMode(echo_,    PI_INPUT);
   
    gpioSetAlertFunc(trigger_, cbf);
    gpioSetAlertFunc(echo_, cbf);
    
    gpioSetTimerFunc(1, 100, fnTrigger);
    
    while(1) {}
    
    gpioTerminate();

}

void fnTrigger() {
    ping_ = false;
    high_ = 0;
    time_ = 0;
    triggered_ = 0;
    
    gpioWrite(trigger_, PI_ON);

    gpioDelay(10); //10us trigger pulse

    gpioWrite(trigger_, PI_OFF);
}

void cbf(int gpio, int level, uint32_t tick) {
    if(gpio == trigger_){
        if(level == 0){ //trigger sent
            triggered_ = true;
            high_ = 0;
            printf("triggered");
        }
    } else {
        if(triggered_) {
            if(level == 1) {
                high_ = tick;
            } else {
                if(high_ != 0) {
                    time_ = tick - high_;
                    high_ = 0;
                    ping_ = true;
                    printf("%u\n", time_/1000000 * 34030);
                }
            }
        }
    }
}

