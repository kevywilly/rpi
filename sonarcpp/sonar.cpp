#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>
#include "sonar_sensor.h"
/*

P1  Name  gpio    used for

 2  5V    ---     5V
 6  GND   ---     Ground
24  CE0   8       Sonar echo
26  CE1   7       Sonar trigger

*/

#define SONAR1_TRIGGER 19
#define SONAR1_ECHO    26


/* forward prototypes */
void sonarCallback(int gpio, int level, uint32_t tick);
void sonar1Trigger();

SonarSensor *getSonar(int gpio);
SonarSensor * sonar1;

int main(int argc, char *argv[])
{
   
   if (gpioInitialise()<0) return 1;
   
   sonar1 = new SonarSensor(SONAR1_TRIGGER, SONAR1_ECHO, 0, sonar1Trigger, sonarCallback);
   sonar1->start();

   while (1) sleep(1);

   gpioTerminate();

   return 0;
}


SonarSensor *getSonar(int gpio) {
   switch(gpio) {
         case SONAR1_ECHO:
            return sonar1;
            break;
   }
   
   return NULL;
}
void sonarCallback(int gpio, int level, uint32_t tick){
      SonarSensor *sensor = getSonar(gpio);
      
      if(sensor != NULL) {
         sensor->sonarEcho(gpio, level, tick);
         printf("gpio %u: %u\n", gpio, sensor->distance);
      }
}

void sonar1Trigger(){
      sonar1->sonarTrigger();
}
