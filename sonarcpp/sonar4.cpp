#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>

/*

P1  Name  gpio    used for

 2  5V    ---     5V
 6  GND   ---     Ground
24  CE0   8       Sonar echo
26  CE1   7       Sonar trigger

*/

#define SONAR_TRIGGER 19
#define SONAR_ECHO    26
#define MICROSECONDS_PER_CM  (1000000/34321)

/* forward prototypes */

void sonarTrigger(void);

void sonarEcho(int gpio, int level, uint32_t tick);

//void triggered(int gpio, int level, uint32_t tick);

static uint32_t startTick, firstTick=0;

int main(int argc, char *argv[])
{
   if (gpioInitialise()<0) return 1;

   gpioSetMode(SONAR_TRIGGER, PI_OUTPUT);
   gpioWrite  (SONAR_TRIGGER, PI_OFF);

   gpioSetMode(SONAR_ECHO,    PI_INPUT);

   /* update sonar 20 times a second, timer #0 */

   gpioSetTimerFunc(0, 50, sonarTrigger); /* every 50ms */

   /* monitor sonar echos */

   //gpioSetAlertFunc(SONAR_TRIGGER, triggered);
   

   while (1) sleep(1);

   gpioTerminate();

   return 0;
}

void sonarTrigger(void)
{
   /* trigger a sonar reading */
   //uint32_t startTick, firstTick=0;
   
   //gpioSetAlertFunc(SONAR_ECHO, NULL);
   
   gpioTrigger(SONAR_TRIGGER, 10, PI_ON);
   
   //gpioSetAlertFunc(SONAR_ECHO, sonarEcho);
   
   //gpioDelay(10); /* 10us trigger pulse */

   //gpioWrite(SONAR_TRIGGER, PI_OFF);
}

/*
void triggered(int gpio, int level, uint32_t tick) {
    if(level == 1) {
        gpioSetAlertFunc(SONAR_ECHO, sonarEcho);
    }
}
*/

void sonarEcho(int gpio, int level, uint32_t tick)
{
   //static uint32_t startTick, firstTick=0;

   int diffTick;

   if (!firstTick) firstTick = tick;

   if (level == PI_ON)
   {
      startTick = tick;
   }
   else if (level == PI_OFF)
   {
      diffTick = ((tick >> 0) - (startTick >> 0)) / 2 / MICROSECONDS_PER_CM;
      //diffTick = tick - startTick;
      firstTick = startTick = 0;
      
      diffTick = (diffTick > 400 || diffTick <= 5) ? 0 : diffTick;   
   
      printf("%u\n",  diffTick);
   }
}