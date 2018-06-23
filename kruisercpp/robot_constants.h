//============================================================================
// Sonar
//============================================================================
#define SONAR1_TRIGGER 19
#define SONAR2_TRIGGER 16
#define SONAR3_TRIGGER 12
#define SONAR4_TRIGGER	7
#define SONAR1_ECHO    26
#define SONAR2_ECHO	   20
#define SONAR3_ECHO	   21
#define SONAR4_ECHO		1
#define SONARS 4

//============================================================================
// ADCS
//============================================================================
#define CE0 8
#define CE1 7
#define MISO 9
#define MOSI 10
#define CLK 11

#define ADCS 4

//============================================================================
// IR
//============================================================================
#define IR_MAX_DISTANCE 30

//============================================================================
// CAMERA MOUNT
//============================================================================
#define SERVO_MIN_PULSE 600
#define SERVO_MAX_PULSE 2300
#define CAMERA_PITCH_PIN 27
#define CAMERA_YAW_PIN 17
#define CAMERA_DEFAULT_PITCH 90
#define CAMERA_DEFAULT_YAW 0
/*
  enable dev/video0
  sudo modprobe bcm2835-v4l2 
*/

//============================================================================
// MOTORS
//============================================================================
#define M_REAR1 14
#define M_REAR2 15
#define M_REAR_PWM 18

#define M_FRONT1 5
#define M_FRONT2 6
#define M_FRONT_PWM 13

#define FORWARD 0
#define FORWARD_RIGHT 1
#define FORWARD_LEFT 2
#define REVERSE_RIGHT 3
#define REVERSE_LEFT 4
#define REVERSE 5

//============================================================================
// DATA FILES
//============================================================================
const string CONFIG_FILE = "/home/pi/neural_network/kruiser/kruiser.yml";
const string TRAINING_FILE = "/home/pi/neural_network/kruiser/live_training.csv";