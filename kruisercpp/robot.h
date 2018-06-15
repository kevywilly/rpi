#include "pigpio.h"
#include "camera_mount.hpp"
#include "drivetrain.hpp"
#include "utils.h"
#include <cstdlib>
#include <iostream>
#include "tcpserver.h"
#include "json.hpp"
#include "mcp3008.h"
#include "sonar_sensor.h"
#include "neurocore/neuronic.h"
//#include "opencv2/opencv.hpp"

// FOR CONVENIENCE
using json = nlohmann::json;
using namespace std;
using namespace kruiser;

//Sonar
#define SONAR1_TRIGGER 19
#define SONAR2_TRIGGER 16
#define SONAR3_TRIGGER 12
#define SONAR4_TRIGGER	7
#define SONAR1_ECHO    26
#define SONAR2_ECHO	   20
#define SONAR3_ECHO	   21
#define SONAR4_ECHO		1
#define SONARS 4

//Neural Network
#define NUM_TARGETS 4

// ADCS
#define CE0 8
#define CE1 7
#define MISO 9
#define MOSI 10
#define CLK 11

#define ADCS 4

// IR
#define IR_MAX_DISTANCE 30

// CAMERA MOUNT
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

// MOTORS
#define M_REAR1 14
#define M_REAR2 15
#define M_REAR_PWM 18

#define M_FRONT1 5
#define M_FRONT2 6
#define M_FRONT_PWM 13



// ***************************************************
// ROBOTSTATUS
// ***************************************************
class Robot { 
	public:
		// RobotStatus Members
		int * Adcs;
		int * SonarValues;
		float Pitch;
		float Yaw;
		float Speed;
		float Turn;
		float * IRProximity;
		float * SonarProximity;
		double * RecommendedAction;
		void (*sonarCallback)(int, int, uint32_t);
		void (*sonarTrigger)(void);
		
		// Object pointers
		Neuronic * neuralNetwork;
		Drivetrain * motors;
		CameraMount * cameraMount;
		MCP3008 * adcSensor;
		SonarSensor * sonar1;
		SonarSensor * sonar2;
		SonarSensor * sonar3;
		SonarSensor * sonar4;
	
		double TEST_INPUTS[10] = {
				-1,-0.479167,0.0,0.0,0.0,0.0,0.8375,0.8375,0.8049999999999999,0.8049999999999999
		};
	
		// RobotStatus Constructor
		Robot(void(*sonarCallbackFunction)(int, int, uint32_t), void(*sonarTriggerFunction)(void)) {
			
			sonarCallback = sonarCallbackFunction;
			sonarTrigger = sonarTriggerFunction;
			
			Pitch = 0.0;
			Yaw = 0.0;
			Speed = 0.0;
			Turn = 0.0;
			
			RecommendedAction = new double[NUM_TARGETS];
			
			Adcs = new int[ADCS];
			SonarValues = new int[SONARS];
			
			IRProximity = new float[ADCS];
			SonarProximity = new float[SONARS];
			
			for(int i=0; i < ADCS; i++) {
				Adcs[i] = IR_MAX_DISTANCE;
				IRProximity[i] = 0;
			}
			
			for(int i=0; i < SONARS; i++) {
				SonarValues[i] = MAX_SONAR_DISTANCE;
				SonarProximity[i] = 0;
			}
		}
		
		void initialize() {
			
			Logger::debug("initializing neural network...\n");
			neuralNetwork = new Neuronic("/home/pi/neural_network/kruiser/kruiser.yml");
			
			Logger::debug("starting sonars...\n");
			startSonars();
			
			Logger::debug("starting camera mount...\n");
			cameraMount = new CameraMount(CAMERA_PITCH_PIN, CAMERA_YAW_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
			
			Logger::debug("starting motors...\n");
			motors = new Drivetrain(M_REAR1,M_REAR2,M_REAR_PWM,M_FRONT1,M_FRONT2,M_FRONT_PWM);
			
			Logger::debug("starting adcSensor...\n");
			adcSensor = new MCP3008(CE0, MISO, MOSI, CLK);
			
			Logger::debug("moving camera into position...\n");
			cameraMount->move_to(CAMERA_DEFAULT_PITCH,CAMERA_DEFAULT_YAW);
			
			Logger::debug("reading adc...\n");
			readAdcs();
			
			Logger::debug("reading sonars...\n");
			readSonars();
			
			Logger::debug("getting recommended action...\n");
			getRecommendedAction();
			
			Logger::debug("generating status json...\n");
			cout << statusToJsonString() << endl;
		}
		
		void startSonars() {
			
			sonar1 = new SonarSensor(SONAR1_TRIGGER, SONAR1_ECHO);
			sonar2 = new SonarSensor(SONAR2_TRIGGER, SONAR2_ECHO);
			sonar3 = new SonarSensor(SONAR3_TRIGGER, SONAR3_ECHO);
			sonar4 = new SonarSensor(SONAR4_TRIGGER, SONAR4_ECHO);
			
			gpioSetTimerFunc(0, 50, sonarTrigger);
			gpioSetAlertFunc(SONAR1_ECHO, sonarCallback);
			gpioSetAlertFunc(SONAR2_ECHO, sonarCallback);
			gpioSetAlertFunc(SONAR3_ECHO, sonarCallback);
			gpioSetAlertFunc(SONAR4_ECHO, sonarCallback);
			
		}
		
		void stopSonars() {
			gpioSetTimerFunc(0, 50, NULL);
			gpioSetAlertFunc(SONAR1_ECHO, NULL);
			gpioSetAlertFunc(SONAR2_ECHO, NULL);
			gpioSetAlertFunc(SONAR3_ECHO, NULL);
			gpioSetAlertFunc(SONAR4_ECHO, NULL);
		}
		
		// Read ADC values
		void readAdcs() {
			adcSensor->readMulti(Adcs, ADCS);
			for(int i=0; i < ADCS; i++) {
				IRProximity[i] = 1.0 - (Adcs[i]/(IR_MAX_DISTANCE*1.0));
			}
		}
		
		SonarSensor *getSonar(int gpio) {
		   switch(gpio) {
		        case SONAR1_ECHO:
		            return sonar1;
		            break;
		        case SONAR2_ECHO:
		            return sonar2;
		            break;
		        case SONAR3_ECHO:
		            return sonar3;
		            break;
		        case SONAR4_ECHO:
		            return sonar4;
		            break;
		        default:
		        	return NULL;
		   }
		   return NULL;
		}
		
		void readSonars() {
			SonarValues[0] = sonar1->distance;
			SonarValues[1] = sonar2->distance;
			SonarValues[2] = sonar3->distance;
			SonarValues[3] = sonar4->distance;
			for(int i=0; i < SONARS; i++) {
				SonarProximity[i] = 1.0 - ((SonarValues[i] > 100 ? 100.0 : SonarValues[i])/(100.0));
			}
		}
		
		// Generate status as json string
		string statusToJsonString() {
			int i;
			
			stringstream oss;
			
			// adc values
			oss << "{\"adc\":[";
			for(i=0; i < ADCS; i ++) {
				oss << Adcs[i];
				if(i < (ADCS-1))
					oss << ",";
			}
			oss << "]";
			
			// sonar values
			oss << ",\"sonar\":[";
			for(i=0; i < SONARS; i ++) {
				oss << SonarValues[i];
				if(i < (SONARS-1))
					oss << ",";
			}
			oss << "]";
			
			// proximity values
			oss << ",\"proximity\":[";
			for(i=0; i < ADCS; i++) {
				oss << IRProximity[i] << ",";
			}
			for(i=0; i < SONARS; i++) {
				oss << SonarProximity[i];
				if(i < (SONARS-1))
					oss << ",";
			}
			oss << "]";
			
			// recommended action
			oss << ",\"rec\":[";
			for(i=0; i < NUM_TARGETS; i ++) {
				oss << RecommendedAction[i];
				if(i < (7))
					oss << ",";
			}
			oss << "]";
			
			// speed, pitch, turn, yaw
			oss << "," << "\"speed\":" << Speed;
			oss << "," << "\"turn\":" << Turn;
			oss << "," << "\"pitch\":" << Pitch;
			oss << "," << "\"yaw\":" << Yaw;
			oss << "}";
			
			return oss.str();
		}
		
		
		// Print ADC Values
		void printAdcs() {
			cout << endl; 
			for(int i=0; i < ADCS; i++) {
				cout << i << ":" << Adcs[i] << "  ";
			}
			cout << endl; 
		}
		
		void printSonarValues() {
			cout << endl; 
			for(int i=0; i < SONARS; i++) {
				cout << i << ":" << SonarValues[i] << "  ";
			}
			cout << endl; 
		}
	    
		double * buildFeatures() {
			double * features = new double[10];
			features[0] = Speed;
			features[1] = Turn;
			for(int i=0; i < 4; i++) {
				features[2+i] = IRProximity[i];
				features[6+i] = SonarProximity[i];
			}
			return features;
		}
		
		double * buildTargets(double speed, double turn) {
			double * targets = new double[NUM_TARGETS];
			return targets;
		}
		
		double * getRecommendedAction() {
			double * features = buildFeatures();
			RecommendedAction = neuralNetwork->network.FeedInputs(features);
			return RecommendedAction;
		}
		
		
};