#include "pigpio.h"
#include "camera_mount.hpp"
#include "drivetrain.hpp"
#include "utils.h"
#include <cstdlib>
#include <iostream>
//#include <list>
#include "tcpserver.h"
#include "mcp3008.h"
#include "sonar_sensor.h"
#include "neurocore/neuronic.h"
//#include "opencv2/opencv.hpp"

// FOR CONVENIENCE
//using json = nlohmann::json;
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
#define REVERSE_THRESHOLD 0.25
#define TURN_THRESHOLD 0.1
#define RISK_THRESHOLD 0.75
#define TURN_NEURON_TRIGGER 0.5
#define REVERSE_NEURON_TRIGGER 0.75

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

#define FORWARD 0
#define FORWARD_RIGHT 1
#define FORWARD_LEFT 2
#define REVERSE_RIGHT 3
#define REVERSE_LEFT 4
#define REVERSE 5

const string CONFIG_FILE = "/home/pi/neural_network/kruiser/kruiser.yml";
const string TRAINING_FILE = "/home/pi/neural_network/kruiser/live_training.csv";


	
// ***************************************************
// ROBOTSTATUS
// ***************************************************
class Robot { 
    private:
        int * Adcs;
		int * SonarValues;
		float Pitch;
		float Yaw;
		float Speed, PrevSpeed;
		float Turn, PrevTurn;
		float * IRProximity;
		float * SonarProximity;
		double * RecommendedAction;
		void (*sonarCallback)(int, int, uint32_t);
		void (*sonarTrigger)(void);
		void(*trainingCallback)(int,double);
		
		double drive_parameters[6][4] = {
			{ 0.5, 0.0, 0.0}, // FORWARD
			{ 0.4, 0.5, 1.0}, // FORWARD RIGHT
			{ 0.4,-0.5,-1.0}, // FORWARD LEFT
			{-0.4, 0.5, 1.0}, // REVERSE RIGHT
			{-0.4,-0.5,-1.0}, // REVERSE LEFT
			{-0.4, 0.0, 0.0} // REVERSE
		};
		
		
		
		int NumInputs = 0;
		int NumOutputs = 0;
		
		std::ofstream trainingOfs;
		
		//std::list<std::list<double>> TrainingValues;
		
	public:
		// RobotStatus Members
		bool IsAutonomous;
		bool IsTraining;
		
		// Object pointers
		Neuronic * neuralNetwork;
		Drivetrain * motors;
		CameraMount * cameraMount;
		MCP3008 * adcSensor;
		SonarSensor * sonar1;
		SonarSensor * sonar2;
		SonarSensor * sonar3;
		SonarSensor * sonar4;
		
		
	
	    virtual ~Robot() {
	    	if(IsTraining) {
	    		saveNetwork();
	    	}
	        if(trainingOfs.is_open()) {
	            trainingOfs.flush();
	            trainingOfs.close();
	        }
	    }
		// RobotStatus Constructor
		Robot(void(*sonarCallbackFunction)(int, int, uint32_t), void(*sonarTriggerFunction)(void), void(*trainingCallbackFunction)(int,double)) {

			
			int i;
			sonarCallback = sonarCallbackFunction;
			sonarTrigger = sonarTriggerFunction;
			trainingCallback = trainingCallbackFunction;
			
			IsAutonomous = false;
			IsTraining = false;
			Pitch = 0.0;
			Yaw = 0.0;
			Speed = 0.0;
			Turn = 0.0;
			PrevSpeed = 0.0;
			PrevTurn = 0.0;
			
			
			Adcs = new int[ADCS];
			SonarValues = new int[SONARS];
			
			IRProximity = new float[ADCS];
			SonarProximity = new float[SONARS];
			
			for(i=0; i < ADCS; i++) {
				Adcs[i] = IR_MAX_DISTANCE;
				IRProximity[i] = 0;
			}
			
			for(i=0; i < SONARS; i++) {
				SonarValues[i] = MAX_SONAR_DISTANCE;
				SonarProximity[i] = 0;
			}
			
		}
		
		void saveNetwork() {
			neuralNetwork->SaveNetwork();
		}
		double getSpeed() {
		    return Speed;
		}
		
		double getTurn() {
		    return Turn;
		}
		
		void setPrevSpeed(double value) {
		    PrevSpeed = value;
		}
		
		void setPrevTurn(double value) {
		    PrevTurn = value;
		}
		
		void setSpeed(double value) {
		    PrevSpeed = Speed;
		    Speed = value;
		}
		
		void setTurn(double value) {
		    PrevTurn = Turn;
		    Turn = value;
		}
		
		
		void initialize() {
			
			Logger::debug("initializing neural network...\n");
			neuralNetwork = new Neuronic(CONFIG_FILE);
			
			NumInputs = neuralNetwork->network.NumInputNeurons;
			NumOutputs = neuralNetwork->network.NumOutputNeurons;
			
			RecommendedAction = new double[NumOutputs];
			
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
		    int i;
			adcSensor->readMulti(Adcs, ADCS);
			for(i=0; i < ADCS; i++) {
				IRProximity[i] = ((double)Adcs[i])/IR_MAX_DISTANCE;
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
		    int i;
			SonarValues[0] = sonar1->distance;
			SonarValues[1] = sonar2->distance;
			SonarValues[2] = sonar3->distance;
			SonarValues[3] = sonar4->distance;
			for(i=0; i < SONARS; i++) {
				SonarProximity[i] = ((double)SonarValues[i])/MAX_SONAR_DISTANCE;
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
		    int i;
			cout << endl; 
			for(i=0; i < ADCS; i++) {
				cout << i << ":" << Adcs[i] << "  ";
			}
			cout << endl; 
		}
		
		void printSonarValues() {
		    int i;
			cout << endl; 
			for(i=0; i < SONARS; i++) {
				cout << i << ":" << SonarValues[i] << "  ";
			}
			cout << endl; 
		}
	    
		
		// Build target outputs based on speed and turn
		
	
		
		
		double * buildTargets(double sp, double tr) {
		    
		    int i;
		    
		    // holder for targets
			double * targets = new double[NumOutputs];
			
			for(i=0; i<NumOutputs; i++)
				targets[i] = 0.1;
            
            if(sp < 0) { // Reversing
            	if(tr == 0)
            		targets[REVERSE] = 0.9;
            	else if(tr < 0)
            		targets[REVERSE_LEFT] = 0.9;
            	else
            		targets[REVERSE_RIGHT] = 0.9;
            } else {
            	if(tr == 0)
            		targets[FORWARD] = 0.9;
            	else if(tr < 0)
            		targets[FORWARD_LEFT] = 0.9;
            	else
            		targets[FORWARD_RIGHT] = 0.9;
            }
            
            
			return targets;
		}
		
		
		double * buildFeatures() {
			double * features = new double[NumInputs];
			double * prevTargets = buildTargets(PrevSpeed, PrevTurn);
			int i;
			
			readAdcs();
		    readSonars();
			
			features[0] = 1.0 - ((double)Adcs[0]) / 30.0;
			features[1] = 1.0 - ((double)Adcs[1]) / 30.0;
			features[2] = 1.0 - ((double)Adcs[2]) / 30.0;
			features[3] = 1.0 - ((double)Adcs[3]) / 30.0;
			
			features[4] = 1.0 - (SonarValues[0] > 100 ? 100.0 : SonarValues[0]) / 100.0;
			features[5] = 1.0 - (SonarValues[1] > 100 ? 100.0 : SonarValues[1]) / 100.0;
			features[6] = 1.0 - (SonarValues[2] > 100 ? 100.0 : SonarValues[2]) / 100.0;
			features[7] = 1.0 - (SonarValues[3] > 100 ? 100.0 : SonarValues[3]) / 100.0;
			
			return features;
		}
		
	
		void train() {
		    if(IsAutonomous || (Speed == 0.0))
		        return;
		    
		    
		        
		    int i;
		    
		
		    double * inputs = buildFeatures();
		    double * targets = buildTargets(Speed, Turn);
		    
            
            if(IsTraining) {
		        double err = neuralNetwork->network.TrainOne(inputs, targets, 0.00001, 10);
		        cout << "training error = " << err << endl;
            }
		    
		    writeTrainingData(inputs, targets);
		    
		    
		}
		double * getRecommendedAction() {
		    int i;
			double * features = buildFeatures();
			RecommendedAction = neuralNetwork->network.FeedInputs(features);
			
			return RecommendedAction;
		}
		
		void drive(double speed, double turn) {
		    setSpeed(speed);
		    setTurn(turn);
		    motors->drive(Speed*100, Turn*100);
		    cout << "drive(" << speed << "," << turn << ")" << endl;
		}
		
		void moveCamera(double pitch, double yaw) {
		    Pitch = pitch;
		    Yaw = yaw;
		    double actual_pitch = Pitch+90;
			if(actual_pitch > 90) {
				actual_pitch = 90;
			}
			cameraMount->move_to(actual_pitch, Yaw);
		}
		
		bool getSeverity() {
		    int i;
		    
		    for(i=0; i < ADCS; i++) {
		        if(IRProximity[i] > 0.2 || SonarProximity[i] > 0.5)
		            return true;
		    }
		    return false;
		}
		
		
		// Drives autonomously based on feedback from neural net
		void runAutonomously() {
		    if(!IsAutonomous)
		        return;
		    
		    // Declarations
		    
		    int i;
		    int recommendedIndex = 0;
		    
		    double recSpeed = 0.0;
		    double recTurn = 0.0;
			double maxScore = 0.0;
			double score;
			
		    // Read Sonar and IR Values
		    readAdcs();
		    readSonars();
		    
		    // Get recommended action from neural network
		    // This is an array of outputs stored in variable RecommendedAction[]
		    getRecommendedAction();
		    
		   
			// Loop through recommended actions and find the highest ranking action
			cout << "Rec: ";
			for(i=0; i < NumOutputs; i++) {
				score = RecommendedAction[i];
			    cout << score << ",";
			    if(score > maxScore) {
			    	recommendedIndex = i;
			    	maxScore = score;
			    }
			}
			cout << endl;
		    
		   
		    // Lookup drive parameters table to find the speed and turn associated with recommended actions 
		    
		    // Speed
		    recSpeed = drive_parameters[recommendedIndex][0];
		    
		    // If severity is high (one ore more sensors read object way too close, use the high severity value of turn pct)
		    recTurn = getSeverity() > 0.5 ? drive_parameters[recommendedIndex][2] : drive_parameters[recommendedIndex][1];
		    
		    // Tell motors to drive and turn
		    drive(recSpeed, recTurn);
		    
		    // Delay
		    delay(100);
		    
		}
	
		void writeTrainingData(double * features, double * targets) {
		    int i;
		    if(!trainingOfs.is_open())
		        trainingOfs.open(TRAINING_FILE, std::ofstream::out | std::ofstream::app);
		        
		    if(!trainingOfs.is_open())
		        return;
		        
		    for(i=0; i < NumInputs; i++) 
		        trainingOfs << features[i] << ",";
		        
		    for(i=0; i < NumOutputs; i++) {
		        trainingOfs << targets[i];
		        if(i < (NumOutputs -1))
		            trainingOfs << ",";
		    }
		    
		    trainingOfs << endl;
		}
		
};