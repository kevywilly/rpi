//============================================================================
// Name        : robot.h
// Author      : Kevin Williams
// Version     :
// Copyright   : Your copyright notice
// Description : robot.h
//============================================================================

#include <ctime>
#include "pigpio.h"
#include "camera_mount.hpp"
#include "drivetrain.hpp"
#include "utils.h"
#include <cstdlib>
#include <iostream>
#include "tcpserver.h"
#include "mcp3008.h"
#include "sonar_sensor.h"
#include "neurocore/neuronic.h"
#include "robot_constants.h"
#include "navigation.h"
//#include "opencv2/opencv.hpp"

// FOR CONVENIENCE

using namespace std;
using namespace kruiser;

	
//============================================================================
// ROBOTSTATUS
//============================================================================

class Robot { 
	
//============================================================================
// Private Declarations
//============================================================================
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

//============================================================================
// Public Declarations
//============================================================================	
	public:
		
		bool IsAutonomous;
		bool IsTraining;
		Neuronic * neuralNetwork;
		Drivetrain * motors;
		CameraMount * cameraMount;
		MCP3008 * adcSensor;
		SonarSensor * sonar1;
		SonarSensor * sonar2;
		SonarSensor * sonar3;
		SonarSensor * sonar4;
		Navigation navigation;
		
//============================================================================
// Deconstructor
//============================================================================
	    virtual ~Robot() {
	    	if(IsTraining) {
	    		saveNetwork();
	    	}
	        if(trainingOfs.is_open()) {
	            trainingOfs.flush();
	            trainingOfs.close();
	        }
	    }
	    
//============================================================================
// Constructor
//============================================================================
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


//============================================================================
// Getters & Setters
//============================================================================
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
		

//============================================================================
// Initialization Methods
//============================================================================
				
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
			
			Logger::debug("refreshing navigation distances...\n");
			navUpdateDistances();
			
			Logger::debug("getting recommended action...\n");
			getRecommendedAction();
			
		}
		
//============================================================================
// Actuator Methods
//============================================================================

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
		
//============================================================================
// Sensor Methods
//============================================================================

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
				cout << "IR-" << i << ":" << Adcs[i] << ",";
			}
			cout << endl;
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
				cout << "Sonar-" << i << ":" << SonarValues[i] << ",";
			}
			cout << endl;
		}
	    
//============================================================================
// Neural Network Methods
//============================================================================

		// Save Neural Network
		void saveNetwork() {
			neuralNetwork->SaveNetwork();
		}
		
		// Build targets for Neural Network
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
		
		// Build Features for Neural Network
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
		
		// Train Neural Network
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
		
//============================================================================
// Rules Based Navigation Methods
//============================================================================

		clock_t clock_begin = clock();
		
		inline void startClock() {clock_begin = clock();}
		
		inline double elapsedSeconds() {return (double)(clock() - clock_begin) / CLOCKS_PER_SEC;}
		
		void navUpdateDistances() {
			cout << "updating distances" << endl;
			readAdcs();
			readSonars();
			
			navigation.FrontLeft = SonarValues[0];
			navigation.FrontRight = SonarValues[1];
			navigation.Left = SonarValues[2];
			navigation.Right = SonarValues[3];
			
			navigation.FrontLeft2 = Adcs[0];
			navigation.FrontRight2 = Adcs[1];
			navigation.RearLeft = Adcs[2];
			navigation.RearRight = Adcs[3];
			
			cout << navigation.FrontLeft << ",";
			cout << navigation.FrontRight << ",";
			cout << navigation.FrontLeft2 << ",";
			cout << navigation.FrontRight2 << ",";
			cout << navigation.Left << ",";
			cout << navigation.Right << ",";
			cout << navigation.RearLeft << ",";
			cout << navigation.RearRight << endl << endl;;
			
		}
		
		void navForward() {
			while(elapsedSeconds() < 0.2 && navigation.FrontIsClear()) {
				drive(0.4, 0.0);
				navUpdateDistances();
			}
		}
		
		// Shift slightly to the left
		void navShiftLeft() {
			startClock();
			//while(elapsedSeconds() < 1.0 && navigation.FrontIsClear() && navigation.LeftIsGt() && navigation.LeftIsClear()) {
				drive(0.4, -0.2);
				//delay(100);
				//navUpdateDistances();
			//}
		}
		
		// Shift slightly to the right
		void navShiftRight() {
			startClock();
			//while(elapsedSeconds() < 1.0 && navigation.FrontIsClear() && navigation.RightIsGt() && navigation.RightIsClear()) {
			drive(0.4, 0.2);
			//delay(100);
				//navUpdateDistances();
		
			//}
		}
		
		// Avoid obstacle on the left (turn right)
		void navAvoidLeft() {
			startClock();
			while(elapsedSeconds() < 0.5 && !(navigation.FrontLeftIsClear())) {
				drive(0.4, 1);
				navUpdateDistances();
		
			}
		}
		
		// Avoid obstacle on the right (turn left)
		void navAvoidRight() {
			startClock();
			while(elapsedSeconds() < 0.25 && !(navigation.FrontRightIsClear())) {
				drive(0.4, -1);
				navUpdateDistances();
			}
		}
		
		void navReverseLeft() {
			startClock();
			while(elapsedSeconds() < 0.25 && navigation.RearRightIsClear())
			{
				drive(-0.4, 1.0);
				navUpdateDistances();
			}
		}
		
		void navReverseRight() {
			startClock();
			//while(elapsedSeconds() < 1.0 && navigation.RearLeftIsClear())
			//{
				drive(-0.4, -1.0);
				navUpdateDistances();
			//}
		}
		
		void navExecute() {
			if(!IsAutonomous)
		        return;
		    navUpdateDistances();
		    
			if(navigation.FrontIsClear()) { 
				//front is clear adjust to center of lane or drive forward
				if(navigation.LeftIsGt()) {
					navShiftLeft();
				} else if(navigation.RightIsGt()) {
					navShiftRight();
				} else {
					navForward();
				}
			} else if(navigation.FrontRightIsClear() && (navigation.FrontRightIsGt() || navigation.RightIsGt())) { // right looks better
				navAvoidLeft();
			} else if(navigation.FrontLeftIsClear()) {
				navAvoidRight();
			} else if(navigation.FrontLeftIsGt()) { // reverse, left is more clear
				navReverseLeft();
			} else  { // reverse, right is more clear
				navReverseRight();
			}
			
			delay(50);
		}
};