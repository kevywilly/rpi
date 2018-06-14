/* 

	PI-ZERO PINOUT
	
   3V3  (1) (2)  5V    
 GPIO2  (3) (4)  5V    
 GPIO3  (5) (6)  GND   
 GPIO4  (7) (8)  GPIO14*
  *GND  (9) (10) GPIO15*
*GPIO17 (11) (12) GPIO18*
*GPIO27 (13) (14) GND   
 GPIO22 (15) (16) GPIO23
   *3V3 (17) (18) GPIO24
*GPIO10 (19) (20) GND   
 *GPIO9 (21) (22) GPIO25
*GPIO11 (23) (24) GPIO8* 
   *GND (25) (26) GPIO7* trigger4
  GPIO0 (27) (28) GPIO1* echo4
 *GPIO5 (29) (30) GND   
 *GPIO6 (31) (32) GPIO12*
 *GPIO13 (33) (34) GND   
 *GPIO19 (35) (36) GPIO16*
 *GPIO26 (37) (38) GPIO20*
    GND (39) (40) GPIO21*

*/

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


// ***************************************************
// SONAR 
// ***************************************************
#define SONAR1_TRIGGER 19
#define SONAR2_TRIGGER 16
#define SONAR3_TRIGGER 12
#define SONAR4_TRIGGER	7
#define SONAR1_ECHO    26
#define SONAR2_ECHO	   20
#define SONAR3_ECHO	   21
#define SONAR4_ECHO		1
#define SONARS 6

SonarSensor * sonar1;
SonarSensor * sonar2;
SonarSensor * sonar3;
SonarSensor * sonar4;



// ***************************************************
// NEURAL NETWORK
// ***************************************************
Neuronic neural("/home/pi/neural_network/kruiser/kruiser.yml");
double TEST_INPUTS[10] = {
			-0.175,-0.479167,0.0,0.0,0.0,0.0,0.8375,0.8375,0.8049999999999999,0.8049999999999999
	};

#define NUM_ACTIONS 8

// ***************************************************
// TCP SERVER 
// ***************************************************

TCPServer tcp;


// ***************************************************
// ADC & Status
// ***************************************************
#define CE0 8
#define CE1 7
#define MISO 9
#define MOSI 10
#define CLK 11
#define ADCS 8
#define IR_MAX_DISTANCE 30



// ***************************************************
// CAMERA MOUNT 
// ***************************************************
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

// ****************************************************
// MOTORS
// ****************************************************

#define M_REAR1 14
#define M_REAR2 15
#define M_REAR_PWM 18

#define M_FRONT1 5
#define M_FRONT2 6
#define M_FRONT_PWM 13

// ***************************************************
// ROBOTSTATUS
// ***************************************************
class RobotStatus { 
	public:
	
	int * Adcs;
	int * SonarValues;
	float Pitch;
	float Yaw;
	float Speed;
	float Turn;
	float * IRProximity;
	float * SonarProximity;
	double * RecommendedAction;
	
	Drivetrain * motors;
	CameraMount * camera_mount;
	
	RobotStatus() {
		Pitch = 0.0;
		Yaw = 0.0;
		Speed = 0.0;
		Turn = 0.0;
		
		RecommendedAction = new double[NUM_ACTIONS];
		
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
	
	// Read ADC values
	void readAdcs() {
		adc_sensor_->readMulti(Adcs, ADCS);
		for(int i=0; i < ADCS; i++) {
			IRProximity[i] = 1.0 - (Adcs[i]/(IR_MAX_DISTANCE*1.0));
		}
	}
	
	void readSonars() {
		SonarValues[0] = sonar1->distance;
		SonarValues[1] = sonar2->distance;
		SonarValues[2] = sonar3->distance;
		SonarValues[3] = sonar4->distance;
		for(int i=0; i < SONARS; i++) {
			SonarProximity[i] = 1.0 - (SonarValues[i]/(MAX_SONAR_DISTANCE*1.0));
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
		for(i=0; i < NUM_ACTIONS; i ++) {
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
	
	void initialize() {
		camera_mount = new CameraMount(CAMERA_PITCH_PIN, CAMERA_YAW_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
		motors = new Drivetrain(M_REAR1,M_REAR2,M_REAR_PWM,M_FRONT1,M_FRONT2,M_FRONT_PWM);
		adc_sensor_ = new MCP3008(CE0, MISO, MOSI, CLK);	
		camera_mount->move_to(CAMERA_DEFAULT_PITCH,CAMERA_DEFAULT_YAW);
		readAdcs();
		printAdcs();
	}

	double adcToFeature(double value) { return 1.0 - value/30.0;}

    double sonarToFeature(double value) {return 1.0 - value/400.0;}
    
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
	
	double * getRecommendedAction() {
		double * features = buildFeatures();
		/*
		for(int i=0; i < 10; i++) {
			cout << features[i] << endl;
		}
		*/
		RecommendedAction = neural.network.FeedInputs(features);
		/*
		for(int i =0; i < neural.network.NumOutputNeurons; i++) {
			cout << outputs[i] << endl;
		}
		*/
		
		
		return RecommendedAction;
		
	}

	private:
		MCP3008 * adc_sensor_;
		
		
		
};

RobotStatus robotstatus;

// *******************************************
// Sonar Methods
// *******************************************

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

void sonarCallback(int gpio, int level, uint32_t tick){
      SonarSensor *sensor = getSonar(gpio);
      
      if(sensor != NULL) {
         sensor->sonarEcho(gpio, level, tick);
         //printf("gpio %u: %u\n", gpio, sensor->distance);
      }
}

void sonarTrigger(){
      sonar1->sonarTrigger();
      gpioDelay(10);
      sonar2->sonarTrigger();
      gpioDelay(10);
      sonar3->sonarTrigger();
      gpioDelay(10);
      sonar4->sonarTrigger();
}

void startSonars() {
	Logger::debug("initializing sonar...");
	sonar1 = new SonarSensor(SONAR1_TRIGGER, SONAR1_ECHO);
	sonar2 = new SonarSensor(SONAR2_TRIGGER, SONAR2_ECHO);
	sonar3 = new SonarSensor(SONAR3_TRIGGER, SONAR3_ECHO);
	sonar4 = new SonarSensor(SONAR4_TRIGGER, SONAR4_ECHO);
	
	gpioSetTimerFunc(0, 50, sonarTrigger);
	gpioSetAlertFunc(SONAR1_ECHO, sonarCallback);
	gpioSetAlertFunc(SONAR2_ECHO, sonarCallback);
	gpioSetAlertFunc(SONAR3_ECHO, sonarCallback);
	gpioSetAlertFunc(SONAR4_ECHO, sonarCallback);
	
	Logger::debug("success\n");
}

void stopSonars() {
	gpioSetTimerFunc(0, 50, NULL);
	gpioSetAlertFunc(SONAR1_ECHO, NULL);
	gpioSetAlertFunc(SONAR2_ECHO, NULL);
	gpioSetAlertFunc(SONAR3_ECHO, NULL);
	gpioSetAlertFunc(SONAR4_ECHO, NULL);
}

// Dispatch TCP Message
string dispatch(const string& data) {
	
	try {
		auto j = json::parse(data);
		string cmd = j["cmd"];
		
		robotstatus.readAdcs();
		robotstatus.readSonars();
		robotstatus.getRecommendedAction();
		
		if(cmd == "setmode") {
			int mode = j["mode"];
		} else if(cmd == "drive") {
			robotstatus.Speed = j["speed"];
			robotstatus.Turn = j["turn"];
			robotstatus.motors->drive(robotstatus.Speed*100,robotstatus.Turn*100);
		} else if(cmd == "look") {
			robotstatus.Yaw = j["yaw"];
			robotstatus.Pitch = j["pitch"];
			float actual_pitch = robotstatus.Pitch+90;
			if(actual_pitch > 90) {
				actual_pitch = 90;
			}
			robotstatus.camera_mount->move_to(actual_pitch, robotstatus.Yaw);
		} else if(cmd == "adc") {
			
		}
	} catch (nlohmann::detail::parse_error& e) {
		cerr << "Exception in tcp dispatch: " << e.what() << endl;
		stringstream s;
		s << "{\"error\": ";
		s << e.what();
		s << "}";
		return s.str();
	}
	string js = robotstatus.statusToJsonString();
	cout << js << endl;
	return js;
}

// Loop listen and respond to messages
void * loop(void * m)
{
    pthread_detach(pthread_self());
	while(1)
	{
		srand(time(NULL));
		char ch = 'a' + rand() % 26;
		string s(1,ch);
		string str = tcp.getMessage();
		
		// Process message if any
		if( str != "" )
		{
			cout << "Message:" << str << endl;
			string result = dispatch(str);
			//tcp.Send(result+s);
			tcp.Send(result+"\n");
			tcp.clean();
		}
		
		
		usleep(1000);
	}
	tcp.detach();
}


// Cleanup
void cleanup() {
	stopSonars();
	gpioTerminate();
	
}


void start_tcp(int port) {
	pthread_t msg;
	
	tcp.setup(port);
	
	cout << "Listening on port: " << port << endl;
	// start receive thread
	if( pthread_create(&msg, NULL, loop, (void *)0) == 0)
	{
		tcp.receive();
	}
	
	
}

// Main entry point
int main(int argc, char* argv[])
{
	/*
	int r = capture();
	cout << capture << endl;
	return 0;
	*/
	Logger::debug("testing neural network...\n");
	double * outputs = neural.network.FeedInputs(TEST_INPUTS);
	for(int i =0; i < neural.network.NumOutputNeurons; i++) {
		cout << outputs[i] << endl;
	}
	Logger::debug("ok\n");
	
	Logger::debug("initializing pigpio...");
	
	if (gpioInitialise() < 0) {
		   Logger::debug("failed\n");
		   exit(1);
	}
	
	Logger::debug("success\n");
	
	
	try{ 
		
		startSonars();
		
		Logger::debug("initializing devices...");
		robotstatus.initialize(); 
		Logger::debug("success\n");
		
		
		
		Logger::debug("initializing tcp ...");
		start_tcp(8000); 
		Logger::debug("success\n");
		
	} catch (std::exception& e) {
		std::cerr << "Exception: " << e.what() << endl;
		exit(1);
	}
		
	
	
	while(true);
	
	return 0;
   
}


// Capture image using opencv
/*
int capture()
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    //namedWindow("edges",1);
        Mat frame;
        cap >> frame; // get a new frame from camera
        cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imwrite("edges.jpg", edges);
        //if(waitKey(30) >= 0) break;
        // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
*/
