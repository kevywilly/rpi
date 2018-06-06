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
   *GND (25) (26) GPIO7* 
  GPIO0 (27) (28) GPIO1 
 *GPIO5 (29) (30) GND   
 *GPIO6 (31) (32) GPIO12
 *GPIO13 (33) (34) GND   
 GPIO19 (35) (36) GPIO16
 GPIO26 (37) (38) GPIO20
    GND (39) (40) GPIO21

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
//#include "opencv2/opencv.hpp"

// FOR CONVENIENCE
using json = nlohmann::json;
using namespace std;
using namespace kruiser;


// ***************************************************
// SONAR 
// ***************************************************
#define SONAR1_TRIGGER 19
#define SONAR1_ECHO    26
#define SONARS 6
SonarSensor * sonar1;



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
#define ADCMAXDISTANCE 30



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
	
	Drivetrain * motors;
	CameraMount * camera_mount;
	
	RobotStatus() {
		Pitch = 0.0;
		Yaw = 0.0;
		Speed = 0.0;
		Turn = 0.0;
		Adcs = new int[ADCS];
		SonarValues = new int[SONARS];
		
		for(int i=0; i < ADCS; i++) {
			Adcs[i] = ADCMAXDISTANCE;
		}
		
		for(int i=0; i < SONARS; i++) {
			SonarValues[i] = MAX_SONAR_DISTANCE;
		}
	}
	
	// Read ADC values
	void readAdcs() {
		adc_sensor_->readMulti(Adcs, ADCS);
	}
	
	void readSonars() {
		SonarValues[0] = sonar1->distance;
	}
	
	// Generate status as json string
	string statusToJsonString() {

		
		stringstream oss;
		oss << "{\"adc\":[";
		for(int i=0; i < ADCS; i ++) {
			oss << Adcs[i];
			if(i < (ADCS-1))
				oss << ",";
		}
		oss << "]";
		
		oss << ",\"sonar\":[";
		for(int i=0; i < SONARS; i ++) {
			oss << SonarValues[i];
			if(i < (SONARS-1))
				oss << ",";
		}
		oss << "]";
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
}

void startSonars() {
	Logger::debug("initializing sonar...");
	sonar1 = new SonarSensor(SONAR1_TRIGGER, SONAR1_ECHO, 0, sonarTrigger, sonarCallback);
	sonar1->start();
	Logger::debug("success\n");
}

void stopSonars() {
	sonar1->stop();
}

// Dispatch TCP Message
string dispatch(const string& data) {
	
	try {
		auto j = json::parse(data);
		string cmd = j["cmd"];
		
		robotstatus.readAdcs();
		robotstatus.readSonars();
		
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