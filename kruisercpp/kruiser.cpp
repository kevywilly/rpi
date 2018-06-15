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
#include <cstdlib>
#include <iostream>
#include "tcpserver.h"
#include "json.hpp"
#include "robot.h"
//#include "opencv2/opencv.hpp"

// FOR CONVENIENCE
using json = nlohmann::json;
using namespace std;
using namespace kruiser;

// FORWARD DECLARATIONS
void sonarCallback(int gpio, int level, uint32_t tick);
void sonarTrigger();
void * loop(void * m);
void cleanup();
void start_tcp(int port);
string dispatch(const string& data);

// TCP Server
TCPServer tcp;

// Robot
Robot robot(sonarCallback, sonarTrigger);

// Capture image using opencv

// Dispatch TCP Message
string dispatch(const string& data) {
	
	try {
		auto j = json::parse(data);
		string cmd = j["cmd"];
		
		robot.readAdcs();
		robot.readSonars();
		robot.getRecommendedAction();
		
		if(cmd == "setmode") {
			int mode = j["mode"];
		} else if(cmd == "drive") {
			robot.Speed = j["speed"];
			robot.Turn = j["turn"];
			robot.motors->drive(robot.Speed*100,robot.Turn*100);
		} else if(cmd == "look") {
			robot.Yaw = j["yaw"];
			robot.Pitch = j["pitch"];
			float actual_pitch = robot.Pitch+90;
			if(actual_pitch > 90) {
				actual_pitch = 90;
			}
			robot.cameraMount->move_to(actual_pitch, robot.Yaw);
		} else if(cmd == "adc") {
			
		}
	} catch (nlohmann::detail::parse_error& e) {
		cout << "Exception in tcp dispatch: " << e.what() << endl;
		stringstream s;
		s << "{\"error\": ";
		s << e.what();
		s << "}";
		return s.str();
	}
	string js = robot.statusToJsonString();
	cout << js << endl;
	return js;
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
	robot.stopSonars();
	gpioTerminate();
	
}


void sonarCallback(int gpio, int level, uint32_t tick){
      SonarSensor *sensor = robot.getSonar(gpio);
      
      if(sensor != NULL) {
         sensor->sonarEcho(gpio, level, tick);
         //printf("gpio %u: %u\n", gpio, sensor->distance);
      }
}

void sonarTrigger(){
	robot.sonar1->sonarTrigger();
	gpioDelay(10);
	robot.sonar2->sonarTrigger();
	gpioDelay(10);
	robot.sonar3->sonarTrigger();
	gpioDelay(10);
	robot.sonar4->sonarTrigger();
}


// Main entry point
int main(int argc, char* argv[]){
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
	Logger::debug("\n");
	
	
	try{ 
		
		robot.initialize(); 
		
		Logger::debug("initializing tcp...\n");
		start_tcp(8000); 
		
		Logger::debug("ready!\n");
		
	} catch (std::exception& e) {
		std::cerr << "Exception: " << e.what() << endl;
		exit(1);
	}
		
	
	
	while(true);
	
	return 0;
   
}


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



