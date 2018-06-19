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
#include "robot.h"
#include <ctime>
//#include "opencv2/opencv.hpp"

#define FULL_DATA 0

// Command

#define CMD_AUTO 0
#define CMD_TRAIN 1
#define CMD_DRIVE 2
#define CMD_CAMERA 3
#define CMD_STATUS 4

// FOR CONVENIENCE
//using json = nlohmann::json;
using namespace std;
using namespace kruiser;

// FORWARD DECLARATIONS
void sonarCallback(int gpio, int level, uint32_t tick);
void sonarTrigger();
void * loop(void * m);
void cleanup();
void start_tcp(int port);
string dispatch(const string& data);
void trainingCallback(int count, double error);

bool isAuto = false;
bool isTraining = false;

// TCP Server
TCPServer tcp;

// Robot
Robot robot(sonarCallback, sonarTrigger, trainingCallback);


// Dispatch TCP Message
void dispatch(const char * data) {
	
	double dirDrive, gotSpeed, dirTurn, gotTurn;
	double dirPitch, gotPitch, dirYaw, gotYaw;

	int cmd = static_cast<unsigned>(data[0]);
	
	switch(cmd) {
		case CMD_AUTO: //auto
			robot.IsAutonomous = (static_cast<unsigned>(data[1]) == 1 ? true : false);
			robot.drive(0,0);
			break;
		case CMD_TRAIN:
			robot.IsTraining = (static_cast<unsigned>(data[1]) == 1 ? true : false);
			break;
		case CMD_DRIVE:
			robot.IsAutonomous = false;
			
			dirDrive = static_cast<unsigned>(data[1]) == 0 ? 1.0 : -1.0;
			gotSpeed = dirDrive * (static_cast<unsigned>(data[2])*1.0)/100.0;
			dirTurn = static_cast<unsigned>(data[3]) == 0 ? 1.0 : -1.0;
			gotTurn = dirTurn * (static_cast<unsigned>(data[4])*1.0)/100.0;
			
			robot.drive(gotSpeed,gotTurn);
			
			robot.train();
			
			break;
		case CMD_CAMERA:
			dirPitch = static_cast<unsigned>(data[1]) == 0 ? 1.0 : -1.0;
			gotPitch = dirPitch * (static_cast<unsigned>(data[2])*1.0);
			dirYaw = static_cast<unsigned>(data[3]) == 0 ? 1.0 : -1.0;
			gotYaw = dirYaw * (static_cast<unsigned>(data[4])*1.0);
			
			robot.moveCamera(gotPitch,gotYaw);
			if(isTraining && (gotSpeed > 0 || gotSpeed < 0)) {
				robot.train();
			}
			break;
		case CMD_STATUS:
		
			break;
			
		//char resp[] = {data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9]};
		//return data;
	}
		
			
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
    clock_t begin = clock();
    double elapsed_time;
    
    
	while(1)
	{
		srand(time(NULL));
		//char ch = 'a' + rand() % 26;
		//string s(1,ch);
		char * str = tcp.getMessage();
		
		// Process message if any
		if( str != NULL )
		{
			cout << "Message:";
			for(int i=0; i < MAXPACKETSIZE; i++) {
				cout << static_cast<unsigned>(str[i]) << ",";
			}
			cout << endl;
			
			dispatch(str);
			//tcp.Send(result+s);
			//tcp.Send(result+"\n");
			tcp.Send(str);
			tcp.clean();
			begin = clock();
		} else {
			robot.runAutonomously();
			
			if(!robot.IsAutonomous) {
				elapsed_time = (double)(clock()-begin) / CLOCKS_PER_SEC;
				if((elapsed_time >= 2) && (robot.getSpeed() > 0.0 || robot.getSpeed() < 0.0)) {
						robot.setPrevSpeed(robot.getSpeed());
						robot.setPrevTurn(robot.getTurn());
						robot.readAdcs();
						robot.readSonars();
						robot.train();
						elapsed_time = 0;
				}
			}
			
			/*
			if(!robot.IsAutonomous) {
				if((double(begin - clock()) / CLOCKS_PER_SEC) > 3) {
					if(true) {
						robot.readAdcs();
						robot.readSonars();
						robot.setPrevSpeed(robot.getSpeed());
						robot.setPrevTurn(robot.getTurn());
						robot.train();
					}
				}
			}
			*/
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

void trainingCallback(int count, double error) {
	cout << count << ": " << error << endl;
}
// Main entry point
int main(int argc, char* argv[]){
	/*
	int r = capture();
	cout << capture << endl;
	return 0;
	*/
	srand(time);
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



