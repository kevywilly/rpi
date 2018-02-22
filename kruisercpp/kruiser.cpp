#include "pigpio.h"
#include "camera_mount.hpp"
#include "drivetrain.hpp"
#include "utils.hpp"
#include <cstdlib>
#include <iostream>
#include "TCPServer.h"
#include <json.hpp>
#include "JsonPayload.hpp"


// for convenience
using json = nlohmann::json;

using namespace std;
using namespace kruiser;

#define SERVO_MIN_PULSE 600
#define SERVO_MAX_PULSE 2300
#define CAMERA_PITCH_PIN 27
#define CAMERA_YAW_PIN 17
#define CAMERA_DEFAULT_PITCH 90
#define CAMERA_DEFAULT_YAW 0

TCPServer tcp;

// Camera Mount
CameraMount * camera_mount;
Drivetrain * motors;

void dispatch(const string& data) {
	auto j = json::parse(data);
	string cmd = j["cmd"];
	cout << j["cmd"] << endl;
	
	if(cmd == "setmode") {
		int mode = j["mode"];
	} else if(cmd == "drive") {
		float speed = j["speed"];
		float turn = j["turn"];
		motors->drive(speed*100,turn*100);
	} else if(cmd == "look") {
		float yaw = j["yaw"];
		float pitch = j["pitch"];
		camera_mount->move_to(pitch+90, yaw);
	}
	
}

void * loop(void * m)
{
    pthread_detach(pthread_self());
	while(1)
	{
		srand(time(NULL));
		char ch = 'a' + rand() % 26;
		string s(1,ch);
		string str = tcp.getMessage();
		if( str != "" )
		{
			cout << "Message:" << str << endl;
			tcp.Send(" [client message: "+str+"] "+s);
			tcp.clean();
			dispatch(str);
		}
		usleep(1000);
	}
	tcp.detach();
}



// Cleanup
void cleanup() {
	gpioTerminate();
}

// Main entry point
int main(int argc, char* argv[])
{
	
//	try {
			
			if (gpioInitialise() < 0) {
			   cout << "pigpio initialization failed" << endl;
			   return 0;
			}
		
			camera_mount = new CameraMount(CAMERA_PITCH_PIN, CAMERA_YAW_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
			motors = new Drivetrain(14,15,18,5,6,13);

			camera_mount->move_to(CAMERA_DEFAULT_PITCH,CAMERA_DEFAULT_YAW);
		
			//motors->drive(-50,50);
			//delay(1000);
		//	motors->brake();
		
			pthread_t msg;
			tcp.setup(8000);
			
			if( pthread_create(&msg, NULL, loop, (void *)0) == 0)
			{
				tcp.receive();
			}
		
//	  	}	
//	catch (std::exception& e) {
//	    	std::cerr << "Exception: " << e.what() << "\n";
//	}

	while(true);
	
	return 0;
   
}