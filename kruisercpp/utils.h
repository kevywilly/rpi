#ifndef UTILS_H
#define UTILS_H

#include <chrono>
#include <thread>
#include <iostream>

using namespace std;

namespace kruiser {
  
    class Logger {
       public:
          static inline void debug(const string& s) {
          	cout << s;
          }
          static inline void debugln(const string &s) {
          	debug(s);
          	cout << endl;
          }
    }; 
    // # Declarations
    void delay(int millis);
    void map();
    
    // Delay Microseconds
    void delay(int millis) {
    	this_thread::sleep_for(chrono::milliseconds(millis));
    }
    
    // Map value from one range to another
    long map(long x, long in_min, long in_max, long out_min, long out_max)
    {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    
    

}
#endif