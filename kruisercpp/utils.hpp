#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <chrono>
#include <thread>

using namespace std;

namespace kruiser {
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