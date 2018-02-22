#ifndef JSON_PAYLOAD_
#define JSON_PAYLOAD_

#include <iostream>
#include <json.hpp>

/*****************************************************
 * Manages conversion from char * to json dictionary
 *****************************************************/
 
using namespace std;
using json = nlohmann::json;

class JsonPayload {
  
public:  
    json js;
    
    JsonPayload(const string& data) {
        try {
            js = json::parse(data);
        } catch (exception& e) {
            cout << e.what() << endl;
            
            
        }
    }
    
    
    int getIntOr(const string& key, int value) {
        try {
            return js[key];
        } catch (std::exception& e) {
            return value;
        }
    }
    
    string getStringOr(const string& key, string value) {
        try {
            return js["key"];
        } catch (std::exception& e) {
            return value;
        }
    }
    
    float getFloatOr(const string& key, float value) {
        try {
            return js[key];
        } catch (std::exception& e) {
            return value;
        }
    }
    
    
    


    
    
       
};


#endif