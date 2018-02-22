#ifndef JSON_PAYLOAD_
#define JSON_PAYLOAD_

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

/*****************************************************
 * Manages conversion from char * to json dictionary
 *****************************************************/
 
using namespace std;
using boost::property_tree::ptree;

class JsonPayload {
  
public:  
    JsonPayload(const string& data) {
        try {
            istringstream is(data);
            read_json(is, json_);
        } catch (exception& e) {
            cout << e.what() << endl;
            json_ = empty_json();
            
        }
    }
    
   
    
    int getIntOr(const string& key, int value) {
        try {
            return json_.get<int>(key);
        } catch (std::exception& e) {
            return value;
        }
    }
    
    string getStringOr(const string& key, string value) {
        try {
            return json_.get<std::string>(key);
        } catch (std::exception& e) {
            return value;
        }
    }
    
    float getFloatOr(const string& key, float value) {
        try {
            return json_.get<float>(key);
        } catch (std::exception& e) {
            return value;
        }
    }
    
    
    
private:
    ptree json_;
    
    ptree empty_json() {
        ptree ej;
        std::string empty = "{}";
        istringstream is(empty);
        read_json(is, ej);
        return ej;
    }
       
};




#endif /* JSON_PAYLOAD_ */