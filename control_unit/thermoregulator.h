#ifndef thermoregulator_h
#define thermoregulator_h

#include <Arduino.h>
#include "max6675.h" // Thermopar library's

#define MAX_DATA 10
#define LAST_DATA MAX_DATA-1
#define INITIAL 30

class thermoregulator {
  public:
     thermoregulator(uint8_t pin_SLK, uint8_t pin_CS, uint8_t pin_DO, uint8_t pin); //Constructor
     void initial_temp(); //Set the 10 initial temperatures
     void regulator(boolean activated);
     void state();
     int temp_cons;

     
      
  private:

     MAX6675 thermocouple;
     uint8_t pin;
     boolean activated;
     float temp[MAX_DATA];
     float average;
     int LIM_SUP;
     int LIM_INF;
     int message;
     void add_temp_cons();
     void average_calc();
     void messages_to_UR10();
};


#endif
