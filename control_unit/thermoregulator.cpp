#include <Arduino.h>
#include "max6675.h" // Thermopar library's
#include "thermoregulator.h"

thermoregulator::thermoregulator(uint8_t pin_SLK, uint8_t pin_CS, uint8_t pin_DO, uint8_t pin)
    : thermocouple(pin_SLK, pin_CS, pin_DO) {
    this->pin = pin;
}



void thermoregulator::initial_temp(){
  int i = 0;
  delay(500); //Wait Max chip to stabilize
  Serial.println("Tomando medidas iniciales del entorno");
  Serial.println();
  //Serial.print("Temperatura del vector inicial");
  for(i; i <= LAST_DATA; i++){
    temp[i] = thermocouple.readCelsius();
    //Serial.print(temp[i]);
    //Serial.print(" ");
    delay(2000);
  }
  Serial.println("Toma de medidas realizada");
  Serial.println("inicio de programa");
  Serial.println();
  delay(2000);
}


void thermoregulator::add_temp_cons(){
    LIM_SUP = temp_cons + 3;
    LIM_INF = temp_cons - 3;
}



void thermoregulator::regulator(boolean activated){ 
   this->activated = activated;
   average_calc();
   add_temp_cons();
   if(activated == true) {
       if(temp[LAST_DATA] <= temp_cons) {
           digitalWrite(pin, HIGH);
       } 
       else {
           digitalWrite(pin, LOW);  
       }
   }

  else{
      digitalWrite(pin, LOW); }
  }


void thermoregulator::average_calc() { 

   int i = 0; 
   int plus = 0;

   for(i; i < LAST_DATA; i++) {
     temp[i] = temp[i+1];
     plus = temp[i] + plus;
     // Serial.println(temp[i]);
   }

   temp[LAST_DATA] = thermocouple.readCelsius();
   plus = temp[LAST_DATA] + plus;
 
   average = plus/10.0;
   
}

void thermoregulator::messages_to_UR10() { // Mirar tema de temperaturas inferiores y superiores
  
   if(activated) {
       if(average < LIM_INF) message = 0; //Heating
       if(average > LIM_INF && average < LIM_SUP) message = 1; //Heated
       if(temp[LAST_DATA] > (LIM_SUP + 10) && average > LIM_SUP) message = 2; //Overheated
   }

   else{
       if(average > INITIAL && average < LIM_SUP) message = 3; //Cooling
       if(average < INITIAL) message = 4; //Cooled
    };
}


void thermoregulator::state() {
  
  messages_to_UR10();
  Serial.println();
  Serial.print("Estado -> "); Serial.println(activated);
  Serial.print("Temperatura -> "); Serial.println(temp[LAST_DATA]);
  Serial.print("Temperatura consigna -> "); Serial.println(temp_cons);
  //Serial.print("Límite superior -> "); Serial.println(LIM_SUP);
  //Serial.print("Límite inferior -> "); Serial.println(LIM_INF);
  Serial.print("Temperatura media -> "); Serial.println(average);
  Serial.print("Mensaje -> "); Serial.println(message);
  Serial.println();
  delay(2000);
}
