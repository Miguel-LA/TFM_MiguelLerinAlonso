
/* Created: 22/02/2024
 * Version: 2
 * Modify: 08/04/2024
*/ 

#include "max6675.h" // Thermopar library's
#include "pinout_MAX6675.h"
#include "thermoregulator.h"

#define TEMP_CONS_INF 50
#define TEMP_CONS_SUP 90

//Define the message from the UR10
const byte BUFFER_SIZE = 4; // Size of UR's themperatue. From 50 to 100
char buffer[BUFFER_SIZE];    // Buffer para almacenar la cadena
byte index = 0;
volatile boolean activated = false; //Relay's state

thermoregulator thermoregulator(MAX6675_SLK, MAX6675_CS, MAX6675_DO,RELAY);

void setup() {
  
  int i = 0;
  pinMode(RELAY, OUTPUT); //Relay's pin
  Serial.begin(9600); //Data channel's
  thermoregulator.initial_temp();
  
}

void loop() {
  Serial.println();
  thermoregulator.regulator(activated);
  thermoregulator.state();
}


//Comunicación UR10 - Arduino. Código difícil de implementar en al librería debido a las interacciones que
// debe tener con el puerto serie.

void serialEvent() {
    while (Serial.available()) {
    char incomingByte = Serial.read(); // Read bytes
    
    //Read until it reach the end character.
    if (incomingByte != '\r' && incomingByte != '\n') {
      buffer[index] = incomingByte; // Almacena el byte en el buffer
      index++;                      // Incrementa el índice
      // Verifica si se alcanzó el final del buffer
      if (index >= BUFFER_SIZE) {
        // Si el buffer está lleno, detén la recepción
        index = BUFFER_SIZE - 1; // Establece el índice al último elemento del buffer
        break;
      }
      
    } else {
      // Si se recibió el carácter de terminación, procesa la cadena
      buffer[index] = '\0'; // Agrega un terminador nulo al final del buffer para formar una cadena válida
      // Procesa la cadena recibida
      processString(buffer, activated, thermoregulator.temp_cons);
      // Reinicia el índice para la próxima recepción
      
      index = 0;
    }
  }
}

void processString(char* str, volatile boolean &activated, int &temp_cons) {
  
  if (strcmp(str, "OFF") == 0 && activated == true) {
    activated = false;
    Serial.println("Termostato apagado");
  } 
  
  else{

    if(activated == false) {
       thermoregulator.temp_cons = atoi(str); // Convierte la cadena a un entero

       if (thermoregulator.temp_cons >= TEMP_CONS_INF && thermoregulator.temp_cons <= TEMP_CONS_SUP) {
            Serial.println("El número está en el rango de 50 a 90");
            Serial.println("Termostato encendido");
            activated = true;
         } 
         
       else {
            Serial.println("ERROR: Temperatura no válida");
            activated = false;
         }
       }

    else 
      {
      // Serial.println("Termostato endendido. No se admiten temperaturas consigna nuevas.");
      }
   }
}
