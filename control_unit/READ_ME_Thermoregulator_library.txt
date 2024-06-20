Thermoregulator Library
=========================

This library provides functions to control a temperature regulator using a temperature sensor MAX6675 and a relay. It works taking values of the temperature veery 2 seconds. It makes a average of the last 10 values to know the state of the cylindrical-bed printer.


-> WARNING! To use it you MUST install MAX6675 library by Adafruit 1.1.2 version <-


Variables needed to be introduce
-----------

You must introduce the SLK, CS and DO pins of the MAX6675

You must introduce the SIGNAL or STATE pin of the relay 

You must introduce this values:
      
       - Activated: it is a boolean variable that defines if it is ON (1) or OFF(0) the thermoregulator

       - temp_cons: it is the set temperature.


Constructor
-----------

thermoregulator write_the_name(uint8_t pin_SLK, uint8_t pin_CS, uint8_t pin_DO, uint8_t pin);

   It defines the pin where the relay and the sensor MAX 6675 are connected in the Arduino.


Functions
-----------

->  void initial_temp();

    Get in the first 10 seconds 10 temperatures values and save it in a matrix.
    Then, it wait 2 seconds.

   
->  void regulator(boolean activated, int temp_cons);

    Enter the set temperature and the state of the thermoregulator (On or OFF).
    
    If the state is ON, the relay will be ON if the instant temperature is lower that 
    the set temperature. Else, it will be OFF.
    
    Otherwise, Relay will be OFF.


->  int messages_to_UR10(boolean activated, int temp_cons);
    
    It will sent a number between 0 to 4 to inform about the state of the process.
    
    - 0: The thermoregulator is ON and it is heating. The average temperature is lower than the lower limit.

    - 1: The thermoregulator is ON and it is heated. It means the average temperature is between the limits.

    - 2: The thermoregulator is ON and it is overheated. The instant temperature is higher than the limits and the average calc is higher that the superior limit. The process could be working in non desirable conditions. Must be warning and recommended to check. 

    - 3: The thermoregulator is OFF and it is cooling. The average temperature is higher than 30 grades. 

    - 4: The thermoregulator is OFF and it is cooling. The average temperature is lower than 30 grades. 


->  void state();

    To print values to know the state. It will print: the instantaneous temperature, the limits of the interval, the average temperature and the message.



