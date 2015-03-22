//-------------------------------------------- Include Files --------------------------------------------//
#include <Streaming.h>

//-------------------------------------------- Global variables --------------------------------------------//
volatile unsigned long timeold;
volatile int engine_rpm;            // current engine rpm
float map_value_us;                 // map value in microseconds
int rev_limit = 4550;               // rev limitter
float map_value = 0.0;              // initialise map_value to be 0 degrees advance
boolean output = false;
boolean fixed = false;              // declare whether to use fixed advance values
int fixed_advance = 15;             // fixed advance value
boolean multispark = false;         // declare whether to use multispark
int interrupt_X = 2;                // declare interrupt pin
int SAW_pin = 13;                   // declare Spark advance word output 

// manifold pressure averaging array
const int numReadings = 20;         // size of pressure averaging array
int readings[numReadings];          // Array of pressure readings
int current_index = 0;              // the index of the current pressure reading
unsigned long total = 0;            // the running pressure total
unsigned int manifold_pressure = 0;        // Initial average manifold pressure
int map_pressure_kpa;

//rpm averaging array
const int num_rpm_readings = 10;               //size of rpm averaging array
volatile int rpm_readings[num_rpm_readings];   // array of rpm readings
volatile int current_index_rpm = 0;            // the index of the current rpm reading
volatile unsigned long rpm_total = 0;          // the running rpm total
volatile unsigned int engine_rpm_average = 0;  // Initial average engine rpm

//Serial input initailisation
String inputString = "";            // a string to hold incoming data
boolean stringComplete = false;     // whether the string is complete

// Global array for the ignition map, with values as degrees of advance
// RPM values seperated into columns, Manifold Pressure (kpa) seperated into rows.

const int pressure_axis[17] = { 
  20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};  // 17
const int rpm_axis[17]  = { 
  0, 500, 600, 900, 1200, 1500, 1800, 2100, 2400, 2700, 3000, 3300, 3600, 3900, 4200, 4500, 4550};  // 17
const int ignition_map [17] [17] = {
  //RPM 0, 500, 600, 900 etc.
  {
    18,18,18,30,34,36,38,40,43,46,48,50,53,55,58,58,0    }
  ,      //20kpa
  {
    18,18,18,30,34,36,38,40,43,46,48,50,53,55,58,58,0    }
  ,      //25kpa
  {
    18,18,18,30,34,36,38,40,43,46,48,50,53,55,58,58,0    }
  ,      //30kpa
  {
    18,18,18,30,34,36,38,40,43,46,48,50,53,55,58,58,0    }
  ,      //35kpa
  {
    18,18,18,30,34,36,38,40,43,46,48,50,53,55,58,58,0    }
  ,      //40kpa
  {
    18,18,22,30,34,36,38,40,43,45,48,50,53,55,57,58,0    }
  ,      //45kpa
  {
    18,18,21,29,34,35,37,39,43,45,48,50,52,55,57,58,0    }
  ,      //50kpa
  {
    17,17,21,29,33,35,37,39,42,45,47,50,52,54,57,58,0    }
  ,      //55kpa
  {
    17,17,21,29,33,35,37,39,42,45,47,49,52,54,57,58,0    }
  ,      //60kpa
  {
    15,15,19,27,31,33,35,37,40,43,45,47,50,52,55,57,0    }
  ,      //65kpa
  {
    13,13,17,25,29,31,33,35,38,41,43,45,48,50,53,55,0    }
  ,      //70kpa
  {
    11,11,15,23,27,29,31,33,36,39,41,43,46,48,51,53,0    }
  ,      //75kpa
  {
    9,9,13,21,25,27,29,31,34,37,39,41,44,46,49,51,0    }
  ,        //80kpa
  {
    7,7,11,19,23,25,27,29,32,35,37,39,42,44,47,49,0    }
  ,        //85kpa
  {
    6,6,10,18,22,24,26,28,31,34,36,38,41,43,46,48,0    }
  ,        //90kpa
  {
    6,6,10,18,22,24,26,28,31,34,36,38,41,43,46,48,0    }
  ,        //95kpa
  {
    6,6,10,18,22,24,26,28,31,34,36,38,41,43,46,48,0    }
  ,        //100kpa
};


//-------------------------------------------- Initialise Parameters --------------------------------------------//
void setup(){
  timeold = 0;

  attachInterrupt(0, pip_interupt, FALLING);                                // initialise interupt to dectect the falling edge of the PIP signal coming in on digital pin 2 (interupt 0)
  pinMode(SAW_pin, OUTPUT);                                                 // Assign SAW_pin as a digital output
  pinMode(interrupt_X, INPUT);
  digitalWrite (interrupt_X, HIGH);

  for (int thisReading = 0; thisReading < numReadings; thisReading++)       //populate manifold pressure averaging array with zeros
    readings[thisReading] = 0;  

  for (int thisReading = 0; thisReading < num_rpm_readings; thisReading++)   //populate rpm averaging array with zeros
    rpm_readings[thisReading] = 0;

  Serial.begin(38400);                                                       // Initialise serial communication at 38400 Baud  
  inputString.reserve(200);                                                  //reserve 200 bytes for serial input - equates to 25 characters

}


//-------------------------------------------- he loop routine runs over and over again forever: whilst device is powered --------------------------------------------// 
void loop(){

  if (stringComplete){   
    if(inputString == "fixed") {
      Serial.println("Fixed Advance Selected");
      fixed = true;
      inputString = "";
      stringComplete = false;
    }
    else if (inputString == "map") {
      Serial.println("Ignition Advance Selected");
      fixed = false;
      inputString = "";
      stringComplete = false;
    }
    else if (inputString == "ms on") {
      Serial.println("Multispark Enabled");
      multispark = true;
      inputString = "";
      stringComplete = false;
    }
    else if (inputString == "ms off") {
      Serial.println("Multispark Disabled");
      multispark = false;
      inputString = "";
      stringComplete = false;
    }
    else if (inputString == "output on"){
      output = true; 
      inputString = "";
      stringComplete = false;
    }
    else if (inputString == "output off"){
      output = false;
      inputString = "";
      stringComplete = false;
    }
  } 

  // Averaging the Manifold Pressure Reading 
  total = total - readings[current_index];        //subtract the previous reading in current array element from the total reading:
  readings[current_index] = 400;//analogRead(A0);       //take a read from the sensor and place in current array element 
  total = total + readings[current_index];        //add the reading to the total
  current_index++;              //advance to the next element in the array      
  if (current_index >= numReadings - 1){              //at end of the array...
    current_index = 0;                            // ...wrap around to the beginning
  }  
  manifold_pressure = total / numReadings;   //calculate the average - change from constant value for testing 
  map_pressure_kpa = map(manifold_pressure,0,1023,0,260);  
  delay(1);                                  //delay in between reads for stability  
} 


//-------------------------------------------- Triggered when serial input detected --------------------------------------------//
void serialEvent() {
  while (Serial.available()) {                    //whilst the serial port is available...
    inputString = Serial.readStringUntil('\n');   //... read in the string until a new line is recieved
    stringComplete = true;                      
  }
}
// ------------------------  Continued below---------------------------//
//-------------------------------------------- PIP signal interupt --------------------------------------------// 
void pip_interupt()  {
  //  detachInterrupt(1);                                       // Used for testing
  if ( digitalRead(interrupt_X) == LOW ){                      // continue only if interrupt pin is LOW - Eliminates false rising edge triggering.
    generate_SAW(map_value);                                // Generate SAW using previous average engine rpm and pressure - move to bottom of this function to use current rpm and pressure

      engine_rpm = 30000000/(micros() - timeold);             //take the time of a reading and subtract the time of previous reading to determine rpm. Note: 30000ms as two PIP per revolution
    timeold = micros();
    rpm_total= rpm_total - rpm_readings[current_index_rpm]; //subtract the previous reading in current array element from the total reading      //set time of current reading as the new time of the previous reading
    rpm_readings[current_index_rpm] = engine_rpm;           //place current rpm value into current array element
    rpm_total= rpm_total + rpm_readings[current_index_rpm]; //add the reading to the total     
    current_index_rpm++;                                    //advance to the next element in the array
    if (current_index_rpm >= (num_rpm_readings - 1))   {    //at end of the array...
      current_index_rpm = 0;                               // ...wrap around to the beginning
    }
    engine_rpm_average = rpm_total / num_rpm_readings;      //calculate the average
    map_value = rpm_pressure_to_spark(engine_rpm_average, map_pressure_kpa);    // Retrieve ignition map value as degrees of advance.
  }                                          
  // attachInterrupt(1, pip_interupt, FALLING);
  
}


//-------------------------------------------- Function to map the engine rpm to value for the array element --------------------------------------------// 
int decode_rpm(int rpm_) {
  int idle_rpm = 500;                 // Idle Speed
  int map_rpm = 0;
  if(rpm_ <idle_rpm){                // if rpm less than idle_rpm, use values from ignition map column [1]
    map_rpm = 1;
  } 
  else if(rpm_ >=rev_limit) {      // if rpm more than the rev limitter, use values from ignition map column [16] - rev limitter
    map_rpm = 16;      
  }
  else{
    while(rpm_ > rpm_axis[map_rpm]){ // otherwise find which array element corresponds the the rpm value  
      map_rpm++;
    }
  }
  
  return map_rpm;
}

//-------------------------------------------- Function to map the engine manifold absolute pressure to value for the array element --------------------------------------------//
int decode_pressure(int pressure_) {

  int map_pressure = 0;
  if(pressure_ < 20){
    map_pressure = 0;
  }
  else if (pressure_ > 100){
    map_pressure = 16;
  }
  else{
    while(pressure_ > pressure_axis[map_pressure]){
      map_pressure++;
    }
  }
  return map_pressure;
}


//-------------------------------------------- Function to determine Ignition table value based on manifold pressure and engine rpm --------------------------------------------//
int rpm_pressure_to_spark(int rpm, int pressure){
  float table_value;
  int map_rpm_index = decode_rpm(rpm);                      // call function decode_rpm
  int map_pressure_index = decode_pressure(pressure);       // call function decode_pressure
  float pressure_index_high;
  float pressure_index_low;

  pressure_index_high = mapfloat(pressure, pressure_axis[map_pressure_index-1], pressure_axis[map_pressure_index], ignition_map[map_pressure_index][map_rpm_index], ignition_map[map_pressure_index-1][map_rpm_index]);
  pressure_index_low  = mapfloat(pressure, pressure_axis[map_pressure_index-1], pressure_axis[map_pressure_index], ignition_map[map_pressure_index][map_rpm_index-1], ignition_map[map_pressure_index-1][map_rpm_index-1]);


  table_value = mapfloat(rpm, rpm_axis[map_rpm_index-1], rpm_axis[map_rpm_index], pressure_index_low, pressure_index_high);



  return table_value;

}


//-------------------------------------------- Function to generate SAW signal to return to EDIS --------------------------------------------//
int generate_SAW(float advance_degrees){
  if (fixed == true){                                    // If serial input has been set to fixed, set map_value_us to the fixed value
    map_value_us = 1536 - (25.6 * fixed_advance);
  }
  else if (multispark && engine_rpm_average <= 1000){    // If engine rpm below 1000 and multispark has been set, add 2048us to map_value_us
    map_value_us = (1536 - (25.6 * advance_degrees))+2048;
  } 
  else{                                                // Otherwise read from map
    map_value_us = 1536 - (25.6 * advance_degrees);
  }

  if (map_value_us < 64){                                // If map_value_us is less than 64 (smallest EDIS will accept), set map_value_us to 64us
    map_value_us = 64;
  }

  int code_delay = 50;
  long ten_ATDC_delay = (((60000000/engine_rpm_average)/36)-code_delay);
  delayMicroseconds(ten_ATDC_delay);
  digitalWrite(SAW_pin,HIGH);                                 // send output to logic level HIGH (5V)
  delayMicroseconds(map_value_us);                            // hold HIGH for duration of map_value_us
  digitalWrite(SAW_pin,LOW);                                  // send output to logic level LOW (0V)
Serial.println(map_value_us);
}


//-------------------------------------------- Function to relicate map() function with floating point values --------------------------------------------//
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return ((x - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min;
}


