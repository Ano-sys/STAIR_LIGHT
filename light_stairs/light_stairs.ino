/*
Created by: Timo Niemann
Date: 18.05.2024

What is this program doing:
This programm lets you setup an array of lights, expandable at your descretion, DEFAULT would be 5V LEDs.
The lights are turned on by setting pin 9 or 11 HIGH, the pins determine the direction of the flow.
You can configure the Microcontroller by Bluetooth with simple isntructions.

Usage:
Please refer the pinouts by the defines i've taken, I will provide a schematic.

Flash the software to a Board with an RP2040, the ESP32 should work too (not tested), why especially the RP2040,
because I threaded and the RP2040 is a DualCore ARM Processor.
*/

//----------INCLUDE----------
#include <stdlib.h>
#include <SoftwareSerial.h>
#include <pico/multicore.h>
#include <pico/sync.h>

//----------DEFINES----------
#define DEFAULT_ONTIME_DELAY 20000
#define DEFAULT_STAIR_LIGHT_DELAY 100 // 35
#define DEFAULT_BRIGHTNESS 100

// -> LIGHTS
#define l01 0
#define l02 1
#define l03 2
#define l04 3
#define l05 4
#define l06 5
#define l07 6
#define l08 14
#define l09 15
#define l10 26
#define l11 27
#define l12 28
#define l13 29

// -> Sensorpins (up-/downwards)
#define s_up 13
#define s_dwn 11

// -> Define new Serial pins for HC-05 Bluetooth Module
#define _RX 9
#define _TX 10

// END DEFINE

// this enum contains all instructions which are able to be received via bluetooth
enum INSTRUCTION{
  DIM = 0,
  STEP_D = 1,
  ONTIME_D = 2,
  PAUSE = 3,
  RESUME = 4,
  RUNNING = 5,
  RESET = 6,
  HELP = 7,
};

// simple struct to store one bluetooth package
typedef struct _bt_packet{
  INSTRUCTION instr;  // holds current instruction
  int value;          // holds value if needed
}BT_PACKET;

// *lights is the array of the LED Pins 
int *lights;
int lights_array_length;

// Variables set via Bluetooth
int ONTIME_DELAY = 25000;
int STAIR_LIGHT_DELAY = 100;
int BRIGHTNESS = 100;
int BRIGHTNESS_STEP_D = 2;

// to set new rx and tx pins for bluetooth
SoftwareSerial SSerial(_RX, _TX);

// threading specific defines
volatile bool _SENSOR_IN_USE = false;
critical_section_t cs;

/**
* turns the lights on one by one in the given direction
*/
void lights_on(int *lights, int direction) {
  // TODO raise the lights to the brightness level
  switch (direction) {
    case -1:
      for (int i = 0; i < lights_array_length; i++) {
        // set pins "HIGH" based on set BRIGHTNESS Level
        for(int j = 0; j < BRIGHTNESS; j++){
          analogWrite(lights[i], HIGH * j);
          delay(BRIGHTNESS_STEP_D);
        }
        delay(STAIR_LIGHT_DELAY);
        // delay between each stairwaystep to light up
      }
      break;
    case 1:
      for (int i = lights_array_length - 1; i >= 0; i--) {
        for(int j = 0; j < BRIGHTNESS; j++){
          analogWrite(lights[i], HIGH * j);
          delay(BRIGHTNESS_STEP_D);
        }
        delay(STAIR_LIGHT_DELAY);
      }
      break;
  }
  // while one of the sensors is HIGH -> wait before turing lights of after delay 
  while(true){
    // enter critical section for read of _SENSOR_IN_USE
    critical_section_enter_blocking(&cs);
    if(!_SENSOR_IN_USE){
      critical_section_exit(&cs);
      break;
    }
    critical_section_exit(&cs);
  }
  
  // Delays the lights after user is walking past sensor so he can walk with lights until at destination
  // function delays seconds not on point
  unsigned long starter = millis();
  int counter = ONTIME_DELAY;
  while(counter > 0){
    unsigned long current = millis();
    if(current - starter >= 1){
      starter = current;
      counter--;
    }

    critical_section_enter_blocking(&cs);
    bool sensorTriggered = _SENSOR_IN_USE;
    critical_section_exit(&cs);

    if(sensorTriggered){
      counter = ONTIME_DELAY;
    }
  }

  // turns of the lights in the opposite direction
  kill_lights(lights, -direction);
}

/**
* Turn off the lights in the given direction
*/
void kill_lights(int *lights, int direction) {
  //while(digitalRead(s_up) == HIGH || digitalRead(s_dwn) == HIGH){}
  switch (direction) {
    case -1:
      for (int i = lights_array_length - 1; i >= 0; i--) {
        for(int j = BRIGHTNESS; j >= 0; j--){
          analogWrite(lights[i], HIGH * j);
          delay(BRIGHTNESS_STEP_D);
        }
        digitalWrite(lights[i], LOW);
        delay(STAIR_LIGHT_DELAY);
      }
      break;
    case 1:
      for (int i = 0; i < lights_array_length; i++) {
        for(int j = BRIGHTNESS; j >= 0; j--){
          analogWrite(lights[i], HIGH * j);
          delay(BRIGHTNESS_STEP_D);
        }
        digitalWrite(lights[i], LOW);
        delay(STAIR_LIGHT_DELAY);
      }
      break;
  }
}

/**
* sets the brightness level of the leds
*/
int dim_lights(BT_PACKET packet){
  if(packet.value < 0 || packet.value > 100){
    return -1;
  }
  BRIGHTNESS = packet.value;
  return 0;
}

/**
* little caller function which is called infront of lights_on to determine the right direction
*/
void set_lights(int *lights) {
  if (digitalRead(s_up) == HIGH) {
    SSerial.println("Walking up!");
    lights_on(lights, -1);
  } else if (digitalRead(s_dwn) == HIGH) {
    SSerial.println("Walking down!");
    lights_on(lights, 1);
  }
}

/**
* Creates the light list lights
*/
int *create_light_list() {
  // if you have more LEDs they need to be listed here also
  int lights[] = { l01, l02, l03, l04, l05, l06, l07, l08, l09, l10, l11, l12, l13 };
  // doesn't need to be freed, because on one side the program runs indefinitely and on the other the device is restarted
  int *ret = (int *)malloc(sizeof(lights));
  int i;
  for (i = 0; i < (int)(sizeof(lights) / sizeof(lights[0])); i++) {
    ret[i] = lights[i];
  }
  lights_array_length = i;
  return ret;
}

/**
* checks the sent isntruction via bluetooth, splits it up in INSTRUCTION and Value
* handles the gotten instruction
*/
void check_bt(){
  BT_PACKET packet = {RUNNING, 0};
  long start_pause = 0;
  do{
    if(SSerial.available()){
      // Get instructions
      String userinput = SSerial.readString();                    // Read from bluetooth
      int delimiter_index = userinput.indexOf(' ');               // get the space between isntruction and value
      String instr = userinput.substring(0, delimiter_index);     // read until isntruction is over or end of string if delimiter_index < 0
      instr.toLowerCase();                                        // convert to lower to ignore Lower- or Uppercase missspellings
      delimiter_index++;                                          // do not start on whitespace for value, example 'dim 50' del_in would have been 3 which is the ' ' 

      if(instr.equals("dim")){
        packet.instr = DIM;
        SSerial.println("-ACK DIM-");
      }
      else if(instr.equals("step")){
        SSerial.println("-ACK STEP-");
        if(delimiter_index <= 0){
          SSerial.println("To delay the step intervall, please use the command as following: 'STEP 200' -> this delays the light to 200 Milliseconds");
        }
        packet.instr = STEP_D;
      }
      else if(instr.equals("ontime")){
        SSerial.println("-ACK ONTIME-");
        if(delimiter_index <= 0){
          SSerial.println("To delay the light intervall, please use the command as following: 'ONTIME 20' -> this delays the light to 20 Seconds");
        }
        packet.instr = ONTIME_D;
      } 
      else if(instr.equals("pause")){
        SSerial.println("-ACK PAUSE-");
        packet.instr = PAUSE;
        start_pause = millis();
      }
      else if(instr.equals("resume")){
        SSerial.println("-ACK RESUME-");
        packet.instr = RESUME;
      }
      else if(instr.equals("reset")){
        SSerial.println("-ACK RESET-");
        STAIR_LIGHT_DELAY = DEFAULT_STAIR_LIGHT_DELAY;
        ONTIME_DELAY = DEFAULT_ONTIME_DELAY;
        packet.instr = RESUME;
        packet.value = 0;
      }
      else if(instr.equals("help")){
        SSerial.println("-ACK HELP-");
        SSerial.println("--------------------------");
        SSerial.println("Commands:");
        SSerial.println("\t-> DIM 50 - Sets Brightness to 50%");
        SSerial.println("\t-> STEP 35 - Sets the Stair Light Delay to 35ms");
        SSerial.println("\t-> ONTIME 20 - Sets the Delay for the lights to turn off after to 20s");
        SSerial.println("\t-> PAUSE [1] - Stops the lights from turning on, [OPTINAL] with delay in Minutes");
        SSerial.println("\t-> RESUME - breaks the pause");
        SSerial.println("\t-> RESET - Resets all values to DEFAULT, same as restarting the Device");
        SSerial.println("\t-> HELP - Prints this text");
        SSerial.println("--------------------------");
      }
      else{
        SSerial.println("--------------------False Instruction!--------------------");
        SSerial.println("Try 'HELP'");
      }

      // get value from string and pass to packet.value
      if(delimiter_index > 0){
        packet.value = (int)atol(userinput.substring(delimiter_index).c_str());
        String ret_val = "Value: " + packet.value;
        SSerial.println(ret_val.c_str());
      }

      // Handle Instructions
      if(packet.instr == DIM){
        if(dim_lights(packet) == -1){
          SSerial.println("---ERROR-did not set value---");
        }
      }
      else if(packet.instr == STEP_D){
        if(packet.value >= 0 && packet.value <= 5000){
          STAIR_LIGHT_DELAY = packet.value; 
          String ret_val = "Setting Delay to Value: " + packet.value;
          SSerial.println(ret_val.c_str());
        }
      }
      else if(packet.instr == ONTIME_D){
        if(packet.value >= 0 && packet.value <= 120){
          ONTIME_DELAY = packet.value * 1000;
          String ret_val = "Setting Delay to Value: " + packet.value;
          SSerial.println(ret_val.c_str());
        }
      }
      else if(packet.instr == PAUSE && packet.value > 0){
        // loop to get RESUME instruction or delay until the pause timer is finished
        while (millis() - start_pause < (long unsigned int)packet.value * 1000 && packet.instr == PAUSE) {
          if (SSerial.available()) {
            String userinput = SSerial.readString();
            if (userinput.equalsIgnoreCase("resume")) {
              packet.instr = RESUME;
              SSerial.println("-ACK RESUME-");
              break;
            }
          }
          // the user is likely not faster
          delay(100);
        }
        if (packet.instr == PAUSE) { // Wenn immer noch pausiert, dann setzen auf RUNNING
        packet.instr = RUNNING;
        }
      }
    }
  }
  while(packet.instr == PAUSE);

  // not neccessary but fancier
  packet.instr = RUNNING;
}

/**
* this function runs on Core1
* determines if sensor is in use, free from current programm state -> used to hold the lights HIGH until nobody is in front of one sensor (does not interrupt)
*/
void observe_sensors_thread(){
  while(true){
    if(digitalRead(s_up) == HIGH || digitalRead(s_dwn) == HIGH){
      critical_section_enter_blocking(&cs);
      _SENSOR_IN_USE = true;
      critical_section_exit(&cs);
    }
    else{
      critical_section_enter_blocking(&cs);
      _SENSOR_IN_USE = false;
      critical_section_exit(&cs);
    }
  }
}

/**
* Initializer Function so that the setup function is clean
*/
void _init() {
  SSerial.begin(9600);
  pinMode(l01, OUTPUT);
  pinMode(l02, OUTPUT);
  pinMode(l03, OUTPUT);
  pinMode(l04, OUTPUT);
  pinMode(l05, OUTPUT);
  pinMode(l06, OUTPUT);
  pinMode(l07, OUTPUT);
  pinMode(l08, OUTPUT);
  pinMode(l09, OUTPUT);
  pinMode(l10, OUTPUT);
  pinMode(l11, OUTPUT);
  pinMode(l12, OUTPUT);
  pinMode(l13, OUTPUT);

  pinMode(s_up, INPUT);
  pinMode(s_dwn, INPUT);

  lights = create_light_list();

  critical_section_init(&cs);
}

void setup() {
  _init();
  // gives Core1 the Task to observe the sensors
  multicore_launch_core1(observe_sensors_thread);
}

void loop() {
  check_bt();
  set_lights(lights);
}
