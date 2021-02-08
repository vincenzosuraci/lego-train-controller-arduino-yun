//--------------------------------------------------------------------------------------------------
// Title: Lego Train Controller for Arduino YUN
// Author: Vincenzo Suraci
// Created: 25-Gen-2021
//
// Description: Lego Train Controller for:
// > Power Function (IR) Trains
// > TrixBrix Switches, Boom barriers, Position sensors and Signals
//
// Hardware requirements:
// > Arduino YUN ()
// > DFRoboot IO Expansion Shield v7.1 (https://www.dfrobot.com/product-1009.html)
// > Stacking Headers (1x 6pin, 2x 8pin, 1x 10pin) - needed to separate Arduino YUN from the DFRobot board
// > DFRoboot IR Trasmitter Module V2 (x2)
//
// Acknowledgements:
// > Lego Power Function library: https://github.com/jurriaan/Arduino-PowerFunctions
// > TrixBrix connection with Arduino Boards: https://trixbrix.eu/en_US/i/Control-Automation-FAQ/23
//
// Disclaimer:
// > LEGO, the LEGO logo, the PowerFunctions and PoweredUp logos are trademarks of the LEGO Group.
// > TrixBrix, the TrixBrix logo are trademarks of the TrixBrix Company.
//
// License:
// (c) Copyright 2021 - Vincenzo Suraci
// Released under MIT License
//--------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------
// LIBRARIES
//------------------------------------------------------------------------------------------------ 

// Include libraries for OpenWrt-Yun <--> Arduino communications 
// it is based on a TCP client-server socket
#include <Bridge.h>
#include <BridgeServer.h>
#include <BridgeClient.h>

// Include Lego Power Function IR-LED libraries
#include "legopowerfunctions.h"

// Include servo-motor libraries to actuate the switches
#include <Servo.h>

// MQTT Publish-Subscribe Client
#include <PubSubClient.h>

//------------------------------------------------------------------------------------------------
// GLOBAL CONSTRAINTS
//------------------------------------------------------------------------------------------------ 

// PC <--> Yun Serial speed
const int YUN_PC_SERIAL_BAUDRATE = 9600;

// Num of power function IR led
const int NUM_LEGO_PF = 2;

// TRAIN POSITION SENSORS
const unsigned char NUM_POSITION_SENSORS = 2;
const unsigned char POSITION_SENSOR_PINS[NUM_POSITION_SENSORS] = {2, 3};
const unsigned char POSITION_SENSOR_IDS[NUM_POSITION_SENSORS] = {0, 1};

// TRAIN SWITCHES (SERVO)
const unsigned char SERVO_SWITCH_MIN_ANGLE = 68; // original value 58 (https://trixbrix.eu/en_US/i/Control-Automation-FAQ/23)
const unsigned char SERVO_SWITCH_MAX_ANGLE = 85; // original value 100 (https://trixbrix.eu/en_US/i/Control-Automation-FAQ/23)
const unsigned char SERVO_SWITCH_ANGLES[2] = {SERVO_SWITCH_MIN_ANGLE, SERVO_SWITCH_MAX_ANGLE};
const unsigned char NUM_SERVO_SWITCHES = 4;
const unsigned char SERVO_SWITCH_PINS[NUM_SERVO_SWITCHES] = {4, 5, 6, 7};
const unsigned char SERVO_SWITCH_IDS[NUM_SERVO_SWITCHES] = {0, 1, 2, 3};

// TRAIN BARRIER (SERVO)
const unsigned char SERVO_BARRIER_MIN_ANGLE = 30; // original value 30 (https://trixbrix.eu/en_US/i/Control-Automation-FAQ/23)
const unsigned char SERVO_BARRIER_MAX_ANGLE = 128; // original value 128 (https://trixbrix.eu/en_US/i/Control-Automation-FAQ/23)
const unsigned char SERVO_BARRIER_ANGLES[2] = {SERVO_BARRIER_MIN_ANGLE, SERVO_BARRIER_MAX_ANGLE};
const unsigned char NUM_SERVO_BARRIERS = 4;
const unsigned char SERVO_BARRIER_PINS[NUM_SERVO_BARRIERS] = {8, 9, 10, 11};
const unsigned char SERVO_BARRIER_IDS[NUM_SERVO_BARRIERS] = {0, 1, 2, 3};

// ArduinoYUN status Led (blinks when a IR command is executed)
const int STATUS_LED_PIN = 13;

// TRAIN SIGNALS
const unsigned char NUM_SIGNALS = 2;
const unsigned char SIGNAL_PINS[2*NUM_SIGNALS] = {A0, A1, A2, A3};
const unsigned char SIGNAL_IDS[NUM_SIGNALS] = {0, 1};

// LEGO POWER FUNCTION IR LEDS
const unsigned char NUM_PF_IR_LEDS = 2;
const unsigned char PF_IR_LED_PINS[NUM_PF_IR_LEDS] = {A4, A5};
const unsigned char PF_IR_LED_IDS[NUM_PF_IR_LEDS] = {0, 1};

// Forward/Reverse Configuration speeds from 0 to 7
const int fwdSpeed[8] = {PWM_FLT, PWM_FWD1, PWM_FWD2, PWM_FWD3, PWM_FWD4, PWM_FWD5, PWM_FWD6, PWM_FWD7};
const int revSpeed[8] = {PWM_FLT, PWM_REV1, PWM_REV2, PWM_REV3, PWM_REV4, PWM_REV5, PWM_REV6, PWM_REV7};

// MQTT Configuration
const char* MQTT_HOST = "192.168.1.250";
const char* MQTT_USER = "mqtt_client";
const char* MQTT_PASS = "***";
const char* MQTT_CLIENT_ID = "arduino-yun-lego-train-controller";
const int MQTT_PORT = 1883;

// JSON Text
const char* COMMA = ",";
const char* CONFIG = "config";
const char* COLONS = ":";
const char* SEMICOLONS = ";";
const char* ISOK = "isok";
const char* TRUE = "true";
const char* FALSE = "false";
const char* CURLY_BRACE_ON = "{";
const char* CURLY_BRACE_OFF = "}";
const char* BRACKET_ON = "[";
const char* BRACKET_OFF = "]";
const char* _SPEED = "speed";
const char* ES = "es";
const char* S = "s";
const char* PIN = "pin";
const char* ANGLE = "angle";
const char* LIGHT = "light";
const char* VALUE = "value";
const char* CHANNEL = "channel";
const char* COLOR = "color";
const char* QUOTES = "\"";
const char* SLASH = "/";
const char* STATUS = "status";
const char* ID = "id";
const char* WILDCARD = "#";

const char* SWITCH = "switch";
const char* SIGNAL = "signal";
const char* BARRIER = "barrier";
const char* SENSOR = "sensor";
const char* POSITION = "position";
const char* MOTOR = "motor";

const char* LEGO = "lego";
const char* TRAIN = "train";

const char* HASH = "#";

const char* PU = "pu";
const char* PF = "pf";
const char* _12V = "12V";
const char* _9V = "9V";
const char* _4_5V = "4.5V";

// max buffer size
const unsigned char MAX_BUFFER_SIZE = 255;

// 1 second in microseconds
const unsigned long ul_1_second = 500000;

//------------------------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------------------------ 

// LEGO POWER FUNCTION
LEGOPowerFunctions lego_pf[NUM_PF_IR_LEDS] = {
  LEGOPowerFunctions(PF_IR_LED_PINS[0]), 
  LEGOPowerFunctions(PF_IR_LED_PINS[1])
};

// lego power function messages' queue
// 0 > repetitions 
// 1 > channel (0-3)
// 2 > output (0-1)
// 3 > value (0-7) 
unsigned char queue[4] = {};

// YUN server
BridgeServer server;

// An Arduino based HTTP client, modeled after the EthernetClient class.
BridgeClient yun_client;
PubSubClient mqtt_client(yun_client);

// Flag indicating the ArduinoYUN LED status
bool status_led_is_on = false;

// Servo switch
Servo servo_switch[NUM_SERVO_SWITCHES];
Servo servo_barrier[NUM_SERVO_BARRIERS];

// Train sensors (1 when no train is detected, 0 otherwise)
// Setting 2, forces to send their value at start up
unsigned char position_sensor[NUM_POSITION_SENSORS] = {2, 2};

// Status led blinking loop
unsigned long ul_status_led_switch_timeout;

// Useful char buffers to play with chars' concatenation
char buffer1[MAX_BUFFER_SIZE];
char buffer2[MAX_BUFFER_SIZE];
  
//------------------------------------------------------------------------------------------------
// MAIN SETUP
//------------------------------------------------------------------------------------------------ 

void setup() {
  init_status_led();   
  status_led_on();
  init_serial();
  init_yun_bridge();
  init_web_server();
  init_servo_switches();
  init_servo_barriers();
  init_signals();
  init_pf_ir_leds(); 
  init_position_sensors();   
  init_mqtt();
  status_led_off();
  init_status_led_blinking();
}

//------------------------------------------------------------------------------------------------
// MAIN LOOP
//------------------------------------------------------------------------------------------------ 

void loop() {    
  
  //Instance of the YunClient for managing the connection. 
  //If the client connects, process the requests in a custom function (described below) 
  //and close the connection when finished.
  BridgeClient client = server.accept();

  if (client) {
    process_request_by_client(client);
    client.stop();
  }

  // Update train position sensor values
  update_position_sensors();

  // Process power function ir queue
  update_pf_ir_queue();

  // Manage status led blinking
  update_status_led(); 

  // Update MQTT 
  // > to stay always connected
  // > to ensure the subscription to topics
  update_mqtt();
    
  // Putting a delay at the end of loop() 
  // will be helpful in keeping the processor from doing too much work.
  delay(10);
}

//------------------------------------------------------------------------------------------------
// SETUP FUNCTIONS
//------------------------------------------------------------------------------------------------

void init_mqtt(){
  // During MQTT initialization you can
  // 1) configure the server (Ip and port)  
  mqtt_client.setServer(MQTT_HOST, MQTT_PORT); 
  // 2) set the subscribe callback
  mqtt_client.setCallback(mqtt_callback);   
}

void init_signals(){
  for (unsigned char i = 0; i < NUM_SIGNALS; i++){
    for (unsigned char j = 0; j < 2; j++){      
      unsigned char pin = SIGNAL_PINS[(2*i)+j];
      if (pin > 13){
        pinMode(pin, INPUT_PULLUP);  
      }      
      pinMode(pin,OUTPUT);
      if (j == 0){
        set_signal(i,j,true);
      } else {
        set_signal(i,j,false);
      }
    } 
  }      
}

void init_pf_ir_leds() {
  for (unsigned char i = 0; i < NUM_PF_IR_LEDS; i++){    
    unsigned char pin = PF_IR_LED_PINS[i];
    if (pin > 13){
      pinMode(pin, INPUT_PULLUP);  
    }      
    pinMode(pin,OUTPUT);       
  }
}

void init_position_sensors(){
  for (unsigned char i = 0; i < NUM_POSITION_SENSORS; i++){
    for (unsigned char j = 0; j < 2; j++){
      unsigned char pin = POSITION_SENSOR_PINS[(2*i)+j];           
      pinMode(pin,INPUT);
    } 
  }
}

void init_serial(){
  Serial.begin(YUN_PC_SERIAL_BAUDRATE);
}

void init_status_led_blinking(){
  ul_status_led_switch_timeout = micros() + ul_1_second;
}

void init_yun_bridge(){  
  // Attiviamo il bridge
  // Bridge.begin() is blocking, and should take about 2 seconds to complete...
  Bridge.begin();
}

void init_web_server(){
  // tell the instance of YunServer to listen for incoming connections only coming from localhost. 
  // Connections made to OpenWrt-Yun will be passed to the 32U4 processor for parsing and 
  // controlling the pins. This happens on port 5555. Start the server with server.begin().
  server.listenOnLocalhost();
  server.begin();  
}

void init_status_led(){
  pinMode(STATUS_LED_PIN,OUTPUT);    
}

void init_servo_switches(){   
  // Reset servo switches
  for(unsigned char i = 0; i < NUM_SERVO_SWITCHES; i++){  
    servo_switch[i].attach(SERVO_SWITCH_PINS[i]);        
    switch_off(i);
  }
}

void init_servo_barriers(){   
  // Reset servo barriers
  for(unsigned char i = 0; i < NUM_SERVO_BARRIERS; i++){  
    servo_barrier[i].attach(SERVO_BARRIER_PINS[i]);        
    barrier_off(i);
  }
}

//------------------------------------------------------------------------------------------------
// BRIDGE LININO-ARDUINO >>> CLIENT >>> REQUEST
// Accepeted commands:
// 1) http://<ip-address>/arduino/lego/train/config
// 2) http://<ip-address>/arduino/lego/train/switch/<id>/<value>
// 3) http://<ip-address>/arduino/lego/train/barrier/<id>/<value>
// 4) http://<ip-address>/arduino/lego/train/pf/<channel>/<output>/<speed>
// Where:
// <id> is a non negative integer ( >= 0 )
// <value> is a boolean value ( 0 or 1 )
// <channel> is in { 0, 1, 2, 3 } set
// <output> is a boolean value ( 0 >>> Red or 1 >>> Blue )
// <speed> is a non negative integer in { 0, 1, 2, 3, 4, 5, 6, 7 } set
//------------------------------------------------------------------------------------------------

void process_request_by_client(BridgeClient &client) { 
  
  client.print(CURLY_BRACE_ON);

  char request[MAX_BUFFER_SIZE];
  
  // get the request from client
  get_char_from_client(client, request);
          
  client.print(QUOTES);
  client.print(ISOK);
  client.print(QUOTES);
  client.print(COLONS);

  bool send_configuration = false;

  // process the request  
  if ( process_request_by_char(request, send_configuration) ) {
    client.print(TRUE);
    if (send_configuration){
      client.print(COMMA);
      send_json_configuration(client);                
    }      
  } else {
    client.print(FALSE);
  }
      
  client.print(CURLY_BRACE_OFF);  
}

bool process_request_by_char(char *request, bool &send_json_configuration) {

  bool isok = false;
  send_json_configuration = false;
  
  unsigned char index = 0;
  get_char_until_next_slash(request, buffer1, index);

  if ( strcmp(buffer1,LEGO) == 0 ) {

    //----------------------------------------------------------------------------------------------
    // LEGO
    //----------------------------------------------------------------------------------------------

    get_char_until_next_slash(request, buffer1, index);

    if ( strcmp(buffer1,TRAIN) == 0 ) {
  
      //--------------------------------------------------------------------------------------------
      // LEGO - TRAIN
      //--------------------------------------------------------------------------------------------
  
      get_char_until_next_slash(request, buffer1, index);

      if ( strcmp(buffer1,CONFIG) == 0 ) {
    
        //------------------------------------------------------------------------------------------
        // LEGO - TRAIN - CONFIGURATION
        //------------------------------------------------------------------------------------------

        send_json_configuration = true;          
    
        isok = true;
        
        
      } else if ( strcmp(buffer1,MOTOR) == 0 ) {
    
        //------------------------------------------------------------------------------------------
        // LEGO - TRAIN - MOTOR
        //------------------------------------------------------------------------------------------
    
        get_char_until_next_slash(request, buffer1, index);

        if (strcmp(buffer1,PF) == 0) {
    
          //----------------------------------------------------------------------------------------
          // LEGO - TRAIN - MOTOR - POWER FUNCTIONS
          //----------------------------------------------------------------------------------------
          
          // Selezione del canale (da 0 a 3)
          get_char_until_next_slash(request, buffer1, index);
          unsigned char CH = atoi(buffer1); 
          
          // Selezione del colore (0 ==> RED e 1 ==> BLU)
          get_char_until_next_slash(request, buffer1, index);
          unsigned char CO = atoi(buffer1);   
          
          // Selezione del valore (da -7 a 7)
          get_char_until_next_slash(request, buffer1, index);
          int VA = atoi(buffer1);
          if (VA >= 0) {
            VA = fwdSpeed[VA]; 
          } else {
            VA = revSpeed[-VA];
          }   
      
          // Aggiungiamo il comando alla coda
          queue[0] = 5;
          queue[1] = PWM;
          queue[2] = VA;
          queue[3] = CO;
          queue[4] = CH;
      
          isok = true;
        }    
        
      } else if (strcmp(buffer1,SIGNAL) == 0) {  
    
        //------------------------------------------------------------------------------------------
        // LEGO - TRAIN - SIGNAL
        //------------------------------------------------------------------------------------------
    
        // Selezione del semaforo
        get_char_until_next_slash(request, buffer1, index);
        unsigned char Signal = atoi(buffer1);
    
        if (Signal >= 0 && Signal < NUM_SIGNALS){ 
    
          // Selezione della luce
          get_char_until_next_slash(request, buffer1, index);
          unsigned char Light = atoi(buffer1);
    
          if (Light >= 0 && Light < 2){
    
            // Selezione del valore
            get_char_until_next_slash(request, buffer1, index);
            unsigned char Value = atoi(buffer1);
    
            if (Value >= 0 && Value <= 1){
              set_signal(Signal, Light, (Value==1));
            }        
          }              
        }
    
        isok = true;
        
      } else if (strcmp(buffer1,SWITCH) == 0) {
    
        //------------------------------------------------------------------------------------------
        // LEGO - TRAIN - SWITCH
        //------------------------------------------------------------------------------------------
    
        // Selezione della porta digitale
        get_char_until_next_slash(request, buffer1, index);
        unsigned char s = atoi(buffer1);
    
        // Selezione dell'angolo
        get_char_until_next_slash(request, buffer1, index);
        unsigned char a = atoi(buffer1); 
    
        if (s >= 0 && s < NUM_SERVO_SWITCHES && a >= 0 && a < 2){      
          
          unsigned char ang = SERVO_SWITCH_ANGLES[a];
          set_switch(s, ang);
                
        }  
    
        isok = true;
        
      } else if (strcmp(buffer1,BARRIER) == 0) {
    
        //------------------------------------------------------------------------------------------
        // LEGO - TRAIN - BARRIER
        //------------------------------------------------------------------------------------------
    
        // Selezione della porta digitale
        get_char_until_next_slash(request, buffer1, index);
        unsigned char b = atoi(buffer1);
    
        // Selezione dell'angolo
        get_char_until_next_slash(request, buffer1, index);
        unsigned char a = atoi(buffer1); 
    
        if (b >= 0 && b < NUM_SERVO_BARRIERS && a >= 0 && a < 2){            
          unsigned char ang = SERVO_BARRIER_ANGLES[a];
          set_barrier(b, ang);            
        }
    
        isok = true;
        
      }        
    }    
  }

  return isok;
}


//------------------------------------------------------------------------------------------------
// POWER FUNCTION IR-LED QUEUE
//------------------------------------------------------------------------------------------------

void update_pf_ir_queue() {
  if (queue[0] > 0) {
    for (int i = 0; i < NUM_LEGO_PF; i++){
      lego_pf[i].SingleOutput(queue[1], queue[2], queue[3], queue[4]);
    }     
    queue[0]--;    
  }
}

//------------------------------------------------------------------------------------------------
// STATUS LED FUNCTIONS
//------------------------------------------------------------------------------------------------

void update_status_led(){
  if ( micros() > ul_status_led_switch_timeout ) {
    ul_status_led_switch_timeout += ul_1_second;    
    status_led_switch();
  }
}

void status_led_switch(){
  if (status_led_is_on){
    digitalWrite(STATUS_LED_PIN,LOW);
    status_led_is_on = false;
  } else {
    digitalWrite(STATUS_LED_PIN,HIGH);
    status_led_is_on = true;
  }
}

void status_led_on(){
  status_led_is_on = true;
  digitalWrite(STATUS_LED_PIN,HIGH);
}

void status_led_off(){
  status_led_is_on = false;
  digitalWrite(STATUS_LED_PIN,LOW);
}

//------------------------------------------------------------------------------------------------
// TRAIN CONTROLLER FUNCTIONS
// >>> CONFIGURATION <<<
//------------------------------------------------------------------------------------------------

void send_json_configuration(BridgeClient &client) {
  
  //--------------------------------------------------
  // Lego >>> OPEN
  //--------------------------------------------------
  
  client.print(QUOTES);
  client.print(LEGO);        
  client.print(QUOTES);
  client.print(COLONS);
  client.print(CURLY_BRACE_ON);

    //--------------------------------------------------
    // Lego - Train >>> OPEN
    //--------------------------------------------------
    
    client.print(QUOTES);
    client.print(TRAIN);        
    client.print(QUOTES);
    client.print(COLONS);
    client.print(CURLY_BRACE_ON);

      //--------------------------------------------------
      // Lego - Train - Motor >>> OPEN
      //--------------------------------------------------

      client.print(QUOTES);
      client.print(MOTOR);
      client.print(S);        
      client.print(QUOTES);
      client.print(COLONS);
      client.print(CURLY_BRACE_ON);
        
        //--------------------------------------------------
        // Lego - Train - Motor - Power Functions >>> OPEN
        //--------------------------------------------------
        
        client.print(QUOTES);
        client.print(PF);        
        client.print(QUOTES);
        client.print(COLONS);
        
        client.print(BRACKET_ON);
        
        for (unsigned char i = 0; i < NUM_PF_IR_LEDS; i++) {
        
          if (i > 0){
            client.print(COMMA);
          }
        
          client.print(CURLY_BRACE_ON);
        
          // Lego - PF - id
          client.print(QUOTES);
          client.print(ID);    
          client.print(QUOTES);
          client.print(COLONS);
          client.print(PF_IR_LED_IDS[i]);      
          client.print(COMMA);
          
          // Lego - PF - pin
          client.print(QUOTES);
          client.print(PIN);    
          client.print(QUOTES);
          client.print(COLONS);
          client.print(PF_IR_LED_PINS[i]);   
        
          client.print(CURLY_BRACE_OFF);
          
        }
        
        client.print(BRACKET_OFF);
        
        //--------------------------------------------------
        // Lego - Train - Motor - Power Functions <<< CLOSE
        //--------------------------------------------------

      client.print(CURLY_BRACE_OFF);

      //--------------------------------------------------
      // Lego - Train - Motor <<< CLOSE
      //--------------------------------------------------
  
      client.print(COMMA); 
  
      //--------------------------------------------------
      // Lego - Train - Position >>> OPEN
      //--------------------------------------------------
            
      client.print(QUOTES);
      client.print(POSITION);
      client.print(S);                   
      client.print(QUOTES);
      client.print(COLONS);       
      client.print(BRACKET_ON);
      
      for (unsigned char i = 0; i < NUM_POSITION_SENSORS; i++) {
      
        if (i > 0){
          client.print(COMMA);
        }
      
        client.print(CURLY_BRACE_ON);
      
        // TrixBrix - Position sensors - id
        client.print(QUOTES);
        client.print(ID);    
        client.print(QUOTES);
        client.print(COLONS);
        client.print(POSITION_SENSOR_IDS[i]);      
        client.print(COMMA);
        
        // TrixBrix - Position sensors - pin
        client.print(QUOTES);
        client.print(PIN);    
        client.print(QUOTES);
        client.print(COLONS);
        client.print(POSITION_SENSOR_PINS[i]);      
        
        client.print(CURLY_BRACE_OFF);
        
      }
      
      client.print(BRACKET_OFF);
      
      //--------------------------------------------------
      // Lego - Train - Position <<< CLOSE
      //--------------------------------------------------
      
      client.print(COMMA);
      
      //--------------------------------------------------
      // Lego - Train - Switches >>> OPEN
      //--------------------------------------------------
            
      client.print(QUOTES);
      client.print(SWITCH);
      client.print(ES);     
      client.print(QUOTES);
      client.print(COLONS);       
      client.print(BRACKET_ON);
      
      for (unsigned char i = 0; i < NUM_SERVO_SWITCHES; i++) {
      
        if (i > 0){
          client.print(COMMA);
        }
      
        client.print(CURLY_BRACE_ON);
      
        // TrixBrix - Switches - id
        client.print(QUOTES);
        client.print(ID);    
        client.print(QUOTES);
        client.print(COLONS);
        client.print(SERVO_SWITCH_IDS[i]);      
        client.print(COMMA);
        
        // TrixBrix - Switches - pin
        client.print(QUOTES);
        client.print(PIN);    
        client.print(QUOTES);
        client.print(COLONS);
        client.print(SERVO_SWITCH_PINS[i]);      
        
        client.print(CURLY_BRACE_OFF);
        
      }
      
      client.print(BRACKET_OFF);
      
      //--------------------------------------------------
      // Lego - Train - Switches <<< CLOSE
      //--------------------------------------------------
      
      client.print(COMMA);
      
      //--------------------------------------------------
      // Lego - Train - Barriers >>> OPEN
      //--------------------------------------------------
            
      client.print(QUOTES);
      client.print(BARRIER);
      client.print(S);     
      client.print(QUOTES);
      client.print(COLONS);       
      client.print(BRACKET_ON);
      
      for (unsigned char i = 0; i < NUM_SERVO_BARRIERS; i++) {
      
        if (i > 0){
          client.print(COMMA);
        }
      
        client.print(CURLY_BRACE_ON);
      
        // TrixBrix - Barrier - id
        client.print(QUOTES);
        client.print(ID);    
        client.print(QUOTES);
        client.print(COLONS);
        client.print(SERVO_BARRIER_IDS[i]);      
        client.print(COMMA);
        
        // TrixBrix - Barrier - pin
        client.print(QUOTES);
        client.print(PIN);    
        client.print(QUOTES);
        client.print(COLONS);
        client.print(SERVO_BARRIER_PINS[i]);      
        
        client.print(CURLY_BRACE_OFF);
        
      }
      
      client.print(BRACKET_OFF);
      
      //--------------------------------------------------
      // Lego - Train - Barriers <<< CLOSE
      //--------------------------------------------------
      
      client.print(COMMA);
      
      //--------------------------------------------------
      // Lego - Train - Signals >>> OPEN
      //--------------------------------------------------
            
      client.print(QUOTES);
      client.print(SIGNAL);
      client.print(S);     
      client.print(QUOTES);
      client.print(COLONS);       
      client.print(BRACKET_ON);
      
      for (unsigned char i = 0; i < NUM_SIGNALS; i++) {
      
        if (i > 0){
          client.print(COMMA);
        }
      
        client.print(CURLY_BRACE_ON);
      
        // TrixBrix - Signal - id
        client.print(QUOTES);
        client.print(ID);    
        client.print(QUOTES);
        client.print(COLONS);
        client.print(SERVO_SWITCH_IDS[i]);      
        client.print(COMMA);
      
        // TrixBrix - Signal - pins
        client.print(QUOTES);
        client.print(PIN);    
        client.print(S);    
        client.print(QUOTES);
        client.print(COLONS);
      
        client.print(BRACKET_ON);
      
        for (unsigned char j = 0; j < 2; j++) {
          if (j > 0){
            client.print(COMMA);
          }
          // TrixBrix - Signal - pin (j-th)
          client.print(SIGNAL_PINS[(2*i)+j]);     
        }     
      
        client.print(BRACKET_OFF);
        
        client.print(CURLY_BRACE_OFF);
        
      }
      
      client.print(BRACKET_OFF);
      
      //--------------------------------------------------
      // Lego - Train - Signals <<< CLOSE
      //--------------------------------------------------
      
    client.print(CURLY_BRACE_OFF);

    //--------------------------------------------------
    // Lego - Train <<< CLOSE
    //--------------------------------------------------

  client.print(CURLY_BRACE_OFF);
  
  //--------------------------------------------------
  // TrixBrix << CLOSE
  //-------------------------------------------------- 
}

//------------------------------------------------------------------------------------------------
// TRAIN CONTROLLER FUNCTIONS
// >>> POSITION SENSORS <<<
//------------------------------------------------------------------------------------------------

void update_position_sensors(){
  for(unsigned char i = 0; i < NUM_POSITION_SENSORS; i++){ 
    // 0 => train detected, 1 => no train
    unsigned char v = digitalRead(POSITION_SENSOR_PINS[i]);
    // Change the values
    // 1 => train detected, 0 => no train
    v = ((v+1)%2);
    if (v != position_sensor[i]) {
      // value, has changed!      
      position_sensor[i] = v;
      mqtt_publish_position_sensor(i,v);
    }       
  }
}

//------------------------------------------------------------------------------------------------
// TRAIN CONTROLLER FUNCTIONS
// >>> SWITCHES <<<
//------------------------------------------------------------------------------------------------

void all_switches_off(){  
  for(unsigned char i = 0; i < NUM_SERVO_SWITCHES; i++){  
    switch_off(i);
  }
}

void all_switches_on(){
  for(unsigned char i = 0; i < NUM_SERVO_SWITCHES; i++){  
    switch_on(i);
  }
}

void switch_on(unsigned char i){
  int ang = SERVO_SWITCH_ANGLES[1];
  set_switch(i, ang);
}

void switch_off(unsigned char i){
  int ang = SERVO_SWITCH_ANGLES[0];
  set_switch(i, ang);
}

void set_switch(unsigned char i, int a){
  set_servo(true, i, a);
}

//------------------------------------------------------------------------------------------------
// TRAIN CONTROLLER FUNCTIONS
// >>> BARRIERS <<<
//------------------------------------------------------------------------------------------------

void all_barriers_off(){  
  for(unsigned char i = 0; i < NUM_SERVO_BARRIERS; i++){  
    barrier_off(i);
  }
}

void all_barriers_on(){
  for(unsigned char i = 0; i < NUM_SERVO_BARRIERS; i++){  
    barrier_on(i);
  }
}

void barrier_on(unsigned char i){
  int ang = SERVO_BARRIER_ANGLES[1];
  set_barrier(i, ang);
}

void barrier_off(unsigned char i){
  int ang = SERVO_BARRIER_ANGLES[0];
  set_barrier(i, ang);
}

void set_barrier(unsigned char i, int a){
  set_servo(false, i, a);
}

void set_servo(bool switch_or_barrier, unsigned char i, int a){       
  if (switch_or_barrier){
    Serial.print("switch #");
    Serial.print(i);
    Serial.print(" > pin:");
    Serial.print(SERVO_SWITCH_PINS[i]);
    Serial.print(" > angle:");
    Serial.println(a);
    servo_switch[i].write(a);     
    delay(250);        
    //servo_switch.detach();
  } else {
    Serial.print("barrier #");
    Serial.print(i);
    Serial.print(" > angle:");
    Serial.println(a);
    servo_barrier[i].write(a);     
    delay(250);        
    //servo_barrier.detach();
  }  
}

//------------------------------------------------------------------------------------------------
// TRAIN CONTROLLER FUNCTIONS
// >>> SIGNALS <<<
//------------------------------------------------------------------------------------------------

void set_signal(unsigned char s, unsigned char l, bool v){
  for(unsigned char j = 0; j <2; j++){
    unsigned char pin = SIGNAL_PINS[(2*s)+j];
    if ( v == (j==l) ) {    
      // Accendi
      digitalWrite(pin,LOW);
    } else {      
      // Spegni
      digitalWrite(pin,HIGH);
    }    
  }  
}

//------------------------------------------------------------------------------------------------
// MQTT FUNCTIONS
//------------------------------------------------------------------------------------------------

void update_mqtt(){
  if (!mqtt_client.connected()) {
    reconnect_mqtt();
  }  
  // MQTT client loop >>> Once connected, the mqtt_client.loop() function 
  // must be called regularly. This allows the client to maintain the connection 
  // and check for any incoming messages.
  mqtt_client.loop();  
}

void reconnect_mqtt() {
  if (!mqtt_client.connect(MQTT_CLIENT_ID , MQTT_USER, MQTT_PASS)) {
    mqtt_connection_failed();    
  } else {
    mqtt_subscribe();
  }
}

void mqtt_subscribe(){

  // subscribe to motor   
  strcpy(buffer1,LEGO);
  strcat(buffer1,SLASH);
  strcat(buffer1,TRAIN);
  strcat(buffer1,SLASH);
  strcat(buffer1,MOTOR);
  strcat(buffer1,SLASH);    
  strcat(buffer1,PF);
  strcat(buffer1,SLASH);    
  strcat(buffer1,WILDCARD);
  mqtt_client.subscribe(buffer1);
  Serial.print("Subscribed to ");
  Serial.println(buffer1);

  // subscribe to switches
  for (unsigned char i = 0; i < NUM_SERVO_SWITCHES; i++) {

    strcpy(buffer1,LEGO);
    strcat(buffer1,SLASH);
    strcat(buffer1,TRAIN);
    strcat(buffer1,SLASH);
    strcat(buffer1,SWITCH);
    strcat(buffer1,SLASH);  
    itoa(i, buffer2, 10);    
    strcat(buffer1,buffer2);
    mqtt_client.subscribe(buffer1);
    Serial.print("Subscribed to ");
    Serial.println(buffer1);

  }

  // subscribe to barrier
  for (unsigned char i = 0; i < NUM_SERVO_BARRIERS; i++) {

    strcpy(buffer1,LEGO);
    strcat(buffer1,SLASH);
    strcat(buffer1,TRAIN);
    strcat(buffer1,SLASH);
    strcat(buffer1,BARRIER);
    strcat(buffer1,SLASH);
    itoa(i, buffer2, 10);    
    strcat(buffer1,buffer2);
    mqtt_client.subscribe(buffer1);
    Serial.print("Subscribed to ");
    Serial.println(buffer1);

  }

  // subscribe to signals
  for (unsigned char i = 0; i < NUM_SIGNALS; i++) {
    for (unsigned char j = 0; j < 2; j++) {
      strcpy(buffer1,LEGO);
      strcat(buffer1,SLASH);
      strcat(buffer1,TRAIN);
      strcat(buffer1,SLASH);
      strcat(buffer1,SIGNAL);
      strcat(buffer1,SLASH);    
      itoa(i, buffer2, 10);
      strcat(buffer1,buffer2);
      strcat(buffer1,SLASH);
      itoa(j, buffer2, 10);    
      strcat(buffer1,buffer2);
      mqtt_client.subscribe(buffer1);
      Serial.print("Subscribed to ");
      Serial.println(buffer1);
    }
  }
}


void mqtt_publish_position_sensor(int i, int v){      
  if (mqtt_client.connected()){    
    strcpy(buffer1,LEGO);
    strcat(buffer1,SLASH);
    strcat(buffer1,TRAIN);
    strcat(buffer1,SLASH);
    strcat(buffer1,POSITION);           
    strcat(buffer1,SLASH);
    itoa(POSITION_SENSOR_IDS[i], buffer2, 10);
    strcat(buffer1,buffer2);               
    itoa(v, buffer2, 10);
    if (mqtt_client.publish(buffer1, buffer2)){      
      Serial.print(SENSOR);
      Serial.print(i);
      Serial.println(" published");      
    } else {
      mqtt_publish_failed();
    }
  }           
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {

  char request[MAX_BUFFER_SIZE];
  char c[2];
  c[1] = 0;
  strcpy(request,topic);
  strcat(request,SLASH);    
  for (int i=0;i<length;i++) {
    c[0] = (char)payload[i];    
    strcat(request,c);
  }
  Serial.println(request);
  bool b = false;
  process_request_by_char(request, b);  
  
}

void mqtt_connection_failed() {
  Serial.println("Mqtt connection failed");
}

void mqtt_publish_failed() {
  Serial.println("Mqtt publish failed");
}

//------------------------------------------------------------------------------------------------
// SERIAL FUNCTIONS
//------------------------------------------------------------------------------------------------

void get_char_from_client(BridgeClient &client, char *buf){  
  unsigned char i = 0;  
  while(client.available() && i < MAX_BUFFER_SIZE){    
    char c = client.read();     
    // 13 = '/r'  
    // 10 = '/n'  
    if (c != 13 && c!= 10){            
      buf[i++] = c;        
    }
  }
  // terminate string
  buf[i] = 0;   
}

void get_char_until_next_slash(char *str, char *buf, unsigned char &i){  
  unsigned char j = 0;  
  while(str[i] != 0 && i < MAX_BUFFER_SIZE){           
    if (str[i] != 47){
      buf[j++] = str[i];
      i++;
    } else {
      // exit from while loop        
      i++;
      break;
    }        
  }
  // terminate string
  buf[j] = 0;   
}
