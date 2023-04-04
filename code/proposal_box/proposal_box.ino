#include <LiquidCrystal.h>
#include <math.h>
#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <SoftwareSerial.h>     // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
#include <Streaming.h>          // For serial debugging output - https://www.arduino.cc/reference/en/libraries/streaming/
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#define OPEN 1
#define BUZZER 2
#define UNLOCK 3

#define IRIS_TX         3   
#define IRIS_RX         2   
#define THETA_RX        4 
#define THETA_TX        5 
#define Z_RX            6 
#define Z_TX            7
#define LOCK_4_READ_PIN 8
#define EXT_LED_4       9
#define LOCK_3_READ_PIN 12
#define EXT_LED_3       13
#define DIR_Z           27
#define DIR_THETA       29
#define DIR_IRIS        31
#define IRIS_LIM_HIGH   33
#define STATE_RESET     34
#define IRIS_LIM_LOW    35
#define RED             41
#define GREEN           43
#define BLUE            45
#define WHITE           47
#define Z_LIM_HIGH      (55)
#define Z_LIM_LOW       (56)
#define LOCK_1_READ_PIN (60)
#define LOCK_2_READ_PIN (61)
#define EXT_LED_1       (66)
#define EXT_LED_2       (68)

#define THETA_VEL 1000
#define IRIS_VEL 500
#define Z_VEL 1000

#define DRIVER_ADDRESS   0b00
#define R_SENSE 0.11f  

//int Z_LIM_HIGH = PIN_A1;
//int Z_LIM_LOW = PIN_A2;
//int LOCK_1_READ_PIN = PIN_A6;
//int LOCK_2_READ_PIN = PIN_A7;
//int EXT_LED_1 = PIN_A12;
//int EXT_LED_2 = PIN_A14;

SoftwareSerial iris_serial(IRIS_RX, IRIS_TX);
SoftwareSerial theta_serial(THETA_RX, THETA_TX);
SoftwareSerial z_serial(Z_RX, Z_TX);
TMC2209Stepper iris_driver(&iris_serial, R_SENSE, DRIVER_ADDRESS); 
TMC2209Stepper theta_driver(&theta_serial, R_SENSE, DRIVER_ADDRESS); 
TMC2209Stepper z_driver(&z_serial, R_SENSE, DRIVER_ADDRESS); 

SoftwareSerial speaker_serial(10, 11); // RX, TX
DFRobotDFPlayerMini player;
void printDetail(uint8_t type, int value);

const int rs = 44, en=42, d4=48, d5=46,d3=50,d2=52;
LiquidCrystal lcd(rs, en, d5, d4, d3, d2);

int counter=0;
int lock_state=0;
int main_state=0;
int z_state=0;
int iris_state=0;


void set_lock_status(int lock_num, int state){
  int lock_1_pos = 9;
  int lock_2_pos = 14;
  switch(lock_num){
    case 0:
      lcd.setCursor(lock_1_pos, 0);
      break;
    case 1:
      lcd.setCursor(lock_2_pos, 0);
      break;
    case 2:
      lcd.setCursor(lock_1_pos, 1);
      break;
    case 3:
      lcd.setCursor(lock_2_pos, 1);
      break;
    default:
      break;
  }
  lcd.print(state ? "*" : " ");
  
}

void lock_reset(){
  lock_state = 0;
  player.volume(22);
  player.play(BUZZER);
  delay(1000);
  lcd_start_state();
  write_leds();
}

void next_lock_level(){
  delay(100);
  player.volume(25);
  player.play(UNLOCK);
  delay(100);
  set_lock_status(lock_state, 1);
  lock_state +=1;
  write_leds();
}

void write_leds(){
  int pin = 0;

  switch(lock_state){
    case 0:
      digitalWrite(EXT_LED_1, LOW);
      digitalWrite(EXT_LED_2, LOW);
      digitalWrite(EXT_LED_3, LOW);
      digitalWrite(EXT_LED_4, LOW);
      break;
    case 1:
      digitalWrite(EXT_LED_1, HIGH);
      break;
    case 2:
      digitalWrite(EXT_LED_2, HIGH);
      break;
    case 3:
      digitalWrite(EXT_LED_3, HIGH);
      break;
    case 4:
      digitalWrite(EXT_LED_4, HIGH);
      break;
    default:
      break;
    
  }
}

int build_lock_state(){
  int l1 = digitalRead(LOCK_1_READ_PIN);
  int l2 = digitalRead(LOCK_2_READ_PIN);
  int l3 = digitalRead(LOCK_3_READ_PIN);
  int l4 = digitalRead(LOCK_4_READ_PIN);
  if (l1 + l2 + l3 + l4 > 1){
     lock_reset();
     return -1;
  }
  return l1 + (l2 << 1) + (l3 << 2) + (l4 << 3);
}

void play_opening_message(){
  int scroll_time = 3000;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Congratulations");
  lcd.setCursor(0,1);
  lcd.print("on solving the");
  delay(scroll_time);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("on solving the");
  lcd.setCursor(0,1);
  lcd.print("puzzle Annie...");
  delay(scroll_time + 2000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("puzzle Annie...");
  lcd.setCursor(0,1);
  lcd.print("Now are you ready");
  delay(scroll_time);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Now are you ready");
  lcd.setCursor(0,1);
  lcd.print("for the next");
  delay(scroll_time);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("for the next");
  lcd.setCursor(0,1);
  lcd.print("adventure?");
//  delay(scroll_time);
//  lcd.clear();
//  lcd.setCursor(0,0);
//  lcd.write((uint8_t)0);
}

void read_locks(){
  int sound_delay = 20;
  int state_val = build_lock_state();

  switch(lock_state){
    case -1:
      //Do nothing
      break;
    case 0:
      if (state_val == 1){
        next_lock_level();
      } else if (state_val) {
        lock_reset();
      }
      break;
    case 1:
      if (state_val != 0 && (state_val >= 2 && state_val <= 3)){
        next_lock_level();
      } else if (state_val > 3){
        lock_reset();
      }
      break;
    case 2:
      if (state_val != 0 && (state_val >= 4 && state_val <= 7)){
        next_lock_level();
      } else if (state_val > 7){
        lock_reset();
      }
      break;
    case 3:
      if (state_val != 0 && (state_val >= 8 && state_val <= 15)){
        next_lock_level();
        digitalWrite(WHITE, HIGH);
        digitalWrite(BLUE, HIGH);
        delay(1000);
        player.volume(27);
        player.play(OPEN);
        main_state = 1;
      } else if (state_val > 15){
        lock_reset();
      }
      break;
    default:
      break;
  }
}

void lcd_start_state(){
  lcd.setCursor(0,0);
  lcd.print("Locks   [ ]  [ ]");
  lcd.setCursor(0,1);
  lcd.print("Status  [ ]  [ ]");

  for (int i=0; i<4; i++){
    set_lock_status(i, 0);
  }
}

void initialize_speaker(){
  speaker_serial.begin(9600);
  Serial.println();
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!player.begin(speaker_serial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));

  player.volume(0);  //Set volume value. From 0 to 30
}

void initialize_lcd(){
  Serial.println("Initializing LCD screen...");
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Begin Initialize..");

  byte customChar[8] = {
      0b01110,
      0b00100,
      0b01110,
      0b10001,
      0b10001,
      0b10001,
      0b01110,
      0b00000
  };
  lcd.createChar(0, customChar);
  
  Serial.println("LCD screen started sucessfully!");  
}

void initialize_steppers(){
  Serial.println("Initializing Stepper drivers...");
  iris_serial.begin(11520);           // initialize software serial for UART motor control
  iris_driver.beginSerial(11520);      // Initialize UART
  iris_driver.begin();                 // UART: Init SW UART (if selected) with default 115200 baudrate
  iris_driver.toff(5);                 // Enables driver in software
  iris_driver.rms_current(1000);        // Set motor RMS current
  iris_driver.microsteps(64);         // Set microsteps
  iris_driver.en_spreadCycle(false);
  iris_driver.pwm_autoscale(true);     // Needed for stealthChop

  Serial.println("Sucessfully started iris stepper driver!");

  theta_serial.begin(11520);           // initialize software serial for UART motor control
  theta_driver.beginSerial(11520);      // Initialize UART
  theta_driver.begin();                 // UART: Init SW UART (if selected) with default 115200 baudrate
  theta_driver.toff(5);                 // Enables driver in software
  theta_driver.rms_current(1000);        // Set motor RMS current
  theta_driver.microsteps(64);         // Set microsteps
  theta_driver.en_spreadCycle(false);
  theta_driver.pwm_autoscale(true);     // Needed for stealthChop

  Serial.println("Sucessfully started theta stepper driver!");

  z_serial.begin(11520);           // initialize software serial for UART motor control
  z_driver.beginSerial(11520);      // Initialize UART
  z_driver.begin();                 // UART: Init SW UART (if selected) with default 115200 baudrate
  z_driver.toff(5);                 // Enables driver in software
  z_driver.rms_current(2000);        // Set motor RMS current
  z_driver.microsteps(64);         // Set microsteps
  z_driver.en_spreadCycle(false);
  z_driver.pwm_autoscale(true);     // Needed for stealthChop

  Serial.println("Sucessfully started z stepper driver!");
}

void initialize_io(){
  Serial.println("Initializing all I/O...");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LOCK_1_READ_PIN, INPUT);
  pinMode(LOCK_2_READ_PIN, INPUT);
  pinMode(LOCK_3_READ_PIN, INPUT);
  pinMode(LOCK_4_READ_PIN, INPUT);

  pinMode(Z_LIM_LOW, INPUT);
  pinMode(Z_LIM_HIGH, INPUT);
  pinMode(IRIS_LIM_LOW, INPUT);
  pinMode(IRIS_LIM_HIGH, INPUT);
  
  pinMode(STATE_RESET, INPUT);
  
  pinMode(EXT_LED_1, OUTPUT);
  pinMode(EXT_LED_2, OUTPUT);
  pinMode(EXT_LED_3, OUTPUT);
  pinMode(EXT_LED_4, OUTPUT);

  pinMode(WHITE, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
}

void initialize_iris_position(){
  if(!digitalRead(IRIS_LIM_LOW)){
    Serial.println("Iris not in intial closed State, setting now");
    iris_driver.shaft(0);
    iris_driver.VACTUAL(500);
    while(!digitalRead(IRIS_LIM_LOW)){
      //do nothing
    }
    delay(150);
    iris_driver.VACTUAL(0);
  }
  iris_state=0;
}

void initialize_z(){
  if (!digitalRead(Z_LIM_LOW)){
    Serial.println("Z not in initial low state, setting now");
    z_driver.shaft(0);
    z_driver.VACTUAL(500);
    while(!digitalRead(Z_LIM_LOW)){
      //do nothing
    }
    delay(150);
    z_driver.VACTUAL(0);
  }
  z_state = 0;
}

void cycle_iris(){
  switch(iris_state){
    case 0:
      //Iris fully closed
      Serial.println("Beginning opening iris");
      iris_driver.shaft(1);
      iris_driver.VACTUAL(500);
      iris_state = 1;
      break;
    case 1:
      //currently opening
      if (digitalRead(IRIS_LIM_HIGH)){
        Serial.println("Iris fully opened!");
        delay(150);
        iris_driver.VACTUAL(0);
        iris_state = 2;
      }
      break;
    case 2:
      //Is fully opened
      Serial.println("Beginning closing iris");
      iris_driver.shaft(0);
      iris_driver.VACTUAL(500);
      iris_state = 3;
      break;
    case 3:
      if (digitalRead(IRIS_LIM_LOW)){
        Serial.println("Iris fully closed!");
        delay(150);
        iris_driver.VACTUAL(0);
        iris_state = 0;
      }
      break;
    default:
      break;
  }
}

void cycle_z(){
  switch(z_state){
    case 0:
      Serial.println("Beginning Z up");
      z_driver.shaft(1);
      z_driver.VACTUAL(1000);
      z_state = 1;
      break;
    case 1:
      if (digitalRead(Z_LIM_HIGH)){
        Serial.println("High triggered");
        delay(150);
        z_driver.VACTUAL(0);
        z_state = 2;
      }
      break;
    case 2:
      Serial.println("Beginning Z down");
      z_driver.shaft(0);
      z_driver.VACTUAL(1000);
      z_state = 3;
      break;
    case 3:
      if (digitalRead(Z_LIM_LOW)){
        Serial.println("Low triggered");
        delay(150);
        z_driver.VACTUAL(0);
        z_state = 0;
      }
      break;
    default:
      break;
  }
}



void opening_routine(){
  
  //Start spinning the Theta motor
  switch(main_state){
    case 0:
      theta_driver.VACTUAL(0);
      digitalWrite(WHITE, HIGH);
      digitalWrite(BLUE, HIGH);
    case 1:
      //Start spinning theta motor, and start opening the iris
      theta_driver.shaft(0);
      theta_driver.VACTUAL(1000);
      Serial.println("Beginning opening iris");
      iris_driver.shaft(1);
      iris_driver.VACTUAL(500);
      main_state = 2;
      break;
    case 2:
      if (digitalRead(IRIS_LIM_HIGH)){
        Serial.println("Iris fully opened!");
        delay(150);
        iris_driver.VACTUAL(0);
        main_state = 3;
      }
      break;
    case 3:
      Serial.println("Starting Z Up now");
      z_driver.shaft(1);
      z_driver.VACTUAL(1000);
      delay(150);
      main_state = 4;
      break;
    case 4:
      if (digitalRead(Z_LIM_HIGH)){
        Serial.println("Z at the top");
        delay(150);
        z_driver.VACTUAL(0);
        main_state = 5;
        delay(1000);
      }
      break;
    case 5:
      if (digitalRead(STATE_RESET)){
        reset_all();
      }
//      z_driver.shaft(0);
//      z_driver.VACTUAL(1000);
      break;
    case 6:
      
      break;
//    case 6:
//      if (digitalRead(Z_LIM_LOW)){
//        delay(150);
//        z_driver.VACTUAL(0);
//        main_state = 7;
//      }
//      break;
//    case 7:
//      iris_driver.shaft(0);
//      iris_driver.VACTUAL(500);
//      main_state = 8;
//      break;
//    case 8:
//      if (digitalRead(IRIS_LIM_LOW)){
//        delay(150);
//        iris_driver.VACTUAL(0);
//        main_state = 1;
//        delay(500);
//      }
//      break;
    default:
      break;
  }
}

void reset_all(){
  digitalWrite(WHITE, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  theta_driver.VACTUAL(0);
  initialize_z();
  initialize_iris_position();
  main_state = 0;
  lcd_start_state();
  lock_state = 0;
}

void pwm_led(){
  if (counter <= 200){
    digitalWrite(GREEN, HIGH);
  } else if (counter <=400){
    digitalWrite(GREEN,LOW);
  } else if (counter <= 600) {
    digitalWrite(RED, HIGH);
  } else if (counter <= 800){
    digitalWrite(RED, LOW);
    counter = 0;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initialize_lcd();
  initialize_io();
  initialize_speaker();
  initialize_steppers();
  lcd_start_state();
  initialize_z();
  initialize_iris_position();
 Serial.println("Ready to start up!");
  delay(500);

}
1



void loop() {

  if (main_state == 0){
    read_locks();
  } else {
    opening_routine();
}
