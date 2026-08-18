 /*  Code

 * */
#include <PS4Controller.h>
#include "DifferentialSteering.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Mounted in the back left corner with pins facing left
MPU6050 mpu(Wire);

// Use a timer to print debug every 10ms
unsigned long timer = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// use to store the current angle when steering input goes to 0 - relative to starting angle
int16_t lockangle;
int adjangle = 0;

// - fPivYLimt  : The threshold at which the pivot action starts
//                This threshold is measured in units on the Y-axis
//                away from the X-axis (Y=0). A greater value will assign
//                more of the joystick's range to pivot actions.
//                Allowable range: (0..+127)
int fPivYLimit = 32;
DifferentialSteering DiffSteer;

// Used to toggle assists on or off
boolean  assist = false;

// Display mode
String mode = "angles";

// Used to signal if assist is on or off
const int onboardled = 2; // ESP32 Pin to which onboard LED is connected

// Motor A
int motorAPin1 = 27; 
int motorAPin2 = 26; 
int enableAPin = 14;

// Motor B
int motorBPin1 = 32; 
int motorBPin2 = 33; 
int enableBPin = 25; 

// Setting PWM properties
const int freq = 30000;
const int motorA_PWMch = 0;
const int motorB_PWMch = 1;
const int resolution = 8;
int dutyCycleA = 0;
int dutyCycleB = 0;

int adj_map = 3;
int adj_out = 20;

int lowLimit = -115;
int highLimit = 115;  

int l_speed = 0;
int r_speed = 0;

int LStickXvalue = 0;
int prevXValue = 0;
int XValue = 0;
int YValue = 0;

// takes left and right motor speeds and drives motors (-128..+128)
int drive(int l_speed, int r_speed) {

  // Motor A left
  if (l_speed > 1) { //forwards
    //Serial.print("Left Forwards at %d\n", l_speed);
    digitalWrite(motorAPin1, HIGH);
    digitalWrite(motorAPin2, LOW);
    dutyCycleA = (map(l_speed, 3, 128, 150, 255 ) );
    //Serial.print("dutyCycleA: "); Serial.print(dutyCycleA);
    ledcWrite(motorA_PWMch, dutyCycleA);
    
  } else if (l_speed < -1) { //backwards
    //Serial.print("Left Backwards at %d\n", l_speed);
    digitalWrite(motorAPin1, LOW);
    digitalWrite(motorAPin2, HIGH);
    dutyCycleA = (map(l_speed, -3, -128, 150, 255 ) );
    ledcWrite(motorA_PWMch, dutyCycleA);
    
  } else {
    digitalWrite(motorAPin1, LOW);
    digitalWrite(motorAPin2, LOW);
  }

  // Motor B right
  if (r_speed > 1) { //forwards
    //Serial.print("Right Forwards at %d\n", r_speed);
    digitalWrite(motorBPin1, HIGH);
    digitalWrite(motorBPin2, LOW);
    dutyCycleB = (map(r_speed, 0, 128, 150, 255 ) );
    ledcWrite(motorB_PWMch, dutyCycleB);
    
  } else if (r_speed < -1) { //backwards
    //Serial.print("Right Backwards at %d\n", r_speed);
    digitalWrite(motorBPin1, LOW);
    digitalWrite(motorBPin2, HIGH);
    dutyCycleB = (map(r_speed, 0, -128, 150, 255 ) );
    ledcWrite(motorB_PWMch, dutyCycleB);
    
  } else {
    digitalWrite(motorBPin1, LOW);
    digitalWrite(motorBPin2, LOW);
  }
  
}
      

// Initialise pins, screen, gyro and bluetooth
void setup() {
  Serial.begin(115200);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  // Clear the buffer so the adafruit splash screen is not shown
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  
  // Show on oled
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println(F("MPU6050: "));
  display.setCursor(60,0);
  display.println(status);
  display.setCursor(0,16);
  display.println(F("Calulating offsets..."));
  display.setCursor(0,24);
  display.println(F("keep still!"));
  display.display();
  
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  PS4.begin("48:b0:2d:37:2d:4c");
  
  // sets the pins as outputs:
  pinMode(onboardled, OUTPUT);
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(enableAPin, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  pinMode(enableBPin, OUTPUT);
  
  // configure motor (esp32 ledc) PWM functionalitites
  ledcSetup(motorA_PWMch, freq, resolution);
  ledcSetup(motorB_PWMch, freq, resolution);
  
  // attach the PWM channel to the GPIO to be controlled
  ledcAttachPin(enableAPin, motorA_PWMch);
  ledcAttachPin(enableBPin, motorB_PWMch);

  DiffSteer.begin(fPivYLimit);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println(F("Ready."));
  display.display();
  delay(200); // Pause for 2 seconds

}

void loop() {

  if (PS4.isConnected()) {

    if (PS4.Share()) {
      Serial.println("Share Button");
      assist = !assist;
      delay(100); // should do a better debounce
    }

    digitalWrite(onboardled, assist);

    
    if (PS4.Options()) {
      Serial.println("Options Button");
      if (mode  == "angles") {
        mode = "mspeed";
      } else if (mode == "mspeed") {
        mode = "valadjust";
      } else if (mode == "valadjust") {
        mode = "angles";
      }
      delay(100);
    }

    if (PS4.Triangle()) {
      //Serial.println("Triangle Button");
      Serial.printf("Battery Level : %d\n", PS4.Battery());
      if (PS4.Charging()) Serial.println("The controller is charging");
    }

  // Steering
  // Get current stick pos
  LStickXvalue = PS4.LStickX();
  //Serial.print("LStickX at: "); Serial.print(LStickXvalue);
  // -+3 from 0 is deadzone
  if ((LStickXvalue > 4) or (LStickXvalue < -4)) {
    XValue = LStickXvalue;
  } else { XValue = 0; }

  if (PS4.R2()) { // Forwards
    YValue = map(PS4.R2Value(), 0, 255, 0, 115);
    //Serial.printf("R2 button at %d\n", PS4.R2Value());
  }
  if (PS4.L2()) { // Reverse overrides
    YValue = map(PS4.L2Value(), 0, 255, 0, -115);
    //Serial.printf("L2 button at %d\n", PS4.L2Value());
  }
  if (!( (PS4.R2()) or (PS4.L2()) )) { YValue = 0; }

  // Outside no action limit joystick
  if (!((XValue > -128) && (XValue < 128) && (YValue > -lowLimit) && (YValue < highLimit)))
  {
      DiffSteer.computeMotors(XValue, YValue);
      l_speed = DiffSteer.computedLeftMotor();
      r_speed = DiffSteer.computedRightMotor();

      // map motor outputs to your desired range

      //Serial.println("Differential | " + DiffSteer.toString());
  } else {
      //Serial.println("idle");
  }

  //Serial.printf("original l_speed:  %d\n", l_speed);

  // only use the gyro when not steering and when assist is on, needs to be done after diffsteer
  if ((!XValue) && (assist)) {
    if (prevXValue) {
      mpu.update();
      lockangle = mpu.getAngleZ();
    }
    
    mpu.update();

    gz = mpu.getGyroZ();
    az = mpu.getAngleZ();

    // map will not limit the output, use constrain() if rqd
    adjangle = map((az - lockangle), -adj_map, adj_map, -adj_out, adj_out);
    //Serial.printf("mapped adj:  %d\n", az);
    adjangle = constrain(adjangle,-20,20);
    // gz (Z axis rotation) is negative when the rear swings left, bot is heading right so to compensate the right motor should speed up
     
    l_speed = constrain((l_speed + adjangle),-128,128);
    r_speed = constrain((r_speed - adjangle),-128,128);
  
    //Serial.printf("adjusted l_speed:  %d\n", l_speed);
 
    
  }

  if (mode == "angles") {
      display.clearDisplay();
      display.setTextSize(1);             // Normal 1:1 pixel scale
      display.setCursor(0,0);             // Start at top-left corner
      display.println(F("Angle Z: "));
      display.setCursor(60,0);             
      display.println(az);
      display.setCursor(0,12);            
      display.println(F("adjangle: "));
      display.setCursor(60,12);            
      display.println(adjangle);
      display.setCursor(0,24);             
      display.println(F("Lock A: "));
      display.setCursor(60,24);            
      display.println(lockangle);
      display.display();
    } else if (mode == "mspeed") {
      display.clearDisplay();
      display.setTextSize(1);             // Normal 1:1 pixel scale
      display.setCursor(0,0);             // Start at top-left corner
      display.println(F("L speed: "));
      display.setCursor(60,0);             
      display.println(l_speed);
      display.setCursor(0,12);            
      display.println(F("R speed: "));
      display.setCursor(60,12);            
      display.println(r_speed);
      display.setCursor(0,24);             
      display.println(F("adj: "));
      display.setCursor(60,24);            
      display.println(adjangle);
      display.display();
    } else if (mode == "valadjust") {
      
      if (PS4.Up()) {
        Serial.println("Up Button");
        adj_map++;
        delay(100);
      } else if (PS4.Down()) {
        Serial.println("Down Button");
        adj_map--;
        delay(100);
      } else if (PS4.Left()) {
        Serial.println("Left Button");
        adj_out--;
        delay(100);
      } else if (PS4.Right()) {
        Serial.println("Right Button");
        adj_out++;
        delay(100);
      }
        
        
      display.clearDisplay();
      display.setTextSize(1);             // Normal 1:1 pixel scale
      display.setCursor(0,0);             // Start at top-left corner
      display.println(F("Adj map: "));
      display.setCursor(60,0);             
      display.println(adj_map);
      display.setCursor(0,12);            
      display.println(F("Adj out:  "));
      display.setCursor(60,12);            
      display.println(adj_out);
      display.setCursor(0,24);             
      display.println(F("adj: "));
      display.setCursor(60,24);            
      display.println(adjangle);
      display.display();
    }

  prevXValue = XValue;
  
  // makes the bot drive
  drive(l_speed, r_speed);

  }

    // Debug without controller connected
    /*
        
    mpu.update();
      if(millis() - timer > 10){ // print data every 10ms
        Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
        Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
        Serial.print("\tY: ");Serial.print(mpu.getAccY());
        Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
      
        Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
        Serial.print("\tY: ");Serial.print(mpu.getGyroY());
        Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
      
        Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
        Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
        
        Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
        Serial.print("\tY: ");Serial.print(mpu.getAngleY());
        Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
        
        timer = millis();
      }

      */
    
}
