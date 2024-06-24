/**
 Code related to preliminary testing of the TB vacuum test rig.
 @author Andrew Scott-George
 
 
 Changelog
 24/06/24 
 - Created
 - Imported code from tb_complete
**/

/** LIBRARIES **/
#include <Wire.h>
#include "TeensyThreads.h"
#include <Adafruit_INA219.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <time.h>

/** TEENSY PINMAP **/
#define L298_ENA 8     // ENA pin on L298
#define L298_IN1 9     // IN1 pin on L298
#define L298_IN2 10    // IN2 pin on L298
#define SDA 18            // The I2C SDA line for INA219
#define SCL 19            // The I2C SCL line for INA219

/** SYSTEM **/
bool debug = true;
bool logging = false;
int debugDelay = 200;
struct state {         // The struct that will capture system states at each timestep and report them to the serial
  unsigned long time;  // Time in ms

  // Current
  float currentVibe;  // From current sensor

  // Accelerometer
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t; // Temperature

};
state X = { 0, 0, 0, 0, 0, 0, 0, 0, 0};  // The state object which will track system state.

int i = 0;                                   // An increment used throughout program
int j = 0;                                   // A secondary increment used throughout program
String inputString;                          // The string being input from the serial terminal. Used to receive commands.
unsigned char rxBuf[8];                      // Buffer used when receiving input from serial terminal.
bool stringComplete = false;                 // Set to true when string has been fully received from serial terminal, false at start of loop.
uint32_t errorMsg = 0;                       // Used when canSniff reveals an error message
String errorOutput = "";

/** SENSORS **/
#define VIBE_CURRENT 0x41                   // I2C of vibe current sensor
Adafruit_INA219 ina219_VIBE(VIBE_CURRENT);  // Vibration current sensor
Adafruit_MPU6050 mpu; // Accelerometer 

/** L298 Motor Controllers **/
int linear_count = 0;  // The amount of forward/backward progression

struct L298 {  // State model for L298 motor
  unsigned char ENA;
  unsigned char IN1;
  unsigned char IN2;
  unsigned char PWMValue;
  unsigned int Pos;
  unsigned char Oper;
};

L298 vibrator = { LA_L298_ENA, LA_L298_IN1, LA_L298_IN2, 0, 0, 0 };  // Instantiate linear actuator object


/**
 * Function to reverse motor.
 * @param {struct} motor  
 */
void Reverse(struct L298 motor) {
  digitalWrite(motor.IN1, LOW);
  digitalWrite(motor.IN2, HIGH);
}

/**
 * Function to forward motor.
 * 
 */
void Forward(struct L298 motor) {
  digitalWrite(motor.IN1, HIGH);
  digitalWrite(motor.IN2, LOW);
}

/**
 * Function to switch off motor.
 * 
 */
void MotorOff(struct L298 motor) {
  digitalWrite(motor.IN1, LOW);
  digitalWrite(motor.IN2, LOW);
}

/**
 * Function to set PWM of linear actuator.
 * @param {int} PWM - The PWM value to be set, from 0-100 (%).
 */
void setPWM(int PWM, struct L298 motor) {
  Serial.print(PWM);
  if (PWM > 255) {
    PWM = 255;
  }
  motor.PWMValue = PWM;
  Serial.print("Set motor PWM to: ");
  Serial.print(PWM);
  analogWrite(motor.ENA, PWM);
}

void setup() {
  // Serial Initialisation
  Serial.begin(115200);
  while (!Serial);  // time to get serial running
  inputString.reserve(200);  // Buffer for serial input


  // Current sensors
  Serial.println("---------(1)");
  Serial.println("Setting up current sensor.");
  if (!ina219_VIBE.begin()) {
    Serial.println("Failed to find VIBE INA219 chip");
  } 
  if (ina219_VIBE.begin()) {
      Serial.println("ina219_VIBE begun.");
  }
  Serial.println("Current sensor setup complete.");

  // Try to initialize!
  Serial.println("---------(2)");
  Serial.println("Setting up accelerometer.");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Linear Actuator Setup
  Serial.println("---------(2)");
  Serial.println("Setting up vibrator.");
  setPWM(255, Linear);  // Set PWM to max value
  pinMode(Linear.IN1, OUTPUT);
  pinMode(Linear.IN2, OUTPUT);
  Serial.println("Vibrator setup complete.");

  Serial.println("System ready.");
}



void loop() {
  // Process CAN Events
  can1.events();

  // Update and print states to terminal
  threads.addThread(updateStates);
  if (logging == true) {
    threads.addThread(printStates);
  }


  // User input
  if (stringComplete) {
    Serial.println("");
    switch (inputString[0]) {
      // Debug commands
      case 'e':
        Serial.println(atoi(&inputString[1]));
        Serial.println("");
        break;  // Test that values being taken from serial correctly
      case 't':
        updateStates();
        Serial.println("Updated states!");
        break;
      case 'y': printStates(); break;
      case 'u':
        logging = false;
        Serial.println("Logging off.");
        break;
      case 'i':
        logging = true;
        Serial.println("Logging on.");
        break;

      // L298 Commands

      // Vibrator
      case 'h':
        MotorOff(vibrator);
        Serial.println("Vibrator MotorOff()");
        Serial.println("");
        break;  // Turn LA of
      case 'j':
        Forward(vibrator);
        Serial.println("Linear Forward()");
        Serial.println("");
        break;  // Progress LA
      case 'k':
        Reverse(vibrator);
        Serial.println("Linear Reverse()");
        Serial.println("");
        break;  // Retract LA
      case 'l':
        setPWM(atoi(&inputString[1]), Linear);
        Serial.print("setPWM() called, PWM set to: ");
        Serial.print(vibrator.PWMValue);
        Serial.println("");
        break;  // Change PWM value
      case ';':
        Serial.print("IN1 set to: ");
        Serial.print(digitalRead(vibrator.IN1));
        Serial.print(", IN2 set to: ");
        Serial.print(digitalRead(vibrator.IN2));
        Serial.println("");
        break;  // Read PWM value

      case 'c':
        analogWrite(vibrator.ENA, atoi(&inputString[1]));
        Serial.print("vibrator.ENA set to ");
        Serial.print(atoi(&inputString[1]));
        Serial.println("");
        break;

      // Current sensor
      case '.':
        Serial.print("(");
        Serial.print(ina219_VIBE.getCurrent_mA());
        Serial.print("mA)");
        Serial.println("");
        break;
    }
  }

  // Clear the string:
  rxBuf[0] = 0;
  inputString = "";
  stringComplete = false;

  // Delay if debug is on
  if (debug == true) {
    delay(debugDelay);
  }
}



/**
 * Function to update all system states in a state vector X.
 * @param {state} X The system state struct.
 */
void updateStates() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  X.time = millis();                        // Time in ms
  X.currentVibe = ina219_VIBE.getCurrent_mA();  // Servo 1 current
  X.ax = a.acceleration.x;
  X.ay = a.acceleration.y;
  X.az = a.acceleration.z;
  X.gx = g.gyro.x;
  X.gy = g.gyro.y;
  X.gz = g.gyro.z;
  X.t = temp.temperature;
}


/**
 * Function to print to serial all system states in a state vector X.
 * @param {state} X The system state struct.
 */
void printStates() {
  Serial.print(X.time);
  Serial.print(", ");
  Serial.print(X.currentS1);
  Serial.print(", ");
  Serial.print(X.positionS1);
  Serial.print(", ");
  Serial.print(X.currentS2);
  Serial.print(", ");
  Serial.print(X.positionS2);
  Serial.print(", ");
  Serial.print(X.velocityM1);
  Serial.print(", ");
  Serial.print(X.currentM1);
  Serial.print(", ");
  Serial.print(X.currentVibe);
  Serial.print(", ");
  Serial.print(X.positionLA);
  Serial.print(", ");
  Serial.print(X.forceLA);
  Serial.print(", ");
  Serial.println("");
}

/**
 * Function to process an incoming serial message from operator.
 */
void serialEvent() {
  while (Serial.available()) {          // While message available
    char inChar = (char)Serial.read();  // Read a char
    inputString += inChar;              // Append char to inputString
    rxBuf[j] = inChar;                  // Append char to input buffer
    j = j + 1;                          // Increment
    if (inChar == '\n') {               // If return pressed
      for (i = 0; i < j; i++) {
        rxBuf[i] = inputString[i];  // Reverse rxBuf, why did you bother with it then?
      }
      stringComplete = true;  // Set stringComplete to true
      j = 0;
    }
  }
}
