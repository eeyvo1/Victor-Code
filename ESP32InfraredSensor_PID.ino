//****************************************//
//* Example Code for Sending             *//
//* Signed Integers over I2C             *//
//* ESP32 (Master) to Arduino (Slave)    *//
//*                                      *//
//* Master Code                          *//
//*                                      *//
//* UoN 2022 - Nat Dacombe               *//
//****************************************//

// read through all of the code and the comments before asking for help
// research 'two's complement' if this is not familiar to you as it is used to represented signed (i.e. positive and negative) values


#define NUM_SENSORS 6  // number of sensors used
#define IR1 A18        //
#define IR2 A17        // Channel A direction
#define IR3 A16        // Channel B direction
#define IR4 A15
#define IR5 A14
#define IR6 A13

int mode = 0;
const int power = 500;
const int iniMotorPower = 250;
const int adj = 1;
float adjTurn = 8;

#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2

int SensorValues[NUM_SENSORS] = { 0, 0, 0, 0, 0, 0 };

// PID controller
float Kp = 50;
float Ki = 0;
float Kd = 0;

float error = 0, P = 0, I = 0, D = 0, PIDvalue = 0;
float previousError = 0, previousI = 0;


// the minimum and maximum values here are determined by the amount of bits used by the chosen variable type
// for int, this is either 16-bits or 32-bits
// due to two's complement, the minimum value is -2^(N-1), and the maximum is (2^(N-1))-1; where N is the number of bits
int LeftMotor = 0;
int RightMotor = 0;
int ServoAngle = 85;


#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04  // 4 in hexadecimal

void setup() {
  Serial.begin(9600);
  Wire.begin();  // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)

  pinMode(IR1, OUTPUT);
  pinMode(IR2, OUTPUT);
  pinMode(IR3, OUTPUT);
  pinMode(IR4, OUTPUT);
  pinMode(IR5, OUTPUT);
  pinMode(IR6, OUTPUT);
}


void calculatePID()
{
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
}

void motorPIDcontrol() {
  LeftMotor = iniMotorPower + 0.7 * PIDvalue;
  RightMotor = iniMotorPower - 0.7 * PIDvalue;
  ServoAngle = 85 + PIDvalue;
}


void readSsensors() {
  SensorValues[0] = digitalRead(IR1);
  SensorValues[1] = digitalRead(IR2);
  SensorValues[2] = digitalRead(IR3);
  SensorValues[3] = digitalRead(IR4);
  SensorValues[4] = digitalRead(IR5);
  SensorValues[5] = digitalRead(IR6);  
  //Creating error values based on the sensor readings from the array
  if ((SensorValues[0] == 0) && (SensorValues[1] == 0) && (SensorValues[2] == 0) && (SensorValues[3] == 0) && (SensorValues[4] == 0) && (SensorValues[5] == 1)) {
    mode = FOLLOWING_LINE;
    error = 5;
  } else if ((SensorValues[0] == 0) && (SensorValues[1] == 0) && (SensorValues[2] == 0) && (SensorValues[3] == 0) && (SensorValues[4] == 1) && (SensorValues[5] == 1)) {
    mode = FOLLOWING_LINE;
    error = 4;
  } else if ((SensorValues[0] == 0) && (SensorValues[1] == 0) && (SensorValues[2] == 0) && (SensorValues[3] == 0) && (SensorValues[4] == 1) && (SensorValues[5] == 0)) {
    mode = FOLLOWING_LINE;
    error = 3;
  } else if ((SensorValues[0] == 0) && (SensorValues[1] == 0) && (SensorValues[2] == 0) && (SensorValues[3] == 1) && (SensorValues[4] == 1) && (SensorValues[5] == 0)) {
    mode = FOLLOWING_LINE;
    error = 2;
  } else if ((SensorValues[0] == 0) && (SensorValues[1] == 0) && (SensorValues[2] == 0) && (SensorValues[3] == 1) && (SensorValues[4] == 0) && (SensorValues[5] == 0)) {
    mode = FOLLOWING_LINE;
    error = 1;
  } else if ((SensorValues[0] == 0) && (SensorValues[1] == 0) && (SensorValues[2] == 1) && (SensorValues[3] == 1) && (SensorValues[4] == 0) && (SensorValues[5] == 0)) {
    mode = FOLLOWING_LINE;
    error = 0;
  } else if ((SensorValues[0] == 0) && (SensorValues[1] == 0) && (SensorValues[2] == 1) && (SensorValues[3] == 0) && (SensorValues[4] == 0) && (SensorValues[5] == 0)) {
    mode = FOLLOWING_LINE;
    error = -1;
  } else if ((SensorValues[0] == 0) && (SensorValues[1] == 1) && (SensorValues[2] == 1) && (SensorValues[3] == 0) && (SensorValues[4] == 0) && (SensorValues[5] == 0)) {
    mode = FOLLOWING_LINE;
    error = -2;
  } else if ((SensorValues[0] == 0) && (SensorValues[1] == 1) && (SensorValues[2] == 0) && (SensorValues[3] == 0) && (SensorValues[4] == 0) && (SensorValues[5] == 0)) {
    mode = FOLLOWING_LINE;
    error = -3;
  } else if ((SensorValues[0] == 1) && (SensorValues[1] == 1) && (SensorValues[2] == 0) && (SensorValues[3] == 0) && (SensorValues[4] == 0) && (SensorValues[5] == 0)) {
    mode = FOLLOWING_LINE;
    error = -4;
  } else if ((SensorValues[0] == 1) && (SensorValues[1] == 0) && (SensorValues[2] == 0) && (SensorValues[3] == 0) && (SensorValues[4] == 0) && (SensorValues[5] == 0)) {
    mode = FOLLOWING_LINE;
    error = -5;
  } else if ((SensorValues[0] == 1) && (SensorValues[1] == 1) && (SensorValues[2] == 1) && (SensorValues[3] == 1) && (SensorValues[4] == 1) && (SensorValues[5] == 1)) {
    mode = STOPPED;
    error = 0;
  } else if ((SensorValues[0] == 0) && (SensorValues[1] == 0) && (SensorValues[2] == 0) && (SensorValues[3] == 0) && (SensorValues[4] == 0) && (SensorValues[5] == 0)) {
    mode = NO_LINE;
    error = 0;
  }
}



void loop() {

  readSsensors();
  switch (mode) {
    case STOPPED:
      LeftMotor = 0;
      RightMotor = 0;
      delay(200);
      previousError = error;
      break;

    case NO_LINE:
      LeftMotor = 0;
      RightMotor = 0;
      delay(200);
      ServoAngle = 30;          
      previousError = 0;
      break;

    case FOLLOWING_LINE:
      calculatePID();
      motorPIDcontrol();
      break;
  }


  Wire.beginTransmission(I2C_SLAVE_ADDR);  // transmit to device #4
  /* depending on the mirocontroller, the int variable is stored as 32-bits or 16-bits
     if you want to increase the value range, first use a suitable variable type and then modify the code below
     for example; if the variable used to store x and y is 32-bits and you want to use signed values between -2^31 and (2^31)-1
     uncomment the four lines below relating to bits 32-25 and 24-17 for x and y
     for my microcontroller, int is 32-bits hence x and y are AND operated with a 32 bit hexadecimal number - change this if needed
     >> X refers to a shift right operator by X bits
  */
  //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
  //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
  Wire.write((byte)((LeftMotor & 0x0000FF00) >> 8));   // first byte of x, containing bits 16 to 9
  Wire.write((byte)(LeftMotor & 0x000000FF));          // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((RightMotor & 0x0000FF00) >> 8));  // first byte of y, containing bits 16 to 9
  Wire.write((byte)(RightMotor & 0x000000FF));         // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((ServoAngle & 0x0000FF00) >> 8));  // first byte of y, containing bits 16 to 9
  Wire.write((byte)(ServoAngle & 0x000000FF));
  Wire.endTransmission();  // stop transmitting
  delay(100);
}
