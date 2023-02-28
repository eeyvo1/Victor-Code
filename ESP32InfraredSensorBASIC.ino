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

#define IR1 A18  //     
#define IR2 A17  // Channel A direction 
#define IR3 A16  // Channel B direction 
#define IR4 A15
#define IR5 A14
#define IR6 A13


#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#define HIGH 1
#define LOW -1

void setup()
{
  Serial.begin(9600);
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  pinMode(IR1, OUTPUT);
  pinMode(IR2, OUTPUT);
  pinMode(IR3, OUTPUT);
  pinMode(IR4, OUTPUT);
  pinMode(IR5, OUTPUT);
  pinMode(IR6, OUTPUT);

}

// the minimum and maximum values here are determined by the amount of bits used by the chosen variable type
// for int, this is either 16-bits or 32-bits
// due to two's complement, the minimum value is -2^(N-1), and the maximum is (2^(N-1))-1; where N is the number of bits
int LeftMotor = 0;
int RightMotor = 0;
int ServoAngle = 85;





void loop()
{
  ReadIR3 = digitalRead(IR3);
  ReadIR4 = digitalRead(IR4);
 
   
  if((ReadIR3 == HIGH) && (ReadIR4 == HIGH))
  {
    LeftMotor = 230;
    RightMotor = 230;
    ServoAngle = 85;

  }
  else if((ReadIR3 == HIGH) &&(ReadIR4 == LOW))
  {
    LeftMotor = 80;
    RightMotor = 230;
    ServoAngle = 40;
  }
    else if((ReadIR3 == LOW) &&(ReadIR4 == HIGH))
  {
    LeftMotor = 230;
    RightMotor = 80;
    ServoAngle = 120;
  }
    else if((ReadIR3 == LOW) &&(ReadIR4 == LOW))
  {
    LeftMotor = 0;
    RightMotor = 0;
    ServoAngle = 85;
  }
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  /* depending on the mirocontroller, the int variable is stored as 32-bits or 16-bits
     if you want to increase the value range, first use a suitable variable type and then modify the code below
     for example; if the variable used to store x and y is 32-bits and you want to use signed values between -2^31 and (2^31)-1
     uncomment the four lines below relating to bits 32-25 and 24-17 for x and y
     for my microcontroller, int is 32-bits hence x and y are AND operated with a 32 bit hexadecimal number - change this if needed
     >> X refers to a shift right operator by X bits
  */
  //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
  //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
  Wire.write((byte)((LeftMotor & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(LeftMotor & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((RightMotor & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(RightMotor & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((ServoAngle & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(ServoAngle & 0x000000FF));
  Wire.endTransmission();   // stop transmitting
  delay(100);
}
