#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>


const int motorPin1 = 2;     // motor connected to D2
const int motorPin2 = 3; 
const int motorPin3 = 4;     // motor connected to D3
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

void setup() {

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  Serial.begin(115200); //setup serial monitor, uncomment Serial.print()'s to debug
  Serial.println("Begin LilyPad Vibe Motor Tests");
    if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring!"));
    while(1);
  }

  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);
 
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);



  digitalWrite(motorPin1, HIGH);  //turn motor
  delay(1000);
  digitalWrite(motorPin1, LOW);  //turn motor
  delay(1000);
  //Serial.println("Turn LilyPad Vibe Motor And LED OFF");
  //digitalWrite(ledPin, LOW);  // turn the LED off
  digitalWrite(motorPin2, HIGH);   //turn motor off
  delay(1000);
  digitalWrite(motorPin2, LOW);  //turn motor
  delay(1000);

  digitalWrite(motorPin3, HIGH);   //turn motor off
  delay(1000);
  digitalWrite(motorPin3, LOW);  //turn motor
  delay(1000);
  //Quick Test 1: Check If LED and Motor Can Turn On

  //Serial.println("Turn LilyPad Vibe Motor And LED ON");
  //digitalWrite(ledPin, HIGH);  // turn the LED on
  
 
}

void loop() {
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp);
  if (gyro.gyro.x>10)
  {
  digitalWrite(motorPin1, HIGH);
  //digitalWrite(motorPin2, HIGH);
   delay(200);
   digitalWrite(motorPin1, LOW);
  }

   if (gyro.gyro.y>40.0)
  {
  digitalWrite(motorPin2, HIGH);
  delay(200);
  digitalWrite(motorPin2, LOW);
  }

  if (gyro.gyro.z<-80.0)
  {
  digitalWrite(motorPin3, HIGH);
  delay(200);
  digitalWrite(motorPin3, LOW);
  }
  
  // digitalWrite(motorPin1, HIGH);
  // digitalWrite(motorPin2, HIGH); //turn motor
  
  //  //turn motor
  // // delay(200);
  // //Serial.println("Turn LilyPad Vibe Motor And LED OFF");
  // //digitalWrite(ledPin, LOW);  // turn the LED off
  //    //turn motor off
  // delay(random(200,2000));
  // digitalWrite(motorPin2, LOW);
  // delay(random(300,3000));
  // digitalWrite(motorPin1, LOW);   //turn motor
  // delay(1000);
}
