#include <TinyGPS++.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
TinyGPSPlus gps;

// pin used:
#define echoPin 2 // pin D2 Arduino to pin Echo
#define trigPin 3 // pin D3 Arduino to pin Trig
//gps using Serial1 () 19, 18;
//imu using: SCL A5, SDA A4/ 21 scl, 20sda
const int encoderPin1  = 2;
const int encoderPin2  = 3;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

/* sensor used:
 * HCSR04
 * Ublox M8N
 * MPU6050
 * Rotary Encoder
 */

// data variable
// distance measurement
float distance=0; 
// coordinate measurement
float latitude = 0, longitude=0;
// Pitch, Roll and Yaw values
float pitch =0;
// encoder speed
float readRPM = 0, readRPS = 0, Output = 0, bikeSpeed=0;
String data ="";

int lastEncoded = 0;
long encoderValue = 0, correctEncoderValue = 0, timelast = 0, lastencoderValue = 0;
int lastMSB = 0, lastLSB = 0;

/*
function of sensors
*/
float gearRatio(){
  long duration; // duration of sound wave travel
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;
}

void coordinate(){
  while (Serial1.available() > 0){
    gps.encode(Serial1.read());
    if (gps.location.isUpdated()){
//      latitude = gps.location.lat();
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);
//      longitude = gps.location.lng();
      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);
    }
  }
}

void tilt(){
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
}

void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
  
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
  
  //LMSD - LMSDB -MSB - LSB
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
  {encoderValue++;}
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) 
  {encoderValue--;}
  
  lastEncoded = encoded; //store this value for next time
}

float calculateSpeed(){
  //1 detik - 100rpm - 4040 pulse
  //float tickToRPM = 100.0/4040.0; //sampling 1 detik
  float tickToRot = 1400;
  float dt = 0.05;
  if (millis() - timelast > 50){
    readRPS = (correctEncoderValue / tickToRot) / dt;
    readRPM = readRPS * 60.0;
    Serial.println(readRPM);
    
    correctEncoderValue = 0;
    encoderValue = 0;
    timelast = millis();        
  }
  else{
    correctEncoderValue = encoderValue;
  }  
}

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);

  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
  
  Serial.begin(9600); // usb serial
  Serial1.begin(9600); // gps serial

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}

void loop() {
  // put your main code here, to run repeatedly:
  distance = gearRatio();
  coordinate();
  tilt(); //ambil pitch
  bikeSpeed = calculateSpeed();
  data = data.concat(distance);
  data = data.concat(latitude);
  data = data.concat(longitude);
//  data = data.concat(tilt);
  data = data.concat(bikeSpeed);
  // imu timestep delay
  delay((timeStep*1000) - (millis() - timer));
}
