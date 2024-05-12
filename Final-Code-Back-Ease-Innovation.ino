#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define vibOutPin 2 // Pin for vibration motor

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
const int initialAngleX = -180; // Initial angle for X-axis (in degrees)
const int initialAngleY = -180; // Initial angle for Y-axis (in degrees)
const int deviationThreshold = 90; // Deviation threshold (in degrees)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(vibOutPin, OUTPUT); // Define vibration motor pin as output
  mpu.initialize();
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate angle deviation for X and Y axes
  float angleX = map(ax, -17000, 17000, -180, 180); // Map accelerometer data to degrees
  float angleY = map(ay, -17000, 17000, -180, 180); // Map accelerometer data to degrees

  // Calculate deviation from initial values
  float deviationX = abs(angleX - initialAngleX);
  float deviationY = abs(angleY - initialAngleY);

  // Print angle data
  Serial.print("Angle X: ");
  Serial.print(angleX);
  Serial.print("  Angle Y: ");
  Serial.println(angleY);
  
  // Check if angle deviation exceeds threshold
  if (deviationX > deviationThreshold || deviationY > deviationThreshold) {
    digitalWrite(vibOutPin, HIGH); // Activate vibration motor
  } else {
    digitalWrite(vibOutPin, LOW); // Turn off vibration motor
  }
  
  delay(1000); // Delay for 1 second before the next reading
}
