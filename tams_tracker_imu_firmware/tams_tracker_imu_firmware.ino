#include<Wire.h>

int imu_address = 0x68; 

void setup() {
  
  Serial.begin(9600);
  
  Wire.begin();
  
  // wake up
  Wire.beginTransmission(imu_address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // set gyro range to 250deg/s 
  Wire.beginTransmission(imu_address); //I2C address of the MPU
  Wire.write(0x1B);
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  
  // set accelerometer range to 2g
  Wire.beginTransmission(imu_address);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void loop() {
  
  Wire.beginTransmission(imu_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  Wire.requestFrom(imu_address, 14, true);
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t t = Wire.read() << 8 | Wire.read();
  int16_t rx = Wire.read() << 8 | Wire.read();
  int16_t ry = Wire.read() << 8 | Wire.read();
  int16_t rz = Wire.read() << 8 | Wire.read();
  
  Serial.print("imu ");
  Serial.print(ax);
  Serial.print(" ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.print(az);
  Serial.print(" ");
  Serial.print(rx);
  Serial.print(" ");
  Serial.print(ry);
  Serial.print(" ");
  Serial.print(rz);
  Serial.print(" ");
  Serial.println(ax + ay + az + rx + ry + rz); // checksum
  delay(20);
}
