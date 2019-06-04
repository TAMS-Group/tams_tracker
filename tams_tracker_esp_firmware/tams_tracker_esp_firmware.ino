#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <stdlib.h>
#include "os_type.h"
#include "osapi.h"

// markers

const int marker_frequencies[] = {
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
};
const int marker_pins[] = {
    D3,
    TX,
    D5,
    D6,
    D7,
    D8,
    D0,
    RX,
};
const size_t marker_count = 8;

os_timer_t marker_timer;

void marker_timer_callback(void *arg) {
  static uint8_t pwm = 1;
  pwm = ((pwm + 1) & 3);
  uint32_t t = millis();
  for (size_t i = 0; i < marker_count; i++) {
    digitalWrite(marker_pins[i], ((t * marker_frequencies[i]) / 500) & 1 || !pwm);
  }
}

void marker_init() {
  for (size_t i = 0; i < marker_count; i++) {
    pinMode(marker_pins[i], OUTPUT);
    digitalWrite(marker_pins[i], HIGH);
  }
  os_timer_setfn(&marker_timer, marker_timer_callback, nullptr);
  os_timer_arm(&marker_timer, 1, true);
}



// imu

int imu_address = 0x68; 

int16_t imu_ax = 0;
int16_t imu_ay = 0;
int16_t imu_az = 0;

int16_t imu_rx = 0;
int16_t imu_ry = 0;
int16_t imu_rz = 0;

void imu_init() {
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

void imu_read() {
  Wire.beginTransmission(imu_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  Wire.requestFrom(imu_address, 14, true);
  imu_ax = Wire.read() << 8 | Wire.read();
  imu_ay = Wire.read() << 8 | Wire.read();
  imu_az = Wire.read() << 8 | Wire.read();
  int16_t t = Wire.read() << 8 | Wire.read();
  imu_rx = Wire.read() << 8 | Wire.read();
  imu_ry = Wire.read() << 8 | Wire.read();
  imu_rz = Wire.read() << 8 | Wire.read();
}



// wifi

const char *ssid = "tams_tracker_object";
const char *password = "password";

char wifi_buffer[256];

IPAddress localIp(192,168,1,200);   
WiFiServer server(10000);

void wifi_init() {
  IPAddress subnet(255,255,255,0);   
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(localIp, localIp, subnet);
  WiFi.softAP(ssid, password, 1, 0, 32);
  WiFi.begin();
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  IPAddress myIP = WiFi.softAPIP();
  server.begin();
}



// main

void setup() {
  Serial.begin(9600);
  wifi_init();
  imu_init();
  marker_init();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("connected");
    while (client.connected()) {
      while (client.available() > 0) {
        client.read();
      }
      imu_read();
      delay(20);
      sprintf(wifi_buffer, "imu %i %i %i %i %i %i %i\n", imu_ax, imu_ay, imu_az, imu_rx, imu_ry, imu_rz, (int16_t)(imu_ax + imu_ay + imu_az + imu_rx + imu_ry + imu_rz));
      client.write(wifi_buffer);
    }
    client.stop();
    Serial.println("disconnected");
  }
}
