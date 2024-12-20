/**
 * PCA9548 I2C Multi Sensor Example
 *
 * Using two LSM6D imu sensors on ports 0
 CHANNEL ON 1 AND 2: ERM 
 *
 */
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <Arduino.h>
#include <Wire.h>
//#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <DallasTemperature.h>
#include "Adafruit_DRV2605.h" //for Haptic driver
#include "SparkFun_LSM6DSV16X.h"
#include "TCA9548A.h" //
#include "OrientationEffects.h"
#include <Adafruit_Sensor.h>

#include <math.h>
SparkFun_LSM6DSV16X myLSM;

TCA9548A I2CMux;                  // Address can be passed into the constructor

//Adafruit_DRV2605 drv, drv1, drv2; // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
// The value of the last bit of the I2C address.
// Structs for X,Y,Z data
sfe_lsm_data_t accelData;
sfe_lsm_data_t gyroData;
OneWire  ds(0);  // on pin 10 (a 4.7K resistor is necessary)

// Define the roll and pitch angles for triggering vibration
const float triggerRoll = 30.0;  // Roll angle in degrees
const float triggerPitch = 20.0; // Pitch angle in degrees
const float threshold_O = 30.0;     // Threshold for triggering vibration


// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

//dellas temerature sensor
#define TEMPERATURE_PRECISION 9 // Lower resolution
#define ONE_WIRE_BUS 0 // the pin number connected to the DS18B20 data pin


#define PCAADDR 0x70
#define DEV_I2C Wire

#define SerialPort Serial
//Adafruit_DRV2605 drv;
//ICM_20948_I2C myICM; //create an ICM_20948_I2C object
const int pinIRd = 1;
const int pinIRa = A0;
const int pinLED = 13;
int IRvalueA = 0;
int IRvalueD = 0;


// Parameters for waveform generation
const int numSamples = 100;
float rollWaveform[numSamples];
float pitchWaveform[numSamples];
float yawWaveform[numSamples];
float rollAmplitude = 30.0;
float pitchAmplitude = 20.0;
float yawAmplitude = 45.0;
float frequency = 0.1;

// Calibration constants (adjust these based on your sensor and environment)
const float m = 1.0;// 5.2; // Calibration slope  Calibration slope (convert from cm to mm)
const float b = 1.367;//0.8; // Calibration intercept (convert from cm to mm)
float ax_corrected, ay_corrected, az_corrected, gx_corrected, gy_corrected, gz_corrected;

uint8_t hapticEffect, hapticEffect_d;
//float ax, ay, az;
float roll, pitch, up_roll, up_pitch;
float yaw = 0;
//static unsigned long lastTime = 0;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
  

//variables and params for wifi connection to UE5 
const char* ssid = "rando";
const char* password = "youthought101";

// Replace with your computer's IP and port
const char* targetIP = "192.168.39.191"; // Update with your computer's IP
const int targetPort = 12345;            // Same as the port in UE5
unsigned int localPort = 8888; // Arduino's local port
char packetBuffer[255]; //buffer to hold incoming packet
WiFiUDP udp;

//float roll = 0.0, pitch = 0.0, yaw = 0.0;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 50; // Send data every 50ms (20Hz)
float prevAccelX = 0.0, prevAccelY = 0.0, prevAccelZ = 0.0;
float rollOffset = 0.0, pitchOffset = 0.0, yawOffset = 0.0; // Orientation offsets
float posXOffset = 0.0, posYOffset = 0.0, posZOffset = 0.0; // Position offsets
float posX = 0.0, posY = 0.0, posZ = 0.0;                  // Current position
bool offsetsSet = false; // Ensure offsets are set only once

/*void pcaselect(uint8_t i) {
  if (i > 4) return;
 
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}*/

// standard Arduino setup()
void setup()
{
    while (!Serial);
    delay(1000);
    Serial.begin(115200); //[[[[[]]]]]
    pinMode(pinIRd,INPUT);
    pinMode(pinIRa,INPUT);
    pinMode(pinLED,OUTPUT);
    Wire.begin();
    
    Serial.begin(115200);
    Serial.println("\nMultiSensor PCA9548");
    // define the port on the PCA9548 for the first sensor
    pcaselect(0);
    myLSM.begin();
    myLSM.deviceReset();
    myLSM.getDeviceReset();
    myLSM.enableBlockDataUpdate();
    myLSM.setAccelDataRate(LSM6DSV16X_ODR_AT_15Hz);
    myLSM.setAccelFullScale(LSM6DSV16X_16g);
     myLSM.setGyroDataRate(LSM6DSV16X_ODR_AT_15Hz);
    myLSM.setGyroFullScale(LSM6DSV16X_2000dps);

    // Configure the LSM6DSV16X
 // myLSM.setAccelRange(LSM6DSV16X_ACCEL_RANGE_2G);
  //myLSM.setGyroRange(LSM6DSV16X_GYRO_RANGE_250DPS);

    myLSM.enableFilterSettling();

    myLSM.enableAccelLP2Filter();
    myLSM.setAccelLP2Bandwidth(LSM6DSV16X_XL_STRONG);

    myLSM.enableGyroLP1Filter();
    myLSM.setGyroLP1Bandwidth(LSM6DSV16X_GY_ULTRA_LIGHT);
     GetcalibrateIMU(ax_corrected, ay_corrected, az_corrected, gx_corrected, gy_corrected, gz_corrected);
  
    // define the port on the PCA9548 for the 2nd sensor
    pcaselect(1);
     drv.begin();
     drv.setMode(DRV2605_MODE_REALTIME);
     drv.setRealtimeValue(0x11);
     drv.setMode(DRV2605_MODE_INTTRIG);
     pcaselect(2);
     drv2.begin();

      sensors.begin();

    //connect to wifi
    Serial.print("Connecting to Wi-Fi...");
    while (WiFi.begin(ssid, password) != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("Board IP Address: ");
    Serial.println(WiFi.localIP());
  }

float calDistance(void)
{
  int IRvalueA = analogRead(pinIRa);
  //IRvalueD = digitalRead(pinIRd);
  float voltage = IRvalueA * (5 / 1023.0); // Assuming 5V Arduino
  float distance = (m * voltage + b) * 10;

  return distance;
}

float tempF(void)
{
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
  Serial.print("Temperature in Celsius : ");
  Serial.print(temperatureC);
  Serial.print(" C");
  Serial.print("\tTemperature in Fahrenheit : ");
  Serial.print(temperatureF);
  Serial.println(" F");
  return temperatureF;
}

float GetcalibrateIMU(float &ax_corrected, float &ay_corrected, float &az_corrected,
                             float &gx_corrected, float &gy_corrected, float &gz_corrected)
{
  if (myLSM.checkStatus())
   {
      float ax = 0, ay = 0, az = 0;
      float gx = 0, gy = 0, gz = 0;
      int samples = 100;
      ax = accelData.xData;
       Serial.print("******************");
      for (int i = 0; i < samples; i++) {
      myLSM.getAccel(&accelData);
      ax += accelData.xData; ay +=accelData.yData;  az += accelData.zData;
      delay(10);
      myLSM.getGyro(&gyroData); gx += gyroData.xData; gy += gyroData.yData; gz += gyroData.zData;
    delay(10);
  }
 //average collected data
  ax /= samples;
  ay /= samples;
  az /= samples;

  Serial.print("Accelerometer Bias (X, Y, Z): ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az);
  //average collected data
  gx /= samples;
  gy /= samples;
  gz /= samples;

  Serial.print("Gyroscope Bias (X, Y, Z): ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);
  float ax_corrected = accelData.xData- ax; //ax_bias;
  float ay_corrected =accelData.yData - ay;// ay_bias;
  float az_corrected = accelData.zData - az;//az_bias;

  float gx_corrected = gyroData.xData - gx; //gx_bias;
  float  gy_corrected = gyroData.yData- gy; // gy_bias;
  float gz_corrected = gyroData.zData - gz;// gz_bias;
  
  Serial.print("Corrected Calibrated Accelerometer (X, Y, Z) AND  Calibrated Gyroscope (X, Y, Z): ");
  Serial.print(ax_corrected); Serial.print(", "); Serial.print(ay_corrected); Serial.print(", "); Serial.print(az_corrected);

  //Serial.print("Corrected Calibrated Gyroscope (X, Y, Z): ");
  Serial.print(gx_corrected); Serial.print(", "); Serial.print(gy_corrected); Serial.print(", "); Serial.println(gz_corrected);
  delay(10);
 
   }
  return ax_corrected, ay_corrected, az_corrected;
  
}





void playMicroVibration(uint8_t intensity)
{
  // Set the intensity
  drv.setRealtimeValue(intensity);
  // Load a waveform sequence: we will use effect #1 which is a strong click
  drv.setWaveform(0, 51);  // Play effect #51 (Buzz5 -20%)
  drv.setWaveform(1, 0);  // End of sequence
  // Start the waveform sequence playback
  drv.go();
  // Optional: Print status for debugging
  //Serial.print("Playing micro vibration effect with intensity: ");
  //Serial.println(intensity);
}

void playVibrationType(uint8_t intensity,uint8_t effect)
{
  // Set the intensity
  drv.setRealtimeValue(intensity);
  // Load a waveform sequence: we will use effect #1 which is a strong click
  drv.setWaveform(0, effect);  // Play effect #51 (Buzz5 -20%)
  drv.setWaveform(1, 0);  // End of sequence
  // Start the waveform sequence playback
  drv.go();
  // Optional: Print status for debugging
  Serial.print("Playing micro vibration effect with type: ");
  Serial.println(effect);
}

void controlERM(float roll, float pitch) 
{
  uint8_t intensity = mapOrientationToIntensity(roll, pitch);
  // Set the intensity
  drv2.setRealtimeValue(intensity);

  // Play a simple effect with the specified intensity
  drv2.setWaveform(0,119 );  // Play effect #1 (Strong Click)
  //drv.setWaveform(55, 0);  // End of sequence
  drv2.go();
  delay(10);

  // Optional: Print intensity for debugging
  Serial.print("ERM Intensity: ");
  Serial.println(intensity);
}



void createPulsedEffect() {
  // Strong pulse
   uint8_t intensity = 70, intensity2 =90;
  drv.setRealtimeValue(50);
  drv.setWaveform(0, 47); // Effect number 47: Strong Click
  // Pause (effect number 0 has no effect)
  delay(30);
  drv.setWaveform(1, 0);  // Wait for the next effect
   delay(30);
  // Repeat the strong pulse
  drv.setWaveform(2, 47); // Strong Click
   delay(30);
  // Pause
  drv.setWaveform(3, 0);  // Wait for the next effect
   delay(30);
  // Another strong pulse
 drv.setWaveform(4, 47); // Strong Click
  delay(30);
  // End the effect sequence
  //drv.setWaveform(5, 0);  // End the effect sequence
}





uint8_t mapOrientationToIntensity(float roll, float pitch) {
  // Map roll and pitch to an intensity value (0 to 127)
  // Customize the mapping function as needed

  float magnitude = sqrt(roll * roll + pitch * pitch);
  uint8_t intensity = (uint8_t)map(magnitude, 0, 360, 0, 127);
  return intensity;
}



void loop() 
{
  uint8_t NewDataReady = 0;
  float acceleration = 0.0;
  float combinegyroDataMagnitude =0.0;
  // Threshold for triggering haptic feedback
  float threshold = 1000, threshold_a = 400, threshold_band =2500;
  float Threshold_accel = 0.0, Threshold_accel_up = 0.0;
  //VL53L4CD_Result_t results;
  uint8_t status;
  char report[64];
  
  IRvalueA = analogRead(pinIRa);
  IRvalueD = digitalRead(pinIRd);
 //  Serial.print("Analog Reading="); Serial.print(IRvalueA);
  // Convert the raw analog value to voltage
  float voltage = IRvalueA * (5 / 1023.0); // Assuming 5V Arduino
  // Convert voltage to distance using calibration equation
  //float distance = (m * voltage + b) * 10;
  float distance = calDistance();
  // Print the distance in millimeters
 Serial.print("IR Sensor Value: ");
 Serial.print(IRvalueA);
 Serial.print(", Voltage: ");
 Serial.print(voltage, 2); // Print voltage with 2 decimal places
 Serial.print("V, Distance: ");
 Serial.print(distance);
 Serial.println(" mm");
 if (udp.begin(localPort)) {
  Serial.println("UDP initialized successfully.");
} else {
  Serial.println("UDP initialization failed.");
}

  sfe_lsm_data_t accelData, gyroData;

    // Check if the IMU has new data
    if (myLSM.checkStatus()) {
        // Retrieve scaled accelerometer and gyroscope data
        myLSM.getAccel(&accelData);
        myLSM.getGyro(&gyroData);

        // Calculate roll, pitch, yaw
        roll = atan2(accelData.yData, accelData.zData) * 180.0 / PI;
        pitch = atan2(-accelData.xData, sqrt(accelData.yData * accelData.yData + accelData.zData * accelData.zData)) * 180.0 / PI;
        yaw += gyroData.zData * 0.05; // Integrate gyroscope data over time

        // Normalize yaw to 0-360 degrees
        yaw = fmod(yaw + 360.0, 360.0);
    }
  int incomingByte = 0; // for incoming serial data
  Serial.println("Effects based on: 1. Acceleration 2. Orientation 3. Pulsed 5.Temprature 4. Distance Mode");
  uint8_t hapticEffect0 = 0;
  float prevRoll = 0.0; // Initialize this globally or persist across frames
  while(1){ 
  uint8_t intensity1 = 70, intensity2 =90;
  unsigned long lastTime = 0;
 

  if (Serial.available() > 0) {
    // read the incoming byte:
      incomingByte = Serial.read();
   
    // say what you got:b
     Serial.print("I received: ");
     Serial.println(incomingByte, DEC);
    switch (incomingByte){
      case '1':
        Serial.println("Acceleration Mode Demonstration");
        while(1){
          pcaselect(0);
          {
           if (myLSM.checkStatus())
            {
           
                myLSM.getAccel(&accelData);
                myLSM.getGyro(&gyroData);
                /*Serial.print("Accelerometer: "); 
                Serial.print("X: "); Serial.print(accelData.xData); Serial.print(" "); Serial.print("Y: "); Serial.print(accelData.yData); Serial.print(" "); Serial.print("Z: "); Serial.print(accelData.zData); Serial.println(" ");
                Serial.print("Gyroscope: "); 
                Serial.print("X: "); Serial.print(gyroData.xData);  Serial.print(" ");Serial.print("Y: "); Serial.print(gyroData.yData);Serial.print(" "); Serial.print("Z: "); Serial.print(gyroData.zData); Serial.println(" "); 
                */
                GetcalibrateIMU(ax_corrected, ay_corrected, az_corrected, gx_corrected, gy_corrected, gz_corrected);
                //acceleration = sqrt(ax_corrected * ax_corrected + ay_corrected * ay_corrected + az_corrected * az_corrected);
                acceleration = sqrt(accelData.xData * accelData.xData + accelData.yData * accelData.yData + accelData.zData * accelData.zData); 
                //Serial.println(acceleration);
                unsigned long currentTime = millis();
                float deltaTime = (currentTime - lastTime) / 1000.0;
                lastTime = currentTime;
                float gravity=9.8;
                // Convert accelerometer data to m/s²
                  float accelX = accelData.xData / 1000.0 * gravity; 
                  float accelY = accelData.yData / 1000.0 * gravity;
                  float accelZ = (accelData.zData / 1000.0 * gravity) - gravity; // Subtract gravity
                  float gyroX = gyroData.xData;
                  float gyroY = gyroData.yData;
                  float gyroZ = gyroData.zData;
                // Set offsets on the first iteration
                if (!offsetsSet) {
                    rollOffset = atan2(accelY, accelZ) * 180.0 / PI;
                    pitchOffset = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
                    yawOffset = atan2(accelY, accelX) * 180.0 / PI;
                    // Set position offsets
                    posXOffset = posX;
                    posYOffset = posY;
                    posZOffset = posZ;
                    offsetsSet = true;
                }
                  

                  // Low-pass filter for acceleration
                  const float alpha = 0.98;
                  accelX = alpha * accelX + (1 - alpha) * prevAccelX;
                  accelY = alpha * accelY + (1 - alpha) * prevAccelY;
                  accelZ = alpha * accelZ + (1 - alpha) * prevAccelZ;
                  prevAccelX = accelX;
                  prevAccelY = accelY;
                  prevAccelZ = accelZ;

                  // Integrate acceleration to get velocity
                  float velX,velY,velZ=0;
                  velX += accelX * deltaTime;
                  velY += accelY * deltaTime;
                  velZ += accelZ * deltaTime;

                  // Apply dead zone to velocity
                  const float accelDeadZone = 0.05;
                  if (abs(velX) < accelDeadZone) velX = 0.0;
                  if (abs(velY) < accelDeadZone) velY = 0.0;
                  if (abs(velZ) < accelDeadZone) velZ = 0.0;

                  // Integrate velocity to get position
                  posX += velX * deltaTime;
                  posY += velY * deltaTime;
                  posZ += velZ * deltaTime;

                  // Apply position offsets
                  posX -= posXOffset;
                  posY -= posYOffset;
                  posZ -= posZOffset;

                
                // Calculate roll, pitch, yaw with offsets
                roll = atan2(accelY, accelZ) * 180.0 / PI - rollOffset;
                pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI - pitchOffset;
                const float yawDeadZone = 0.5; // Dead zone threshold for minor fluctuations
                //yaw += gyroZ * deltaTime; // Integrate gyroscope for yaw
                if (abs(gyroZ) < yawDeadZone) {
                    gyroZ = 0.0; // Ignore small fluctuations
                }
                // Update yaw using complementary filter
                yaw += gyroZ * deltaTime; // Integrate gyroscope for yaw
                yaw = alpha * yaw + (1 - alpha) * atan2(accelY, accelX) * 180.0 / PI; // Blend with accelerometer

                // Subtract yaw offset
                yaw -= yawOffset;
                //yaw = alpha * (yaw + gyroZ * deltaTime) + (1 - alpha) * atan2(accelY, accelX) * 180.0 / PI - yawOffset;

                // Normalize to 0-360 degrees
                roll = fmod(roll + 360.0, 360.0);
                pitch = fmod(pitch + 360.0, 360.0);
                yaw = fmod(yaw + 360.0, 360.0);
                // Send positional and rotational data
                if (currentTime - lastSendTime >= sendInterval) {
                    lastSendTime = currentTime;

                    // Format: "roll,pitch,yaw,posX,posY,posZ"
                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", roll, pitch, yaw, posX, posY, posZ);

                    // Send UDP packet
                    udp.beginPacket(targetIP, targetPort);
                    udp.write(buffer);
                    udp.endPacket();

                    // Debug Output
                    /*Serial.print("Position: X="); Serial.print(posX);
                    Serial.print(" Y="); Serial.print(posY);
                    Serial.print(" Z="); Serial.println(posZ);
                    Serial.print("Rotation: Roll="); Serial.print(roll);
                    Serial.print(" Pitch="); Serial.print(pitch);
                    Serial.print(" Yaw="); Serial.println(yaw);
                    */
                }
              int packetSize = udp.parsePacket();
              if (packetSize) {
                Serial.print("Received packet of size ");
                Serial.println(packetSize);
                //Serial.print("From ");
                //IPAddress remoteIp = udp.remoteIP();
                //Serial.print(remoteIp);
                //Serial.print(", port ");
                //Serial.println(udp.remotePort());
                // read the packet into packetBufffer
                int len = udp.read(packetBuffer, 255);
                 pcaselect(1);
                if (len > 0) {
                  packetBuffer[len] = 0;
                }
                Serial.println("Contents:");
                Serial.println(packetBuffer);
                if (strcmp(packetBuffer, "Gingivitis") == 0) {
                      drv.go();
                     // playMicroVibration(intensity1);
                      playVibrationType(intensity1, 14); // Strong Buzz
                  } else if (strcmp(packetBuffer, "Tooth Decay") == 0) {
                      drv.go();
                     //  playMicroVibration(intensity1);
                      playVibrationType(intensity1, 52); // Pulsing Strong
                  } else if (strcmp(packetBuffer, "Periodontitis") == 0) {
                      drv.go();
                     //  playMicroVibration(intensity1);
                     playVibrationType(intensity1, 7); // Soft Bump
                  } else if (strcmp(packetBuffer, "Necrotic Pulp") == 0) {
                      drv.go();
                      playVibrationType(intensity1, 13); // Soft Fuzz
                  } else {
                      // Default or unknown packet handling
                      Serial.println("Unknown packet received");
                  }


              }

            }
                 /*Threshold_accel = acceleration;
                  Threshold_accel_up = Threshold_accel + 100;
         
                   if (acceleration >= (threshold) && acceleration <= (Threshold_accel + 300))
                    {
                      pcaselect(1); pcaselect(2);
         
                      drv.go();
                      playMicroVibration(intensity1);
                      int inByte = Serial.read();
      
                       if(inByte == 'b')
                       {
                        Serial.println("Breaking from the loop");
                        break;
                       } //end of if
         
                    }*/
            } //end of myLSM.checkStatus
          //pcaselect (0)
       
        }  //end of while
        break;
      case '2':
        Serial.println("Orientation Mode Demonstration");
         // define port on PCA9548
         
        while(1){
          pcaselect(0);
         {
          if (myLSM.checkStatus())
          { myLSM.getAccel(&accelData);
                      
          // Calculate orientation
          roll = atan2(accelData.yData, accelData.zData) * 180.0 / PI;
          pitch = atan2(-accelData.xData, sqrt(accelData.yData * accelData.yData + accelData.zData * accelData.zData)) * 180.0 / PI;
          Serial.print("Orientation: ");
           Serial.print("Roll: "); Serial.print(roll, 2);
            Serial.print(" Pitch: "); Serial.println(pitch, 2);
             // Normalize the pitch value to be within 0 to 360 degrees
           //  pitch += 90;
            // roll +=90;
            // Normalize the roll value to 0-360 degrees
             roll = fmod(roll + 360.0, 360.0);

             // Normalize the pitch value to 0-360 degrees
            pitch = fmod(pitch + 360.0, 360.0);
              
          // Calculate yaw from gyroscope data
          // static float yaw = 0;
           static unsigned long lastTime = 0;
           unsigned long currentTime = millis();
           float deltaTime = (currentTime - lastTime) / 1000.0;
           lastTime = currentTime;
           yaw += gyroData.zData * deltaTime;
           Serial.print(" Yaw: "); Serial.print(yaw, 2);
          // re-initialisation 
            up_roll = roll; up_pitch = pitch;
           Serial.print("UP_Roll: "); Serial.print(up_roll, 2);
           Serial.print("UP_ Pitch: "); Serial.println(up_pitch, 2);
          // Control the ERM motor based on orientation

            myLSM.getGyro(&gyroData);
            //myLSM.enableBlockDataUpdate();
            myLSM.setAccelDataRate(LSM6DSV16X_ODR_AT_7Hz5);

          combinegyroDataMagnitude = sqrt(abs(gyroData.xData) * abs(gyroData.xData) + abs(gyroData.yData) * abs(gyroData.yData) + abs(gyroData.zData) * abs(gyroData.zData));
          Serial.print("CombinegyroDataMagnitude");  Serial.print(combinegyroDataMagnitude);
         //}
    
          float thre_roll = up_roll, thre_pitch = up_pitch, thre_yaw = yaw;

          // Call the function to trigger the appropriate vibration effect
  triggerVibrationEffect(roll, pitch);

          //if (abs(roll) > (thre_roll-90) && abs(roll) <= thre_roll || abs(pitch) > (thre_pitch +90) && abs(pitch) <=  (thre_pitch))
          //if (abs(roll) > -30 && abs(roll) < 30 && abs(pitch) > 30 && abs(pitch) < 90)
          // Region 1: Front (Roll: 330 to 30, Pitch: 30 to 90)
         /* if ((roll > 330 || roll < 30) && pitch > 30 && pitch < 90) 
           {
            pcaselect(1);  pcaselect(2);
            Serial.println("--------generate effect 1 ----------");
           // uint8_t intensity = mapOrientationToIntensity(roll, pitch);
            // Set the intensity
               drv2.setRealtimeValue(intensity2);
              drv.setWaveform(0, 5); 
              drv.setWaveform(0, 90);  // Effect 2
              drv.setWaveform(0, 90);  // Effect 2
              drv.setWaveform(1, 0);  // End of effects 
            }
          // else if (abs(thre_roll - 90) > abs(thre_roll-180) && abs(roll) <= abs(thre_roll-270) || abs(thre_pitch +90) > abs(thre_pitch +180) && abs(pitch) <=  abs(thre_pitch-270))
          // Region 2: Back (Roll: -30 to +30, Pitch: 90 to 150)
         // else if (abs(roll) > -30 && abs(roll) < 30 && abs(pitch) > 90 && abs(pitch) < 150)
          // Region 2: Back (Roll: 150 to 210, Pitch: 90 to 150)
        //else if (roll > 150 && roll < 210 && pitch > 90 && pitch < 150)
        // Region 2: Back (Roll: 150 to 210, Pitch: 90 to 150)
           else if (roll > 150 && roll < 210 && pitch > 90 && pitch < 150)
            {
              pcaselect(1);  pcaselect(2);
              Serial.print("-----------generate effect 2---------- ");
            //  uint8_t intensity = mapOrientationToIntensity(roll, pitch);
            // Set the intensity
            drv2.setRealtimeValue(intensity1);
             drv2.setWaveform(0, 30);  
             drv2.setWaveform(0, 30);  
             drv2.setWaveform(0, 30);  
            }
         
          //else if (abs(roll) >= (thre_roll + 45) && abs(pitch) >= (thre_pitch + 45) || abs(pitch) >= (thre_pitch + 60) )  
          // Region 3: Left side (Roll: 30 to 90, Pitch: 60 to 120)
          // else if (abs(roll) > 30 && abs(roll) < 90 && abs(pitch) > 60 && abs(pitch) < 120)
          // Region 3: Left side (Roll: 60 to 120, Pitch: 60 to 120)
  else if (roll > 60 && roll < 120 && pitch > 60 && pitch < 120)
           {
          pcaselect(1);  pcaselect(2);
           Serial.print("-----------generate effect 3---------- ");
          //uint8_t intensity = mapOrientationToIntensity(roll, pitch);
            // Set the intensity
         drv.go();
         // playMicroVibration(intensity1);
            drv.setRealtimeValue(intensity1);
          //      drv.go();
     // Control Roll Motor         controlMotor(drv, roll);
          drv.setWaveform(0, 117);  // Effect 2
          drv.setWaveform(2, 117);
          drv.setWaveform(0, 117);
          drv.setWaveform(1, 0);  // End of effects
          }
      
     // else if((thre_roll >= up_roll && thre_roll <= (up_roll + 10)) && (thre_pitch >= up_pitch && thre_pitch <= (up_pitch + 10))) 
     // Region 4: Right side (Roll: -90 to -30, Pitch: 60 to 120)
     // else if (abs(roll) > -90 && abs(roll) < -30 && abs(pitch) > 60 && abs(pitch) < 120) 
     // Region 4: Right side (Roll: 240 to 300, Pitch: 60 to 120)
  else if (roll > 240 && roll < 300 && pitch > 60 && pitch < 120){
      pcaselect(1); pcaselect(2);
      Serial.print("-----------generate effect 4---------- ");
      // Condition 2: Moderate Roll or Pitch
         drv.setWaveform(0, 117);  // Effect 2
          drv.setWaveform(0, 117); 
         drv.setWaveform(1, 0);  // End of effects
      } 
      // Region 5: Upside down (Roll: -150 to +150, Pitch: 150 to 210)
      // else if (abs(pitch) > 150 && abs(pitch) < 210)
       // Region 5: Upside down (Pitch: 150 to 210)
  else if (pitch > 150 && pitch < 210){
         Serial.print("-----------generate effect 5---------- ");
      pcaselect(1);pcaselect(2);

       // Condition 3: Large Roll or Pitch
       drv.setWaveform(0,117);  // Effect 3
       // Series of softer pulses to simulate smooth gear engagement
      //drv.setWaveform(1, 6);  // Effect number 6: Transition Click
         drv.setWaveform(2, 6);  // Repeating to create a series of transitions
         drv.setWaveform(3, 6);  
         drv.setWaveform(4, 0);  // End the effect sequence
      } 
       // Region 6: Upright (Pitch: 0 to 30 or 330 to 360)
  else if (pitch > 0 && pitch < 30 || pitch > 330 && pitch < 360) {
       pcaselect(1); pcaselect(2);
       Serial.print("-----------generate effect 6---------- ");
      // Condition 4: Small Yaw
      drv.setWaveform(0,117);  // Effect 4
      // Series of softer pulses to simulate smooth gear engagement
      // drv.setWaveform(1, 6);  // Effect number 6: Transition Click
      drv.setWaveform(2, 6);  // Repeating to create a series of transitions
      drv.setWaveform(3, 6);  
       drv.setWaveform(4, 0);  // End the effect sequence
     } 
    /* else if (abs(yaw) > 20) {
      pcaselect(1);
      // Condition 5: Large Yaw
      // Initial strong pulse to simulate gear engagement
      drv.setWaveform(0, 117); // Effect number 47: Strong Click
      // Series of softer pulses to simulate smooth gear engagement
      drv.setWaveform(1, 6);  // Effect number 6: Transition Click
     drv.setWaveform(2, 6);  // Repeating to create a series of transitions
     drv.setWaveform(3, 6);  
     drv.setWaveform(4, 0);  // End the effect sequence
     } 
     else {
    // No significant movement
     pcaselect(1);
     Serial.print("-----------generate NO EFFECT---------- ");
   
    drv.setWaveform(1, 0);
    } */

   drv.go();  // Start the effect

          
             // controlERM(roll, pitch);
             // if
           
            int inByte = Serial.read();
            if(inByte == 'b')
            {
              Serial.println("Breaking from the loop");
              break;
            }
        }
         }
      }

        break;
        case '3':
/////////////////
        Serial.println("Pulse Mode Demonstration");
         // define port on PCA9548
         
        while(1){
         // intensity =50;
         float dist = calDistance();
          Serial.println(dist);
          if(dist > 31 && dist <= 36)
          
         {
          pcaselect(1); pcaselect(2);
          playMicroVibration(100);
          Serial.println("-------------Pulse Mode Demonstration---------");
          createPulsedEffect();
          delay(400);
         } 
     else {
    // No significant movement
    drv.setWaveform(0, 0);  // No effect
    }

   //drv.go();  // Start the effect

          
             // controlERM(roll, pitch);
             // if
           
            int inByte = Serial.read();
            if(inByte == 'b')
            {
              Serial.println("Breaking from the loop");
              break;
            }
        }
     // }
///////
break;
        case '4':
        
        Serial.println("Temperature-based control Mode DemonstrationXXXX");
         // define port on PCA9548
         
        while(1){
         // intensity =50;
         float temp = tempF();
        // sensors.requestTemperatures();
        //float temperatureC = sensors.getTempCByIndex(0);
        // float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
         //float temp = calDistance();
       //  Serial.print("Temperature in Celsius : "); Serial.print(temperatureC); Serial.print(" C");
        // Serial.print("\tTemperature in Fahrenheit : "); Serial.print(temperatureF); Serial.println(" F");
        Serial.println(temp);
         pcaselect(2);
          //if(temperatureF >= 70 && temperatureF <= 100 )
          if(temp >= 70 && temp <= 100 )
          {
           Serial.print("\tEntered into safe temperature loop: "); Serial.print(temp); Serial.println(" F");
            drv.go();  // Start the effect
          playMicroVibration(30);
         // createPulsedEffect();
         // delay(400);
         } 
     else {
    // No significant movement
    drv.setWaveform(0, 0);  // No effect
     Serial.print("\tNot safe temp range: No EFFECT ");
    }

   //drv.go();  // Start the effect

          
             // controlERM(roll, pitch);
             // if
           
            int inByte = Serial.read();
            if(inByte == 'b')
            {
              Serial.println("Breaking from the loop");
              break;
            }


        }
  
////////////
      break;  
      case '5':
        Serial.println("Distance Mode Demonstration");
        pcaselect(2);
        while(1){
          uint8_t intensity1 = 40;
          float dist = calDistance();
          Serial.println(dist);
          if(dist > 31 && dist <= 33)
          {
            pcaselect(1); pcaselect(2);
          playMicroVibration(50);
      // hapticEffect0 = map(dist, 20, 88, 99, 121);
       hapticEffect = constrain(hapticEffect, 111, 112);

       //drv.setRealtimeValue(hapticEffect0);
     drv.setWaveform(1,hapticEffect);
      drv.go();
      delay(5);
      drv.stop();
          }
      int inByte = Serial.read();
      if(inByte == 'b')
      {
        Serial.println("Breaking from the loop");
        break;
      }
        }

        break;
      default:
        Serial.println("Mode Not supported");
        break;  
    }
  }
 }
 

  }

  
/*void controlMotor(Adafruit_DRV2605 &drv, float angle) {
  // Map angle to a vibration pattern
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG);
  int effect = map(abs(angle), 0, 90, 117, 123);  // Effect range 1-123
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);  // End of effects
  drv.go();
} */




float combineDataVectorMagnitude(float gyroX, float gyroY, float gyroZ) {
    return sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
}

float combineDataRMS(float gyroX, float gyroY, float gyroZ) {
    return sqrt((gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ) / 3.0);
}

float combineDataSumAbsolute(float gyroX, float gyroY, float gyroZ) {
    return abs(gyroX) + abs(gyroY) + abs(gyroZ);
}

void generateWaveforms() {
  for (int i = 0; i < numSamples; i++) {
    float t = i * frequency;
    rollWaveform[i] = rollAmplitude * sin(t);
    pitchWaveform[i] = pitchAmplitude * sin(t + PI / 4);
    yawWaveform[i] = yawAmplitude * sin(t + PI / 2);
  }
}

void triggerHapticFeedback(float rollIntensity, float pitchIntensity, float yawIntensity) {
  drv.setWaveform(0, rollIntensity); // Roll effect
  drv.setWaveform(1, pitchIntensity); // Pitch effect
  drv.setWaveform(2, yawIntensity); // Yaw effect
  drv.setWaveform(3, 0); // End of sequence

  drv.go();
}

