#ifndef ORIENTATION_EFFECTS_H
#define ORIENTATION_EFFECTS_H
#define PCAADDR 0x70
#include <Arduino.h>
//#include "Adafruit_DRV2605.h" //for Haptic driver
//#include "SparkFun_LSM6DSV16X.h"
//#include "TCA9548A.h" //
//SparkFun_LSM6DSV16X myLSM;

//TCA9548A I2CMux;                  // Address can be passed into the constructor
Adafruit_DRV2605 drv, drv1, drv2; // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
uint8_t intensity1 = 70, intensity2 =90;
//pasted pcaselect into orientationEffects from ino file
void pcaselect(uint8_t i) {
  if (i > 4) return;
 
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


// Function to trigger the appropriate vibration effect based on roll and pitch
void triggerVibrationEffect(float roll, float pitch) {
  // Region 1: Front (Roll: 330 to 30, Pitch: 30 to 90)
  if ((roll > 330 || roll < 30) && pitch > 30 && pitch < 90) {
    Serial.println("Vibration Effect 1: Front");
    // Code to trigger front vibration
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
  // Region 2: Back (Roll: 150 to 210, Pitch: 90 to 150)
  else if (roll > 150 && roll < 210 && pitch > 90 && pitch < 150) {
    Serial.println("Vibration Effect 2: Back");
    // Code to trigger back vibration
     pcaselect(1);  pcaselect(2);
              Serial.print("-----------generate effect 2---------- ");
            //  uint8_t intensity = mapOrientationToIntensity(roll, pitch);
            // Set the intensity
            drv2.setRealtimeValue(intensity1);
             drv2.setWaveform(0, 30);  
             drv2.setWaveform(0, 30);  
             drv2.setWaveform(0, 30); 
  }
  // Region 3: Left side (Roll: 60 to 120, Pitch: 60 to 120)
  else if (roll > 60 && roll < 120 && pitch > 60 && pitch < 120) {
    Serial.println("Vibration Effect 3: Left side");
    // Code to trigger left side vibration
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
  // Region 4: Right side (Roll: 240 to 300, Pitch: 60 to 120)
  else if (roll > 240 && roll < 300 && pitch > 60 && pitch < 120) {
    Serial.println("Vibration Effect 4: Right side");
    // Code to trigger right side vibration
    pcaselect(1); pcaselect(2);
      Serial.print("-----------generate effect 4---------- ");
      // Condition 2: Moderate Roll or Pitch
         drv.setWaveform(0, 117);  // Effect 2
          drv.setWaveform(0, 117); 
         drv.setWaveform(1, 0);  // End of effects
  }
  // Region 5: Upside down (Pitch: 150 to 210)
  else if (pitch > 150 && pitch < 210) {
    Serial.println("Vibration Effect 5: Upside down");
    // Code to trigger upside down vibration
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
    Serial.println("Vibration Effect 6: Upright");
    // Code to trigger upright vibration
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
  else {
    Serial.println("No Effect Vibration");
  }
}

#endif  // ORIENTATION_EFFECTS_H