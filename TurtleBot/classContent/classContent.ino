#include "config.h"
#include "pitches.h"
#include "PlayPiratesTheme.h"
#include "IMU_Read_RollPitchYaw.h"

void warningBeep() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER, NOTE_A5, 200);
    delay(3000);
  }
  noTone(BUZZER);
}


void setup() {
  // put your setup code here, to run once:
  pinMode(BUZZER, OUTPUT);
  Serial.begin(115200);
  playPiratesTheme();  // 🎶 speel bij opstart

  IMU.begin();
  pinMode( led_pin, OUTPUT );
}

void loop() {
  
  int adc_value;
  float vol_value;

  // put your main code here, to run repeatedly:
  adc_value = analogRead(BDPIN_BAT_PWR_ADC);

  Serial.print("BAT_PWR = ");
  Serial.print(adc_value/100);
  Serial.print("\t");
  vol_value = map(adc_value, 0, 1023, 0, 330*57/10);
  vol_value = vol_value/100;
  Serial.print(vol_value);
  Serial.println("V\t");

  if (vol_value <= 10.5) {
    Serial.println("low battery");
    warningBeep();
  }
  delay(500);

  // IMU CODE
  static uint32_t tTime[3];
  static uint32_t imu_time = 0;


  if( (millis()-tTime[0]) >= 500 )
  {
    tTime[0] = millis();

    digitalWrite( led_pin, led_tog );
    led_tog ^= 1;
  }

  tTime[2] = micros();
  if( IMU.update() > 0 ) imu_time = micros()-tTime[2];



  if( (millis()-tTime[1]) >= 50 )
  {
    tTime[1] = millis();

    Serial.print(imu_time);
    Serial.print("Roll:");
    Serial.print(IMU.rpy[0]);
    Serial.print("Pitch:");
    Serial.print(IMU.rpy[1]);
    Serial.print("Yaw:");
    Serial.println(IMU.rpy[2]);
  }


  if( Serial.available() )
  {
    char Ch = Serial.read();

    if( Ch == '1' )
    {
      Serial.println("ACC Cali Start");

      IMU.SEN.acc_cali_start();
      while( IMU.SEN.acc_cali_get_done() == false )
      {
        IMU.update();
      }

      Serial.print("ACC Cali End ");
    }
  }
 
}
