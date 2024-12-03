/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   This example file shows how to calibrate the load cell and optionally store the calibration
   value in EEPROM, and also how to change the value manually.
   The result value can then later be included in your project sketch or fetched from EEPROM.

   To implement calibration in your project sketch the simplified procedure is as follow:
       LoadCell.tare();
       //place known mass
       LoadCell.refreshDataSet();
       float newCalibrationValue = LoadCell.getNewCalibration(known_mass);
*/
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif


//pins:
const int HX711_dout = 3; //mcu > HX711 dout pin
const int HX711_sck = 2; //mcu > HX711 sck pin

const int CS = 10;
const int SO = 12;
const int sck = 13;
float temperature = 0.0;
float L_temperature = 0.0;

int in1 = 6;
int in2 = 7;
int ena1 = 5;
int pwm = 0;
char count = 's';
int num = 0;
int Lnum = 0;
char state = 'l';

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

//Parameters
#define pwmPin 3
//const int pwmPin = 3;

int i = 0;
float c = 0;
char control;

float max_current = 0.77; //Change max current
int delay_time = 500; //Change delay time

void setup() {
  Serial.begin(115200); delay(10);
  pinMode(CS, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(sck, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena1, OUTPUT);

  //Init pwm output
  pinMode(pwmPin, OUTPUT);
  Serial.println();
  Serial.println("Starting...");

  LoadCell.begin();
  LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 5000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate(); //start calibration procedure
}

void loop() {
  if (Serial.available() > 0) {
    count = Serial.read();
  }
  static boolean newDataReady = 0;
  const int serialPrintInterval = 300; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena1, pwm);
      ReadTemp();


      float l = LoadCell.getData();
      Serial.print("   Load_cell : ");
      Serial.print(l);
      Serial.print(" ");
      Serial.print("Current Temperature : "); Serial.print(temperature); Serial.print(" ");

      newDataReady = 0;

      // if (pwm == 0) count = "up";
      // else if (pwm == 255) count = "down";

      if (num == 2) {
        while (temperature < 28) pwm = 0;
      }

      if (count == 'u') {
        pwm += 1;
        if (pwm >= 255) {
          pwm = 255;
        }
        Serial.print("pwm = ");
        Serial.println(pwm);
      }
      else if (count == 'd') {
        pwm -= 1;
        if (pwm <= 0) pwm = 0;
        Serial.print("pwm = ");
        Serial.println(pwm);

      }

      else if (count == 's') {
        pwm = 0;
        Serial.print("pwm = ");
        Serial.println(pwm);
      }

      t = millis();


      // check if last tare operation is complete
      if (LoadCell.getTareStatus() == true) {
        Serial.println("Tare complete");
      }
    }
  }
}

void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      if (Serial.available() > 0) {
        char inByte = Serial.read();
        if (inByte == 't') LoadCell.tareNoDelay();
      }
    }
    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");

  _resume = false;
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;

      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }

  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");
  Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
  Serial.println("***");

  delay(5000);
}

void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  Serial.println("***");
  Serial.print("Current value is: ");
  Serial.println(oldCalibrationValue);
  Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("New calibration value is: ");
        Serial.println(newCalibrationValue);
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }
  Serial.println("End change calibration value");
  Serial.println("***");
}




void ReadTemp() {
  int error = 0;
  int Value = 0;

  // 온도변환을 초기화 합니다.
  digitalWrite(CS, LOW);
  delay(2);
  digitalWrite(CS, HIGH);
  delay(2);

  // CS핀을 LOW로 만들어 값을 읽어옵니다
  digitalWrite(CS, LOW);

  // D15 더미 비트를 위한 클럭을 발생합니다.
  digitalWrite(sck, HIGH);
  delay(1);
  digitalWrite(sck, LOW);

  // D14 ~ D3의 값을 읽어옵니다.
  for (int i = 11; i >= 0; i--) {
    digitalWrite(sck, HIGH);
    Value += digitalRead(SO) << i;
    digitalWrite(sck, LOW);
  }

  // D2값을 읽어서 에러여부를 확인합니다.
  digitalWrite(sck, HIGH);
  error = digitalRead(SO);
  digitalWrite(sck, LOW);

  // D1, D0을 위한 클럭을 발생합니다.
  for (int i = 1; i >= 0; i--) {
    digitalWrite(sck, HIGH);
    delay(1);
    digitalWrite(sck, LOW);
  }

  // CS핀을 HIGH로 만들어 읽기를 끝냅니다
  digitalWrite(CS, HIGH);

  if (error != 0) {
    temperature = -CS;
  } else {
    temperature = Value * 0.25;
  }
}
