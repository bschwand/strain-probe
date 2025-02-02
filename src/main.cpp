#include <Arduino.h>
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

const int HX711_dout = 3; //PD3 mcu > HX711 dout pin, must be external interrupt capable!
const int HX711_sck = 16; //A2 pin 16, PC2 mcu > HX711 sck pin

const int PROBE_PIN = 13;
const int PROBE_ENABLE_PIN = 14;

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// variables stored in eeprom
float calibrationValue = 1.0;
float thresholdValue = 100.0;
char eepromProgrammed = 0;
const char eepromProgrammedFlag = 0xAE;

// eeprom variables addresses in eeprom
const int eepromProgrammedAdr = 0; // we set to 0xAE to signal the eeprom has been programmed with valid threshold and calibration
const int calVal_eepromAdress = sizeof(eepromProgrammed);
const int threshold_eepromAddress = calVal_eepromAdress + sizeof(calibrationValue);

int serialPrintInterval = 0; //increase value to slow down serial print activity
bool dumpEnabled = false;

unsigned long t = 0;
volatile boolean newDataReady;

// constants won't change. Used here to set a pin number:
const int ledPin = LED_BUILTIN;  // the number of the LED pin

// Variables will change:
int ledState = LOW;  // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change:
const long interval = 200;  // interval at which to blink (milliseconds)

// put function declarations here:
void dataReadyISR();
void calibrate();
void changeSavedCalFactor(int eepromAddress);

void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");
  pinMode(ledPin, OUTPUT);
  pinMode(PROBE_ENABLE_PIN, INPUT); // for debug to always enable probe, use INPUT_PULLUP otherwise INPUT
  pinMode(PROBE_PIN, OUTPUT);
  digitalWrite(PROBE_PIN, LOW);

  calibrationValue = 1.0;

#if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
#endif
  
  EEPROM.get(eepromProgrammedAdr, eepromProgrammed);
  // fetch values from eeprom if the eeprom flag is set
  if(eepromProgrammed == eepromProgrammedFlag){
    EEPROM.get(calVal_eepromAdress, calibrationValue);
    EEPROM.get(threshold_eepromAddress, thresholdValue);
  } else {
    // otherwise store default values in eeprom
    EEPROM.put(calVal_eepromAdress, calibrationValue);
    EEPROM.put(threshold_eepromAddress, thresholdValue);
    EEPROM.put(eepromProgrammedAdr, eepromProgrammedFlag);
  }
  LoadCell.begin(128);
  //LoadCell.setReverseOutput();
  //LoadCell.disableTareTimeout();
  unsigned long stabilizingtime = 2000; // tare precision can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }

  attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);
}

//interrupt routine:
void dataReadyISR() {
  if (LoadCell.update()) {
    newDataReady = 1;
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  // get smoothed value from the dataset and process probe pins:
  if (newDataReady) {
    float i = LoadCell.getData();
    newDataReady = 0;

  char probe_pin = LOW;
  char probe_enabled = LOW;
  static char previous_probe_enabled = LOW;

    if(digitalRead(PROBE_ENABLE_PIN) == HIGH){
      probe_enabled = HIGH;
      // do quick tare when the probe first gets enabled
      if(previous_probe_enabled == LOW)
        LoadCell.tareNoDelay();

      if(i > thresholdValue){
        digitalWrite(PROBE_PIN, HIGH);
        digitalWrite(ledPin, HIGH);
        probe_pin = HIGH;
      } else {
        digitalWrite(PROBE_PIN, LOW);
        digitalWrite(ledPin, LOW);
        probe_pin = LOW;
      }
    } else 
      digitalWrite(ledPin, LOW);

    previous_probe_enabled = probe_enabled;

    if (dumpEnabled && (millis() > t + serialPrintInterval)) {
      Serial.print("Probe: ");
      Serial.print((int)probe_pin);
      Serial.print("  Enabled: ");
      Serial.print((int)probe_enabled);
      Serial.print("  Load_cell : ");
      Serial.println(i);
      //Serial.println(millis() - t);
      t = millis();
    }
  }

  // receive command from serial terminal
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay(); //tare
    else if (inByte == 'r') calibrate(); //calibrate
    else if (inByte == 'c') changeSavedCalFactor(calVal_eepromAdress); //edit calibration value manually
    else if (inByte == 's') changeSavedCalFactor(threshold_eepromAddress); //edit threshold value manually
    else if (inByte == 'd') { dumpEnabled = !dumpEnabled;} // toggle weight dump on serial
    else if (inByte == '+') { serialPrintInterval += 100;} // increase weight dump interval
    else if (inByte == '-') { serialPrintInterval -= (serialPrintInterval == 0)? 0 : 100;} // decrease weight dump interval
    else Serial.print("t : tare\r\n"
                      "r : calibrate\r\n"
                      "c : manually change calibration factor\r\n"
                      "s : manually change threshold\r\n"
                      "d : toggle weight dump\r\n"
                      "+ : increase dump frequency\r\n"
                      "- : decrease dump frequency\r\n");
  }


  //check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
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
}

void changeSavedCalFactor(int eepromAddress) {
  float oldCalibrationValue;
  
  if (eepromAddress == calVal_eepromAdress)
    oldCalibrationValue = LoadCell.getCalFactor();
  
  if(eepromAddress == threshold_eepromAddress)
    oldCalibrationValue = thresholdValue;

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
        Serial.print("New value is: ");
        Serial.println(newCalibrationValue);
        if(eepromAddress == calVal_eepromAdress)
          LoadCell.setCalFactor(newCalibrationValue);
        if(eepromAddress == threshold_eepromAddress)
          thresholdValue = newCalibrationValue;
        _resume = true;
      }
    }
  }
  _resume = false;
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(eepromAddress);
  Serial.println("? y/n");
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(eepromAddress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(eepromAddress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(eepromAddress);
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }
  Serial.println("End change value");
  Serial.println("***");
}

