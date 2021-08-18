#include<Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <EEPROM.h>
#include <mcp2515.h>

#define IDLE_AMPS_VALUE -0.05

// TO check whether eeprom value is not a number
#define isnan(n) (n!=n)?true:false

MCP2515 mcp2515(10);
MCP2515 bmsside(9);

struct can_frame canMsg;
struct can_frame canMsg1;
struct can_frame canMsg2;

#define ledpn 13
float amps_true = 0.0;
float voltage_true = 0.0;
float bms_amps_true = 0.0;
float bms_voltage_true = 0.0;
int soc=0;
int voltage_true_a, voltage_true_b, voltage_true_fin;

unsigned long a, b, c;
long double calc, wattsec, ampsec;

float Voltage = 0.0;
float watts;
double watthour = 0;
double amphour = 0;
double watthour_rom = 0;
double amphour_rom = 0;
double watthour_preset = 3758;
double amphour_preset = 52200;
byte amps_true_send_a;
byte amps_true_send_b;
byte amps_true_send_c;
byte amps_true_send_d;
long amphour_preset_a, amphour_preset_b, amphour_preset_c, amphour_preset_d;
long watthour_preset_a, watthour_preset_b, watthour_preset_c, watthour_preset_d;

long watthour_a, watthour_b, watthour_c, watthour_d, watthour_fin;
long amphour_a, amphour_b, amphour_c, amphour_d, amphour_fin;

unsigned long canMessageTimer, canMessageTimerDelay = 50;

int security = 3;
int i;
double offsetTemp = 0;
Adafruit_ADS1115 ads(0x49);

bool ignitionStatus = true;
bool isACOn = true;
bool needToCalibrate = false;
double calibrationOffset = 0.1937;
unsigned long calibrationTimer = 0;
unsigned long calibrationInterval = 100;
float calibrationValues[15];
int calibrationIndex = 0;

void sender();
void commands();
void bmsvalues();
void session();
void wattcalc();
void checkCurrentCalibration();

void setup() {
  Serial.begin(9600);
  // watthour=watthour_preset;
  //Presetting code
//  EEPROM.put(20, watthour_preset);
//  EEPROM.put(40, amphour_preset);
//  watthour_preset = 2462;
//  amphour_preset = 41608;
//  EEPROM.put(0, watthour_preset);
//  EEPROM.put(10, amphour_preset);
//  while(true){}

  double watthr_rom_temp;
  double amphr_rom_temp;
  EEPROM.get( 0, watthr_rom_temp);
  EEPROM.get(10, amphr_rom_temp);
  if(!isnan(watthr_rom_temp)) {
    watthour_rom = watthr_rom_temp;
  } else {
    watthour_rom = 0;
    EEPROM.put( 0, 0);
  }
  if(!isnan(amphr_rom_temp)) {
    amphour_rom = amphr_rom_temp;
  } else {
    amphour_rom = 0;
    EEPROM.put( 10, 0);
  }

  double watthr_preset_temp;
  double amphr_preset_temp;
  EEPROM.get( 20, watthr_preset_temp);
  EEPROM.get(40, amphr_preset_temp);

  if(!isnan(watthr_preset_temp)) {
    watthour_preset = watthr_preset_temp;
  } else {
    EEPROM.put(20, watthour_preset);
  }

  if(!isnan(amphr_preset_temp)) {
    amphour_preset = amphr_preset_temp;
  } else {
    EEPROM.put(40, amphour_preset);
  }

  double offsetTemp = 0;
  EEPROM.get(50, offsetTemp);
  if(!isnan(offsetTemp)) {
    calibrationOffset = offsetTemp;
  } else {
    EEPROM.put(50, calibrationOffset);
  }
  //Serial.print("Current Calibration Offset = ");
  //Serial.println(calibrationOffset);
  //watthour_rom=8736;
//  amphour_rom = 46000;
  ads.begin();


  mcp2515.reset();
  bmsside.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  bmsside.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  bmsside.setNormalMode();

  // Initialize Outputs
  pinMode(ledpn, OUTPUT);
  digitalWrite(ledpn, HIGH);
  delay(500);
  digitalWrite(ledpn, LOW);

  // Initialize CAN messages
  canMsg1.can_id  = 0x095;
  canMsg1.can_dlc = 8;

  canMsg2.can_id  = 0x096;
  canMsg2.can_dlc = 7;

  delay(1000);
  canMessageTimer = millis();

}

void loop()
{

  // Put watthour and amphour in EEPROM
  EEPROM.put( 0, watthour);
  EEPROM.put(10, amphour);
  // bmsvalues();
  wattcalc();
  //secure();
  session();
  checkCurrentCalibration();

  // Keep sending the CAN message after some delay
  if(millis() - canMessageTimer >= canMessageTimerDelay) {
    bmsvalues();
    sender();
    canMessageTimer = millis();
  }
  
  commands();

}

// Code to Callibrate CT if values are Drifted too much
void checkCurrentCalibration() {

  // return in no need to calibrate
  if(!needToCalibrate) {
    return;
  }

  // Do not attempt to calibrate if ignition is ON
  if(ignitionStatus) {
    Serial.println("Ignition On Detected skipping Calibration...");
    calibrationInterval = millis();
    calibrationIndex = 0;
    return;
  }

  // Do not attempt to calibrate if charger is ON
  if(isACOn) {
    Serial.println("Charger Detected skipping Calibration...");
    calibrationInterval = millis();
    calibrationIndex = 0;
    return;
  }

  // Useless... just to debug that it has started to attempt calibration
  if(calibrationIndex == 0) {
    Serial.println("Attempting to Calibrate...");
  }

  //G9et the current value after some interval
  if(abs(millis() - calibrationTimer) >= calibrationInterval) {
    
    // store the current value at that instance in array and reset the timer
    calibrationTimer = millis();

    // subtract previously calibrated offset from current to get the value without any offset
    calibrationValues[calibrationIndex] = amps_true - calibrationOffset;
    //Increment the array pointer
    calibrationIndex = (calibrationIndex + 1) % 15;

    // Stuff to do when we get 10 current values. i.e. the array is full
    if(calibrationIndex >= 10) {
      // get average of all the values in the array
      offsetTemp = 0;
      for(i = 0; i < 10; i++) {
        offsetTemp += calibrationValues[i];
      }
      offsetTemp = offsetTemp / 10.0;

      // current_value_with_offset = current_value_without_offset + offset
      // IDLE_VALUE = value we got by averaging the array + offet we need to get the idle value
      // -> offet we need to get the idle value = IDLE_VALUE - value we got by averaging the array
      offsetTemp = IDLE_AMPS_VALUE - offsetTemp;
      calibrationOffset = offsetTemp;
      // store the offset value in the EEPROM
      EEPROM.put(50, calibrationOffset);
      calibrationIndex = 0;                   // reset the array pointer
      needToCalibrate = false;                // clear the need to calibrate flag
    }
  }
}


void sender()
{
  //split the voltage values so that we can send it on CAN (8 bit ka limit he)
  voltage_true_fin = voltage_true * 100;
  voltage_true_a = voltage_true_fin & 0b0000000011111111;
  voltage_true_b = (voltage_true_fin >> 8) & 0b0000000011111111;

  // again same for watthour
  watthour_fin = watthour * 100;
  watthour_a = (watthour_fin & 0x000000FF);
  watthour_b = (watthour_fin & 0x0000FF00) >> 8;
  watthour_c = (watthour_fin & 0x00FF0000) >> 16;
  watthour_d = (watthour_fin & 0xFF000000) >> 24;

 // don't know ye kya he contact Elnino Rosario or Rahul Maurya
  amphour_fin = amphour * 100;

  // to monitor values using bluetooth
  // Serial.print(" Amp hour:   ");
  // Serial.print(amphour_fin);

  //printnaidekhna
  Serial.print("V:   ");
  Serial.print(voltage_true);
  Serial.print(" ");
  Serial.print("A:  ");
  Serial.print(amps_true);
  Serial.print(" ");

  Serial.print("whr: ");
  Serial.print(watthour);
  Serial.print("mah:   ");
  Serial.println(amphour);

  //split the amphour values so that we can send it on CAN (8 bit ka limit he)
  amphour_a = (amphour_fin & 0x000000FF);
  amphour_b = (amphour_fin & 0x0000FF00) >> 8;
  amphour_c = (amphour_fin & 0x00FF0000) >> 16;
  amphour_d = (amphour_fin & 0xFF000000) >> 24;

  // assign the values to can data bytes
  canMsg1.data[0] = watthour_a;
  canMsg1.data[1] = watthour_b;
  canMsg1.data[2] = watthour_c;
  canMsg1.data[3] = watthour_d;
  canMsg1.data[4] = amphour_a;
  canMsg1.data[5] = amphour_b;
  canMsg1.data[6] = amphour_c;
  canMsg1.data[7] = amphour_d;

  canMsg2.data[0] = voltage_true_a;
  canMsg2.data[1] = voltage_true_b;

//split the current values so that we can send it on CAN (8 bit ka limit he) (idar wapis kiya he coz upar wale method me issue tha overflow ka)
  float amps_true_temp = amps_true * 1000.0;
  amps_true_send_a = (int32_t)amps_true_temp & 0xFF;
  amps_true_send_b = ((int32_t)amps_true_temp & 0xFF00) >> 8;
  amps_true_send_c = ((int32_t)amps_true_temp & 0xFF0000) >> 16;
  amps_true_send_d = ((int32_t)amps_true_temp & 0xFF000000) >> 24;
//  //Serial.print(" Current Fin A:  ");
//  //Serial.print(amps_true_send_a);
//  //Serial.print(" ");
//  //Serial.print(" Current Fin B:  ");
//  //Serial.print(amps_true_send_b);
//  //Serial.print(" ");
//  //Serial.print(" Current Fin C:  ");
//  //Serial.print(amps_true_send_c);
//  //Serial.print(" ");
//  //Serial.print(" Current Fin D:  ");
//  //Serial.print(amps_true_send_d);
//  //Serial.print(" ");
//  //Serial.print(" Current Fin Reverted:  ");
//  //Serial.print((float)(amps_true_send_a | (amps_true_send_b << 8) | (amps_true_send_c << 16) | (amps_true_send_d << 24)) /1000.0);
//  Serial.println(" ");
// assign the values to can data bytes
  canMsg2.data[2] = amps_true_send_a;
  canMsg2.data[3] = amps_true_send_b;
  canMsg2.data[6] = amps_true_send_c;
  canMsg2.data[5] = amps_true_send_d;

  //  canMsg2.data[5] = ;
  //  canMsg2.data[6] = ;
  //  canMsg2.data[7] = ;

  // send the messasges on CAN
  mcp2515.sendMessage(&canMsg1);
  mcp2515.sendMessage(&canMsg2);



  //  amphour=amphour+0.001;
    //watthour=watthour_preset;
}


void bmsvalues()
{
  if (bmsside.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id==0x98FF28F4){


      for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data

      bms_amps_true=(5000-((256.0*canMsg.data[3])+canMsg.data[2]))/10.0;//BMS wala ncurrent
      // Serial.print(bms_amps_true);
      // Serial.print("  ");
      bms_voltage_true=(((256.0*canMsg.data[5])+canMsg.data[4]))/10.0;//BMS wala Voltage
      // Serial.println(bms_voltage_true);
      soc=canMsg.data[1];//SOC
      // Serial.println(soc);

      }
    }
  }
}


//Function to read CAN data
void commands()
{


  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
  {
    // to change the preset values
    if(canMsg.can_id == 0x97) {
      amphour_preset_a = canMsg.data[0];
      amphour_preset_b = canMsg.data[1];
      amphour_preset_c = canMsg.data[2];
      amphour_preset_d = canMsg.data[3];
      watthour_preset_a = canMsg.data[4];
      watthour_preset_b = canMsg.data[5];
      watthour_preset_c = canMsg.data[6];
      watthour_preset_d = canMsg.data[7];

      double amphour_temp, watthour_temp;
      amphour_temp = amphour_preset_a + (amphour_preset_b << 8) + (amphour_preset_c << 16) + (amphour_preset_d << 24);
      watthour_temp = watthour_preset_a + (watthour_preset_b << 8) + (watthour_preset_c << 16) + (watthour_preset_d << 24);
      
      watthour_preset = watthour_temp;
      amphour_preset = amphour_temp;

      watthour_rom = watthour_preset;
      amphour_rom = amphour_preset;
      
      EEPROM.put(20,watthour_preset);
      EEPROM.put(40,amphour_preset);
    }

    // reading ignition and charging status for calibration purposes
    if(canMsg.can_id == 0x072) {
      ignitionStatus = canMsg.data[1] & 0x01;
      isACOn = canMsg.data[1] & 0x02;

      //Calibration related
      // check whether ignition and charging is not on
      if(!ignitionStatus && !isACOn) {
        // if both of them are not on then check the current value, if the values are outside the threshold then we need to calibrate
        if(amps_true < IDLE_AMPS_VALUE*2.5 || amps_true > 0) {
          if(!needToCalibrate) {
            needToCalibrate = true;
            calibrationTimer = millis();
            calibrationIndex = 0;
          }
        }
      }
    }

    // don't know this contact Elnino Rosario or Rahul Maurya
    // probably to clear/preset the amphour values using CAN, and the relay stuff is not used now
    if (canMsg.can_id == 0x045)
    {

      if (canMsg.data[1] == 5)
      {
        wattsec = 0;
        ampsec = 0;
        watthour_rom = 0;
        amphour_rom = 0;
        amphour = 0;
        watthour = 0;
        EEPROM.put(0,0);
        EEPROM.put(10,0);
        //Serial.println("you asked for this on CAN .............................................. the data is gone forever..............................................");
      }
      if (canMsg.data[1] == 6)
      {
        EEPROM.put(0,watthour_preset);
        EEPROM.put(10,amphour_preset);
        watthour_rom = watthour_preset;
        amphour_rom = amphour_preset;
        wattsec = 0;
        ampsec = 0;
        watthour = watthour_preset;
        amphour = amphour_preset;
        //Serial.println("Presetted aka full charge.....");
      }
      if (canMsg.data[1] == 7)
      {
        EEPROM.put(20, watthour);
        EEPROM.put(40, amphour);
        //Serial.println("Preset sets to this value from Can");
      }

    }


  }
}

unsigned long timee;

// function to do stuff from serial monitor
void session()
{
  if (Serial.available() && security == 3)
  {
    i = Serial.read();

    // clear the amphour and watthour values
    if (i == 'c' && security)
    {
      wattsec = 0;
      ampsec = 0;
      watthour_rom = 0;
      amphour_rom = 0;
      amphour = 0;
      watthour = 0;
      EEPROM.put(0,0);
      EEPROM.put(10,0);
      //Serial.println("you asked for this .............................................. the data is gone forever..............................................");
    }

    // force calibrate (note: this won't force calibrate if ignition or charging is on, it will start calibratin after both of these are turned off)
    if (i == 'k' && security)
    {
      //Serial.println("--------------------------------Forcing Current Calibration--------------------------------------------");
      //Serial.println("Calibration Will Start if the ignition is off!");
      needToCalibrate = true;
      calibrationTimer = millis();
      calibrationIndex = 0;

    }

    // set amphour and watthour values to the preset values
    if (i == 'p' && security)
    {
      EEPROM.put(0,watthour_preset);
      EEPROM.put(10,amphour_preset);
      watthour_rom = watthour_preset;
      amphour_rom = amphour_preset;
      wattsec = 0;
      ampsec = 0;
      watthour = watthour_preset;
      amphour = amphour_preset;
      //Serial.println("you asked for preset .............................................. the data is gone forever..............................................");

    }

    // no clue the purpose of this probably useless
    if (i == 'T' || i == 't')
    { security = 0;
      //Serial.println("your session has been terminated Sucessfully for further queries contact Earth Energy EV \n The readings obtained Were: \nVoltage Current Watthour and amphour Respectively");

    }
  }
}

double ampsArr[5];
int ampsArrPointer = 0;

// most important function to calculate all the values
void wattcalc()
{
  a = millis();
  c = a - b;
  b = a;
  uint16_t adc0, adc1, adcv; // we read from the ADC, we have a sixteen bit integer as a result

  // read the adc values
  // adc0 = current adc value
  // adc1 = adc of voltage reference given to CT
  // adcv = adc of voltage of battery
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adcv = ads.readADC_SingleEnded(2) + 15;

  // amps1 = ((Voltage * 1000.0 + 3.19) * 5) / 99.75;
  // amps1 = (Voltage + 124) * 5 / calibrationScale + calibrationOffset; // 280 - 0.91; // 160 + 0.95; 
//  double dummy = map(adc0, 11550.0, 15150.0, -300.0, 300.0);

  // take the difference of current adc value and reference value
  double raw = adc0;
  double rawVref = adc1;
  double diff = rawVref - raw;
//  double dummy = (13350 - raw) / 34.0;
//  double dummy = 0.0295*diff + 0.2204;

  // refer to CT_Calibration_data.xlsx for the derivation of this equation
  double dummy = 0.0000000006*diff*diff*diff + 0.0000006*diff*diff + 0.0294*diff + calibrationOffset;

//  if(dummy > 0) {
//    dummy += 0.14;
//  } else {
//    dummy += 0.1;
//  }
  
  // average out last 5 current values
  // ampsArr[ampsArrPointer] = dummy;
  // ampsArrPointer = (ampsArrPointer + 1) % 5;
  // double averagedAmps = 0;
  // for(int i = 0; i < 5; i++) {
  //   averagedAmps += ampsArr[i];
  // }
  // averagedAmps = averagedAmps / 5.0;

  amps_true = bms_amps_true; //idar
  // if(amps_true < 0.09 && amps_true > -0.15) {
  //   amps_true = -0.01;
  // }
  ////Serial.println(amps1);
//  if(amps_true > calibrationIgnitionOffValue + 0.5 || amps_true < calibrationIgnitionOffValue - 0.5) {
//    isCalibrated = false;
//  }
//  if ((amps1 >= -0.10 && amps1 <= 0.10 ) || amps1 > 300)
//  {
//    amps_true = 0;
//  }

  
  // read and convert voltage values from adc value to Volts
  // Voltage = ((adcv));
  // Voltage = Voltage ;
  // voltage_true = (Voltage - 13) * 30 / 7622;
  voltage_true = bms_voltage_true;// idar
  watts = voltage_true * amps_true;

  if ((Voltage <= 10))
    watts = 0;
  calc = c * watts;

  // calc = watt-millisecond
  // keep on adding calc into wattsec every loop
  // similar logic for amp-millisecond
  wattsec = wattsec + calc; //millisecond actually
  ampsec = ampsec + c * amps_true; //milli second actually

  // convert to watt-hour and milliamp-hour
  watthour = watthour_rom + wattsec / 3600000.0;
  amphour = amphour_rom + ampsec / 3600.0; //milliamp hour actually

 // constrain amp-hour and watt-hour in between 0 and the respective preset values.
  watthour = constrain(watthour, 0, watthour_preset);
  amphour = constrain(amphour, 0, amphour_preset);
}
