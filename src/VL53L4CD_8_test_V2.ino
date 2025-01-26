/*
VL53L4CD I2C Time of Flight Micro Distance Sensor and close-up sensor blocking detection
Fast ranging 10 ms loop robust and error-correcting functionality with detailed error descriptions
The program is coded with a focus on simplicity, ease of understanding and self-explanatory

REMEMBER! disconnet power briefly to all sensors and Arduino MCU for correct reset!

Please read the accompanying detailed documentation

Jørgen Bo Madsen - august 2024 - STM@jorgen-madsen.dk
*/

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
// Visual Studio code debug
//#include "avr8-stub.h"
//#include "app_api.h"

#define SerialPort true  // true: test and debug monitor information - false: production

// Sensor configuration --------------------------------------------------------------------

const bool YES = true, NO = false;  // Easy understanding
const bool VL53L4CD_Laser_I2C_ON[8]    = // Wired and connected to Nano
// sensor      0      1      2      3      4      5      6      7
          {  YES,    NO,   NO,     NO,    NO,    NO,   YES,    NO }; 
const word VL53L4CD_Min_Distance_MM[8] = // Min distance = 20 mm
// sensor      0      1      2      3      4      5      6      7
          {   25,    25,    25,    25,    25,    25,    25,    25 };
const word VL53L4CD_Max_Distance_MM[8] = // Max distance = 800 mm.
// sensor      0      1      2      3      4      5      6      7
          {  800,   800,   800,   800,   800,   800,   800,   800 };
//
// Show range, status, warning and error messages
// 
const bool I2C_wire_Debug_Enabled       = YES;  // List I2C Wire connected devices and other I2C debug information
const bool VL53L4CD_Print_Object_Detect = NO;  // Print distance and min-max zone, when object is detected: 
const bool VL53L4CD_Print_Range_Status  = NO;  // Print status information. Se www.ST.com UM2931 manual.
//
const bool VL53L4CD_Debug_Enabled[8]   = // Print ranging and error messages
// sensor      0      1      2      3      4      5      6      7
          {   NO,    NO,    NO,    NO,    NO,    NO,    NO,    NO }; 
//
// Close-up Blocking detection distance configuration --------------------------------------
//
const bool VL53L4CD_Blocking_Detect[8] =
// sensor      0      1      2      3      4      5      6      7
          {  YES,   YES,   YES,   YES,   YES,   YES,   YES,   YES };

// Close-up Blocking detection distance configuration --------------------------------------

// Optimized to 1,0 mm air gap and 0,35 mm 9H photo lens cover glass (1,35 mm distance in total)
// Close-up total sensor blocking: Matte black = 10 mm, Matte gray = 4 mm, Light yellow = 16 mm, Shiny white = 13 mm
// To lower blocing distance to 15 mm, increase air gap to 2 mm (2,35 mm distance in total)
const word  VL53L4CD_Max_BLock_Dist_MM = 19;   // Starting from top of cover glass (1,35 mm from senesor)
const word  VL53L4CD_Max_Block_Count   = 600;  // Number of loops to calculate average
// Using InterMeasurement (set to 0) will increase the blocking distance
//
// Sensor calibration ----------------------------------------------------------------------
//
// Advanced configuration. Read first www.ST.com UM2931 manual.
const bool  VL53L4CD_Offset_Calibrate   = NO;   // (YES / NO) Activate calibration mode
const short VL53L4CD_Offset_Dist_MM[8]  =       // Subtract from / add to range result
// sensor   0      1      2      3      4      5      6      7
       {   -8,    -8,    -8,    -8,    -8,    -8,    -8,    -8 };

const short VL53L4CD_Xtalk_Distance_MM  = 1000; // Maximum measured raging distance without under-ranging
const bool  VL53L4CD_Xtalk_Calibrate    = NO;   // (YES / NO) Activate calibration mode
const word  VL53L4CD_Xtalk_KCPS[8]      =       // Depends on Cover Glass type or no cover glass
// sensor   0      1      2      3      4      5      6      7
       {    0,     0,     0,     0,     0,     0,     0,     0 };

// Address and ID ---------------------------------------------------------------------------

const byte  VL53L4CD_I2C_Address  = 0x29;    // Adafruit and others 0x29 HEX / ST 0x52 HEX
const long  VL53L4CD_Sensor_ID    = 0xEBAA;  // Sensor ID HEX

// Error correction thresholds -------------------------------------------------------------

// Optimized to 1,0 mm air gap and 0,35 mm 9H photo lens cover glass (1,35 mm distance in total)
// Read www.ST.com AN4907 Application note document DM00326504 
const word  VL53L4CD_Max_Sigma_MM      = 6;   // Default 15
const word  VL53L4CD_Min_Signal_KPCS   = 2048;
//
// Ambien light. Too much ambient light may will entail incorrect mauserments.
const word  VL53L4CD_Max_Ambient_KCPS  = 8000; // Maximum ambient light 
const word  VL53L4CD_Max_Ambient_Count = 600;  // Number of loops to calculate average

// MUX configuration -----------------------------------------------------------------------

// TCA9548A 8-Channel I2C Multiplexer / PCA9548A - sensor channel 0 - 7 [8]
// TCA9546A 4-Channel I2C Multiplexer / PCA9546A - sensor channel 0 - 3 [4]
const bool  TCA_PCA_9548A = true, TCA_PCA_9546A = false;
const bool  TCA_PCA_I2C_Model   = TCA_PCA_9548A;
// YES: I2C MUX connecte to Nano. NO: first VL53L4CD directly connected to Nano.
const bool  TCA_PCA_I2C_MUX_ON  = NO;    // If MUX is off, Only sensor 0 is useable
const byte  TCA_PCA_I2C_Address = 0x70;  // Defalt I2C address 0x70 HEX

// Buzzer error sound ----------------------------------------------------------------------

const  byte Buzzer_Pin           = 8;   // D4 - Passive buzzer 1.000 - 5.000 Hz
const  byte Buzzer_Silent_Mode   = LOW; // Buzzer YL-44 is Silent on HIGH! KY-006 on LOW!
// 1.000 - 5.000 Hertz. 0 = no sound. Duration in milliseconds.
const  byte Error_Sound_Max_Step = 6;   // number og tone / not tone steps
const  word Error_Sound_Pitch_Hz[Error_Sound_Max_Step]    = { 3500,   0, 4250,   0, 3500,    0 };
const  word Error_Sound_Duration_MS[Error_Sound_Max_Step] = {  500, 500, 2000, 500,  500, 3000 };

// Global variables and structures ---------------------------------------------------------

const bool OK = true, Error = false;     // Easy understanding
bool  TCA_PCA_I2C_MUX_Status = OK;       // MUX start status OK
bool  VL53L4CD_Error_Status[8];          // I2C & range error handling
// Ambient light check
long  VL53L4CD_Average_Ambient_KCPS[8];  // Average ambient light level KCPS
word  VL53L4CD_Average_Ambient_Count[8]; // Average ambient count
// Blocking detection
long  VL53L4CD_Average_Block_MM[8];      // Average blocking distance 
word  VL53L4CD_Average_Block_Count[8];   // Average blocking count
// General
byte  VL53L4CD_Status        = 0;  // General function return result
word  Loop_Delay_Time        = 10; // Main loop for parallel processing
word  Offset_Delay_Time      = 0;  // I2C communication delay compensation
// Buzzer
byte  Error_Sound_Count      = 0;    // Number of repeatede sound loops
byte  Buzzer_Step_Count      = 0;    // Buzzer sound process
bool  Error_Sound_ON         = NO;   // Sound process loop
bool  Buzzer_Start_Step      = YES;  // Sound process loop step start
unsigned long Buzzer_Start_Time = 0;// Buzzer sound process
// Sensor processing
byte  VL53L4CD_Max_Count     = 0;  // Max number of lasers to process
byte  VL53L4CD_Active_Sensor = 0;  // Currently active laser [0-7]
byte  VL53L4CD_Max_ON_Index  = 0;  // Max number of activated (ON) lasers
byte  VL53L4CD_Loop_ON_Index = 0;  // Round-Robin laser ON reference Index 
byte  VL53L4CD_Laser_ON_List[8];   // Activated (ON) laser reference list
VL53L4CD_Result_t VL53L4CD_Result; // Measured data

// Sensor structures
VL53L4CD VL53l4CD_0(&Wire, 0);   // xhut = 0 => no pin used
VL53L4CD VL53l4CD_1(&Wire, 0);   // xhut = 0 => no pin used
VL53L4CD VL53l4CD_2(&Wire, 0);   // xhut = 0 => no pin used
VL53L4CD VL53l4CD_3(&Wire, 0);   // xhut = 0 => no pin used
VL53L4CD VL53l4CD_4(&Wire, 0);   // xhut = 0 => no pin used
VL53L4CD VL53l4CD_5(&Wire, 0);   // xhut = 0 => no pin used
VL53L4CD VL53l4CD_6(&Wire, 0);   // xhut = 0 => no pin used
VL53L4CD VL53l4CD_7(&Wire, 0);   // xhut = 0 => no pin used
// Laser pointer array
VL53L4CD* VL53L4CD_Lasers_List[8] = { &VL53l4CD_0, &VL53l4CD_1, &VL53l4CD_2, &VL53l4CD_3, \
                                      &VL53l4CD_4, &VL53l4CD_5, &VL53l4CD_6, &VL53l4CD_7 };

// Serial print slow down execution significantly and affects all functions that use time control.
#if SerialPort == true
  // Test
  #define PortPrint(x)   Serial.print(x)
  #define PortPrintln(x) Serial.println(x)
#else
  // Production
  #define PortPrint(x)
  #define PortPrintln(x)
#endif


// Setup -----------------------------------------------------------------------------------


void setup() {
  // Initialize serial for output.
  Serial.begin(250000);
  Serial.println("Starting...");
  pinMode(Buzzer_Pin, OUTPUT);
  //
  Initialize_I2C_bus();
  VL53L4CD_Calculate_Max_Lasers();
  VL53L4CD_Initialize_Error_check();
  VL53L4CD_Initialize_Read_Data();
  // 
  VL53L4CD_Check_I2C_Connected();
  VL53L4CD_Initialize_Lasers();
  VL53L4CD_Check_Parameters();
} // setup


// Main loop -------------------------------------------------------------------------------


void loop() {
  // Read laser number 0
  // bool Status = VL53L4CD_Read_One_Sensors(0);
  
  //Read all enabled lasers. Round-Robin process
  bool Status = VL53L4CD_Read_Sensors_Sequentially(); 
  if (Status == true) {
    Serial.print(millis()); Serial.print(F("  Laser no [")); 
    Serial.print(VL53L4CD_Active_Sensor); Serial.println(F("] Object detected"));
  }

  delay((unsigned long)(Loop_Delay_Time - Offset_Delay_Time));  // Defaul 10 milliseconds
  Error_Alarm_Sound();


  // Execute non blocking code ...

  /*
  // Global sensor reading data (vl53l4cd_api.h)
  Serial.print(F("range_status          = "));  Serial.println(VL53L4CD_Result.range_status);
  Serial.print(F("distance_mm           = "));  Serial.println(VL53L4CD_Result.distance_mm);
  Serial.print(F("ambient_rate_kcps     = "));  Serial.println(VL53L4CD_Result.ambient_rate_kcps);
  Serial.print(F("ambient_per_spad_kcps = "));  Serial.println(VL53L4CD_Result.ambient_per_spad_kcps);
  Serial.print(F("signal_rate_kcps      = "));  Serial.println(VL53L4CD_Result.signal_rate_kcps);
  Serial.print(F("signal_per_spad_kcps  = "));  Serial.println(VL53L4CD_Result.signal_per_spad_kcps);
  Serial.print(F("number_of_spad        = "));  Serial.println(VL53L4CD_Result.number_of_spad);
  Serial.print(F("sigma_mm              = "));  Serial.println(VL53L4CD_Result.sigma_mm);
  */
}


// I2C laser setup ------------------------------------------------------------------


void Initialize_I2C_bus() {
  char Hex_To_Char[5];
  Wire.begin();
  if (I2C_wire_Debug_Enabled == YES) {
    for (byte index = 0; index < 83; index++) { PortPrint(F("-")); } PortPrintln();
    PortPrintln(F("REMEMBER! disconnet power briefly to all sensors and Arduino MCU for correct reset!"));
    for (byte index = 0; index < 83; index++) { PortPrint(F("-")); } PortPrintln();
    snprintf(Hex_To_Char, sizeof(Hex_To_Char), "0x%02X", VL53L4CD_I2C_Address);
    PortPrint(F("VL53L4CD_I2C_Address [HEX]: ")); PortPrintln(Hex_To_Char);
    snprintf(Hex_To_Char, sizeof(Hex_To_Char), "0x%02X", TCA_PCA_I2C_Address);
    PortPrint(F("TCA_PCA_I2C_Address  [HEX]: ")); PortPrintln(Hex_To_Char);
    I2C_Detect();
  }
  Check_CA_PCA_I2C_MUX();
  Wire.end();
  Wire.begin();
} // Initialize_I2C_bus


void Check_CA_PCA_I2C_MUX() {
  if (TCA_PCA_I2C_MUX_ON == YES) {
    Wire.beginTransmission(TCA_PCA_I2C_Address);
    Wire.write(0);  // Close all TCA_PCA channels
    if (Wire.endTransmission() == 0) {
      TCA_PCA_I2C_MUX_Status = OK;
    }
    else {
      TCA_PCA_I2C_MUX_Status = Error;
      Set_Error_Sound_Count(1);
      Error_Sound_Count = 1;
      PortPrintln(F("ERROR: TCA_PCA_I2C_MUX_ON = YES - but no MUX is I2C connected!"));
    }
  }
} // Check_CA_PCA_I2C_MUX(


void VL53L4CD_Calculate_Max_Lasers() {
  if (TCA_PCA_I2C_MUX_ON == YES  && TCA_PCA_I2C_MUX_Status == OK) {
    if (TCA_PCA_I2C_Model == TCA_PCA_9548A) {
      VL53L4CD_Max_Count = 8;      
    }
    else { // TCA_PCA_9546A
      VL53L4CD_Max_Count = 4;
    }
  }
  else { // First sensor
      VL53L4CD_Max_Count = 1;
  }
  //
  for (byte Index = 0; Index < VL53L4CD_Max_Count; Index ++) {
    if (VL53L4CD_Laser_I2C_ON[Index] == YES) {
      VL53L4CD_Laser_ON_List[VL53L4CD_Max_ON_Index] = Index;
      VL53L4CD_Max_ON_Index++;
    }
    else {
      VL53L4CD_Laser_ON_List[VL53L4CD_Max_ON_Index] = 0;
    }
  }
} // Calculate_Max_Lasers


void VL53L4CD_Initialize_Error_check() {
  // Clear data!
  for (byte Init_Laser_No = 0; Init_Laser_No < VL53L4CD_Max_Count; Init_Laser_No++) {
    VL53L4CD_Error_Status[Init_Laser_No]         = OK;
    VL53L4CD_Average_Block_MM[Init_Laser_No]     = 0;
    VL53L4CD_Average_Block_Count[Init_Laser_No]  = 0;
    VL53L4CD_Average_Ambient_KCPS[Init_Laser_No] = 0;
    VL53L4CD_Average_Ambient_Count[Init_Laser_No]= 0;
  }
} // Initialize_Error_check


void VL53L4CD_Initialize_Read_Data() {
    // Global sensor reading data (vl53l4cd_api.h) */
    VL53L4CD_Result.range_status          = 0;
    VL53L4CD_Result.distance_mm           = 0;
    VL53L4CD_Result.ambient_rate_kcps     = 0;
    VL53L4CD_Result.ambient_per_spad_kcps = 0;
    VL53L4CD_Result.signal_rate_kcps      = 0;
    VL53L4CD_Result.signal_per_spad_kcps  = 0;
    VL53L4CD_Result.number_of_spad        = 0;
    VL53L4CD_Result.sigma_mm              = 0;
} // VL53L4CD_Initialize_Read_Data


void I2C_Detect() {
  byte I2C_status, I2C_address;
  char Hex_To_Char[5];
  short nDevices;
  PortPrintln(F("Scanning I2C address range HEX: 0x01 - 0x77"));
  nDevices = 0;
  for(I2C_address = 1; I2C_address < 127; I2C_address++ ) {
    // Did the device acknowledge to the address?
    Wire.beginTransmission(I2C_address);
    I2C_status = Wire.endTransmission();
    snprintf(Hex_To_Char, sizeof(Hex_To_Char), "0x%02X", I2C_address);
    if (I2C_status == 0)
    {
      PortPrint(F("  I2C device found at address [HEX]: ")); 
      PortPrintln(Hex_To_Char);
      nDevices++;
    }
    else if (I2C_status == 4) {
      PortPrint(F("Unknown error at address 0x / not connected / Pull up resistor missing"));
      PortPrintln(Hex_To_Char);
    }
  }
  if (nDevices == 0) {
    PortPrintln(F("  No I2C devices found!"));
  }
  PortPrintln(F("Done I2C scanning"));
} // I2C_Detect


void VL53L4CD_Check_I2C_Connected() {
  byte I2C_Check_Status = 0;
  byte Check_Laser_No = 0;
  for (byte Index = 0; Index < VL53L4CD_Max_ON_Index; Index++) {
    Check_Laser_No = VL53L4CD_Laser_ON_List[Index];
    if (TCA_PCA_I2C_MUX_ON == YES  && TCA_PCA_I2C_MUX_Status == OK) {
      TCA_PCA_Select_Channel(Check_Laser_No);
    }
    if (VL53L4CD_Error_Status[Check_Laser_No] == OK) {
      Wire.beginTransmission(VL53L4CD_I2C_Address);
      I2C_Check_Status = Wire.endTransmission();
      VL53L4CD_Print_Error(I2C_Check_Status, Check_Laser_No, 1);
      if (I2C_Check_Status == 0  && VL53L4CD_Debug_Enabled[Check_Laser_No] == YES) {
        VL53L4CD_Print_Header(Check_Laser_No); PortPrintln(F("I2C Connected OK")); 
      }
    }
  }  // for
}  // VL53L4CD_Check_I2C_Connected


void VL53L4CD_Initialize_Lasers() {
  byte Init_Laser_No = 0;
  for (byte Index = 0; Index < VL53L4CD_Max_ON_Index; Index++) {
    Init_Laser_No = VL53L4CD_Laser_ON_List[Index];
    if (VL53L4CD_Error_Status[Init_Laser_No] == OK) {
      if (TCA_PCA_I2C_MUX_ON == YES  && TCA_PCA_I2C_MUX_Status == OK) {
        TCA_PCA_Select_Channel(Init_Laser_No);
      }
      VL53L4CD_Set_I2C_Adresse(Init_Laser_No);
      VL53L4CD_Check_Sensor_ID(Init_Laser_No);
      VL53L4CD_Setup_And_Initialize_Sensor(Init_Laser_No);
      VL53L4CD_Calibrate_Sensor();  // Alle enabled lasers
      VL53L4CD_Setup_And_Start_Continuous_Ranging(Init_Laser_No);
      delay (10);  // Stability
    }
  }
} // VL53L4CD_Initialize_Lasers


void VL53L4CD_Set_I2C_Adresse(const byte Set_Laser_No) {
  // Set the samer address for all sensors (Adafruit = 0x29 / TS = 0x52)
  if (VL53L4CD_Error_Status[Set_Laser_No] == OK) {
    VL53L4CD_Status = VL53L4CD_Lasers_List[Set_Laser_No]->VL53L4CD_SetI2CAddress(VL53L4CD_I2C_Address);
    VL53L4CD_Print_Error(VL53L4CD_Status, Set_Laser_No, 2);
  }
} // VL53L4CD_Set_I2C_Adresse


void VL53L4CD_Check_Sensor_ID(const byte ID_Laser_No) {
  char Hex_To_Char[7];
  uint16_t VL53L4CD_ID = 0;
  if (VL53L4CD_Error_Status[ID_Laser_No] == OK) {
    VL53L4CD_Status = VL53L4CD_Lasers_List[ID_Laser_No]->VL53L4CD_GetSensorId(&VL53L4CD_ID);
    }
    if (VL53L4CD_Debug_Enabled[ID_Laser_No] == YES) {
       snprintf(Hex_To_Char, sizeof(Hex_To_Char), "0x%04X", VL53L4CD_ID);
      VL53L4CD_Print_Header(ID_Laser_No); PortPrint(F("Sensor ID: ")); PortPrintln(Hex_To_Char);
    }
    if (VL53L4CD_ID != VL53L4CD_Sensor_ID) {
      VL53L4CD_Print_Error(250, ID_Laser_No, 3);
  }
} // VL53L4CD_Check_Sensor_ID


void VL53L4CD_Setup_And_Initialize_Sensor(const byte Set_Laser_No) {
  // Initialize and setup sensor
  if (VL53L4CD_Error_Status[Set_Laser_No] == OK) { 
    VL53L4CD_Status = VL53L4CD_Lasers_List[Set_Laser_No]->VL53L4CD_SensorInit();  // NOT InitSensor!
    VL53L4CD_Print_Error(VL53L4CD_Status, Set_Laser_No, 4);
  } 
  // Set offset distance mm.
  if (VL53L4CD_Error_Status[Set_Laser_No] == OK) { 
    VL53L4CD_Status = VL53L4CD_Lasers_List[Set_Laser_No]->VL53L4CD_SetOffset(VL53L4CD_Offset_Dist_MM[Set_Laser_No]);
    VL53L4CD_Print_Error(VL53L4CD_Status, Set_Laser_No, 5);
  } 
  // Set crosstalk KCPS
  if (VL53L4CD_Error_Status[Set_Laser_No] == OK) { 
    VL53L4CD_Status = VL53L4CD_Lasers_List[Set_Laser_No]->VL53L4CD_SetXtalk(VL53L4CD_Xtalk_KCPS[Set_Laser_No]);
    VL53L4CD_Print_Error(VL53L4CD_Status, Set_Laser_No, 6);
  }
} // VL53L4CD_Setup_And_Initialize_Sensor


// I2C laser calibration ------------------------------------------------------------------------------


void VL53L4CD_Calibrate_Sensor() {
  // Calibrate Distance first (1)
  if(VL53L4CD_Offset_Calibrate == YES) {
    VL53L4DC_Calibrate_Offset_Distance();  // Loop all active Lasers
  }
  // Calibrate Xtalk next (2)
  if(VL53L4CD_Xtalk_Calibrate == YES) {
    VL53L4DC_Calibrate_Xtalk_Distance();  // Loop all active Lasers
  }
} // VL53L4CD_Calibrate_Sensor


void VL53L4DC_Calibrate_Offset_Distance() {
  const int16_t Target_Distance_MM  = 100; // ST recommandation
  const int16_t Number_Of_Samples   = 20;  // ST recommandation
  int16_t Target_Measured_Offset_MM = 0;   // +/- mm
  byte Offset_Laser_No = 0;
  for (byte Index = 0; Index < VL53L4CD_Max_ON_Index; Index++) {
    Offset_Laser_No = VL53L4CD_Laser_ON_List[Index];
    if (VL53L4CD_Error_Status[Offset_Laser_No] == OK) {
      PortPrintln();
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      VL53L4CD_Print_Header(Offset_Laser_No); PortPrintln(F("Calibrating sensor distance procedure."));
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      PortPrintln(F("Use GRAY cardboard / paper as reflektion object and indoor low light environment."));
      PortPrint  (F("Target distance must be [mm]: ")); PortPrintln(Target_Distance_MM);
      //
      VL53L4CD_Status = VL53L4CD_Lasers_List[Offset_Laser_No]->VL53L4CD_GetOffset(&Target_Measured_Offset_MM);
      PortPrint  (F("Current offset distance [mm]: ")); PortPrintln(Target_Measured_Offset_MM);
      VL53L4CD_Print_Error(VL53L4CD_Status, Offset_Laser_No, 8);
      //
      VL53L4CD_Status = VL53L4CD_Lasers_List[Offset_Laser_No]->VL53L4CD_CalibrateOffset( \
                        Target_Distance_MM, &Target_Measured_Offset_MM, Number_Of_Samples);
      PortPrint  (F("NEW calibration offset [mm]: ")); PortPrintln(Target_Measured_Offset_MM);
      VL53L4CD_Print_Error(VL53L4CD_Status, Offset_Laser_No, 9);
      PortPrint  (F("Remember to type in new value: VL53L4CD_Offset_Dist_MM[")); PortPrint(Offset_Laser_No); PortPrintln(F("]"));
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      delay(10000);
    }
  } 
} // VL53L4DC_Calibrate_Offset_Distance


void VL53L4DC_Calibrate_Xtalk_Distance() {
  const uint16_t Number_Of_Samples = 20;  // ST recommandation
  uint16_t  Measured_Xtalk_KCPS = 0;
  byte Xtalk_Laser_No = 0;
  for (byte Index = 0; Index < VL53L4CD_Max_ON_Index; Index++) {
    Xtalk_Laser_No = VL53L4CD_Laser_ON_List[Index];
    if (VL53L4CD_Laser_I2C_ON[Xtalk_Laser_No] == YES  &&  VL53L4CD_Error_Status[Xtalk_Laser_No] == OK) {
      PortPrintln();
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      VL53L4CD_Print_Header(Xtalk_Laser_No); PortPrintln(F("Calibrating sensor Crosstalk procedure."));
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      PortPrintln(F("Use GRAY cardboard / paper as reflektion object and indoor low light environment."));
      PortPrint  (F("Maximum measured ranging distance without under-ranging [mm]: ")); PortPrintln(VL53L4CD_Xtalk_Distance_MM);
      //
      VL53L4CD_Status = VL53L4CD_Lasers_List[Xtalk_Laser_No]->VL53L4CD_GetXtalk(&Measured_Xtalk_KCPS);
      PortPrint  (F("Current crosstalk compensation [KPCS]: ")); PortPrintln(Measured_Xtalk_KCPS);
      VL53L4CD_Print_Error(VL53L4CD_Status, Xtalk_Laser_No, 10);
      //
      VL53L4CD_Status = VL53L4CD_Lasers_List[Xtalk_Laser_No]->VL53L4CD_CalibrateXtalk( \
                        VL53L4CD_Xtalk_Distance_MM, &Measured_Xtalk_KCPS, Number_Of_Samples);
      PortPrint  (F("NEW crosstalk compensation [KCPS]: ")); PortPrintln(Measured_Xtalk_KCPS);
      VL53L4CD_Print_Error(VL53L4CD_Status, Xtalk_Laser_No, 11);
      PortPrint  (F("Remember to type in new value: VL53L4CD_Xtalk_KCPS[")); PortPrint(Xtalk_Laser_No); PortPrintln(F("]"));
      for (byte index = 0; index < 81; index++) { PortPrint(F("-")); } PortPrintln();
      delay(10000);
    }
  } 
} // VL53L4DC_Calibrate_Xtalk_Distance


//  I2C laser processing ---------------------------------------------------------------------


void TCA_PCA_Select_Channel(const byte TCA_PCA_Channel_No) {
  // 4 or 8 channel I2C Multiplexer
  if (VL53L4CD_Error_Status[TCA_PCA_Channel_No] == OK) {
    byte End_Status = 0;
    Wire.beginTransmission(TCA_PCA_I2C_Address);
    Wire.write(1 << TCA_PCA_Channel_No);  // Shit left one bit
    End_Status = Wire.endTransmission();
    // 
    if (End_Status > 0) {
      Set_Error_Sound_Count(1);
      VL53L4CD_Error_Status[TCA_PCA_Channel_No] = Error;
      PortPrint(F("TCA_PCA channel [")); PortPrint(TCA_PCA_Channel_No); PortPrint(F("] ERROR! "));
      VL53L4CD_Print_Error_Text (End_Status);
    }
  }
} // TCA_PCA_Select_Channel


void VL53L4CD_Setup_And_Start_Continuous_Ranging(const byte Start_Laser_No) {
  // Program the fastest speed. Do not change.
  const word Time_Budget_MS   = 10;  // 10 Milliseconds
  const word Inter_Measure_MS = 0;   //  0 Milliseconds
  if (VL53L4CD_Error_Status[Start_Laser_No] == OK) {
    VL53L4CD_Status = VL53L4CD_Lasers_List[Start_Laser_No]->VL53L4CD_SetRangeTiming(Time_Budget_MS, Inter_Measure_MS);
    VL53L4CD_Print_Error(VL53L4CD_Status, Start_Laser_No, 12);
  }
  // Start Measurements
  if (VL53L4CD_Error_Status[Start_Laser_No] == OK) {
    VL53L4CD_Status = VL53L4CD_Lasers_List[Start_Laser_No]->VL53L4CD_StartRanging();
    VL53L4CD_Print_Error(VL53L4CD_Status, Start_Laser_No, 13);
  }
} // VL53L4CD_Set_And_Start_Continuous_Ranging


bool VL53L4CD_Read_Sensors_Sequentially() {
  if (VL53L4CD_Loop_ON_Index >= VL53L4CD_Max_ON_Index) {
    VL53L4CD_Loop_ON_Index = 0;
  }
  VL53L4CD_Active_Sensor = VL53L4CD_Laser_ON_List[VL53L4CD_Loop_ON_Index];
  VL53L4CD_Loop_ON_Index++;
  return VL53L4CD_Read_One_Sensors(VL53L4CD_Active_Sensor);
} // VL53L4CD_Read_Sensors_Sequentially


bool VL53L4CD_Read_One_Sensors (const byte Read_One_Laser) {
  if (VL53L4CD_Laser_I2C_ON[Read_One_Laser] == YES  &&  VL53L4CD_Error_Status[Read_One_Laser] == OK) {
    VL53L4CD_Active_Sensor = Read_One_Laser;
    if (TCA_PCA_I2C_MUX_ON == YES  &&  TCA_PCA_I2C_MUX_Status == OK) {
      TCA_PCA_Select_Channel(Read_One_Laser);
    }
    if (VL53L4CD_Error_Status[Read_One_Laser] == OK) {
      return VL53L4CD_Read_And_Check(Read_One_Laser);
    }
  }
  return false;
} // VL53L4CD_Read_One_Sensors


bool VL53L4CD_Read_And_Check(const byte Read_Laser_No) {
  bool Return_Status = false;
  unsigned long Start_Time_MS;
  byte New_Data_Ready = 0;
  //
  Start_Time_MS = millis();
  VL53L4CD_Status = VL53L4CD_Lasers_List[Read_Laser_No]->VL53L4CD_CheckForDataReady(&New_Data_Ready); // 1,2 milliseconds
  if ((!VL53L4CD_Status == OK) && (New_Data_Ready == YES)) {
     // (Mandatory) Clear HW interrupt to restart measurements
    VL53L4CD_Lasers_List[Read_Laser_No]->VL53L4CD_ClearInterrupt(); // 0,5 milliseconds
    VL53L4CD_Lasers_List[Read_Laser_No]->VL53L4CD_GetResult(&VL53L4CD_Result);  // 4,0 Milliseconds
    VL53L4CD_Print_Debug_Info(Read_Laser_No);
    VL53L4CD_check_Ambient_Light(Read_Laser_No);
    //
    if (VL53L4CD_Result.range_status == VL53L4CD_ERROR_NONE  &&  
        VL53L4CD_Result.distance_mm > 0  && 
        VL53L4CD_Result.sigma_mm <= VL53L4CD_Max_Sigma_MM  &&
        VL53L4CD_Result.signal_rate_kcps >=  VL53L4CD_Min_Signal_KPCS) { 
      VL53L4CD_Blocking_Calculate_And_Check(Read_Laser_No);
      Return_Status = VL53L4CD_Check_Min_And_Max_Distance(Read_Laser_No);
    }
  }
  // millis(): The number will overflow (go back to zero), after approximately 50 days.
  // https://www.gammon.com.au/millis
  Offset_Delay_Time = (word)(millis() - Start_Time_MS);
  if (Offset_Delay_Time >= Loop_Delay_Time) { 
    Offset_Delay_Time = Loop_Delay_Time;  // Zero delay time
  }
  delay((unsigned long)(Loop_Delay_Time - Offset_Delay_Time));
  return Return_Status;
} //VL53L4CD_Read_And_Check


void VL53L4CD_check_Ambient_Light(const byte Ambient_Laser_No) {
  word Average_Ambient_KCPS = 0;
  VL53L4CD_Average_Ambient_KCPS[Ambient_Laser_No] = VL53L4CD_Average_Ambient_KCPS[Ambient_Laser_No] + (long)VL53L4CD_Result.ambient_rate_kcps;
  if (VL53L4CD_Average_Ambient_Count[Ambient_Laser_No] == VL53L4CD_Max_Ambient_Count) {
    Average_Ambient_KCPS = (word)(VL53L4CD_Average_Ambient_KCPS[Ambient_Laser_No] / VL53L4CD_Average_Ambient_Count[Ambient_Laser_No]);     
    if (Average_Ambient_KCPS > VL53L4CD_Max_Ambient_KCPS) {
      Set_Error_Sound_Count(1);
      VL53L4CD_Print_Header(Ambient_Laser_No); 
      PortPrint(F("ERROR! Ambient light is to high (KCPS): ")); PortPrintln(Average_Ambient_KCPS);
    }
    VL53L4CD_Average_Ambient_KCPS[Ambient_Laser_No] = 0;
    VL53L4CD_Average_Ambient_Count[Ambient_Laser_No] = 0;
  }
  else {
    VL53L4CD_Average_Ambient_Count[Ambient_Laser_No]++;
  }
}  // VL53L4CD_check_Ambient_Light


void VL53L4CD_Print_Debug_Info(const byte Info_Laser_No) {
  char Print_Result[120];
  if (VL53L4CD_Debug_Enabled[Info_Laser_No] == YES) {
    snprintf(Print_Result, sizeof(Print_Result), 
      "Status = %2u, Distance = %4u mm, Signal rate= %5u kcps, Sigma = %3u, Ambient light = %5u",
      VL53L4CD_Result.range_status, VL53L4CD_Result.distance_mm, VL53L4CD_Result.signal_rate_kcps, 
      VL53L4CD_Result.sigma_mm, VL53L4CD_Result.ambient_rate_kcps);
    VL53L4CD_Print_Header(Info_Laser_No); PortPrintln(Print_Result);
    if (VL53L4CD_Print_Range_Status == YES) {
      VL53L4CD_Print_Ranging_Error(Info_Laser_No, VL53L4CD_Result.range_status);
    }
  }
} // VL53L4CD_Print_Debug_Info


bool VL53L4CD_Check_Min_And_Max_Distance(const byte Dist_Laser_No) {
  if (VL53L4CD_Result.distance_mm >= VL53L4CD_Min_Distance_MM[Dist_Laser_No]  &&  \
      VL53L4CD_Result.distance_mm <= VL53L4CD_Max_Distance_MM[Dist_Laser_No]) {
    if (VL53L4CD_Print_Object_Detect == YES) {
      VL53L4CD_Print_Header(Dist_Laser_No); PortPrint(F("Object detected (mm): ")); 
      PortPrint(VL53L4CD_Result.distance_mm); PortPrint(F(" Detection zone: "));
      PortPrint(VL53L4CD_Min_Distance_MM[Dist_Laser_No]); PortPrint(F(" - "));
      PortPrintln(VL53L4CD_Max_Distance_MM[Dist_Laser_No]);
    }
    return true;
  }
  return false;
} // Check_Min_And_Max_Distance


void VL53L4CD_Blocking_Calculate_And_Check(const byte Average_Laser_No) {
  word Average_Distance_MM; 
  if (VL53L4CD_Blocking_Detect[Average_Laser_No] == YES) {
    VL53L4CD_Average_Block_MM[Average_Laser_No] += (long)VL53L4CD_Result.distance_mm;
    if (VL53L4CD_Average_Block_Count[Average_Laser_No] == VL53L4CD_Max_Block_Count) {
      Average_Distance_MM = (word)(VL53L4CD_Average_Block_MM[Average_Laser_No] / VL53L4CD_Average_Block_Count[Average_Laser_No]);
      VL53L4CD_Average_Block_MM[Average_Laser_No]     = 0;
      VL53L4CD_Average_Block_Count[Average_Laser_No]  = 0;
      //
      VL53L4CD_Blocking_Check(Average_Laser_No, Average_Distance_MM);
    }
    else {
      VL53L4CD_Average_Block_Count[Average_Laser_No]++;
    }
  }
} // VL53L4CD_Blocking_Calculate_And_Check


void VL53L4CD_Blocking_Check(const byte Check_Laser_No, const word Check_Average_Distance_MM) {
  if (Check_Average_Distance_MM <= VL53L4CD_Max_BLock_Dist_MM) {
    Set_Error_Sound_Count(1);
    VL53L4CD_Print_Header(Check_Laser_No); 
    PortPrint(F("ERROR! Sensor beam blocked (mm): ")); PortPrint(Check_Average_Distance_MM);
    PortPrint(F(" Blocking zone: 0 - ")); PortPrintln(VL53L4CD_Max_BLock_Dist_MM);
  }  
} // L53L4CD_Blocking_Check


// I2C laser error handling ------------------------------------------------------------------------


void VL53L4CD_Check_Parameters() {
  byte Chek_Laser_No = 0;
  for (byte Index = 0; Index < VL53L4CD_Max_ON_Index; Index++) {
    Chek_Laser_No = VL53L4CD_Laser_ON_List[Index];
    if (VL53L4CD_Min_Distance_MM[Chek_Laser_No] < 25) {
      VL53L4CD_Print_Header(Chek_Laser_No);
      PortPrint(F("Warning! VL53L4CD_Min_Distance_MM[")); PortPrint(Chek_Laser_No);
      PortPrint(F("] = ")); PortPrint(VL53L4CD_Min_Distance_MM[Chek_Laser_No]); 
      PortPrintln(F(" Should be greater than or equal to 25 mm."));
      Set_Error_Sound_Count(1);
    }
    if (VL53L4CD_Min_Distance_MM[Chek_Laser_No] <= VL53L4CD_Max_BLock_Dist_MM) {
      VL53L4CD_Print_Header(Chek_Laser_No);
      PortPrint(F("Warning! VL53L4CD_Min_Distance_MM[")); PortPrint(Chek_Laser_No);
      PortPrint(F("] = ")); PortPrint(VL53L4CD_Min_Distance_MM[Chek_Laser_No]); 
      PortPrint(F(" Should be greater than or equal to VL53L4CD_Max_BLock_Dist_MM = "));
      PortPrintln(VL53L4CD_Max_BLock_Dist_MM);
      Set_Error_Sound_Count(1);
    }
    if (VL53L4CD_Max_Distance_MM[Chek_Laser_No] > 800) {
      VL53L4CD_Print_Header(Chek_Laser_No);
      PortPrint(F("Warning! VL53L4CD_Max_Distance_MM[")); PortPrint(Chek_Laser_No); 
      PortPrint(F("] = ")); PortPrint(VL53L4CD_Max_Distance_MM[Chek_Laser_No]);
      PortPrintln(F(" Should be less than or equal to 800 mm."));
      Set_Error_Sound_Count(1);
    }
    if (VL53L4CD_Min_Distance_MM[Chek_Laser_No] >=VL53L4CD_Max_Distance_MM[Chek_Laser_No]) {
      VL53L4CD_Print_Header(Chek_Laser_No);
      PortPrint (F("ERROR! VL53L4CD_Max_Distance_MM must be greater than VL53L4CD_Min_Distance_MM"));
      Set_Error_Sound_Count(1);
    }
  }
  //
  if (TCA_PCA_I2C_MUX_ON == YES  &&  TCA_PCA_I2C_MUX_Status == OK  &&  VL53L4CD_Max_ON_Index == 0) {
    PortPrintln(F("Warning! TCA_PCA_I2C_MUX_ON = YES - but no lasers [0-7] are enabled!"));
    Set_Error_Sound_Count(1);
  }
} // Check_VL53L4CD_Parameters


void VL53L4CD_Print_Error(const byte Print_Error_Status, const byte Print_Laser_No, const byte Function_Numer) {
  if (Print_Error_Status != 0) {
    Set_Error_Sound_Count(1);
    VL53L4CD_Error_Status[Print_Laser_No] = Error;
    VL53L4CD_Print_Header(Print_Laser_No);
    switch (Function_Numer) {
      case  1: PortPrint(F("Check_I2C_Connected"));    break;
      case  2: PortPrint(F("Set_I2C_Adresse"));        break;
      case  3: PortPrint(F("GetSensorID"));            break;
      case  4: PortPrint(F("SensorInit"));             break;
      case  5: PortPrint(F("SetOffset"));              break;
      case  6: PortPrint(F("SetXtalk"));               break;
      case  7: PortPrint(F("SetDetectionThresholds")); break;
      case  8: PortPrint(F("GetOffset"));              break;
      case  9: PortPrint(F("CalibrateOffset"));        break;
      case 10: PortPrint(F("GetXtalk"));               break;
      case 11: PortPrint(F("CalibrateXtalk"));         break;
      case 12: PortPrint(F("SetRangeTiming"));         break;
      case 13: PortPrint(F("StartRanging"));           break;
      default:
        PortPrintln(F("Unknown function!"));
    }
    PortPrint(F(" - ERROR! ")); 
    VL53L4CD_Print_Error_Text (Print_Error_Status);
  }
}


void VL53L4CD_Print_Error_Text(const byte Print_Error_Status) {
  switch (Print_Error_Status) {
    case   0: break; // VL53L4CD_ERROR_NONE
    case   1: PortPrintln(F("Data too long to fit in transmit buffer.")); break;
    case   2: PortPrintln(F("Received NACK on transmit of address."));    break;
    case   3: PortPrintln(F("Received NACK on transmit of data."));       break;
    case   4: PortPrintln(F("Other error."));                             break;
    case   5: PortPrintln(F("Timeout"));                                  break;
    case 250: PortPrintln(F("Wrong sensor ID."));                         break;
    case 253: PortPrintln(F("VL53L4CD_ERROR_XTALK_FAILED."));             break;
    case 254: PortPrintln(F("VL53L4CD_ERROR_INVALID_ARGUMENT."));         break;
    case 255: PortPrintln(F("VL53L4CD_ERROR_TIMEOUT."));                  break;
    default:
      PortPrintln(F("Unknown error!"));
  }
} // L53L4CD_Print_Error_Text


void VL53L4CD_Print_Ranging_Error(const byte Range_Laser_No, const byte Print_Ranging_Error) {
  // Some error occurred, print it out!
  if (Print_Ranging_Error != 0) {
    VL53L4CD_Print_Header(Range_Laser_No);
  }
  switch (Print_Ranging_Error) {
    case   0: break; // Returned distance is valid
    case   1: PortPrintln(F("WARNING! Sigma is above the defined threshold."));            break;
    case   2: PortPrintln(F("WARNING! Signal is below the defined threshold."));           break;
    case   3: PortPrintln(F("ERROR! Measured distance is below detection threshold."));    break;
    case   4: PortPrintln(F("ERROR! Phase out of valid limit (max 1.200 mm)."));           break;
    case   5: PortPrintln(F("ERROR! Hardware fail."));                                     break;
    case   6: PortPrintln(F("WARNING! Phase valid but no wrap around check performed."));  break;
    case   7: PortPrintln(F("ERROR! Wrapped target, phase does not match."));              break;
    case   8: PortPrintln(F("ERROR! Processing fail."));                                   break;
    case   9: PortPrintln(F("ERROR! Crosstalk signal fail."));                             break;
    case  10: PortPrintln(F("ERROR! Interrupt error."));                                   break;
    case  11: PortPrintln(F("ERROR! Merged target."));                                     break;
    case  12: PortPrintln(F("ERROR! Signal is too low."));                                 break;
    case 255: PortPrintln(F("ERROR! Other error (e.g boot)."));                            break;
    default:
      PortPrintln(F("UNKNOWN error!"));
  }
} // VL53L4CD_Print_Ranging_Error


void VL53L4CD_Print_Header(const byte Header_Laser_No) {
  // General header info
  PortPrint(F("VL53L4CD [")); PortPrint(Header_Laser_No); PortPrint(F("] "));
}


void Set_Error_Sound_Count(byte Set_Error_Count) {
if (Set_Error_Count > Error_Sound_Count) {
  Error_Sound_Count = Set_Error_Count;
  }
} // Set_Error_Sound_Count


void Error_Alarm_Sound() {
  if (Error_Sound_Count > 0) {
    if (Error_Sound_ON == NO) {
      // analogWrite(LED_Red_Pin, LED_Red_ON);
      Error_Sound_ON = YES;
    }
    if (Buzzer_Sound_Process() == true) {  // Sound process ended
      Error_Sound_Count--;  // Next sound process (if any)
      if (Error_Sound_Count == 0) {
        Error_Sound_ON = NO;
        // analogWrite(LED_Red_Pin, LED_Red_OFF);
      }
    }
  } 
}  // Error_Alarm_Sound


bool Buzzer_Sound_Process() {
  if (Buzzer_Start_Step == YES) {
    Buzzer_Start_Step = NO;
    Buzzer_Start_Time = millis();
    if (Error_Sound_Pitch_Hz[Buzzer_Step_Count] == 0) {
      Buzzer_Off();
    } 
    else {
      tone(Buzzer_Pin, Error_Sound_Pitch_Hz[Buzzer_Step_Count]);
    }
  }
  else {
    if (word(millis() - Buzzer_Start_Time) >= Error_Sound_Duration_MS[Buzzer_Step_Count]) {
      Buzzer_Start_Step = YES;
      Buzzer_Step_Count++;
      if (Buzzer_Step_Count > Error_Sound_Max_Step) {
        Buzzer_Step_Count = 0;
        Buzzer_Start_Step = YES;
        Buzzer_Off();
        return true; // End of sound process
      }
    }
  }
  return false; // Continue sound process
} // Buzzer_Sound_Process


void Buzzer_Off() {
  noTone(Buzzer_Pin);
  // Buzzer YL-44 is active on LOW! KY-006 on HIGH!
  digitalWrite(Buzzer_Pin, Buzzer_Silent_Mode);
} // Buzzer_Off

// end of program

