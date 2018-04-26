// software for XL4016X2 solar charger 
// Modified by farmerkeith
// based on version by zopinter
// based on MPPT solar charger Version 3 by deba168
// based on work by Tim Nolan
// modification commenced 13 April 2018
// last change 23 April 2018

// key components:
// Arduino Nano V3 microcontroller
// 2 DC-DC converter modules based on XL4016 with pwm control of output voltage via feedback pin
// XL4016 controlled by the Nano (pwm on D9 and D10)
// solar panel up to 240W Voc<39V Isc<8 Amps
// AGM or flooded lead acid battery nominal 12V
// mains power charge supplement controlled by the Nano (D3)
// LED night time lighting load controlled by the Nano (D4)
// Battery temperature monitored by DS18B20 and used to adjust battery set points (D2)
// LCD display of operational data (I2C on A4, A5)
// LCD backlight controlled by a momentary action push button (A6)
// Micro SD card for data logging (SPI on D11, D12, D13 with CS on D8)
// TinyRTC DC1307 real time clock (I2C on A4, A5)
// Solar controller temperature monitored by DS18B20 mounted on RTC (D2)
// Battery current in and out monitored by ACS712 20A (A3)
// Red and Green status indicators (D5, D6). 

// 2017.01.01: 14938KWh
// Adding HTU21DF sensor
// Adding separate voide for MPPT PO and IC algorith
// Adding EEPROM store
//----------------------------------------------------------------------------------------------------

//////// Arduino pins Connections//////////////////////////////////////////////////////////////////////////////////
// A0 - Solar panel voltage 100K/15K divider
// A1 - Solar panel current ACS 712 5A
// A2 - Battery voltage 100K/47K divider
// A3 - Battery current ACS 712 5A 
// A4 - I2C SDA for LCD, RTC and EEPROM
// A5 - I2C SCL for LCD, RTC and EEPROM
// A6 - Pushbutton for LCD backlight control
// A7 - Spare - use if needed for cooling FAN control 

// D0 - reserved for USB to host
// D1 - reserved for USB to host
// D2 - DS18B20 * 2
// D3 - Mains power control
// D4 - Load control
// D5 - Green LED  
// D6 - Red LED 
// D7 - Spare 
// D8 - CS for SPI to micro SD card
// D9 - PWM control of XL4016 #2
// D10 - PWM control of XL4016 #1
// D11 - SPI MOSI
// D12 - SPI MISO
// D13 - SPI clock SCK; also Nano on board LED

// possible other functions
// COOLING FAN control
// AIR CLEANING FAN control
// ACS712 ENABLE/DISABLE ??

// include libraries ---------------------------------------------------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AnalogSmooth.h>
// #include "Adafruit_HTU21DF.h"
#include <EEPROM.h>

// include tab files ---------------------------------------------------------------------
#include "data_recording.h"

// global constants ----------------------------------------------------------------------
#define wattHours_setup 5983
#define daily_wattHours_setup 0
#define ampHours_setup 0


//-----------------------------------------------------------------------------------------------------------------
///////// Hardware connection constants /////////////////////////////////////////////////////////////////////////////////////////////////
//#define BAT_FLOAT 12.35                  // battery voltage we want to stop charging at

const byte SOL_VOLTS_CHAN =0;               // defining the adc channel to read solar volts on A0
const byte SOL_AMPS_CHAN =1;                // Defining the adc channel to read solar amps on A1
const byte BAT_VOLTS_CHAN =2;               // defining the adc channel to read battery volts on A2
const byte BUCK_AMPS_CHAN =3;               // Defining the adc channel to read battery amps on A3
const byte BACK_LIGHT_PIN =6;               // pin A6 is used to control the lcd back light

const byte DS18B20_pin = 2;  // D2 for One-wire to DS18B20 * 2
const byte mains_pin = 3;    // D3 - Mains power control
const byte load_pin = 4;     // D4 - Load control
const byte greenLED_pin =5;  // D5 - Green LED  
const byte redLED_pin = 6;   // D6 - Red LED 
// D7 - Spare 
const byte sd_CS_pin =8;     // D8 - CS for SPI to micro SD card
const byte pwm_buck2 = 9;    // D9 - PWM control of XL4016 #2
const byte pwm_buck1 =10;    // D10 - PWM control of XL4016 #1
// #define PWM_PIN 11

// #define COOLING_FAN_PWM_PIN 3               // pin used to control air cleaning fan in room
// #define TURN_ON_COOLING_FAN analogWrite(COOLING_FAN_PWM_PIN, 55)
// #define TURN_OFF_COOLING_FAN analogWrite(COOLING_FAN_PWM_PIN, 0)

// #define NIGHT_LIGHT_ENABLE_PIN 7                                             // pin used to control night light
// #define TURN_ON_NIGHT_LIGHT digitalWrite(NIGHT_LIGHT_ENABLE_PIN, HIGH)      // enable Night light
// #define TURN_OFF_NIGHT_LIGHT digitalWrite(NIGHT_LIGHT_ENABLE_PIN, LOW)      // disable Night light

//#define BIG_FAN_PWM_PIN 9                       // pin used to control air cleaning fan in room
//#define TURN_OFF_BIG_FAN analogWrite(BIG_FAN_PWM_PIN, 0)

// #define ACS_ENABLE_PIN 10                                    // pin used to control ACS712 power for energy save purposes
// #define TURN_ON_ACS digitalWrite(ACS_ENABLE_PIN, HIGH)      // enable ACS712 sensors
// #define TURN_OFF_ACS digitalWrite(ACS_ENABLE_PIN, LOW)      // disable ACS712 sensors

#define LED_ON 50                            //milliseconds
#define LED_OFF 5000
unsigned long ms;                          //time from millis()
unsigned long msLast;                      //last time the LED changed state
boolean ledState;                          //current LED state

#define SOL_AMPS_SCALE  0.048828125        // ACS 712 Current Sensor is used. Current Measured = (5/(1024 *0.185))*ADC - (2.5/0.100)
#define BUCK_AMPS_SCALE 0.048828125        // the scaling value for raw adc reading to get solar amps   // 5/(1024*0.100)
#define SOL_VOLTS_SCALE 0.0299        // the scaling value for raw adc reading to get solar volts  // (5/1024)*(R1+R2)/R2 // R1=100k and R2=20k
#define BAT_VOLTS_SCALE 0.0299        // the scaling value for raw adc reading to get battery volts 
#define AVG_NUM 50                          // number of iterations of the adc routine to average the adc readings


//------------------------------------------------------------------------------------------------------
/////////////////////////////////////////BIT MAP ARRAY//////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------
byte solar[8] = //icon for solar panel
{
  0b11111,
  0b10101,
  0b11111,
  0b10101,
  0b11111,
  0b10101,
  0b11111,
  0b00000
};

byte battery[8] =
{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
};

byte _PWM [8] =
{
  0b11101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10111,
};

//-------------------------------------------- Global variables --------------------------------------------------------
float sol_amps_previous = 0;
float sol_volts_previous = 0;
float buck_amps_previous = 0;


float sol_amps;                       // solar amps
float buck_amps;                      // solar amps

float sol_volts;                      // solar volts
float old_sol_volts = 0;

float bat_volts;                      // battery volts

float sol_watts;                      // solar watts

float buck_watts;                     // buck watts
float rbuck_watts;                    // rounded buck watts

float mppt_track;
float rmppt_track;
float old_mppt_track = 0;

//unsigned int seconds = 0;             // seconds from timer routine
//unsigned int prev_seconds = 0;        // seconds value from previous pass
//unsigned int interrupt_counter = 0;   // counter for 20us interrrupt

int fan_pwm = 0;
int bat_volts_map = 0;                 // for mapping bat volts to fan pwm
int night_light_state = 0;             // variable for storing the load output state (for writing to LCD)

int pwm_value = 0;
int trackDirection = 1;                // step amount to change the value of pulseWidth used by MPPT algorithm

unsigned long ms1 = 0;               // variable to store time the back light control button was pressed in millis
unsigned long ms2 = 0;               // variable to store time the back light control button was pressed in millis
unsigned long ms3 = 0;               // variable to store time the back light control button was pressed in millis
int back_light_pin_State = 0;         // variable for storing the state of the backlight button
int back_light_state = 0;

/*
  float gain;
  int gain_sum = 0;
  int gain_temp = 0;
  int gain_counter = 0;
*/

float eff;                             // efficiency
int eff_sum = 0;
int eff_temp = 0;
int eff_counter = 0;

enum charger_mode {SLEEP, MPPT, FLOAT} charger_state;  // enumerated variable that holds state for charger state machine

float max_sol_watts = 0;              // PZ added
float daily_max_sol_watts = 0;        // PZ added

float max_buck_amps = 0;              // PZ added
float daily_max_buck_amps = 0;        // PZ added

unsigned int msec = 0; // PZ added
int last_msec = 0; // PZ added
int elasped_msec = 0; // PZ added
float elasped_time = 0; // PZ added
float ampSecs = 0; // PZ added
float ampHours = 0; // PZ added
float wattSecs = 0; // PZ added
float wattHours_decimal; // PZ added
float wattHours_temp = 0; // PZ added
float daily_wattHours = 0;
unsigned long wattHours = 0;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// Adafruit_HTU21DF htu = Adafruit_HTU21DF();

//AnalogSmooth as = AnalogSmooth();
AnalogSmooth as100 = AnalogSmooth(50);


//------------------------------------------------------------------------------------------------------
// This routine is automatically called at powerup/reset
//------------------------------------------------------------------------------------------------------

void setup()                           // run once, when the sketch starts
{
  // recover data from EEPROM
  wattHours = wattHours_setup;
  //  start_wattHours = wattHours_setup;

  ampHours = ampHours_setup;
  daily_wattHours = daily_wattHours_setup;

  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 (pin 9&10)divisor to 1 for PWM frequency of 31372.55 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 (pin 3&11) divisor to 1 for PWM frequency of 240 Hz

  DIDR0 = 0x0F;  //Turn off digital input function on ADC channel 0-3, since we will use analogue signals

  Serial.begin(250000);

  pwm_value = 100;
  charger_state = FLOAT;               // start with charger state as sleep

  lcd.begin(20, 4);                    // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.noBacklight();                   // turn off the backlight
  back_light_state = 0;
  lcd.createChar(1, solar);            // turn the bitmap into a character
  lcd.createChar(2, battery);          // turn the bitmap into a character
  lcd.createChar(3, _PWM);             // turn the bitmap into a character

//  pinMode(ACS_ENABLE_PIN, OUTPUT);     // sets the digital pin as output
//  TURN_ON_ACS;                         // turn off ACS712

//  pinMode(NIGHT_LIGHT_ENABLE_PIN, OUTPUT);           // output for the LOAD MOSFET (LOW = on, HIGH = off)
//  TURN_OFF_NIGHT_LIGHT;
//  night_light_state = 0;

//  pinMode(BIG_FAN_PWM_PIN, OUTPUT);           // output for the LOAD MOSFET (LOW = on, HIGH = off)

  pinMode (BACK_LIGHT_PIN, INPUT);

  pinMode (pwm_buck1, OUTPUT);
  pinMode (pwm_buck2, OUTPUT);

//  pinMode (COOLING_FAN_PWM_PIN, OUTPUT);

  pinMode(greenLED_pin, OUTPUT);          // sets the digital pin as output
  pinMode(redLED_pin, OUTPUT);            // sets the digital pin as output
}

//------------------------------------------------------------------------------------------------------
// Main loop
//------------------------------------------------------------------------------------------------------
void loop()
{
  read_data();                         // read data from inputs
  mode_select();                       // select the charging state
  set_charger();                       // run the charger state machine
  // print_data();                        // print data
  // cooling_fan();
  // big_fan();
  // night_light();
  // power_save(); // ACS712 uses 10mA at 5V
  keepalive_led();
  lcd_display();                       // lcd display
  // MPPT_test();
  others();                            // max watts and amps;
}

//------------------------------------------------------------------------------------------------------
// This routine reads and averages the analog inputs for this system, solar volts, solar amps and
// battery volts.
//------------------------------------------------------------------------------------------------------
unsigned int read_adc(int channel) {

  unsigned int sum = 0;
  unsigned int temp;
  int i;

  for (i = 0; i < AVG_NUM; i++) {      // loop through reading raw adc values AVG_NUM number of times
    temp = analogRead(channel);        // read the input pin
    sum += temp;                       // store sum for averaging
    delayMicroseconds(100);             // pauses for 50 microseconds
  }
  return (sum / AVG_NUM);              // divide sum by AVG_NUM to get average and return it
}

//------------------------------------------------------------------------------------------------------
// This routine reads all the analog input values for the system. Then it multiplies them by the scale
// factor to get actual value in volts or amps.
//------------------------------------------------------------------------------------------------------
void read_data(void) {
  sol_volts = read_adc(SOL_VOLTS_CHAN) * SOL_VOLTS_SCALE;          // input of solar volts
  //  bat_volts = read_adc(BAT_VOLTS_CHAN) * BAT_VOLTS_SCALE;          // input of battery volts
  bat_volts = as100.analogReadSmooth(BAT_VOLTS_CHAN) * BAT_VOLTS_SCALE;


  if (sol_volts > 9.5) {
    //  TURN_ON_ACS;    // Moved to Power_Save void
    buck_amps = 25.00 - (read_adc(BUCK_AMPS_CHAN) * BUCK_AMPS_SCALE); // input of solar amps
    sol_amps = 25.05 - (read_adc(SOL_AMPS_CHAN) * SOL_AMPS_SCALE);    // input of solar amps
  }
  else {
    //  TURN_OFF_ACS; // Moved to Power_Save void
    sol_amps = 0;
    buck_amps = 0;
  }

  if (charger_state != SLEEP) {
    sol_watts = sol_amps * sol_volts;                               // calculations of solar watts
    buck_watts = buck_amps * bat_volts;

    msec = millis(); // PZ added section
    elasped_msec = msec - last_msec;                                  //Calculate how long has past since last call of this function
    elasped_time = elasped_msec / 1000.0;                             // 1sec=1000 msec
    //  ampSecs = (sol_amps * elasped_time);                              //AmpSecs since last measurement
    ampSecs = (buck_amps * elasped_time);                                //AmpSecs since last measurement
    ampHours = ampHours + ampSecs / 3600;                             // 1 hour=3600sec //Total ampHours since program started
    wattSecs = ampSecs * bat_volts ;                                   //WattSecs since last measurement
    wattHours_decimal = wattSecs / 3600;                          // 1 hour=3600sec //Total wattHours since program started
    wattHours_temp = wattHours_temp + wattHours_decimal;
    daily_wattHours = daily_wattHours + wattHours_decimal;
    last_msec = msec;                                                 //Store 'now' for next time

    if (wattHours_temp >= 1) {
      wattHours++;
      wattHours_temp = wattHours_temp - 1;
    }
  }

  /*
    gain_temp = ((buck_amps - sol_amps) / sol_amps) * 100;
    gain_sum += gain_temp;
    if (gain_counter < 100) (gain_counter = gain_counter + 1);
    else {
      gain_sum = 0;
      gain_counter = 0;
    }
    gain = gain_sum / gain_counter;
    gain = constrain (gain, 0, 99);
  */

  eff_temp = (buck_watts / sol_watts) * 100;
  eff_sum += eff_temp;
  if (eff_counter < 100) (eff_counter = eff_counter + 1);
  else {
    eff_sum = 0;
    eff_counter = 0;
  }
  eff = eff_sum / eff_counter;
  eff = constrain (eff, 0, 99);
}

//------------------------------------------------------------------------------------------------------
// Mode selection routine
//------------------------------------------------------------------------------------------------------
void mode_select() {
  if (sol_volts > bat_volts) { // If battery voltage is in the normal range and there is light on the panel
    if (sol_amps < 0.3) charger_state = FLOAT;                                 // If battery voltage is above 13.5, go into float charging
    else charger_state = MPPT;                                                                 // If battery voltage is less than 13.5, go into bulk charging
  }
  else  {                                                        // If there's no light on the panel, go to sleep
    charger_state = SLEEP;
  }
}

void set_charger(void) {

  switch (charger_state) {                                                                    // skip to the state that is currently set

    case SLEEP:                                                                               // the charger is in the sleep state
      if (pwm_value > 0) pwm_value--;
      pwm_value = constrain (pwm_value, 0, 255);
      analogWrite (pwm_buck1, pwm_value);
      analogWrite (pwm_buck2, pwm_value);
      break;

    case MPPT:                                                                                // the charger is in the bulk state
      if (sol_volts >= 18.00)
      {
        pwm_value--;
        analogWrite (pwm_buck1, pwm_value);
        analogWrite (pwm_buck2, pwm_value);
      }
      else if (sol_volts < 14.00)
      {
        pwm_value++;
        analogWrite (pwm_buck1, pwm_value);
        analogWrite (pwm_buck2, pwm_value);
      }
      else {
        MPPT_PO();
        //MPPT_IC();
      }
      break;

    case FLOAT:                                                                               // the charger is in the float state, it uses PWM instead of MPPT
      if (pwm_value > 0) pwm_value--;
      pwm_value = constrain (pwm_value, 0, 255);
      analogWrite (pwm_buck1, pwm_value);
      analogWrite (pwm_buck2, pwm_value);
      delay (75);
      break;

    default:                                                                                  // if none of the other cases are satisfied,
      analogWrite (pwm_buck1, 0);
      analogWrite (pwm_buck2, 0);
      break;
  }
}

void MPPT_PO (void) {
  rmppt_track = round (buck_amps * sol_volts * 10) ;                  // calculations of hybrid watts
  //  rmppt_track = round (sol_watts * 10);
  mppt_track = rmppt_track / 10 ;


  if ((mppt_track < old_mppt_track) || (pwm_value <= 0) || (pwm_value >= 255)) {
    trackDirection = -trackDirection;                 // if pulseWidth has hit one of the ends reverse the track direction
    pwm_value = pwm_value + trackDirection;           // add (or subtract) track Direction to(from) pulseWidth
  }
  else pwm_value = pwm_value + trackDirection;


  old_mppt_track = mppt_track;

  pwm_value = constrain (pwm_value, 0, 255);
  analogWrite (pwm_buck1, pwm_value);
  analogWrite (pwm_buck2, pwm_value);
  delay (75);
}

void MPPT_IC (void) {
  float dV = sol_volts - sol_volts_previous;
  float dI = sol_amps - sol_amps_previous;

  if (dV == 0)
  { if (dI == 0)
    {
      sol_volts = sol_volts;
      sol_amps = sol_amps;
      //delay(1000);
    }

    else if (dI > 0) // that means you have to make dI=0 decrease I,decraese duty cycle
    {
      pwm_value++;
      analogWrite (pwm_buck1, pwm_value);
      analogWrite (pwm_buck2, pwm_value);
    }

    else if (dI < 0) // that means you have to increase I to make dI=0;increase duty cycle
    {
      pwm_value--;
      analogWrite (pwm_buck1, pwm_value);
      analogWrite (pwm_buck2, pwm_value);
    }
  }

  else
  {
    if (dI / dV + (sol_amps / sol_volts) <= 0.001)
    {
      sol_volts = sol_volts;
      sol_amps = sol_amps;
      //delay(1000);
    }

    else if (dI / dV < -(sol_amps / sol_volts))
    {
      pwm_value--;
      analogWrite (pwm_buck1, pwm_value);
      analogWrite (pwm_buck2, pwm_value);
    }
    else
    {
      pwm_value++;
      analogWrite (pwm_buck1, pwm_value);
      analogWrite (pwm_buck2, pwm_value);
    }
  }
  sol_volts_previous = sol_volts;
  sol_amps_previous = sol_amps;
  delay(75);
}

//------------------------------------------------------------------------------------------------------
// This routine prints all the data out to the serial port.
//------------------------------------------------------------------------------------------------------
void print_data (void) {

  Serial.print(bat_volts_map);
  Serial.print("    ");

  Serial.print("Fan_pwm= ");
  Serial.print(fan_pwm);
  Serial.print("    ");

  Serial.print("P= ");
  Serial.print(sol_watts, 1);
  Serial.print("    ");

  Serial.print("I_s= ");
  Serial.print(sol_amps);
  Serial.print("    ");

  Serial.print("I_b= ");
  Serial.print(buck_amps);
  Serial.print("    ");

  Serial.print("V_s= ");
  Serial.print(sol_volts);
  Serial.print("    ");

  Serial.print("V_b= ");
  Serial.print(bat_volts);
  Serial.print("    ");

  if (charger_state == SLEEP) Serial.print("SLEEP");
  else if (charger_state == MPPT) Serial.print("MPPT ");
  else if (charger_state == FLOAT) Serial.print("FLOAT");
  Serial.print("    ");

  Serial.print("Track_W: ");
  Serial.print(mppt_track);
  Serial.print("  >>  ");

  Serial.print("pwm = ");
  Serial.print(pwm_value);
  Serial.print("    ");

  Serial.print("Ef=");
  Serial.print(eff, 0);
  Serial.print("%   ");

  /*  Serial.print("Gain=");
    Serial.print(gain, 0);
    Serial.print("%   ");
  */
  Serial.print("\n\r");
}

/*
//-------------------------------------------------------------------------------------------------
//------------------------------------- AirCleaning fan -------------------------------------------
//-------------------------------------------------------------------------------------------------
void big_fan (void)
{
  int fan_pot = analogRead(A6);

  if (fan_pot <= 2) {
    TURN_OFF_BIG_FAN;
    lcd.setCursor(14, 1);
    lcd.print("FanOFF");
  }

  else if (fan_pot > 2 && fan_pot <= 100)
  {
    analogWrite (BIG_FAN_PWM_PIN, 55);
    lcd.setCursor(14, 1);
    lcd.print("Silent");
  }

  else if (fan_pot > 100 && fan_pot < 1020)
  {
    fan_pwm = map(fan_pot, 50, 1020, 55, 255);
    fan_pwm = constrain (fan_pwm, 0, 255);
    analogWrite (BIG_FAN_PWM_PIN, fan_pwm);
    lcd.setCursor(14, 1);
    lcd.print("Man");
    lcd.print(fan_pwm);
  }
  else {
    lcd.setCursor(14, 1);
    lcd.print("Aut");
    lcd.print(fan_pwm);

    if ((bat_volts > 12.00) && (sol_volts > 14.50))
    {
      bat_volts_map = (int)((bat_volts * 100) - 1200);
      int fan_pwm_ref = map(bat_volts_map, 0, 30, 50, 255);
      if (fan_pwm_ref > fan_pwm) fan_pwm++;
      else fan_pwm--;
      fan_pwm = constrain (fan_pwm, 0, 255);
      analogWrite (BIG_FAN_PWM_PIN, fan_pwm);
    }
    else {
      fan_pwm--;
      fan_pwm = constrain (fan_pwm, 0, 255);
      analogWrite (BIG_FAN_PWM_PIN, fan_pwm);
    }
  }
}
*/ // end of big_fan control function

//-------------------------------------------------------------------------------------------------
//------------------------------------- Cooling fan -----------------------------------------------
//-------------------------------------------------------------------------------------------------
// void cooling_fan (void)
// {
//   if (buck_amps > 3) TURN_ON_COOLING_FAN;
//   else TURN_OFF_COOLING_FAN;
// }

/*
//-------------------------------------------------------------------------------------------------
//------------------------------------- Night Light -----------------------------------------------
//-------------------------------------------------------------------------------------------------
void night_light (void)
{
  if ((night_light_state == 0) && (sol_volts < 5.00) && (bat_volts > 11.10)) {
    TURN_ON_NIGHT_LIGHT;
    night_light_state = 1;
  }

  if ((night_light_state == 1 && sol_volts > 5.50) || bat_volts < 11.00) {
    TURN_OFF_NIGHT_LIGHT;
    night_light_state = 0;
  }

  if (night_light_state == 1 && sol_volts < 0.15) night_light_state = 2;

  if ((night_light_state == 2 && sol_volts > 0.30) || bat_volts < 11.00) TURN_OFF_NIGHT_LIGHT;

  if (night_light_state == 2 && sol_volts > 10.00) {
    night_light_state = 0;
  }
} // end of night light funtion
*/

/*
//-------------------------------------------------------------------------------------------------
//-------------------------------------- Power Save -----------------------------------------------
//-------------------------------------------------------------------------------------------------
void power_save (void) // ACS712 uses 10 mA at 5 volts = 50 mW
{
  if (sol_volts < 8.50) {
    TURN_OFF_ACS;
    CLKPR = 0x80;
    CLKPR = 0x02; // over-writes previous instruction
  }

  if (sol_volts > 9.00) {
    TURN_ON_ACS;
    CLKPR = 0x80;
    CLKPR = 0x00; // over-writes previous instruction
  }
} // end of power_save
*/

//-------------------------------------------------------------------------------------------------
//----------------------------------- Keepalive indication ----------------------------------------
//-------------------------------------------------------------------------------------------------
void keepalive_led (void)
{
  ms = millis();
  if (ms - msLast > (ledState ? LED_ON : LED_OFF)) {
    digitalWrite(greenLED_pin, ledState = !ledState);
    //    lcd.clear();
    msLast = ms;
  }
}

//------------------------------------------------------------------------------------------------------
//-------------------------- LCD DISPLAY --------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void lcd_display()
{
  back_light_pin_State = digitalRead(BACK_LIGHT_PIN);
  if (back_light_pin_State == HIGH)
  {
    ms2 = millis();                        // If any of the buttons are pressed, save the time in millis to "time"
  }

  backLight_timer();                      // call the backlight timer function in every loop

  if (millis() > (ms3 + 30000)) {        // Clear LCD every minute of remaining characters
    lcd.clear();
    ms3 = millis();
  }

  if (millis() > (ms1 + 150)) {          // Update LCD data every 150ms
    ms1 = millis();
    //-------------------- Solar Watts ---------------
    lcd.setCursor(0, 0);
    lcd.write(1);
    lcd.print(sol_watts, 1);
    lcd.print("W");

    //-------------------- Spinner ---------------
    lcd.setCursor(8, 0);
    //  lcd.write(2);
    spinner();

    //--------------------Battery State Of Charge ---------------
    float pct = 100.0 * (bat_volts - 10.8) / (12.6 - 10.8);
    if (pct < 0)
      pct = 0;
    else if (pct > 100)
      pct = 100;

    lcd.setCursor(10, 0);
    lcd.print(pct, 0);
    lcd.print("%");

    //------------------ Charger state & Duty Cycle-----------------------------------------
    lcd.setCursor(14, 0);
    lcd.write(3);
    if (charger_state == SLEEP)
      lcd.print("sleep");

    if (charger_state == FLOAT)
      lcd.print("float");

    if (charger_state == MPPT) {
      lcd.print(" M");
      if (pwm_value < 10) lcd.print("00");
      else if (pwm_value < 100) lcd.print("0");
      lcd.print(pwm_value);
    }

    //-------------------- Solar Volts ---------------
    lcd.setCursor(0, 1);
    lcd.print(sol_volts, 2);
    lcd.print("V");

    //-------------------- Battery Volts ---------------
    lcd.setCursor(7, 1);
    lcd.print(bat_volts, 2);
    lcd.print("V");

    //-------------------- Solar Current ---------------
    lcd.setCursor(0, 2);
    lcd.print(sol_amps, 1);
    lcd.print("A");

    //-------------------- Buck Current ---------------
    lcd.setCursor(9, 2);
    lcd.print(buck_amps, 1);
    lcd.print("A");

    //-------------------- Temperature and Humidity ---------------
//    lcd.setCursor(15, 2);
//    if (millis() > (ms3 + 15000)) {
//      lcd.print(htu.readTemperature(), 1);
//      lcd.print("C");
//    }
//    else {
//      lcd.print(htu.readHumidity(), 1);
//      lcd.print("%");
//    }

    //-------------------- Efficiency & WattHour ---------------
    lcd.setCursor(5, 2);
    if ((charger_state == MPPT) && (millis() > (ms3 + 15000))) {
      //    lcd.print("Ef:");
      lcd.print(eff, 0);
      lcd.print("%");
    }
    else {
      lcd.print(daily_max_buck_amps , 1);
    }

    lcd.setCursor(0, 3);
    if ((millis() > (ms3 + 20000)) && (charger_state != SLEEP)) {
      lcd.print (wattHours_temp, 4);
      lcd.print("     ");
    }
    else {
      lcd.print(wattHours);
      lcd.print("/");
      lcd.print(daily_wattHours, 0);
      lcd.print("Wh");
    }


    //-------------------- Long-term & Daily Maximum Solar Power ---------------
    lcd.setCursor(12, 3);
    //   if (max_sol_watts < 10) lcd.print("00");
    //   else if (max_sol_watts < 100) lcd.print("0");
    lcd.print(max_sol_watts, 0);
    lcd.print("/");
    //    if (daily_max_sol_watts < 10) lcd.print("00");
    //   else if (daily_max_sol_watts < 100) lcd.print("0");
    lcd.print(daily_max_sol_watts, 0);
    lcd.print("W");

    //    //-------------------- Gain & AmpHour ---------------
    //    lcd.setCursor(7, 3);
    //   if ((charger_state == MPPT) || (charger_state == FLOAT)) {
    /*    if (charger_state == bulk)
          lcd.print("Gb:    ");
        else if (charger_state == Float)
        lcd.print("Gf:    ");
        lcd.setCursor(3, 3);
        lcd.print(gain, 0);
        lcd.print("%");
    */
    //      lcd.print("D:");
    //    lcd.setCursor(2, 3);
    //     lcd.print(daily_max_buck_amps , 1);
    //      lcd.print("A");
    //    }
    //    else {
    //    lcd.print("      ");
    //    lcd.setCursor(0, 3);
    //  lcd.print(ampHours, 0);
    //  lcd.print("Ah");
    //    }
  }
}
//-------------------------------------------------------------------------------------------------
//---------------------------- Maximum Power Point measurement ------------------------------------
//-------------------------------------------------------------------------------------------------
void MPPT_test(void) {
  float sol_amps_MPPT = 0 ;                      // solar amps
  float sol_volts_MPPT = 0;                      // solar volts
  float sol_watts_MPPT = 0;                      // solar watts
  //  int pwm_perc;
  int pwm_MPPT;
  int n;

  back_light_pin_State = digitalRead(BACK_LIGHT_PIN);
  if ((charger_state == MPPT) && (back_light_state == 1) && (back_light_pin_State == HIGH)) {
    analogWrite (pwm_buck1, 255);
    analogWrite (pwm_buck2, 255);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPP test starting...");
    Serial.print("\n\r");
    Serial.print("MPP test starting...");
    Serial.print("\n\r");
    delay(2000);
    lcd.clear();

    for (n = 255; n >= 0 ; n--) {      // loop through reading raw adc values AVG_NUM number of times
      analogWrite (pwm_buck1, n);
      analogWrite (pwm_buck2, n);
      //      delay(1);             // pauses for 50 microseconds
      read_data();                         // read data from inputs

      if (sol_watts > sol_watts_MPPT) {
        sol_watts_MPPT = sol_watts;
        sol_amps_MPPT =  sol_amps;                       // solar amps
        sol_volts_MPPT = sol_volts;
        pwm_MPPT = n;
        //   pwm_perc = pwm_MPPT / 255 * 100;
      }

      //-------------------- Solar Watts ---------------
      lcd.setCursor(0, 0);
      lcd.write(1);
      lcd.print("SOLAR");
      lcd.setCursor(0, 1);
      lcd.print(sol_watts, 1);
      lcd.print("W");

      //-------------------- Solar Volts ---------------
      lcd.setCursor(0, 2);
      lcd.print(sol_volts, 2);
      lcd.print("V");

      //-------------------- Solar Current ---------------
      lcd.setCursor(0, 3);
      lcd.print(sol_amps, 2);
      lcd.print("A");


      //-------------------- MPP Solar Watts ---------------
      lcd.setCursor(8, 0);
      lcd.print("MPPT");
      lcd.setCursor(7, 1);
      lcd.print(sol_watts_MPPT, 1);
      lcd.print("W");

      //-------------------- MPP Solar Volts ---------------
      lcd.setCursor(7, 2);
      lcd.print(sol_volts_MPPT, 2);
      lcd.print("V");

      //-------------------- MPP Solar Current ---------------
      lcd.setCursor(7, 3);
      lcd.print(sol_amps_MPPT, 2);
      lcd.print("A");


      lcd.setCursor(14, 0);
      lcd.write(3);
      lcd.print(" PWM");

      lcd.setCursor(16, 1);
      if (n < 10) lcd.print("  ");
      else if (n < 100) lcd.print(" ");
      lcd.print(n);

      lcd.setCursor(16, 2);
      if (pwm_MPPT < 10) lcd.print("  ");
      else if (pwm_MPPT < 100) lcd.print(" ");
      lcd.print(pwm_MPPT);

      Serial.print(sol_amps);
      Serial.print(" A   ");
      Serial.print(sol_volts);
      Serial.print(" V   ");
      Serial.print(sol_watts);
      Serial.print(" W   ");
      Serial.print(n);
      Serial.print("\n\r");

      if (sol_volts >= 19.00) n = 0;
    }

    lcd.setCursor(14, 3);
    lcd.print("DONE!!");
    pwm_value = pwm_MPPT;
    analogWrite (pwm_buck1, pwm_value);
    analogWrite (pwm_buck2, pwm_value);

    Serial.print("\n\r");
    Serial.print("Done, Maximum Power Point results are:");
    Serial.print("  ");
    Serial.print(sol_amps_MPPT);
    Serial.print("A");
    Serial.print("   ");
    Serial.print(sol_volts_MPPT);
    Serial.print("V");
    Serial.print("   ");
    Serial.print(sol_watts_MPPT);
    Serial.print("W");
    Serial.print("   ");
    Serial.print(pwm_MPPT);
    Serial.print("pwm");
    Serial.print("   ");
    Serial.print("\n\r");
    Serial.print("\n\r");
    delay(10000);
  }
}
//-------------------------------------------------------------------------------------------------
//------------------------------------- Others ----------------------------------------------------
//-------------------------------------------------------------------------------------------------

void others (void) // max watts and amps; 
{
  //--------------------------------- Daily and long term max watts and buck amps -----------------------------------
  if ((sol_watts > max_sol_watts) && (sol_watts < 150)) {
    max_sol_watts = sol_watts;  // store long-term maximum solar watt
  }
  if ((sol_watts > daily_max_sol_watts) && (sol_watts < 150)) {
    daily_max_sol_watts = sol_watts;  // store daily maximum solar watt
  }

  if (buck_amps > max_buck_amps) {
    max_buck_amps = buck_amps;  // store long-term maximum solar watt
  }

  if (buck_amps > daily_max_buck_amps) {
    daily_max_buck_amps = buck_amps;  // store daily maximum solar watt
  }

  if (sol_volts < 0.10) {
    daily_max_sol_watts = 0;
    daily_max_buck_amps = 0;
    daily_wattHours = 0;

  // write to EEPROM moved to data_recording tab
  }
}


//----------------------------------- Backlight timer routine --------------------------------------------
void backLight_timer() {
  if ((millis() - ms2) <= 60000) {        // if it's been less than the 15 secs, turn the backlight on
    lcd.backlight();                   // finish with backlight on
    back_light_state = 1;
  }
  else
  {
    if ((back_light_state == 0) && (sol_watts > 8.0)) {
      lcd.backlight();
      back_light_state = 1;
    }
    else if ((back_light_state == 1) && ((sol_watts < 4.0) && (bat_volts < 12.28)) || (charger_state == SLEEP)) {
      lcd.noBacklight();
      back_light_state = 0;
    }
  }
}

//----------------------------------- Spinner routine --------------------------------------------
void spinner(void) {
  static int cspinner;
  // static char spinner_chars[] = { '*', '*', '*', ' ', ' ', ' '};
  // static char spinner_chars[] = { '*', '*', ' ', ' '};
  static char spinner_chars[] = { (2), (2), (2), ' ', ' ', ' '};
  cspinner++;
  lcd.print(spinner_chars[cspinner % sizeof(spinner_chars)]);

}
