/* ***************************** Pinewood.INO************************************************* 
 
This is a simple project for running a cubcar (Pinewood derby) race using an Arduino NANO
Project will support either a 3 lane or 4 lane track.
Results of a race are displayed via a bank of LEDs which indicates who the winner is,
individual race times and instructions are displayed on an I2C 4x20 LCD.
I used two shift registers to power the bank of LEDs to reduce the overall number of pins 
needed to control up to 16 LEDS.  (LEDS are individually voltage controlled with a 330 Ohm 
resister)  It would not be difficult to switch the code to use an LED Strip instead.

LDR (Light Detecting Resistors) are embedded at the finish line of the track and shadows from
the cars are what trip the timers. The LDRs are connected with external 1000 ohm pulldown 
resistors to make them more responsive. It is important that consistent light is shining directly
overhead for each LDR.  LDRs and light conditions will vary greatly so at each startup
a tuning routine runs which captures light levels with no car on the tracks and then with cars
shading the sensors.

Note: The shift registers, LED banks, LCD collectively draw a lot of power. USB port on a laptop 
may not give enough power to run everything.  I used a Buck converter with a 9V power supply,
which also powers an always on LED strip which provides consistent lighting to the finish line.

I included a debugging mode in the code which outputs verbosely into the serial monitor to help 
with troubleshooting.

Author - Jeff Wilts
Version 1.0 -- Initial Build  May 2023

The circuit:
3v3 -- LDR -- A0(Nano) --10K resistor -- GND
Same pattern is used for LDR's and switches
EXTERNAL PULLDOWN RESISTORS

D0 (RX)     -- Dont put anything on here or cant upload to Arduino
D1 (TX)     -- Dont put anything on here or cant upload to Arduino
D2 (INT0)   -- Push Button on timer (TimerButton)
D3 (INT1)   -- Start timer switch (StartButton)
D4          -- OPEN
D5          -- OPEN
D6          -- OPEN
D7          -- OPEN
D8          -- Shift Register - Latch PIN Pin (12)
D9          -- OPEN
D10 (SS)    -- OPEN
D11 (MOSI)  -- Shift Register - Data Pin Pin (14)
D12 (MISO)  -- Shift Register - SH_Clock Pin (11)
D13 (SCK)   -- OPEN

A0/D14      -- Lane #1 LDR
A1/D15      -- Lane #2 LDR
A2/D16      -- Lane #3 LDR
A3/D17      -- Lane #4 LDR 
A4/D18 (SDA)-- SDA for i2c screen  
A5/D19 (SCL)-- SCL for i2c screen
A6/D20      -- OPEN
A7/D21      -- OPEN

************************************ */

// #define DEBUG 1                                     // Toggle between debug and run mode by commenting & uncommenting these two lines 
#define DEBUG 0

const int NOSWITCH = 0;                                 // Define if start switch in NO - Normally Open (1) or NC - Normally Closed (0)

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#include <LiquidCrystal_I2C.h>                      // lcd display library
#include <Wire.h>                                   // i2c libary for lcd display

//*********************Display Stuff  ***********************************************
LiquidCrystal_I2C lcd(0x27,20,4);                   // set the LCD address to 0x27 for a 20 chars and 4 line display

//*********************Shift Register Stuff *****************************************
const int latchPin = 8;
const int clockPin = 12;
const int dataPin = 11;

int numOfRegisters = 2;                           // number of shift registers
byte* registerState;

long effectSpeed = 130;  

//************************Lane Setup Stuff ***************************************
const int laneCount = 3;                            // number of lanes on the track
int LEDarray[] = { 5, 6, 7, 9, 10, 11, 13, 14, 15 };  // array of LEDS if 3 lanes
int LEDarray4[] = { 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 13, 14, 15, 16 };  // array of LEDS if 4 lanes
const int raceTimeout = 15;                         // How long before race times out in secondes

const int LDR1 = A1;                                // LDR pin Lane #1
const int LDR2 = A3;                                // LDR pin Lanee #2
const int LDR3 = A0;                                // LDR pin Lane #3
const int LDR4 = A2;                                // LDR pin Lane #4
const int TimerButton = 2;                          // the number of the pushbutton pin
int TimerButtonState;                               // the current reading from the input pin

const int StartButton =  3;                         // The track starter button
int StartButtonState;                               // the current reading from the input pin

const uint8_t DebounceDelay = 50;                   // Adjust this value as needed

int input_val1 = 0;                                 // Starting state for LDR1
int input_val2 = 0;                                 // Starting state for LDR2
int input_val3 = 0;                                 // Starting state for LDR3
int input_val4 = 0;                                 // Starting state for LDR4

int thresh1 = 225;                                  // default trigger threshold for lane #1
int thresh2 = 225;                                  // default trigger threshold for lane #2
int thresh3 = 225;                                  // default trigger threshold for lane #3
int thresh4 = 225;                                  // default trigger threshold for lane #3

//************************************ SETUP ***********************************

void setup()
{  
  pinMode(TimerButton, INPUT);                      // initialize the pushbutton pin as an input with External PULLDOWN resistor:
  pinMode(StartButton, INPUT);                      // initialize the start button pin as an input with External PULLDOWN resistor:
    
  Serial.begin(9600);
  //*********************Shift Register Stuff *****************************************
  //Initialize array
  registerState = new byte[numOfRegisters];
  for (size_t i = 0; i < numOfRegisters; i++)     // blank out all the registers
  {
    registerState[i] = 0;
  }                                               
  //set pins to output so you can control the shift register
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  
  //************************* Setup i2c Screen*****************************
  Wire.begin();                                    //start i2c address
  lcd.init();
  lcd.home();                       
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print(F("Startup Complete!"));
   //*************************** End i2c Screen******************************
  
  debugln(F("Splashing Color"));
  effectC(effectSpeed);                         // splash a little colour on LEDs
  debugln(F("End Splashing Color"));
  BlankAll();

  tuneLDR();                                    // run start routine
  
} // end Setup
/* ******************************** Main Loop ************************ */

void loop()
{
  int finishercount = 0;                        // reset count of finishers to 0
  float startTime = 0 ;                         // Start timer
  float Lane1Timer = 0 ;                        // time for lane 1
  float Lane2Timer = 0 ;                        // time for lane 2
  float Lane3Timer = 0 ;                        // time for lane 3
  float Lane4Timer = 0 ;                        // time for lane 4
    
  BlankAll();                                   // Make sure all the LEDS are off
  debugln(F("Close Start Gate"));
  lcd.clear();
  lcd.print(F("Close Start Gate"));  

  if (NOSWITCH){                                // check for normally open switch
    while(!PushedStart()){}                     // Wait for the track start button to be pushed
  } else {                                      // if normally closed switch
    while(!ReleasedStart()){}
  } // endif check for Normally Open Switch
    
  // effectC(effectSpeed/2);                         // splash a little colour on LEDs
  debugln(F("Load Cars"));
  lcd.clear();
  lcd.print(F("Load Cars"));  
    
  effectC(effectSpeed/2);                         // splash a little colour on LEDs
                                                  // wait for start gate to get triggered
  if (NOSWITCH){                                  // check for normally open switch
    while(!ReleasedStart()){}
  } else {                                      // if normally closed switch
    while(!PushedStart()){}                     // Wait for the track start button to be pushed
  } // endif check for Normally Open Switch

  startTime = millis();                           // capture start time
  
  debugln(F("Raceers Away"));
  lcd.setCursor(0,1);
  lcd.clear();
  
  while(1) {
    input_val1 = analogRead(LDR1);
    input_val2 = analogRead(LDR2);
    input_val3 = analogRead(LDR3);
    input_val4 = analogRead(LDR4);
    
    if ((input_val1 < thresh1) & (Lane1Timer == 0 )) 
    {
      finishercount = finishercount+1; 
      debugln(F("Trigger1"));
      Lane1Timer = (millis() - startTime)/1000;
      BankPlaceLight(1,finishercount,laneCount);
      debugln(Lane1Timer);
      lcd.setCursor(0,0);
      lcd.print("Lane 1: " + String(Lane1Timer));
      if (finishercount == 1) 
      {
        lcd.print(F(" WINNER"));
      } // endif WINNER
    }
    if ((input_val2 < thresh2) & (Lane2Timer == 0 ))
    {
      finishercount = finishercount+1; 
      debugln(F("Trigger2"));
      Lane2Timer = (millis() - startTime)/1000;
      BankPlaceLight(2,finishercount,laneCount);
      debugln(Lane2Timer);
      lcd.setCursor(0,1);
      lcd.print("Lane 2: " + String(Lane2Timer));
      if (finishercount == 1) 
      {
        lcd.print(F(" WINNER"));
      } // endif WINNER

    }
    if ((input_val3 < thresh3) & (Lane3Timer == 0 ))
    {
      finishercount = finishercount+1; 
      debugln(F("Trigger3"));
      Lane3Timer = (millis() - startTime)/1000;
      BankPlaceLight(3,finishercount,laneCount);
      debugln(Lane3Timer);
      lcd.setCursor(0,2);
      lcd.print("Lane 3: " + String(Lane3Timer));
      if (finishercount == 1) 
      {
        lcd.print(F( " WINNER"));
      } // endif WINNER
    } // end if threshold check
    if (laneCount ==4)
    {
      if ((input_val4 < thresh4) & (Lane4Timer == 0 ))
      {
        finishercount = finishercount+1; 
        debugln(F("Trigger4"));
        Lane4Timer = (millis() - startTime)/1000;
        BankPlaceLight(4,finishercount,laneCount);
        debugln(Lane4Timer);
        lcd.setCursor(0,3);
        lcd.print("Lane 4: " + String(Lane4Timer));
        if (finishercount == 1) 
        {
          lcd.print(F(" WINNER"));
        } // endif WINNER
      }  // endif threshold check
    } // Endif laneCount == 4
       
    if (finishercount >= laneCount) { break; }      // End the Race

    //  Check race that ran too long
    if ((millis() - startTime) > (raceTimeout*1000) ) { 
      debugln(F("Timed Out"));
      debugln( finishercount );
      lcd.setCursor(5,3);
      lcd.print(F("TIMED OUT"));
      break; } // End the Race
    
  } // End While
  
  debugln(F("Epic Race - Glad its over, Push Timer Button"));
  // lcd.clear();
  // lcd.print(F("Clear Lanes, "));
  // lcd.setCursor(0,1);
  // lcd.print(F("Push Timer Button"));

  while(!ReleasedTimer()){}         // Wait for the button on the timer to be pressed & released
} // end Loop

/* ******************************************************** */
void tuneLDR()
{
  debugln(F("Starting tuning LDR"));
  delay(1000);
  int brightval1 = 0;       // value for LDR with no car shadow
  int brightval2 = 0;       // value for LDR2 with no car shadow
  int brightval3 = 0;       // value for LDR3 with no car shadow
  int brightval4 = 0;       // value for LDR3 with no car shadow
  int shadowval1 = 0;       // value for LDR with a car shadow
  int shadowval2 = 0;       // value for LDR2 with a car shadow
  int shadowval3 = 0;       // value for LDR3 with a car shadow
  int shadowval4 = 0;       // value for LDR3 with a car shadow
    
  debugln(F("Clear Lanes, Then Press Timer Button"));
  BankLight(1,laneCount); 
  lcd.clear();
  lcd.print(F("Clear Lanes, "));
  lcd.setCursor(0,1);
  lcd.print(F("Push Timer Button"));
  
  while(!ReleasedTimer()){}         // Wait for the button on the timer to be pressed & released
 
  brightval1 = analogRead(LDR1);
  brightval2 = analogRead(LDR2);
  brightval3 = analogRead(LDR3);
  brightval4 = analogRead(LDR4);

  debugln(F("Got Start LDR"));
  debug(brightval1);
  debug( F(" ") );
  debug(brightval2);
  debug( F(" ") );
  debug(brightval3);
  debug( F(" ") );
  debugln(brightval4);
  debugln(F("Put cars on sensors, press Timer"));
  delay(900);                   // double fix for stupid debounce

  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(F("Car Noses On Lanes, "));
  lcd.setCursor(0,2);
  lcd.print(F("Shading finish, "));
  lcd.setCursor(0,3);
  lcd.print(F("Push Timer Button"));
  BankLight(2,laneCount); 

  while(!ReleasedTimer()){}         // Wait for the start button on the timer to be pressed & released
  
  shadowval1 = analogRead(LDR1);
  shadowval2 = analogRead(LDR2);
  shadowval3 = analogRead(LDR3);
  shadowval4 = analogRead(LDR4);

  debugln(F("Got shadow LDR"));
  debug(shadowval1);
  debug(F(" ") );
  debug(shadowval2);
  debug( F(" ") );
  debug(shadowval3);
  debug( F(" ") );
  debugln(shadowval4);
  
  thresh1 = shadowval1 + (brightval1 - shadowval1) /2 ;
  thresh2 = shadowval2 + (brightval2 - shadowval2) /2 ;
  thresh3 = shadowval3 + (brightval3 - shadowval3) /2 ;
  thresh4 = shadowval4 + (brightval4 - shadowval4) /2 ;

  debugln(F("Thresholds"));
  debug(thresh1);
  debug( F(" ") );
  debug(thresh2);
  debug( F(" ") );
  debug(thresh3);
  debug( F(" ") );
  debugln(thresh4);

  lcd.clear();
  lcd.print(F("Got Shadow LDR, "));
  lcd.setCursor(0,1);
  lcd.print(thresh1);
  lcd.setCursor(4,1);
  lcd.print(thresh2);
  lcd.setCursor(8,1);
  lcd.print(thresh3);
  lcd.setCursor(13,1);
  lcd.print(thresh4);
  lcd.setCursor(0,3);
  lcd.print(F("Push Timer Button"));
  debugln(F("Push Timer Button To Proceed"));
  BankLight(3,laneCount); 
  delay(1000);                                      // double fix for stupid debounce
  while(!ReleasedTimer()){}                         // Wait for the button on the timer to be pressed & released
  debugln(F("End Tuning LDR"));
} // end TuneLDR

//*************************************************************************************
// These two routines wait until the push button is RELEASED to record a button press.  Perfect for start button and timer button

bool ReleasedStart() {
  static uint16_t state = 0;
  static unsigned long lastDebounceTime = 0;

  if ((millis() - lastDebounceTime) < DebounceDelay) {          // Check if enough time has passed since the last debounce
    return false;                                               // Debounce in progress, return false
  }
  lastDebounceTime = millis();                                  // Update the debounce time
  state = (state << 1) | digitalRead(StartButton) | 0xfe00;     // Update the button state
  return (state == 0xff00);
} // End ReleasedStart

//*************************************************************************************

bool ReleasedTimer() {                                         // Looks for a button release
  static uint16_t state = 0;
  static unsigned long lastDebounceTime = 0;

  if ((millis() - lastDebounceTime) < DebounceDelay) {          // Check if enough time has passed since the last debounce
    return false;                                               // Debounce in progress, return false
  }
  lastDebounceTime = millis();                                  // Update the debounce time
  state = (state << 1) | digitalRead(TimerButton) | 0xfe00;     // Update the button state
  return (state == 0xff00);
} // end ReleasedTimer

//*************************************************************************************
bool PushedTimer() {

  int reading = digitalRead(TimerButton);
      
    // if the button state has changed:
    if (reading != TimerButtonState) {
      TimerButtonState = reading;
    }
  return(reading);
} // end PushedTimer

//********************************************************
bool PushedStart() {
  int reading = digitalRead(StartButton);
  // debug("StartRead: "); 
  // debugln(reading); 
    
    // if the button state has changed:
    if (reading != TimerButtonState) {
      StartButtonState = reading;
    }
  return(reading);
} // end PushedStart


//********************************************************
void effectC(int speed){
  int prevI = 0;
  for (int i = 0; i < 16; i++){
    regWrite(prevI, LOW);
    regWrite(i, HIGH);
    prevI = i;

    delay(speed);
  }

  for (int i = 15; i >= 0; i--){
    regWrite(prevI, LOW);
    regWrite(i, HIGH);
    prevI = i;

    delay(speed);
  }
}

//***********************************************************************************************

void BankLight(int bank,int laneCount)
// lanecount = number of LEDs in each bank
{
  int startIndex; 
  switch (bank) {
    case 1:
      startIndex = 0;
      break;
    case 2:
      startIndex = 3;;
      break;
    case 3:
      startIndex = 6;
      break;
    default:
      // Invalid bank number
      return;
  } 
  // BlankAll() ;
  for (int i = startIndex; i < (startIndex+laneCount); i++) {
    int value = LEDarray[i];
    regWrite(value, HIGH);
    
  }
}

//***********************************************************************************************
  
void BankPlaceLight(int bank,int Place, int laneCount)
{
  // BlankAll() ;
  int startIndex; 
  switch (bank) {
    case 1:
      startIndex = 0;
      break;
    case 2:
      startIndex = 3;;
      break;
    case 3:
      startIndex = 6;
      break;
    default:
      // Invalid bank number
      return;
  }  // end Switch
  for (int i = startIndex; i < (startIndex+Place); i++) {
    int value = LEDarray[i];
    regWrite(value, HIGH);
    
  } // End for loop
} // end BankPlaceLight


//***************************************************************************************

void BlankAll()
// Turn off all the LEDS
{
  for (int i = 0; i < 16; i++)              // Turn off all the LEDS
  {
    regWrite(i, LOW);
  }
} // end BlankAll

//*************************************************************

void regWrite(int pin, bool state){
  //Determines register
  int reg = pin / 8;
  //Determines pin for actual register
  int actualPin = pin - (8 * reg);

  //Begin session
  digitalWrite(latchPin, LOW);

  for (int i = 0; i < numOfRegisters; i++){
    //Get actual states for register
    byte* states = &registerState[i];

    //Update state
    if (i == reg){
      bitWrite(*states, actualPin, state);
    }

    //Write
    // shiftOut(dataPin, clockPin, MSBFIRST, *states);
    shiftOut(dataPin, clockPin, LSBFIRST, *states);
  }

  //End session
  digitalWrite(latchPin, HIGH);
}
