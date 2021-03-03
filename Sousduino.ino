#include <PID_AutoTune_v0.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <TimerOne.h>


/*******************************************************

This program runs a PID controlled sous vide
Dries Willemsens, December 2016

********************************************************/

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// DS18S20 Temperature chip i/o
OneWire ds(12);  // on pin 12
byte addr[8]; // OneWire address

// Input pin for zerocross detection
int zeroCrossPin = 2;

// Output pin for heating element
int triacPin = 11;

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//Define variables for the dim control
volatile int i=0;               // Variable to use as a counter of dimming steps. It is volatile since it is passed between interrupts
volatile boolean zero_cross=0;  // Flag to indicate we have crossed zero
int dim2 = 0;                   // led control
int dim = 128;                  // Power level (0-128)  0 = On, 128 = 0ff
int pas = 8;                    // step for count;
int freqStep = 75;              // This is the delay-per-brightness step in microseconds. It allows for 128 steps
                                // If using 60 Hz grid frequency set this to 65

//Define Variables for the PID control
double Setpoint, Input, Output;
const long interval = 1000;
unsigned long lastConversion;
boolean conversionStarted;
const double kp = 75;
const double ki = 0.020;
const double kd = 0;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT);

// Function to read the buttons
int read_LCD_buttons(){
  adc_key_in = analogRead(0);      // read the value from the sensor 
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  /* if (adc_key_in < 50)   return btnRIGHT;  
  if (adc_key_in < 250)  return btnUP; 
  if (adc_key_in < 450)  return btnDOWN; 
  if (adc_key_in < 650)  return btnLEFT; 
  if (adc_key_in < 850)  return btnSELECT;  
  */
  // For V1.0 comment the other threshold and use the one below:
  if (adc_key_in < 50)   return btnRIGHT;  
  if (adc_key_in < 195)  return btnUP; 
  if (adc_key_in < 380)  return btnDOWN; 
  if (adc_key_in < 555)  return btnLEFT; 
  if (adc_key_in < 790)  return btnSELECT;


  return btnNONE;  // when all others fail, return this...
}

void zeroCross(){
  zero_cross = true;               // set flag for dim_check function that a zero cross has occured
  i=0;                             // stepcounter to 0.... as we start a new cycle
  digitalWrite(triacPin, LOW);
  //Serial.println("Zero cross detected");
}

// Turn on the TRIAC at the appropriate time
// We arrive here every 75 (65) uS
// First check if a flag has been set
// Then check if the counter 'i' has reached the dimming level
// if so.... switch on the TRIAC and reset the counter
void dim_check() {                   
  if(zero_cross == true) {              
    if(i>=dim) {                     
      digitalWrite(triacPin, HIGH);  // turn on light
      //Serial.println("Power on");       
      i=0;  // reset time step counter                         
      zero_cross=false;    // reset zero cross detection flag
    } 
    else {
      i++;  // increment time step counter                     
    }                                
  }    
}     

void setup(){
  Serial.begin(9600);   

  //Initialize timer interrupt
  attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zeroCross, RISING);    // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection
  Timer1.initialize(freqStep);                      // Initialize TimerOne library for the freq we need
  Timer1.attachInterrupt(dim_check, freqStep);      // Go to dim_check procedure every 75 uS (50Hz)  or 65 uS (60Hz)
  // Use the TimerOne Library to attach an interrupt
  
  bool tempSet = true;
  bool buttonPush = true;

  //Initialize LCD
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Set temperature");
  lcd.setCursor(6,1);
  lcd.print("deg C");

  //Initialize pins
  pinMode(triacPin, OUTPUT);
  digitalWrite(triacPin, LOW);
  
  //initialize the variables we're linked to
  Setpoint = 60;
  Output = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(interval);
 
  //Initialize temperature sensor
  conversionStarted = false;
  //Look up DS OneWire address and start initial conversion
  ds.reset_search();
  if ( !ds.search(addr)) {
      Serial.print("No 1-Wire sensor connected.\n");
  }
  
  while(tempSet){
    //Display the current temperature
    lcd.setCursor(0,1);
    lcd.print(Setpoint);
    
    lcd_key = read_LCD_buttons();
    //Increase or decrease the set temperature when up or down is pushed and exit the routine when the select button is pushed
    switch (lcd_key){
      /*case btnRIGHT:
      {
        lcd.print("RIGHT ");
        break;
      }
      case btnLEFT:
      {
        lcd.print("LEFT   ");
        break;
      }*/
      case btnUP:
      {
        if (buttonPush){
          Setpoint = Setpoint + 1;
          buttonPush = false;
        }
        break;
      }
      case btnDOWN:
      {
        if (buttonPush){
          Setpoint = Setpoint - 1;
          buttonPush = false;
        }
        break;
      }
      case btnSELECT:
      {
        lcd.setCursor(0,0);
        lcd.print("Temperature set");
        delay(2000);
        tempSet = false;
        break;
      }
      case btnNONE:
      {
        buttonPush = true;
        break;
      }
    }    
  }
  lcd.setCursor(0,1);
  lcd.print("                ");
}
 
void loop() {
  unsigned long currentMillis;
  
  if(!conversionStarted){
    //Start conversion for the temperature sensor
    convertTemp();
  }

  currentMillis = millis();

  if(currentMillis - lastConversion >= interval){
    Input = readTemp();
    myPID.Compute();

    conversionStarted = false;

    dim = map(Output, 0, 255, 128, 64);

    lcd.setCursor(0,0);
    lcd.print("Ingestelde T:  ");
    lcd.setCursor(14,0);
    lcd.print(Setpoint);
    lcd.setCursor(0,1);
    lcd.print("Huidige T:     ");
    lcd.setCursor(14,1);
    lcd.print(Input);

    Serial.print(Output);
    Serial.print(" ");
    Serial.println(dim);
  }
}

void convertTemp() {

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  lastConversion = millis();
  conversionStarted = true;
}

double readTemp(){
  byte i;
  byte present = 0;
  byte data[12];
  
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
  double temperatuur;

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  /*Serial.print("P=");
  Serial.print(present,HEX);
  Serial.print(" ");*/
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    /*Serial.print(data[i], HEX);
    Serial.print(" ");*/
  }

  /*Serial.print(" CRC=");
  Serial.print( OneWire::crc8( data, 8), HEX);
  Serial.println();*/

  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + (TReading / 4);    // multiply by (100 * 0.0625) or 6.25

  temperatuur = float(Tc_100)/100;
  return temperatuur;
  /*Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  Fract = Tc_100 % 100;*/


  /*if (SignBit) // If its negative
  {
     Serial.print("-");
  }
  Serial.print(Whole);
  Serial.print(".");
  if (Fract < 10)
  {
     Serial.print("0");
  }
  Serial.print(Fract);

  Serial.print("\n");*/
}
