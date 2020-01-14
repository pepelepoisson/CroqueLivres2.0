// Sketch intended to run in a "Croque Livres" little free library. 
// A lithium battery charged by a solar pannel powers a LED LED strip when the library door is opened. 
// The Arduino Pro Mini 3.3V 16MHz controls the WS2812B LEDs, checks the magnetic switch and counts how many times the door is opened.
// At the end of the animations triggered by door opening the sketch displays CPU Vin as a bargraph and door count as binary sequence on the LED strip.
// A sleep mode is currently not used due to some glitches to be investigated.
// January 2020, Pascal Prado, www.pcube.ca, papasinventeurs.org 


// Libraries required
#include "FastLED.h" // Librarie required for addressable LEDs
#include <avr/interrupt.h> // Librairies for sleep mode
#include <avr/power.h> // Librairies for sleep mode
#include <avr/sleep.h> // Librairies for sleep mode
#include <avr/io.h> // Librairies for sleep mode
#include <EEPROM.h>  // To store data to EEPROM library


// Definitions
#define NUM_LEDS 60  // How many leds in your strip?
#define HIGH_BRIGHTNESS 100  // Set LEDS brightness when set to HIGH level
#define LOW_BRIGHTNESS 20  // Set LEDS brightness when set to LOW level
#define DATA_PIN 7  // Pin to WS2812B LED data in
#define MOSFET_GATE 6  // Pin triggering the VCC power to LED strip
#define INDICATOR_LED 13 // Using built-in LED as indicator of MOSFET=WS2812B LED strip ON/OFF status
#define PushB1  2 // Pin corresponding to door magnetic switch  
//#define Door_Opened  (!digitalRead(PushB1))  // Button is on when circuit is closed - used for standard push buttons
#define Door_Opened  (digitalRead(PushB1))  // Button is on when circuit is opened - used for magnetic switch detecting door opening
#define FRAMES_PER_SECOND  120  // Defines the speed of LED animations
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define SET_LED_Animations_MILLISECONDS 30000 // Duration of LED_Animations triggered by door opening before moving to Idle?
#define SET_MAX_Animations_Time_MILLISECONDS 60000 // Max duration of LEDs animations
#define SET_Idle_MILLISECONDS 10000 // how many seconds at Idle before moving to Sleep?
#define IDLE_TRANSITION 2000 //duration of LED animation before moving to Idle (ms)
#define SLEEP_TRANSITION 2000 //duration of LED animation before moving to sleep (ms)
#define SHOW_COUNTER_DURATION 10000 //duration of door counter display
#define LOW_VCC 2700  // lower vcc value when checking battery level - Used for bargraph display purposes
#define MIN_BRIGHTNESS_VCC 3050  // Reference VCC to set LEDs brightness level to min value
#define HIGH_VCC 3200  // higher vcc value when checking battery level - Used for bargraph display purposes
#define MAX_BRIGHTNESS_VCC 3200  // Reference VCC to set LEDs brightness level to max value
#define BIT(n,i) (n>>i&1)  // Function used to read bit i of variable n

unsigned long start_time=0;  // Counter used to count time in a given state and trigger next moves
int Vcc=0;  // MCU VCC value in millivolts
bool Ready_4_Change=false;  // Boolean used to set readiness and debounce
int Door_opening_counter;  // Integer holding the number of times the door is opened. Is stored in EEPROM.
uint8_t brightness_level=50;  // Default brightness level for WS2812B LED strip - ranges between 0 and 255 

CRGB leds[NUM_LEDS];  // Define the array of leds

void setup() {

  Serial.begin(9600);
  while (!Serial);
  
  Serial.println("*** Entering setup routine ***"); 
  
  pinMode(PushB1,INPUT);
  digitalWrite(PushB1,HIGH);  // Configure built-in pullup resitor for push button 1

  pinMode(MOSFET_GATE,OUTPUT);
  digitalWrite(MOSFET_GATE,HIGH);  // Configure pin used to control LED strip VCC

  pinMode(INDICATOR_LED,OUTPUT);  // Using built-in LED as indicator of MOSFET=WS2812B LED strip ON/OFF status
  digitalWrite(INDICATOR_LED,HIGH);  // Indicator LED is turned ON/OFF to mimic MOSFET status
  
  // Bit of code to uncomment only when wake-up counter needs to be resetted
  //Door_opening_counter=0;
  //EEPROM.write(1, highByte(Door_opening_counter ));
  //EEPROM.write(2, lowByte(Door_opening_counter ));
  //Serial.print("Value in EEPROM has been resetted to ");Serial.println(Door_opening_counter);
  //Serial.println("***************************************************");

  Door_opening_counter = word(EEPROM.read(1), EEPROM.read(2));  // Reading Door_opening_counter from EEPROM. Is stored in two variables to allow interger size.
  Serial.print("Value stored in EEPROM - As read in Setup routine: "); Serial.println( Door_opening_counter);  

  Serial.print("As a binary: ");
  for (int i = 15; i >=0; i--){
    Serial.print(BIT(Door_opening_counter,i));
  }
  Serial.println("");
  
  delay(500);
  
  // LEDs strip
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);  
  FastLED.setBrightness(LOW_BRIGHTNESS);  

  //delay(2000);
  Vcc=int(readVcc());
  Serial.print("VCC=");Serial.print(Vcc);Serial.println("mV");
  brightness_level=SetBrightness();  // Evaluate LEDs brightness as a function of battery VCC
  CHECK_BATTERY_LED(Vcc);
  
  delay(2000);
  alloff();
  
  start_time=millis();
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {confetti, sinelon, juggle, bpm, rainbow};
char* SimplePatternNames[]={"confetti","sinelon", "juggle", "bpm", "rainbow" };
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
uint8_t gFrameCount = 0; // Inc by 1 for each Frame of Transition, New/Changed connection(s) pattern

void ShowCounter(int number2show){
    digitalWrite(MOSFET_GATE,HIGH);
    digitalWrite(INDICATOR_LED,HIGH);  // Indicator LED is turned ON/OFF to mimic MOSFET status
    FastLED.setBrightness(LOW_BRIGHTNESS); 
    
    for (int i = NUM_LEDS-1; i >=0; i--){
      if(i<21 || i>=37){leds[i]=CRGB::Green;}
      else {
        if(BIT(number2show,i-21)==1){leds[i]=CRGB::Blue;}
        else {leds[i]=CRGB::Black;}
      }
    }

// send the 'leds' array out to the actual LED strip
  FastLED.show();
}


void LED(String pattern){
  if (pattern=="All_blue"){
    digitalWrite(MOSFET_GATE,HIGH);
    digitalWrite(INDICATOR_LED,HIGH);  // Indicator LED is turned ON/OFF to mimic MOSFET status
    FastLED.setBrightness(brightness_level); 
    for (int i = NUM_LEDS-1; i >=0; i--){
      leds[i]=CRGB::Blue;
    }
  }
  
  if (pattern=="LED_Animations"){
    digitalWrite(MOSFET_GATE,HIGH);
    digitalWrite(INDICATOR_LED,HIGH);  // Indicator LED is turned ON/OFF to mimic MOSFET status
    FastLED.setBrightness(brightness_level); 
    gPatterns[gCurrentPatternNumber]();
    //confetti();
    //rainbow();
  }
      
  if (pattern=="All_green"){
    digitalWrite(MOSFET_GATE,HIGH);
    digitalWrite(INDICATOR_LED,HIGH);  // Indicator LED is turned ON/OFF to mimic MOSFET status
    FastLED.setBrightness(brightness_level); 
    for (int i = NUM_LEDS-1; i >=0; i--){
      leds[i]=CRGB::Green;
    }
  }
       
  if (pattern=="All_red"){
    digitalWrite(MOSFET_GATE,HIGH);
    digitalWrite(INDICATOR_LED,HIGH);  // Indicator LED is turned ON/OFF to mimic MOSFET status
    FastLED.setBrightness(brightness_level); 
    for (int i = NUM_LEDS-1; i >=0; i--){
      leds[i]=CRGB::Red;
    }
  }
  
  if (pattern=="off"){
    for (int i = NUM_LEDS-1; i >=0; i--) {
      //leds[i]=CRGB::Black;
      leds[i].nscale8(230);
    }
    digitalWrite(MOSFET_GATE,LOW);
    digitalWrite(INDICATOR_LED,LOW);  // Indicator LED is turned ON/OFF to mimic MOSFET status
  }

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND);       
  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
}


void CHECK_BATTERY_LED(int vcc){  // bargraph showing battery level
    digitalWrite(MOSFET_GATE,HIGH);
    digitalWrite(INDICATOR_LED,HIGH);  // Indicator LED is turned ON/OFF to mimic MOSFET status
    FastLED.setBrightness(LOW_BRIGHTNESS);
    if (vcc<LOW_VCC){vcc=LOW_VCC;}
    if (vcc>HIGH_VCC){vcc=HIGH_VCC;}
    int pos_led=map(vcc,LOW_VCC,HIGH_VCC,1,NUM_LEDS);
    //Serial.println(vcc);
    //Serial.println(LOW_VCC);
    //Serial.println(HIGH_VCC);
    //Serial.println(pos_led);

    
    for (int i = NUM_LEDS-1; i >=0; i--){
      if (i<=pos_led){
        if (i<=5){leds[i]=CRGB::Red;}
        else if (i>5 && i<=15){leds[i]=CRGB::Orange;}
        else {leds[i]=CRGB::Green;}
        }
      else {leds[i]=CRGB::Black;}
    }
    // send the 'leds' array out to the actual LED strip
    FastLED.show();  
    // insert a delay to keep the framerate modest
    FastLED.delay(1000/FRAMES_PER_SECOND);  
    // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow 
}

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16(13,0,NUM_LEDS);
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void redGlitter() {
  gFrameCount += 1;
  if (gFrameCount % 4 == 1) { // Slow down frame rate
    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(HUE_RED, 0, random8() < 60 ? random8() : random8(64));
    }
  }
}
  
void alloff() {
  for (int i = NUM_LEDS-1; i >=0; i--) {
    leds[i]=CRGB::Black;
    delay(10);
    FastLED.show();
  }
  //FastLED.show();
}

enum {Check_Battery,Show_Counter,Transition_To_Animations,LED_Animations,Transition_To_Idle,Idle,Transition_To_Sleep,Stuck_Opened} condition=Idle;

void loop() {

  switch (condition) {

  case Check_Battery:
    Vcc=int(readVcc());
    Serial.print("VCC=");
    Serial.print(Vcc);
    Serial.println("mV");
    CHECK_BATTERY_LED(Vcc);
    delay(2000);
    alloff();
    condition=Show_Counter;
    break;
      
  case Show_Counter:
    Door_opening_counter = word(EEPROM.read(1), EEPROM.read(2));  // Reading Door_opening_counter from EEPROM. Is stored in two variables to allow interger size. 
    ShowCounter(Door_opening_counter);
    delay(SHOW_COUNTER_DURATION);
    alloff();
    condition=Transition_To_Idle;
    break;  
     
  case Transition_To_Animations:
    delay(20);
    if (Door_Opened){
      Serial.println("*** Entering Transition_To_Animations ***");
      Door_opening_counter = word(EEPROM.read(1), EEPROM.read(2));
      Door_opening_counter=Door_opening_counter+1;
      Serial.print("Door_opening_counter now updated to: ");Serial.println(Door_opening_counter);
      EEPROM.write(1, highByte(Door_opening_counter ));
      EEPROM.write(2, lowByte(Door_opening_counter ));
      brightness_level=SetBrightness();  // Evaluate LEDs brightness as a function of battery VCC
      condition=LED_Animations;
      break;
    }
    else 
    {
      Serial.println("Door is not open - Go to Idle");
      //sleepNow();
      condition=Idle;
      break;
    }
    
  case LED_Animations:      
    if (Door_Opened && Ready_4_Change) { 
      //delay(20); // debouncing
      Ready_4_Change=false;
      nextPattern();  // change light patterns when button is pressed
      start_time=millis();  // restart counter to enjoy new pattern longer
    }
    if (!Door_Opened){Ready_4_Change=true;}  // Triggered when door is confirmed closed
    if (!Door_Opened && (millis()-start_time>SET_LED_Animations_MILLISECONDS)){condition=Check_Battery;start_time=millis();}  // End of animations when door is closed
    if (Door_Opened && (millis()-start_time>SET_MAX_Animations_Time_MILLISECONDS)){Serial.println("*** Door remained open ***");condition=Stuck_Opened;start_time=millis();}  // End of animations when door remains opened
    
    else {LED("LED_Animations");}
    break;
    
   case Transition_To_Idle:
    if (millis()-start_time>IDLE_TRANSITION){Serial.println("*** Moving to Idle ***");condition=Idle;}
    else LED("All_blue");
    break;
    
   case Idle:
    //if (millis()-start_time>SET_Idle_MILLISECONDS){condition=Transition_To_Sleep;start_time=millis();}
    if (Door_Opened){condition=Transition_To_Animations;Ready_4_Change=false;start_time=millis();}
    else LED("off");
    break; 
  
  case Transition_To_Sleep:
    if (millis()-start_time>SLEEP_TRANSITION){Serial.println("*** Moving to sleepNow ***");sleepNow();}
    else LED("All_blue");
    break;   
    
  case Stuck_Opened:
    if (!Door_Opened){Serial.println("*** Door now closing ***");condition=Idle;Ready_4_Change=false;start_time=millis();}
    else LED("off");
    break; 
  }
  FastLED.show();
}



void sleepNow(void)
{
    // Set pin 2 as interrupt and attach handler:
    //attachInterrupt(0, pinInterrupt, LOW);
    //attachInterrupt(0, pinInterrupt, CHANGE);
    attachInterrupt(0, pinInterrupt, RISING);
    delay(100);
    //
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    //
    // Set sleep enable (SE) bit:
    sleep_enable();
    //
    // Put the device to sleep:
    digitalWrite(MOSFET_GATE,LOW);   // turn LEDs off to indicate sleep
    digitalWrite(INDICATOR_LED,LOW);  // Indicator LED is turned ON/OFF to mimic MOSFET status
    Serial.println("Falling asleep... ");
    delay(100);
    
    sleep_mode();
    //
    // Upon waking up, sketch continues from this point.
    sleep_disable();
    digitalWrite(MOSFET_GATE,HIGH);   // turn LED on to indicate awake 
digitalWrite(INDICATOR_LED,HIGH);  // Indicator LED is turned ON/OFF to mimic MOSFET status
    delay(100);

    Serial.println("Restarting... ");
    
    //condition=Check_Battery;
    condition=Transition_To_Animations;
    start_time=millis();
}

void pinInterrupt(void)
{
    detachInterrupt(0);
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

uint8_t SetBrightness() {
  int vcc=readVcc();
  if (vcc<LOW_VCC){vcc=LOW_VCC;}
  if (vcc>HIGH_VCC){vcc=HIGH_VCC;}
  int brightness_level=map(vcc,MIN_BRIGHTNESS_VCC,MAX_BRIGHTNESS_VCC,LOW_BRIGHTNESS,HIGH_BRIGHTNESS);
  Serial.print("VCC=");Serial.print(vcc);Serial.print(" therefore brightness_level is set to ");Serial.println(brightness_level);
  return brightness_level;
}

