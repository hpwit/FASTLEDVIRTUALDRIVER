#define ESP32_VIRTUAL_DRIVER true //To enable virtual pin driver for esp32
#define NBIS2SERIALPINS 8 

/*
 This needs to be declared before #include "FastLed.h" 
 NBIS2SERIALPINS represent the number of pins used by the esp value, it ranges from 1->20  
for each pin you would be able to drive 5 virtuals pins 
i.e for
#define NBIS2SERIALPINS 8   
You will be able to drive up to 8*5=40 pins
If you want to have a number of strips which is not a multiple of 5 no issue 
if you want to drive 21 pins you would need #define NBIS2SERIALPINS 5 
if you want to drive 17 pins you would need #define NBIS2SERIALPINS 4
*/
#include "FastLED.h"`
#define LATCH_PIN 12
#define CLOCK_PIN 27 //for a reason I don't know, the CLOCK_PIN needs to be >=16

#define NUM_LEDS_PER_STRIP 256 //the length of your strip if you have different strips size put here the longuest strip
#define NUM_STRIPS 38
#define NUM_LEDS NUM_LEDS_PER_STRIP * NUM_STRIPS
CRGB leds[NUM_LEDS];

int Pins[NBIS2SERIALPINS]={13,14,26,25,33,32,15,18};

void setup() {

/*
 * Define the esp pins you are using
 * pin 13 will drive strip 1->5
 * pin 14 will drive strip 6->10
 * pin 26 will drive strip 11->15
 * pin 25 will drive strip 16->20
 * pin 33 will drive stip 21->25
 * pin 32 will drive strip 26->30
 * pin 15 will drive strip 31->35
 * pin 18 will drive strip 36->40
 */



FastLED.addLeds<VIRTUAL_DRIVER,Pins,CLOCK_PIN, LATCH_PIN,NUM_LEDS_PER_STRIP>(leds,NUM_LEDS);
 FastLED.setBrightness(128);

}

void loop() {

   /*
  * this code will light up as many pixels as the strip number 
  */
  for(int i=0;i<NUM_STRIPS;i++)
  {
    int offset=i*NUM_LEDS_PER_STRIP;   //this is the offest of the strip number i
    for(int k=0;k<i+1;k++)
    {
      leds[k+offset]=CRGB::Blue;
    }
    
  }
  FastLED.show();
  

}
