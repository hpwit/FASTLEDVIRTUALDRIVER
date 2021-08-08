
#define STOPLED 50 // number of the first led that will not show
#define DECAL 30 //length of the stip that will not be shown
#define FASTLED_ALLOW_INTERRUPTS 0
#include "FastLED.h"
#include "Arduino.h"
#define DATA_PIN 4
#define NUM_LEDS 256
CRGB leds[NUM_LEDS+DECAL];


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
Serial.println("ee");
FastLED.setBrightness(10);
memset(leds,0,NUM_LEDS*3);
FastLED.show();
//start++;
delay(1000);
}

int start=0;

void loop() {
  
for(int i=0;i<STOPLED;i++)
{ 
  leds[i]=CRGB::Green;
}
//you will never see the blue color
for(int i=STOPLED;i<STOPLED+DECAL;i++)
{ 
  leds[i]=CRGB::Blue;
}
for(int i=STOPLED+DECAL;i<NUM_LEDS+DECAL;i++)
{ 
  leds[i]=CRGB::Red;
}
leds[start%(NUM_LEDS+DECAL)]=CRGB::White;

FastLED.show();
start++;
delay(500);
 
}
