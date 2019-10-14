#define ESP32_VIRTUAL_DRIVER true //To enable virtual pin driver for esp32
#define ESP_VIRTUAL_DRIVER_7 1 //to use the 7:1 ratio
#define NBIS2SERIALPINS 8
#define NUM_LEDS_PER_STRIP 256 //the length of your strip if you have different strips size put here the longuest strip

/*
This needs to be declared before #include "FastLed.h"
NBIS2SERIALPINS represents the number of pins used by the esp value, it ranges from 1->15
NUM_LEDS_PER_STRIP the length of your strip if you have different strips size put here the longuest strip
for each pin you would be able to drive 7 virtuals pins
i.e for
#define NBIS2SERIALPINS 8
You will be able to drive up to 8*7=56 pins
If you want to have a number of strips which is not a multiple of 7 no issue
if you want to drive 64 pins you would need #define NBIS2SERIALPINS 10
if you want to drive 17 pins you would need #define NBIS2SERIALPINS 3
*/
#include "FastLED.h"
#define LATCH_PIN 13
#define CLOCK_PIN 27 //for a reason I don't know, the CLOCK_PIN needs to be >=16


#define NUM_STRIPS NBIS2SERIALPINS * 7
#define NUM_LEDS NUM_LEDS_PER_STRIP * NUM_STRIPS
CRGB leds[NUM_LEDS];

int Pins[NBIS2SERIALPINS]={12,14,26,25,33,32,15,18};

void setup() {
    Serial.begin(115200);
    /*
    * Define the esp pins you are using
    * pin 12 will drive strip 1->7
    * pin 14 will drive strip 8->14
    * pin 26 will drive strip 15->21
    * pin 25 will drive strip 22->28
    * pin 33 will drive stip  29->35
    * pin 32 will drive strip 34->42
    * pin 15 will drive strip 43->49
    * pin 18 will drive strip 50->56
    */



    FastLED.addLeds<VIRTUAL_DRIVER,Pins,CLOCK_PIN, LATCH_PIN>(leds,NUM_LEDS_PER_STRIP);
    FastLED.setBrightness(64);

}
int start=0;
void loop() {
    fill_solid(leds, NUM_LEDS, CRGB(15,15,15));
    /*
    * this code will create a snake on each strip where the length is the strips number
    */
    for(int i=0;i<NUM_STRIPS;i++)
    {
        int offset=i*NUM_LEDS_PER_STRIP;   //this is the offest of the strip number i
        for(int k=0;k<i+1;k++)
        {
            leds[(start+k)%NUM_LEDS_PER_STRIP+offset]=CHSV(i*255/NUM_STRIPS,255,255);
        }

    }
    long     lastHandle = __clock_cycles();
    FastLED.show();
    long   lasthandle2=__clock_cycles();
    Serial.printf("FPS fastled: %f \n", (float) 240000000L/(lasthandle2 - lastHandle));
    start++;
    delay(30);

}
