#include <avr/io.h>
#include <util/delay.h>
#include "Arduino.h"
#include <string.h>
#include <stdlib.h>
#include <Wire.h>

#include <FastLED.h>

#define OFF_MODE 0x00
#define BLINK_MODE 0x01
#define ROTATE_MODE 0x02
#define DEBUG_1_MODE 0x11
#define DEBUG_2_MODE 0x12
#define DEBUG_3_MODE 0x13
#define DEBUG_MODE 0x21

const int count[3] = {24, 16, 12};
#define TOTAL_LEDS 52
const int max_time = count[0] * count[1] * count[2];

#define PIN 2

struct Color{
    Color(){
        red = 0;
        green = 0;
        blue = 0;
    }
    Color(uint8_t _red, uint8_t _green, uint8_t _blue){
        red = _red;
        green = _green;
        blue = _blue;
    }

    int8_t red;
    int8_t green;
    int8_t blue;
};

Color color1;
Color color2;
Color color3;

uint8_t mode = OFF_MODE;

void callback(int bytes){
    if (bytes == 4){

        int cmd = Wire.read();
        int red = Wire.read();
        int green = Wire.read();
        int blue = Wire.read();

        Color color(red, green, blue);

        switch (cmd){
            case OFF_MODE:
                mode = OFF_MODE;
                break;

            case BLINK_MODE:
                mode = BLINK_MODE;
                break;

            case ROTATE_MODE:
                mode = ROTATE_MODE;
                break;

            case DEBUG_1_MODE:
                mode = DEBUG_MODE;
                color1 = color;
                break;

            case DEBUG_2_MODE:
                mode = DEBUG_MODE;
                color2 = color;
                break;

            case DEBUG_3_MODE:
                mode = DEBUG_MODE;
                color3 = color;
                break;

            case DEBUG_MODE:
                mode = DEBUG_MODE;
                break;

            default:

                break;
        }
    }
}


int main(void)
{
    init();

    Wire.begin(0x08);
    Wire.onReceive(callback);
    Serial.begin(57600);

    CRGBArray<TOTAL_LEDS> leds;
    FastLED.addLeds<WS2812B, PIN, GRB>(leds, TOTAL_LEDS);
    leds[0].b = 255;

    int time = 0;

    while (true){

        switch (mode){

            case OFF_MODE:
                // Turn off all leds
                for (int i = 0; i < TOTAL_LEDS; i++){
                    leds[i] = CRGB::Black;
                }
                break;

            case BLINK_MODE:
                // Blink all leds
                for (int i = 0; i < TOTAL_LEDS; i++){
                    if (time > max_time / 2) leds[i] = CRGB::White;
                    else leds[i] = CRGB::Black;
                }
                break;

            case ROTATE_MODE:
                // Turn off all leds
                for (int i = 0; i < TOTAL_LEDS; i++){
                    leds[i] = CRGB::Black;
                }

                // Rotate outer ring
                for (int i = 0; i < count[0] / 2; i++){
                    leds[(time / (count[1] * count[2]) + i) % count[0]] = CRGB::Blue;
                }
                // Rotate middle ring
                for (int i = 0; i < count[1] / 2; i++){
                    leds[count[0] + (time / (count[0] * count[2]) + i) % count[1]] = CRGB::Blue;
                }
                // Rotate inner ring
                for (int i = 0; i < count[2] / 2; i++){
                    leds[count[0] + count[1] + (time / (count[0] * count[1]) + i) % count[2]] = CRGB::Blue;
                }
                break;

            case DEBUG_MODE:
                // Show outer ring
                for (int i = 0; i < count[0]; i++){
                    leds[i].r = color1.red;
                    leds[i].g = color1.green;
                    leds[i].b = color1.blue;
                }
                // Show middle ring
                for (int i = 0; i < count[1]; i++){
                    leds[count[0] + i].r = color2.red;
                    leds[count[0] + i].g = color2.green;
                    leds[count[0] + i].b = color2.blue;
                }
                // Show inner ring
                for (int i = 0; i < count[2]; i++){
                    leds[count[0] + count[1] + i].r = color3.red;
                    leds[count[0] + count[1] + i].g = color3.green;
                    leds[count[0] + count[1] + i].b = color3.blue;
                }
                break;
        }

        Serial.println(mode, HEX);
        Serial.println(0, HEX);

        FastLED.show();

        time += 36;
        if (time == max_time) time = 0;
    }


    return 0;
}
