#include <FastLED.h>

#define LED_PIN     4
#define NUM_LEDS    4 // 150 LEDs in the full strip
#define PATTERN_LEN 1
#define BRIGHTNESS  255
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB

// If you want the pattern to move across the strip, change this to
// a number greater than zero
#define UPDATES_PER_SECOND 0


CRGB leds[NUM_LEDS];
CRGB pattern[PATTERN_LEN] = { CRGB::White };

void setup() {
    delay( 3000 ); // power-up safety delay
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS );
}


void loop()
{
    // Start at the first LED
    static uint8_t startIndex = 0;

    // Apply the pattern repeating across all LEDs
    for( int i =0; i < NUM_LEDS; i++) {
        leds[i] = pattern[(PATTERN_LEN - startIndex + i) % PATTERN_LEN];
    }

    // Apply the colors to the LED strip     
    FastLED.show();

    delay(1000);
}