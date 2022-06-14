#include <FastLED.h>

#define LED_PIN 12
#define NUM_LEDS 4  // 4 LEDs in the full strip
#define BRIGHTNESS 255
#define LED_TYPE WS2811
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];
CRGB pattern = CRGB::White;

void led_begin() {
  // led setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  // Apply the pattern repeating across all LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = pattern;
  }
  // Apply the colors to the LED strip
  FastLED.show();
}
