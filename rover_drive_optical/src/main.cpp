#include <Arduino.h>
#include <pid_loops.h>


void setup() {
    Serial.begin(9600);
    pinMode(PIN_SS_LEFT, OUTPUT);
    pinMode(PIN_SS_RIGHT, OUTPUT);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_MOSI, OUTPUT);
    pinMode(PIN_SCK, OUTPUT);

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS );

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);

    robot.begin();

    set_left_optical_cs(true);
    if (mousecam_init() == -1) {
        Serial.println("Left optical flow sensor failed to init");
        while (1);
    } else {
        Serial.println("Left optical flow sensor initialised.");
    }

    set_left_optical_cs(false);
    if (mousecam_init() == -1) {
        Serial.println("Right optical flow sensor failed to init");
        while (1);
    } else {
        Serial.println("Right optical flow sensor initialised.");
    }
}

    void loop() {
        // Start at the first LED
    static uint8_t startIndex = 0;

    // Apply the pattern repeating across all LEDs
    for( int i =0; i < NUM_LEDS; i++) {
        leds[i] = pattern[(PATTERN_LEN - startIndex + i) % PATTERN_LEN];
    }

    // Apply the colors to the LED strip     
    FastLED.show();

    rover_straight(300);
    delay(3000);
    rover_straight(-300);
    delay(3000);
}