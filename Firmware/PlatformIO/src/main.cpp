// This script is cycles the 8 LEDs and takes a reading using each LED being on. The results are display on the G9 Display.

#include "tlc59208.h" // Small homemade definition library for TLC59208
#include <Wire.h>     // The I2C library
#include <Arduino.h>  // Some Arduino thing
#include <unity.h>    // Unit testing for C
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
//#include <Adafruit_NeoPixel.h>
//#include <SPI.h>
//#include <TFT_eSPI.h> // Hardware-specific library
#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>
#include <math.h>
#include <WiFi.h>
// ML
#include <EloquentTinyML.h>
//#include <eloquent_tinyml/tensorflow.h>
#include "barbie.h"
#define N_INPUTS 12
#define N_RESULTS 1
// in future projects you may need to tweak this value: it's a trial and error process
#define TENSOR_ARENA_SIZE 2 * 1024

#define NUMPIXELS 1

//TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
NAU7802 nau;
TLC59208 tlc;
//Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

LilyGo_RGBPanel panel;

static void slider_event_cb(lv_event_t *e);
static lv_obj_t *slider_label;

Eloquent::TinyML::TfLite<N_INPUTS, N_RESULTS, TENSOR_ARENA_SIZE> tf;

static const int N_OUTPUTS = 8; // Number of LEDs to control
static const int RESET = 2;     // //Define reset pin for TLC59208
static const int ADDR = 0x20;   // The TLC59208 I2C address.

void setup()
{
    Serial.begin(115200);
    delay(100); // Startup delay for devices w/o software reset
    Serial.println("boot");
    pinMode(5, INPUT_PULLUP);

    //NEOPIXEL SETUP
    // pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    // pixels.setBrightness(80);
    // pixels.fill(0xFF0000); // Set color of NEO Pixel to RED.
    // pixels.show();         // Indicate the device is now running code from setup.

    Wire.begin(); // Define the I2C library for use.
    tlc.begin();  // Define the TLC library for use.

    // NAU7802 SETUP //
    if (nau.begin() == false)
    {
        Serial.println("NAU7802 not detected.");
    }
    Serial.println("NAU7802 detected!");

    // Initialize T-RGB, if the initialization fails, false will be returned.
    if (!panel.begin()) {
        while (1) {
            Serial.println("Error, failed to initialize T-RGB"); delay(1000);
        }
    }
    // Call lvgl initialization
    beginLvglHelper(panel);

    /*Create a slider in the center of the display*/
    lv_obj_t *slider = lv_slider_create(lv_scr_act());
    lv_obj_set_width(slider, LV_PCT(80));
    lv_obj_center(slider);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /*Create a label below the slider*/
    slider_label = lv_label_create(lv_scr_act());
    lv_label_set_text(slider_label, "0%");

    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    panel.setBrightness(16);
    // TFT SETUP
    // tft.init();
    // tft.fillScreen(TFT_DARKGREEN);
    // tft.setTextColor(TFT_WHITE);
    // tft.drawCentreString("PlasticScanner", 120, 120, 4);
    // tft.drawCentreString("Ready", 120, 140, 4);
    // tft.setTextDatum(MC_DATUM);

    // pixels.fill(0xE6CC00); // Set color of NEO Pixel to YELLOW.
    // pixels.show();         // Indicate the device is now done with setup.

    nau.setGain(NAU7802_GAIN_4);
    // float theSampleRate = nau.setSampleRate;
    // Serial.println(theSampleRate);

    // TF INIT
//    tf.begin(barbie);
//    // check if model loaded fine
//    if (!tf.isOk())
//    {
//        Serial.print("ERROR: ");
//        Serial.println(tf.getErrorMessage());
//
//        while (true)
//            delay(1000);
//    }
}


void runScan()
{
    // pixels.fill(0x02198B); // Set color of NEO Pixel to BLUE.
    // pixels.show();         // Indicate the device is now running a scan.
    // // Function to blink the 8 LEDs and collect the associated data from the ADC.
    // tft.fillScreen(TFT_DARKGREEN);

    // tft.drawCentreString("Scanning", 120, 120, 4);
    Serial.println("Starting scan.");
    // Get a baseline reading without LEDs active.
    long val = nau.getReading();
    Serial.print("Dark reading: ");
    Serial.println(val);

    delay(500);
    // // Flash the LEDs sequentially and read results from the ADC.
    // tft.drawCentreString("Scanning", 120, 120, 4);
    for (int i = 0; i < N_OUTPUTS; i++)
    {
        // Turn on the numerated LED.
        tlc.on(i);
        // Based on comment from Jerry, they tested that 10 ms is fitting for the light to be on, before taking a reading.
        delay(10);
        // Read data from the IR sensor through the ADC.
        long val = nau.getReading();
        // long avgVal = nau.getAverage(5);

        // Serial.print("Read ");
        Serial.println(val);

        // Calculate the progress in % and combine it with a % sign in a string.
        float progress = float(i) / 8 * 100;
        // String progressOutput = String(progress, 2) + "%";
        String progressOutput = String(val);

        // // Make space on the screen.
        // tft.fillRect(0, 140, 240, 180, TFT_DARKGREEN); // clear a region of the display with the background color
        // tft.drawCentreString(progressOutput, 120, 140, 4);
        // delay(500);
        // tlc.off(i);
        // delay(10);
    }

    // tft.fillScreen(TFT_DARKGREEN);
    // tft.drawCentreString("PP", 120, 100, 4);
    // // tft.drawCentreString("...", 120, 140, 4);

    // Waits for button press.
    while (digitalRead(5) == HIGH)
    {
        // Do nothing
    }

    // tft.fillScreen(TFT_DARKGREEN);
    // tft.drawCentreString("Ready to scan", 120, 110, 4);
}


void loop()
{
    // pixels.fill(0x26580F); // Set color of NEO Pixel to GREEN.
    // pixels.show();         // Indicate the device is now running loop code.
    lv_timer_handler();
    // Waits for button press.
    while (digitalRead(5) == HIGH)
    {
        // Do nothing
    }

    // Execute a scan and display the results.
    UNITY_BEGIN();
    RUN_TEST(runScan);
    UNITY_END();

    // Indicate the scan is done.
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
}

static void slider_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d%%", (int)lv_slider_get_value(slider));
    lv_label_set_text(slider_label, buf);
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}
static void reset()
{
    digitalWrite(RESET, LOW);
    delayMicroseconds(1);
    digitalWrite(RESET, HIGH);
    delayMicroseconds(1);
}

static void setBrightness(int output, int percents)
{
    assert((output >= 0) && (output < N_OUTPUTS));
    assert((percents >= 0) && (percents <= 100));

    uint8_t reg = 0x02 + output;
    uint8_t pwm = (percents * 255) / 100;

    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(pwm);
    Wire.endTransmission();
}
