/*
 * Animated GIFs Display Code for SmartMatrix and HUB75 RGB LED Panels
 *
 * Uses SmartMatrix Library written by Louis Beaudoin at pixelmatix.com
 *
 * Written by: Craig A. Lindley
 *
 * Copyright (c) 2014 Craig A. Lindley
 * Refactoring by Louis Beaudoin (Pixelmatix)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * This SmartMatrix Library example displays GIF animations loaded from a SD Card connected to the Teensy 3
 *
 * The example can be modified to drive displays other than SmartMatrix by replacing SmartMatrix Library calls in setup() and
 * the *Callback() functions with calls to a different library (look for the USE_SMARTMATRIX and ENABLE_SCROLLING blocks and replace)
 *
 * This code has been tested with many size GIFs including 128x32, 64x64, 32x32, and 16x16 pixel GIFs, but is optimized for 32x32 pixel GIFs.
 *
 * Wiring is on the default Teensy 3.2 SPI pins, and chip select can be on any GPIO,
 * set by defining SD_CS in the code below.  For Teensy 3.5/3.6/4.1 with the onboard SDIO, SD_CS should be the default BUILTIN_SDCARD
 * Function     | Pin
 * DOUT         |  11
 * DIN          |  12
 * CLK          |  13
 * CS (default) |  15
 *
 * Wiring for ESP32 follows the default for the ESP32 SD Library, see: https://github.com/espressif/arduino-esp32/tree/master/libraries/SD
 *
 * This code first looks for .gif files in the /gifs/ directory
 * (customize below with the GIF_DIRECTORY definition) then plays random GIFs in the directory,
 * looping each GIF for DISPLAY_TIME_SECONDS
 *
 * This example is meant to give you an idea of how to add GIF playback to your own sketch.
 * For a project that adds GIF playback with other features, take a look at
 * Light Appliance and Aurora:
 * https://github.com/CraigLindley/LightAppliance
 * https://github.com/pixelmatix/aurora
 *
 * If you find any GIFs that won't play properly, please attach them to a new
 * Issue post in the GitHub repo here:
 * https://github.com/pixelmatix/AnimatedGIFs/issues
 */

/*
 * CONFIGURATION:
 *  - If you're not using SmartLED Shield V4 (or above), comment out the line that includes <SmartMatrixShieldV4.h>
 *  - update the "SmartMatrix configuration and memory allocation" section to match the width and height and other configuration of your display
 *  - Note for 128x32 and 64x64 displays with Teensy 3.2 - need to reduce RAM:
 *    set kRefreshDepth=24 and kDmaBufferRows=2 or set USB Type: "None" in Arduino,
 *    decrease refreshRate in setup() to 90 or lower to get good an accurate GIF frame rate
 *  - Set the chip select pin for your board.  On Teensy 3.5/3.6, the onboard microSD CS pin is "BUILTIN_SDCARD"
 *  - For ESP32 and large panels, you don't need to lower the refreshRate, but you can lower the frameRate (number of times the refresh buffer
 *    is updaed with new data per second), giving more time for the CPU to decode the GIF.
 *    Use matrix.setMaxCalculationCpuPercentage() or matrix.setCalcRefreshRateDivider()
 */

#include <MatrixHardware_T4Adapter.h>   // Teensy 4 Adapter attached to SmartLED Shield for Teensy 3 V4
#include <SmartMatrix3.h>
#include "Layer_BackgroundInterpolation.h"

#include <SD.h>
#include <GifDecoder.h>
#include "FilenameFunctions.h"

#include "fscale.h"

#define DISPLAY_TIME_SECONDS (5*60)
#define NUMBER_FULL_CYCLES   1

#define USE_SMARTMATRIX         1
#define ENABLE_SCROLLING        1
#define START_WITH_RANDOM_GIF   1

#define DEBUG_PRINT_FRAMESTATS    0

// range 0-255
const int defaultBrightness = 255;

#if (USE_SMARTMATRIX == 1)
/* SmartMatrix configuration and memory allocation */
#define COLOR_DEPTH 24                  // known working: 24, 48 - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24
#if 1
const uint8_t kMatrixWidth = 96;        // known working: 32, 64, 96, 128
const uint8_t kMatrixHeight = 96;       // known working: 16, 32, 48, 64
const uint8_t kPanelType = SMARTMATRIX_HUB75_32ROW_MOD16SCAN; // use SMARTMATRIX_HUB75_16ROW_MOD8SCAN for common 16x32 panels, or use SMARTMATRIX_HUB75_64ROW_MOD32SCAN for common 64x64 panels
#else
const uint8_t kMatrixWidth = 64;        // known working: 32, 64, 96, 128
const uint8_t kMatrixHeight = 64;       // known working: 16, 32, 48, 64
const uint8_t kPanelType = SMARTMATRIX_HUB75_64ROW_MOD32SCAN; // use SMARTMATRIX_HUB75_16ROW_MOD8SCAN for common 16x32 panels, or use SMARTMATRIX_HUB75_64ROW_MOD32SCAN for common 64x64 panels
#endif
const uint8_t kRefreshDepth = 36;       // known working: 24, 36, 48
const uint8_t kDmaBufferRows = 2;       // known working: 2-4
const uint8_t kMatrixOptions = (SMARTMATRIX_OPTIONS_BOTTOM_TO_TOP_STACKING);    // see http://docs.pixelmatix.com/SmartMatrix for options
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);
const uint8_t kScrollingLayerOptions = (SM_SCROLLING_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);

#if 0
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);
#else
SMARTMATRIX_ALLOCATE_BACKGROUND_INTERPOLATION_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);
#endif

#if (ENABLE_SCROLLING == 1)
SMARTMATRIX_ALLOCATE_SCROLLING_LAYER(scrollingLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kScrollingLayerOptions);
#endif
#endif

const SM_RGB COLOR_BLACK = {
    0, 0, 0 };

/* template parameters are maxGifWidth, maxGifHeight, lzwMaxBits
 * 
 * The lzwMaxBits value of 12 supports all GIFs, but uses 16kB RAM
 * lzwMaxBits can be set to 10 or 11 for smaller displays to save RAM, but use 12 for large displays
 * All 32x32-pixel GIFs tested so far work with 11, most work with 10
 */
GifDecoder<kMatrixWidth, kMatrixHeight, 12> decoder;

// Chip select for SD card
#if defined(ESP32)
    #define SD_CS 5
#elif defined (ARDUINO)
    #define SD_CS BUILTIN_SDCARD
    //#define SD_CS 15
#endif

#if defined(ESP32)
    // ESP32 SD Library can't handle a trailing slash in the directory name
    #define GIF_DIRECTORY "/gifs"
#else
    // Teensy SD Library requires a trailing slash in the directory name
    #define GIF_DIRECTORY "/gifs/"
#endif

int num_files;

void screenClearCallback(void) {
#if (USE_SMARTMATRIX == 1)
  backgroundLayer.fillScreen({0,0,0});
#endif
}

void updateScreenCallback(void) {
#if (USE_SMARTMATRIX == 1)
  backgroundLayer.swapBuffers();
#endif
}

void drawPixelCallback(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue) {
#if (USE_SMARTMATRIX == 1)
    backgroundLayer.drawPixel(x, y, rgb24{red, green, blue});
#endif
}

const int readSensorPeriod_ms = 10;
unsigned long lastSensorRead_millis;

// modified Arduino smoothing example
const int analogInPin = A9;  // Analog input pin that the potentiometer is attached to
const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
long total = 0;                  // the running total
double average = 0;                // the average
const float curveBoundary = 1024.0/2;

float curve = 0.0;

float getSliderReading(void) {
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = analogRead(analogInPin);
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    float temp = (float)total / numReadings;

#if 0
    Serial.print(temp);
    Serial.print(" ");

    // I don't know why this delay is needed, but if it's not somewhere in this function (and not before or after the function call, the sketch will stop in some way and the screen will blank frequently
    // It's the combination of the second call to Serial.print(), calling fscale, and having a large panel size that seems to be causing this behavior
    // delay must be in this function, as in there's something important on the stack?
    delayMicroseconds(1);
#endif

    // It's not easy or maybe even possible to choose a single expoential curve that gives good resolution in the lower range, and changes quickly over the higher range, split it into two curves
    if(temp < curveBoundary)
        return fscale( 0, curveBoundary, 1.0, 25.0, temp, 0.0);
    else
        return fscale( curveBoundary, 1023.0, 25.0, 1023, temp, curve);
}

void initSliderReading(void) {
    // initialize all the readings to 0:
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      readings[thisReading] = 0;
    }
}

// Setup method runs once, when the sketch starts
void setup() {
    decoder.setScreenClearCallback(screenClearCallback);
    decoder.setUpdateScreenCallback(updateScreenCallback);
    decoder.setDrawPixelCallback(drawPixelCallback);

    decoder.setFileSeekCallback(fileSeekCallback);
    decoder.setFilePositionCallback(filePositionCallback);
    decoder.setFileReadCallback(fileReadCallback);
    decoder.setFileReadBlockCallback(fileReadBlockCallback);

    initSliderReading();

#if (START_WITH_RANDOM_GIF == 1)
    // Seed the random number generator
    randomSeed(analogRead(14));
#endif

    Serial.begin(115200);
    Serial.setTimeout(1);

    Serial.println("Starting AnimatedGIFs Sketch");


#if (USE_SMARTMATRIX == 1)
    // Initialize matrix
    matrix.addLayer(&backgroundLayer); 
#if (ENABLE_SCROLLING == 1)
    matrix.addLayer(&scrollingLayer); 
#endif

    matrix.setBrightness(defaultBrightness);
    matrix.setRefreshRate(250);

#if !defined(ESP32)
    matrix.begin();
#endif

#if defined(ESP32)
    // for large panels on ESP32, may want to set the max percentage time dedicated to updating the refresh frames lower, to leave more CPU time to decoding GIFs (needed if GIFs are playing back slowly)
    //matrix.setMaxCalculationCpuPercentage(50);

    // alternatively, for large panels on ESP32, may want to set the calculation refresh rate divider lower to leave more CPU time to decoding GIFs (needed if GIFs are playing back slowly) - this has the same effect as matrix.setMaxCalculationCpuPercentage() but is set with a different parameter
    //matrix.setCalcRefreshRateDivider(4);

    // The ESP32 SD Card library is going to want to malloc about 28000 bytes of DMA-capable RAM, make sure at least that much is left free
    matrix.begin(28000);
#endif

    // Clear screen
    backgroundLayer.fillScreen(COLOR_BLACK);
    backgroundLayer.swapBuffers(false);
#endif

    if(initFileSystem(SD_CS) < 0) {
#if (ENABLE_SCROLLING == 1)
        scrollingLayer.start("No SD card", -1);
#endif
        Serial.println("No SD card");
        while(1);
    }

    // Determine how many animated GIF files exist
    num_files = enumerateGIFFiles(GIF_DIRECTORY, true);

    if(num_files < 0) {
#if (ENABLE_SCROLLING == 1)
        scrollingLayer.start("No gifs directory", -1);
#endif
        Serial.println("No gifs directory");
        while(1);
    }

    if(!num_files) {
#if (ENABLE_SCROLLING == 1)
        scrollingLayer.start("Empty gifs directory", -1);
#endif
        Serial.println("Empty gifs directory");
        while(1);
    }
}

double frameDelayMultiplier = 7.0;

bool frameDelayMultiplierUpdated = false;

void loop() {
    static unsigned long displayEndTime_millis, frameStartTime_micros;
    static unsigned int currentFrameDelay_ms, nMinus2FrameDelay_ms, nMinus1FrameDelay_ms;
    unsigned long now = millis();

    // TODO: we need to determine the exponent of the curve through trial and error, allow easily changing curve over Serial
    float tempfloat = Serial.parseFloat();
    if(tempfloat) {
        Serial.print("Got: ");
      Serial.println(tempfloat, 10);

      curve = tempfloat;
    }

#if (START_WITH_RANDOM_GIF == 1)
    static int index = random(num_files);
#else
     static int index = 0;
#endif   

    // default behavior is to play the gif for DISPLAY_TIME_SECONDS or for NUMBER_FULL_CYCLES, whichever comes first
#if 0
    if(now >= displayEndTime_millis || decoder.getCycleNo() > NUMBER_FULL_CYCLES)
#else
    // alt behavior is to play the gif until both DISPLAY_TIME_SECONDS and NUMBER_FULL_CYCLES have passed
    if(now >= displayEndTime_millis && decoder.getCycleNo() > NUMBER_FULL_CYCLES)
#endif
    {
        if (openGifFilenameByIndex(GIF_DIRECTORY, index) >= 0) {
            // Can clear screen for new animation here, but this might cause flicker with short animations
            // matrix.fillScreen(COLOR_BLACK);
            // matrix.swapBuffers();

            decoder.startDecoding();

            // Calculate time in the future to terminate animation
            displayEndTime_millis = now + (DISPLAY_TIME_SECONDS * 1000);
        }

        // get the index for the next pass through
        if (++index >= num_files) {
            index = 0;
        }
    }

    // decode new frame (n), but don't delay, and don't display it
    if(decoder.decodeFrame(false) == ERROR_DONE_PARSING) {
        // when the GIF wraps, decodeFrame doesn't actually decode a frame, start loop() again to either get next frame or switch to new GIF
        return;
    }

#if (DEBUG_PRINT_FRAMESTATS == 1)
    int timeToDecode_ms = millis() - now;
#endif

    // get the delay associated with the current frame (n), the one we're interpolating *to* with the next swapBuffers call
    currentFrameDelay_ms = decoder.getFrameDelay_ms();

    uint32_t t;
    int32_t microsUntilChange;

    // wait for the delay associated with the frame (n-2), the frame that's currently the "previous" frame in backgroundLayer
    do {
        if(millis() - lastSensorRead_millis > readSensorPeriod_ms) {
            lastSensorRead_millis = millis();


            average = (double)getSliderReading();
            Serial.println(average);

            if(average > 1.0)
                frameDelayMultiplier = (1000.0 * average) / 1024;
            else
                frameDelayMultiplier = 1.0;

            frameDelayMultiplierUpdated = true;
        }

        t = micros();

        microsUntilChange = ((nMinus2FrameDelay_ms * 1000) * frameDelayMultiplier) - (t - frameStartTime_micros);

        if(frameDelayMultiplierUpdated) {
            frameDelayMultiplierUpdated = false;

            backgroundLayer.updateInterpolationPeriod(microsUntilChange);
        }
    } while (microsUntilChange > 0);

    // we're now done with frame (n-2).  frame (n-1) is being displayed.  we're going to start interpolating from frame (n-1) to new frame (n)

    // capture the timestamp of when we last called swapBuffers() to begin interpolation between frame (n-1) and (n).  We'll call swapBuffers() again after waiting nMinus1FrameDelay_micros, regardless of how long the next call to decoder.decodeFrame(false) takes
    frameStartTime_micros = t;

    // swap buffers, begin interpolation between frame (n-1) and (n)
    backgroundLayer.swapBuffers(true, (nMinus1FrameDelay_ms * 1000) * frameDelayMultiplier);

    // we're done with this frame loop, setup for the next loop where frame (n) becomes (n-1), (n-1) becomes (n-2)
    nMinus2FrameDelay_ms = nMinus1FrameDelay_ms;
    nMinus1FrameDelay_ms = currentFrameDelay_ms;

#if (DEBUG_PRINT_FRAMESTATS == 1)
    Serial.print(decoder.getFrameNo());
    Serial.print(" ");
    Serial.print(decoder.getFrameDelay_ms());
    Serial.print(" ");
    Serial.print(matrix.getRefreshRate());
    Serial.print(" ");
    Serial.println(timeToDecode_ms);
#endif
}
