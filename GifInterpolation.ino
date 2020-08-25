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

#include <Bounce.h>
#include <Encoder.h>

#include "fscale.h"

#define DISPLAY_TIME_SECONDS (5*60)
#define NUMBER_FULL_CYCLES   1

#define ENABLE_SCROLLING        1
#define START_WITH_RANDOM_GIF   1

#define DEBUG_PRINT_FRAMESTATS          0
#define DEBUG_PRINT_ENCODER_UPDATES     0
#define DEBUG_PRINT_BUTTON_UPDATES      0
#define DEBUG_PRINT_SLIDER_UPDATES      0

// range 0-255
const int defaultBrightness = 255;
int brightness = defaultBrightness;
const int brightnessIncrement = 5;
const int maxBrightness = 255;

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

#define ENABLE_APA102_REFRESH   1

#if (ENABLE_APA102_REFRESH == 1)
// adjust this to your APA matrix/strip - set kApaMatrixHeight to 1 for a strip
const uint8_t kApaMatrixWidth = 16;
const uint8_t kApaMatrixHeight = 16;
const uint8_t kApaRefreshDepth = 36;        // known working: 36
const uint8_t kApaDmaBufferRows = 1;        // known working: 1
const uint8_t kApaPanelType = 0;            // not used for APA matrices as of now
const uint8_t kApaMatrixOptions = (SMARTMATRIX_OPTIONS_NONE);      // no options for APA matrices as of not 
const uint8_t kApaBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);

SMARTMATRIX_APA_ALLOCATE_BUFFERS(apamatrix, kApaMatrixWidth, kApaMatrixHeight, kApaRefreshDepth, kApaDmaBufferRows, kApaPanelType, kApaMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(apaBackgroundLayer, kApaMatrixWidth, kApaMatrixHeight, COLOR_DEPTH, kApaBackgroundLayerOptions);
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
  backgroundLayer.fillScreen({0,0,0});
}

void updateScreenCallback(void) {
  backgroundLayer.swapBuffers();
}

void drawPixelCallback(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue) {
    backgroundLayer.drawPixel(x, y, rgb24{red, green, blue});
}

const int readSensorPeriod_ms = 100;
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

const int buttonPin1 = 18;
Bounce pushbutton1 = Bounce(buttonPin1, 10);  // 10 ms debounce
const int buttonPin2 = 21;
Bounce pushbutton2 = Bounce(buttonPin2, 10);  // 10 ms debounce

Encoder knobLeft(16, 17);
Encoder knobRight(19, 20);

long positionLeft  = -999;
long positionRight = -999;

int gifIndex;
bool gifIndexChanged = true;

void checkEncodersState(void) {
    long newLeft, newRight;
    newLeft = knobLeft.read();
    newRight = knobRight.read();

    if (newLeft != positionLeft) {
#if (DEBUG_PRINT_ENCODER_UPDATES == 1)
        Serial.print("Left = ");
        Serial.print(newLeft);
#endif

        // encoder sends four pulses per notch, so we only care if this pulse position is a multiple of 4
        if(!(newLeft % 4)) {
            gifIndexChanged = true;
            if(newLeft > positionLeft) {
                gifIndex++;
#if (DEBUG_PRINT_ENCODER_UPDATES == 1)
                Serial.print(" index++");
#endif
            } else {
                gifIndex--;
#if (DEBUG_PRINT_ENCODER_UPDATES == 1)
                Serial.print(" index--");
#endif
            }
        }

        gifIndex = (gifIndex + num_files) % num_files;

        positionLeft = newLeft;

#if (DEBUG_PRINT_ENCODER_UPDATES == 1)
        Serial.print(" index=");
        Serial.println(gifIndex);
#endif
    }

    if (newRight != positionRight) {
        // encoder sends four pulses per notch, so we only care if this pulse position is a multiple of 4
        if(!(newRight % 4)) {
            if(newRight > positionRight) {
                brightness += brightnessIncrement;
#if (DEBUG_PRINT_ENCODER_UPDATES == 1)
                Serial.print(" brightness++");
#endif
            } else {
                brightness -= brightnessIncrement;
#if (DEBUG_PRINT_ENCODER_UPDATES == 1)
                Serial.print(" brightness--");
#endif
            }
        }

        if(brightness < 0)
            brightness = 0;
        if(brightness > maxBrightness)
            brightness = maxBrightness;

        matrix.setBrightness(brightness);

        positionRight = newRight;
    }
}

void checkButtonsState(void) {
    if (pushbutton1.update() && pushbutton1.fallingEdge()) {
#if (DEBUG_PRINT_BUTTON_UPDATES == 1)
        Serial.println("********* BUTTON 1 **********");
#endif
    }
    if (pushbutton2.update() && pushbutton2.fallingEdge()) {
#if (DEBUG_PRINT_BUTTON_UPDATES == 1)
        Serial.println("********* BUTTON 2 **********");
#endif
    }
}

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

    // If this delay isn't in this function when we print SmartMatrix Library starts dropping frames
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

    pinMode(buttonPin1, INPUT_PULLUP);
    pinMode(buttonPin1, INPUT_PULLUP);

    initSliderReading();

#if (START_WITH_RANDOM_GIF == 1)
    // Seed the random number generator
    randomSeed(analogRead(14));
#endif

    Serial.begin(115200);
    Serial.setTimeout(1);

    Serial.println("Starting AnimatedGIFs Sketch");


    // Initialize matrix
    matrix.addLayer(&backgroundLayer); 
#if (ENABLE_SCROLLING == 1)
    matrix.addLayer(&scrollingLayer); 
#endif

    matrix.setBrightness(defaultBrightness);
    matrix.setRefreshRate(250);


    matrix.begin();

    // Clear screen
    backgroundLayer.fillScreen(COLOR_BLACK);
    backgroundLayer.swapBuffers(false);

#if (ENABLE_APA102_REFRESH == 1)
    // enable the APA102 buffers to drive out the SPI signals
    pinMode(SMARTLED_APA_ENABLE_PIN, OUTPUT);
    digitalWrite(SMARTLED_APA_ENABLE_PIN, HIGH);  // enable access to LEDs    

    apamatrix.addLayer(&apaBackgroundLayer);
    apamatrix.begin();

    // lower the brightness
    apamatrix.setBrightness(255);

    // Clear screen
    apaBackgroundLayer.fillScreen(COLOR_BLACK);
    apaBackgroundLayer.swapBuffers(false);
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

#if (START_WITH_RANDOM_GIF == 1)
    gifIndex = random(num_files);
#else
    gifIndex = 0;
#endif   
}

double frameDelayMultiplier = 7.0;
bool frameDelayMultiplierUpdated = false;

const int indexChangedFrameDelay_ms = 100;

void loop() {
    static unsigned long displayEndTime_millis, frameStartTime_micros;
    static unsigned int currentFrameDelay_ms, nMinus2FrameDelay_ms, nMinus1FrameDelay_ms;
    unsigned long now = millis();
    bool firstFrame = false;

    // TODO: we need to determine the exponent of the curve through trial and error, allow easily changing curve over Serial
    float tempfloat = Serial.parseFloat();
    if(tempfloat) {
        Serial.print("Got: ");
      Serial.println(tempfloat, 10);

      curve = tempfloat;
    }

    // default behavior is to play the gif for DISPLAY_TIME_SECONDS or for NUMBER_FULL_CYCLES, whichever comes first
#if 0
    if(now >= displayEndTime_millis || decoder.getCycleNo() > NUMBER_FULL_CYCLES)
#else
    // alt behavior is to play the gif until both DISPLAY_TIME_SECONDS and NUMBER_FULL_CYCLES have passed
    if(now >= displayEndTime_millis && decoder.getCycleNo() > NUMBER_FULL_CYCLES)
#endif
    {
        gifIndexChanged = true;

        if (++gifIndex >= num_files)
            gifIndex = 0;
    }

    if(gifIndexChanged) {
        gifIndexChanged = false;
        
        // normally we interpolate to the next frame over a potentially long period of time, but we want to see the first frame of the next GIF quickly, so let's bypass that logic
        firstFrame = true;

        // quickly switch to the next frame
        nMinus2FrameDelay_ms = 0;
        nMinus1FrameDelay_ms = 0;
        frameDelayMultiplierUpdated = true;

        if (openGifFilenameByIndex(GIF_DIRECTORY, gifIndex) >= 0) {
            // Can clear screen for new animation here, but this might cause flicker with short animations
            // matrix.fillScreen(COLOR_BLACK);
            // matrix.swapBuffers();

            decoder.startDecoding();

            // Calculate time in the future to terminate animation
            displayEndTime_millis = now + (DISPLAY_TIME_SECONDS * 1000);
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

    // special case for the first frame - interpolate from the previous GIF to the first frame of the new GIF relatively quickly
    if(firstFrame)
        currentFrameDelay_ms = indexChangedFrameDelay_ms;

    uint32_t t;
    int32_t microsUntilChange;

    // wait for the delay associated with the frame (n-2), the frame that's currently the "previous" frame in backgroundLayer
    do {
        if(gifIndexChanged)
            return;

        if(millis() - lastSensorRead_millis > readSensorPeriod_ms) {
            lastSensorRead_millis = millis();

            average = (double)getSliderReading();
#if (DEBUG_PRINT_SLIDER_UPDATES == 1)
            Serial.println(average);
#endif

            if(average > 1.0)
                frameDelayMultiplier = (1000.0 * average) / 1024;
            else
                frameDelayMultiplier = 1.0;

            frameDelayMultiplierUpdated = true;
        }

        checkEncodersState();
        checkButtonsState();

        t = micros();

        microsUntilChange = ((nMinus2FrameDelay_ms * 1000) * frameDelayMultiplier) - (t - frameStartTime_micros);

        if(frameDelayMultiplierUpdated) {
            frameDelayMultiplierUpdated = false;

            backgroundLayer.updateInterpolationPeriod(microsUntilChange);
        }

        // special case for the first frame - stop interpolating the two previous frames of the old GIF, so we can interpolate to the new GIF quickly
        if(firstFrame) {
            microsUntilChange = 0;
            backgroundLayer.updateInterpolationPeriod(0);
        }

        // If we don't delay at least a bit SmartMatrix Library starts dropping frames
        if(microsUntilChange > 0)
            delayMicroseconds(min(microsUntilChange, 100));
    } while (microsUntilChange > 0);

    // we're now done with frame (n-2).  frame (n-1) is being displayed.  we're going to start interpolating from frame (n-1) to new frame (n)

    // capture the timestamp of when we last called swapBuffers() to begin interpolation between frame (n-1) and (n).  We'll call swapBuffers() again after waiting nMinus1FrameDelay_micros, regardless of how long the next call to decoder.decodeFrame(false) takes
    frameStartTime_micros = t;

    // swap buffers, begin interpolation between frame (n-1) and (n)
    if(!firstFrame) {
        backgroundLayer.swapBuffers(true, (nMinus1FrameDelay_ms * 1000) * frameDelayMultiplier);
    } else {
        backgroundLayer.swapBuffers(true, 0);
    }

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
