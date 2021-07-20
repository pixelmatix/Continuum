/*
 * This sketch is based off the AnimatedGIFs sample sketch in the SmartMatrix Library,
 *   modified to vary the frame rate from 1x (realtime) to up to 1/1000x with smooth
 *   interpolation between frames
 *
 *
 * See the Continuum tutorial on Instructables for more details
 */

#include <MatrixHardware_Teensy4_ShieldV5.h>        // SmartLED Shield for Teensy 4 (V5)
#include <SmartMatrix.h>
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
const uint8_t apaStripLeftLength = 28;
const uint8_t apaStripTopLength = 28;
const uint8_t apaStripRightLength = 29;
const uint8_t apaStripBottomLength = 33;
const uint8_t apaStripLength = apaStripLeftLength + apaStripTopLength + apaStripRightLength + apaStripBottomLength;

// adjust this to your APA matrix/strip - set kApaMatrixHeight to 1 for a strip
const uint8_t kApaMatrixWidth = apaStripLength;
const uint8_t kApaMatrixHeight = 1;
const uint8_t kApaRefreshDepth = 36;        // known working: 36
const uint8_t kApaDmaBufferRows = 1;        // known working: 1
const uint8_t kApaPanelType = 0;            // not used for APA matrices as of now
const uint8_t kApaMatrixOptions = (SMARTMATRIX_OPTIONS_NONE);      // no options for APA matrices as of not 
const uint8_t kApaBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);

SMARTMATRIX_APA_ALLOCATE_BUFFERS(apamatrix, kApaMatrixWidth, kApaMatrixHeight, kApaRefreshDepth, kApaDmaBufferRows, kApaPanelType, kApaMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_INTERPOLATION_LAYER(apaBackgroundLayer, kApaMatrixWidth, kApaMatrixHeight, COLOR_DEPTH, kApaBackgroundLayerOptions);
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
#define SD_CS BUILTIN_SDCARD

// Teensy SD Library requires a trailing slash in the directory name
#define GIF_DIRECTORY "/gifs/"

int num_files;

void screenClearCallback(void) {
  backgroundLayer.fillScreen({0,0,0});
}

void updateScreenCallback(void) {
    // not used in this application
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
        apamatrix.setBrightness(brightness);

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
    decoder.setFileSizeCallback(fileSizeCallback);

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
    matrix.setRefreshRate(240);


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
const int minBrightness = 120;

// function copied from Adafruit Adalight, but disabled for some reason (I took a > 6 month break between writing and attempting to document this code)
rgb24 pixelToMinBrightness(rgb24 pixel) {
#if 1
    int sum = pixel.red + pixel.green + pixel.blue;
    if(sum < minBrightness) {
        if(sum == 0) { // To avoid divide-by-zero
#if 0
            int deficit = minBrightness / 3; // Spread equally to R,G,B
            pixel.red += deficit;
            pixel.green += deficit;
            pixel.blue += deficit;
#endif
        } else {
#if 0
            int deficit = minBrightness - sum;
            int s2      = sum * 2;
            // Spread the "brightness deficit" back into R,G,B in proportion to
            // their individual contribition to that deficit.  Rather than simply
            // boosting all pixels at the low end, this allows deep (but saturated)
            // colors to stay saturated...they don't "pink out."
            pixel.red  += deficit * (sum - pixel.red ) / s2;
            pixel.green += deficit * (sum - pixel.green) / s2;
            pixel.blue += deficit * (sum - pixel.blue) / s2;
#else
            int factor = (minBrightness * 256)/sum;
            pixel.red = (pixel.red * factor)/256;
            pixel.green = (pixel.green * factor)/256;
            pixel.blue = (pixel.blue * factor)/256;
#endif
        }
    }
    return pixel;
#else
    delayMicroseconds(10);
#endif    
}

void fillApaBackgroundLayer(void) {
    // go around display collecting color of pixels (TODO: averaging over area), and filling strip with values
    for(int i=0; i<apaStripLeftLength; i++) {
        // left side, from bottom to top
        rgb24 pixel = backgroundLayer.readPixel(0, kMatrixHeight - 1 - (i*kMatrixHeight)/apaStripLeftLength);
        pixel = pixelToMinBrightness(pixel);
        apaBackgroundLayer.drawPixel(i, 0, pixel);
    }

    for(int i=0; i<apaStripTopLength; i++) {
        // top side, from left to right
        rgb24 pixel = backgroundLayer.readPixel((i*kMatrixWidth)/apaStripTopLength, 0);
        pixel = pixelToMinBrightness(pixel);
        apaBackgroundLayer.drawPixel(i + apaStripLeftLength, 0, pixel);
    }

    for(int i=0; i<apaStripRightLength; i++) {
        // bottom side, from right to left
        rgb24 pixel = backgroundLayer.readPixel(kMatrixHeight-1, 0 + (i*kMatrixHeight)/apaStripRightLength);
        pixel = pixelToMinBrightness(pixel);
        apaBackgroundLayer.drawPixel(i + apaStripLeftLength + apaStripTopLength, 0, pixel);
    }

    for(int i=0; i<apaStripBottomLength; i++) {
        // bottom side, from right to left 
        rgb24 pixel = backgroundLayer.readPixel(kMatrixWidth - 1 - (i*kMatrixWidth)/apaStripBottomLength, 0);
        pixel = pixelToMinBrightness(pixel);
        apaBackgroundLayer.drawPixel(i + apaStripLeftLength + apaStripTopLength + apaStripRightLength, 0, pixel);
    }
}
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
    if(now >= displayEndTime_millis || decoder.getCycleNumber() > NUMBER_FULL_CYCLES)
#else
    // alt behavior is to play the gif until both DISPLAY_TIME_SECONDS and NUMBER_FULL_CYCLES have passed
    if(now >= displayEndTime_millis && decoder.getCycleNumber() > NUMBER_FULL_CYCLES)
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

    fillApaBackgroundLayer();

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
            apaBackgroundLayer.updateInterpolationPeriod(microsUntilChange);
        }

        // special case for the first frame - stop interpolating the two previous frames of the old GIF, so we can interpolate to the new GIF quickly
        if(firstFrame) {
            microsUntilChange = 0;
            backgroundLayer.updateInterpolationPeriod(0);
            apaBackgroundLayer.updateInterpolationPeriod(0);
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
        apaBackgroundLayer.swapBuffers(true, (nMinus1FrameDelay_ms * 1000) * frameDelayMultiplier);
    } else {
        backgroundLayer.swapBuffers(true, 0);
        apaBackgroundLayer.swapBuffers(true, 0);
    }

    // we're done with this frame loop, setup for the next loop where frame (n) becomes (n-1), (n-1) becomes (n-2)
    nMinus2FrameDelay_ms = nMinus1FrameDelay_ms;
    nMinus1FrameDelay_ms = currentFrameDelay_ms;

#if (DEBUG_PRINT_FRAMESTATS == 1)
    Serial.print(decoder.getFrameNumber());
    Serial.print(" ");
    Serial.print(decoder.getFrameDelay_ms());
    Serial.print(" ");
    Serial.print(matrix.getRefreshRate());
    Serial.print(" ");
    Serial.println(timeToDecode_ms);
#endif
}
