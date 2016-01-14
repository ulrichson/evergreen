/*

  Evergreen

  This sketch detects soil humidity and allows to water plants via a water pump.
  Additionally it measures the sunlight.

  It is designed for the Minigarden vertical garden system (http://minigarden.net),
  altough this can be utilized for similar systems with an overflow water collection tank

*/

#include "SoftPWM.h"
#include "SoftPWM_timer.h"

#include "FastRunningMedian.h"        // From http://forum.arduino.cc/index.php?topic=53081.msg1160999#msg1160999

#include <U8glib.h>
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);

// Settings for mode filter
#define NUM_VALS_MEDIAN       32
#define MEDIAN_SAMPLES        6

// Size for buffer to store numeric values (for OLED display)
#define NUM_BUFFER_SIZE       8

// LED intensity
#define INTENSITY_MAX         1.0
#define INTENSITY_MIN         0.1
#define INTENSITY_DEFAULT     0.8

// LED animation
#define FADE_IN_TIME_MS       400
#define FADE_OUT_TIME_MS      700

// Soil sensor thresholds
// based on IKEA coconur soil
#define THR_DISCONNECTED      1000
#define THR_DRY               690
#define THR_HUMID             490

// LED RGB interpolation
#define INTERPOLATION_POWER   3
#define INTERPOLATION_BASE    1.05

// MOSFET gate (N-Channel: 0, P-Channel: 1023)
#define GATE_OFF              0

// INPUT Pins
#define potPin                0
#define lightPin              1
#define waterPin              2
#define soilPin               3
#define btnPin                8

// OUTPUT Pins

// Top LED
#define topRPin               11
#define topGPin               12
#define topBPin               13

// Bottom LED
#define botRPin               2
#define botGPin               3
#define botBPin               4

// MOSFET
#define gatePin               5

struct RGB {
  byte r;
  byte g;
  byte b;
};

enum FadingState {
  none,
  fadingOut,
  fadingIn
};

// Mode filter based on
// https://gist.github.com/ofZach/5082043
// http://www.elcojacobs.com/eleminating-noise-from-sensor-readings-on-arduino-with-digital-filtering/
/*class ModeFilter {

  public:

    float vals[NUM_VALS_MEDIAN];
    float valsSorted[NUM_VALS_MEDIAN];
    boolean firstVal;

    ModeFilter() {
      firstVal = true;
    }

    void addValue(float val) {
      if (firstVal == true) {
        for (int i = 0; i < NUM_VALS_MEDIAN; i++) valsSorted[i] = val;
        for (int i = 0; i < NUM_VALS_MEDIAN; i++) vals[i] = val;
      } else {
        for (int i = 1; i < NUM_VALS_MEDIAN; i++) vals[i - 1] = vals[i];
        vals[NUM_VALS_MEDIAN - 1] = val;
        for (int i = 0; i < NUM_VALS_MEDIAN; i++) valsSorted[i] = vals[i];
        // sort array
        bubbleSort();
      }
      firstVal = false;
    }

    float processValue(float val) {
      addValue(val);
      return getValue();
    }

    float getValue() {
      if (SPAN_MEAN == 0) {
        // Return simple median value
        return valsSorted[(int)floor(NUM_VALS_MEDIAN / 2)];
      } else {
        float vals = 0;
        int medianIndex = (int)floor(NUM_VALS_MEDIAN / 2);
        for (int i = medianIndex - SPAN_MEAN; i < (medianIndex + SPAN_MEAN); i++) {
          vals += valsSorted[i];
        }
        return vals / (SPAN_MEAN * 2);
      }
    }

    void bubbleSort() {
      int out, in;
      float swapper;
      for (out = 0 ; out < NUM_VALS_MEDIAN; out++) { // outer loop
        for (in = out; in < NUM_VALS_MEDIAN; in++) { // inner loop
          if ( valsSorted[in] > valsSorted[in + 1] ) { // out of order?
            // swap them:
            swapper = valsSorted[in];
            valsSorted[in] = valsSorted[in + 1];
            valsSorted[in + 1] = swapper;
          }
        }
      }
    }
  };*/

RGB colorDisconnected =       { 0  , 0  , 0   };
RGB colorDry          =       { 220, 50 , 0   };
RGB colorHumid        =       { 0  , 150, 200 };
RGB colorWater        =       { 0  , 50 , 250 };
RGB colorError        =       { 255, 0  , 0   };

// Variables
unsigned int btnState = 0;
unsigned int potValue = 0;
unsigned int lightValue = 0;
unsigned int soilValue = 0;
unsigned int waterValue = 0;

bool irrigationStarted = false;
unsigned long fadingStartTime = 0;
FadingState fadingState = none;
float intensity = INTENSITY_DEFAULT;

// Buffer for numeric values
char numBuffer[NUM_BUFFER_SIZE];

// ModeFilter soilFilter = ModeFilter();
// ModeFilter lightFilter = ModeFilter();
// ModeFilter waterFilter = ModeFilter();

FastRunningMedian<unsigned int, NUM_VALS_MEDIAN, 0> soilFilter;
FastRunningMedian<unsigned int, NUM_VALS_MEDIAN, 0> lightFilter;
FastRunningMedian<unsigned int, NUM_VALS_MEDIAN, 0> waterFilter;

void setup() {
  Serial.begin(9600);
  Serial.println("Welcome to Evergreen :)");

  // Setup pins
  pinMode(btnPin, INPUT);
  pinMode(gatePin, OUTPUT);
  analogWrite(gatePin, GATE_OFF);

  // Setup software PWM
  SoftPWMBegin();

  SoftPWMSet(topRPin, 0);
  SoftPWMSet(topGPin, 0);
  SoftPWMSet(topBPin, 0);
  SoftPWMSet(botRPin, 0);
  SoftPWMSet(botGPin, 0);
  SoftPWMSet(botBPin, 0);

  SoftPWMSetFadeTime(topRPin, FADE_IN_TIME_MS, FADE_OUT_TIME_MS);
  SoftPWMSetFadeTime(topGPin, FADE_IN_TIME_MS, FADE_OUT_TIME_MS);
  SoftPWMSetFadeTime(topBPin, FADE_IN_TIME_MS, FADE_OUT_TIME_MS);
  SoftPWMSetFadeTime(botRPin, FADE_IN_TIME_MS, FADE_OUT_TIME_MS);
  SoftPWMSetFadeTime(botGPin, FADE_IN_TIME_MS, FADE_OUT_TIME_MS);
  SoftPWMSetFadeTime(botBPin, FADE_IN_TIME_MS, FADE_OUT_TIME_MS);

  // Initialize median filters
  for (int i = 0; i < NUM_VALS_MEDIAN; i++) soilFilter.addValue(analogRead(soilPin));
  for (int i = 0; i < NUM_VALS_MEDIAN; i++) lightFilter.addValue(analogRead(lightPin));
  for (int i = 0; i < NUM_VALS_MEDIAN; i++) waterFilter.addValue(analogRead(waterPin));
}

void loop() {
  // Read sensor values
  btnState = digitalRead(btnPin);
  potValue = analogRead(potPin);

  // Smooth sensor readings
  soilFilter.addValue(analogRead(soilPin));
  lightFilter.addValue(analogRead(lightPin));
  waterFilter.addValue(analogRead(waterPin));

  soilValue = soilFilter.getAverage(MEDIAN_SAMPLES);
  lightValue = lightFilter.getAverage(MEDIAN_SAMPLES);
  waterValue = waterFilter.getAverage(MEDIAN_SAMPLES);

  // lightValue = analogRead(lightPin); // lightFilter.processValue(analogRead(lightPin));
  // soilValue = analogRead(soilPin); // soilFilter.processValue(analogRead(soilPin));
  // waterValue = analogRead(waterPin); // waterFilter.processValue(analogRead(waterPin));

  // Serial.println(lightValue);

  // Check button
  if (btnState == HIGH) {
    if (!irrigationStarted) {
      // Start irrigation
      irrigationStarted = true;
      Serial.println("Irrigation started");
      fadeOut();
    } else {
      // wait for the a fade change
      switch (fadingState) {
        case fadingOut:
          if (fadingStartTime + FADE_OUT_TIME_MS <= millis()) {
            fadeIn();
          }
          break;
        case fadingIn:
          if (fadingStartTime + FADE_IN_TIME_MS <= millis()) {
            fadeOut();
          }
          break;
        default:
          // do nothing
          break;
      }
    }
  } else {
    if (irrigationStarted) {
      // Stop irrigation
      irrigationStarted = false;
      fadingState = none;
      Serial.println("Irrigation stopped");
    }
    // display default soil moisture level
    intensity = INTENSITY_DEFAULT;
  }

  // Run motor when button is pressed
  // Serial.println(map(potValue, 0, 1023, 240, 1), DEC);
  if (irrigationStarted) {
    if (waterValue < THR_HUMID) {
      // Bottom sensor detected water draining from above -> no more watering to avoid collection tank overflow
      setColor(colorError);
    } else {
      analogWrite(gatePin, irrigationStarted ? map(potValue, 0, 1023, 1, 240) : GATE_OFF);
      // analogWrite(gatePin, irrigationStarted ? HIGH : GATE_OFF);

      // Display soil level
      displaySoilLevel(soilValue);
    }
  } else {
    // Display soil level
    analogWrite(gatePin, GATE_OFF);
    displaySoilLevel(soilValue);
  }

  // picture loop
  u8g.firstPage();
  do {
    draw();
  } while (u8g.nextPage());
}

void draw() {
  u8g.setFont(u8g_font_unifont);
  snprintf (numBuffer, NUM_BUFFER_SIZE, "%d", soilValue);
  u8g.drawStr(0, 20, "Soil: ");
  u8g.drawStr(70, 20, numBuffer);
  snprintf (numBuffer, NUM_BUFFER_SIZE, "%d", lightValue);
  u8g.drawStr(0, 40, "Light: ");
  u8g.drawStr(70, 40, numBuffer);
  snprintf (numBuffer, NUM_BUFFER_SIZE, "%d", waterValue);
  u8g.drawStr(0, 60, "Water: ");
  u8g.drawStr(70, 60, numBuffer);
}

void fadeIn() {
  intensity = INTENSITY_MAX;
  fadingState = fadingIn;
  fadingStartTime = millis();
}

void fadeOut() {
  intensity = INTENSITY_MIN;
  fadingState = fadingOut;
  fadingStartTime = millis();
}

RGB getLinearColorInterpolation(RGB startCol, RGB endCol, float p) {
  if (p < 0.0) p = 0.0;
  if (p > 1.0) p = 1.0;

  RGB ret = {
    byte((float)startCol.r * p + (float)endCol.r * (1.0 - p)),
    byte((float)startCol.g * p + (float)endCol.g * (1.0 - p)),
    byte((float)startCol.b * p + (float)endCol.b * (1.0 - p))
  };

  return ret;
}

RGB getPotentialColorInterpolation(RGB startCol, RGB endCol, int val, int minVal, int maxVal, float power) {
  if (val > maxVal) val = maxVal;
  if (val < minVal) val = minVal;

  double p = (pow(val, power) - pow(minVal, power)) / (pow(maxVal, power) - pow(minVal, power));

  RGB ret = {
    byte((float)startCol.r * p + (float)endCol.r * (1.0 - p)),
    byte((float)startCol.g * p + (float)endCol.g * (1.0 - p)),
    byte((float)startCol.b * p + (float)endCol.b * (1.0 - p))
  };

  return ret;
}

RGB getExponentialColorInterpolation(RGB startCol, RGB endCol, int val, int minVal, int maxVal, float base) {
  if (val > maxVal) val = maxVal;
  if (val < minVal) val = minVal;

  double p = (pow(base, val) - pow(base, minVal)) / (pow(base, maxVal) - pow(base, minVal));

  RGB ret = {
    byte((float)startCol.r * p + (float)endCol.r * (1.0 - p)),
    byte((float)startCol.g * p + (float)endCol.g * (1.0 - p)),
    byte((float)startCol.b * p + (float)endCol.b * (1.0 - p))
  };

  return ret;
}

void displaySoilLevel(float val)
{
  if (val >= THR_DISCONNECTED) {
    // Sensor disconnected
    setColor(colorDisconnected);
  } else if (val < THR_HUMID) {
    // Sensor in water
    setColor(colorWater, intensity);
  } else {
    // Sensor detects plausible soil levels
    float valNormalized = float(val - THR_HUMID) / float(THR_DISCONNECTED - THR_HUMID);

    // RGB col = val > THR_DRY ? colorDry : getPotentialColorInterpolation(colorDry, colorHumid, val, THR_HUMID, THR_DRY, INTERPOLATION_POWER);
    RGB col = val > THR_DRY ? colorDry : getExponentialColorInterpolation(colorDry, colorHumid, val, THR_HUMID, THR_DRY, INTERPOLATION_BASE);
    setColor(col, (1.0 - valNormalized) * intensity, intensity);
  }
}

void setColor(RGB col) {
  setColor(col, 1.0, 1.0);
}

void setColor(RGB col, float intensity) {
  setColor(col, intensity, intensity);
}

void setColor(RGB col, float topIntensity, float bottomIntensity) {
  SoftPWMSet(topRPin, byte(col.r * topIntensity));
  SoftPWMSet(topGPin, byte(col.g * topIntensity));
  SoftPWMSet(topBPin, byte(col.b * topIntensity));

  SoftPWMSet(botRPin, byte(col.r * bottomIntensity));
  SoftPWMSet(botGPin, byte(col.g * bottomIntensity));
  SoftPWMSet(botBPin, byte(col.b * bottomIntensity));
}
