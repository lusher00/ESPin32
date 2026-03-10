#include "led_animation.h"
#include "config.h"
#include "motor_control.h"
#include <esp_task_wdt.h>

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastUpdate = 0;
uint16_t animationStep = 0;
uint8_t currentPattern = 0;  // Pattern selector

// Pattern timing
const unsigned long PATTERN_CHANGE_INTERVAL = 10000; // Change pattern every 10 seconds
unsigned long lastPatternChange = 0;

void initLEDs() {
  strip.begin();
  strip.setBrightness(200); // Reduce brightness for spinning (less blur)
  strip.show(); // Initialize all pixels to 'off'
  Serial.println("✓ NeoPixel strip initialized");
}

// Helper: Create rainbow color from position
uint32_t rainbow(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85) {
    return strip.Color(255 - pos * 3, 0, pos * 3);
  } else if (pos < 170) {
    pos -= 85;
    return strip.Color(0, pos * 3, 255 - pos * 3);
  } else {
    pos -= 170;
    return strip.Color(pos * 3, 255 - pos * 3, 0);
  }
}

// Pattern 1: Radial rainbow (creates color wheel when spinning)
void patternRainbowWheel() {
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    uint8_t colorIndex = (i * 256 / LED_COUNT + animationStep) & 255;
    strip.setPixelColor(i, rainbow(colorIndex));
  }
  animationStep++;
}

// Pattern 2: Speed indicator (visual tachometer)
void patternSpeedometer() {
  strip.clear();
  
  // Map RPM to number of lit LEDs (0-1500 RPM → 0-34 LEDs)
  int litCount = map(constrain(currentRPM, 0, 1500), 0, 1500, 0, LED_COUNT);
  
  // Color changes with speed: green → yellow → red
  for (int i = 0; i < litCount; i++) {
    uint32_t color;
    if (i < LED_COUNT / 3) {
      color = strip.Color(0, 255, 0); // Green (low speed)
    } else if (i < (LED_COUNT * 2) / 3) {
      color = strip.Color(255, 255, 0); // Yellow (medium)
    } else {
      color = strip.Color(255, 0, 0); // Red (high speed)
    }
    strip.setPixelColor(i, color);
  }
}

// Pattern 3: Spinning segments (creates pie slices)
void patternSegments() {
  strip.clear();
  
  // Create 4 colored segments
  int segmentSize = LED_COUNT / 4;
  uint32_t colors[4] = {
    strip.Color(255, 0, 0),   // Red
    strip.Color(0, 255, 0),   // Green
    strip.Color(0, 0, 255),   // Blue
    strip.Color(255, 255, 0)  // Yellow
  };
  
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    int segment = (i + animationStep) % LED_COUNT / segmentSize;
    if (segment < 4) {
      strip.setPixelColor(i, colors[segment]);
    }
  }
  animationStep = (animationStep + 1) % LED_COUNT;
}

// Pattern 4: Comet trail (single bright pixel with fade)
void patternComet() {
  strip.clear();
  
  int cometPos = animationStep % LED_COUNT;
  
  // Draw comet with trailing tail
  for (int i = 0; i < 8; i++) {
    int pos = (cometPos - i + LED_COUNT) % LED_COUNT;
    int brightness = 255 - (i * 30);
    if (brightness > 0) {
      strip.setPixelColor(pos, strip.Color(0, brightness, 255));
    }
  }
  
  animationStep++;
}

// Pattern 5: Strobe/pulse (synced with rotation for POV effect)
void patternStrobe() {
  uint8_t brightness = (sin(animationStep * 0.1) + 1) * 127; // Pulse 0-255
  
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(brightness, brightness, 255));
  }
  
  animationStep++;
}

// Pattern 6: Alternating bands (creates spiral when spinning)
void patternSpiral() {
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    if ((i + animationStep) % 4 < 2) {
      strip.setPixelColor(i, strip.Color(255, 0, 255)); // Magenta
    } else {
      strip.setPixelColor(i, strip.Color(0, 255, 255)); // Cyan
    }
  }
  animationStep = (animationStep + 1) % 4;
}

// Pattern 7: Fire effect (warm colors creating ring of fire)
void patternFire() {
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    // Random flicker
    int flicker = random(0, 150);
    int r = 255;
    int g = random(0, 100) + flicker;
    int b = 0;
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
}

void animationTask() {
  if (millis() - lastUpdate < ANIMATION_INTERVAL) return;
  lastUpdate = millis();

  // Auto-change patterns every 10 seconds
  if (millis() - lastPatternChange > PATTERN_CHANGE_INTERVAL) {
    currentPattern = (currentPattern + 1) % 7;
    lastPatternChange = millis();
    Serial.print("Switching to pattern: ");
    Serial.println(currentPattern);
  }

  // Run current pattern
  switch (currentPattern) {
    case 0:
      patternRainbowWheel();
      break;
    case 1:
      patternSpeedometer();
      break;
    case 2:
      patternSegments();
      break;
    case 3:
      patternComet();
      break;
    case 4:
      patternStrobe();
      break;
    case 5:
      patternSpiral();
      break;
    case 6:
      patternFire();
      break;
  }
  
  strip.show();
  esp_task_wdt_reset();  // strip.show() blocks ~37ms — feed watchdog to prevent silent reset
}
void setLEDPattern(uint8_t pattern) {
  currentPattern = pattern % 7;
  animationStep = 0;
  lastPatternChange = millis(); // Reset auto-change timer
}
