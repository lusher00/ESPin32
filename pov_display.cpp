#include "pov_display.h"
#include "motor_control.h"
#include "config.h"
#include "debug.h"
#include <Adafruit_NeoPixel.h>

// Global POV state
POVState povState;

// NeoPixel strip (defined in led_animation.cpp but we need access)
extern Adafruit_NeoPixel strip;

// Upload state
static uint8_t uploadAnimationId = 0;
static uint16_t uploadTotalFrames = 0;
static bool uploadInProgress = false;

void initPOV() {
  // Initialize POV state
  povState.enabled = false;
  povState.currentAnimation = 0;
  povState.currentFrame = 0;
  povState.revolutionCount = 0;
  povState.currentColumn = 0;
  
  // Initialize all animations as inactive
  for (int i = 0; i < MAX_ANIMATIONS; i++) {
    povState.animations[i].active = false;
    povState.animations[i].frames = nullptr;
  }
  
  DEBUG_INFO("✓ POV Display initialized");
}


// Called from encoder ISR on each sync pulse (once per revolution).
// Resets column to 0 and advances frame. Column advancing is handled
// by the hardware timer in motor_control.cpp — not here.
void IRAM_ATTR povEncoderUpdate() {
  if (!povState.enabled) return;

  POVAnimation* anim = &povState.animations[povState.currentAnimation];
  if (!anim->active) return;

  povState.currentColumn = 0;
  povState.revolutionCount++;

  if (povState.revolutionCount >= anim->revolutionsPerFrame) {
    povState.revolutionCount = 0;
    povState.currentFrame = (povState.currentFrame + 1) % anim->frameCount;
  }

  displayPOVColumn();
  povNeedsShow = true;  // signal main loop — no strip.show() in ISR
}

// Set when the pixel buffer has been updated and needs a strip.show()
static volatile bool povDirty = false;

// Called from the encoder ISR (or main loop) — updates pixel buffer only, no strip.show()
void IRAM_ATTR displayPOVColumn() {
  if (!povState.enabled) return;

  POVAnimation* anim = &povState.animations[povState.currentAnimation];
  if (!anim->active || !anim->frames) return;

  POVFrame* frame = &anim->frames[povState.currentFrame];
  POVColumn* col = &frame->columns[povState.currentColumn];

  for (int i = 0; i < POV_LEDS; i++) {
    strip.setPixelColor(i, col->rgb[i][0], col->rgb[i][1], col->rgb[i][2]);
  }
  povDirty = true;
}

// Called from main loop — strip.show() fallback if ISR show was missed.
void povShowTask() {
  if (povDirty) {
    povDirty = false;
    strip.show();
  }
}

void setPOVEnable(bool enable) {
  Serial.printf("[POV] setPOVEnable %d — before state change\n", enable);
  povState.enabled = enable;
  Serial.printf("[POV] setPOVEnable %d — after state change\n", enable);
  if (!enable) {
    stopPOVTimer();
    strip.clear();
    strip.show();
  } else {
    // Seed the column timer with the last known revolution period.
    // allTransitionAvg is the EMA of encoder pulse intervals (us/rev).
    // Defaults to 60ms (1 rev/s) if motor not yet spinning; self-corrects
    // on the first encoder pulse via encoderISR -> timerAlarmWrite.
    extern volatile unsigned long allTransitionAvg;
    unsigned long periodUs = (allTransitionAvg > 0) ? allTransitionAvg : 60000UL;
    startPOVTimer(periodUs);
    Serial.printf("[POV] timer started with period %lu us\n", periodUs);
  }
  Serial.printf("[POV] setPOVEnable %d — done\n", enable);
}

bool getPOVEnable() {
  return povState.enabled;
}

void selectAnimation(uint8_t animationId) {
  Serial.printf("[POV] selectAnimation called with id=%d, MAX=%d, active=%d, povEnabled=%d\n",
                animationId, MAX_ANIMATIONS,
                animationId < MAX_ANIMATIONS ? povState.animations[animationId].active : -1,
                povState.enabled);

  if (animationId >= MAX_ANIMATIONS) {
    Serial.println("[POV] FAILED: invalid ID");
    return;
  }

  if (!povState.animations[animationId].active) {
    Serial.printf("[POV] FAILED: animation %d not active\n", animationId);
    return;
  }
  
  povState.currentAnimation = animationId;
  povState.currentFrame = 0;
  povState.revolutionCount = 0;
  povState.currentColumn = 0;
  
  DEBUG_POVF("Selected animation: %d", animationId);
  
}

void setFrameTiming(uint16_t revolutionsPerFrame) {
  POVAnimation* anim = &povState.animations[povState.currentAnimation];
  anim->revolutionsPerFrame = revolutionsPerFrame;
  
  DEBUG_POVF("Frame timing set to %d revolutions per frame", revolutionsPerFrame);
  
  
}

uint32_t getRevolutionCount() {
  return povState.revolutionCount;
}

void resetRevolutionCount() {
  povState.revolutionCount = 0;
  DEBUG_POV("Revolution count reset");
}

uint16_t getCurrentFrame() {
  return povState.currentFrame;
}

bool startAnimationUpload(uint8_t animationId, uint16_t totalFrames) {
  if (animationId >= MAX_ANIMATIONS) {
    DEBUG_WARN("Invalid animation ID");
    return false;
  }
  
  if (totalFrames > MAX_POV_FRAMES) {
    Serial.println("Too many frames");
    return false;
  }
  
  // Allocate memory for frames
  size_t frameSize = sizeof(POVFrame) * totalFrames;
  Serial.printf("[POV] Allocating animation %d: %d frames × %d bytes = %d bytes\n",
                animationId, totalFrames, sizeof(POVFrame), frameSize);
  POVFrame* frames = (POVFrame*)malloc(frameSize);

  if (!frames) {
    Serial.printf("[POV] MALLOC FAILED for animation %d (need %d bytes, free heap: %u)\n",
                  animationId, frameSize, ESP.getFreeHeap());
    return false;
  }
  
  // Clear existing animation if any
  if (povState.animations[animationId].frames) {
    free(povState.animations[animationId].frames);
  }
  
  // Initialize new animation
  povState.animations[animationId].id = animationId;
  povState.animations[animationId].frameCount = totalFrames;
  povState.animations[animationId].revolutionsPerFrame = 1;
  povState.animations[animationId].frames = frames;
  povState.animations[animationId].active = true;
  
  // Zero out frame data
  memset(frames, 0, frameSize);
  
  uploadAnimationId = animationId;
  uploadTotalFrames = totalFrames;
  uploadInProgress = true;
  
  DEBUG_POVF("Started upload for animation %d with %d frames", animationId, totalFrames);
  
  
  
  
  
  return true;
}

bool uploadFrameData(uint16_t frameNum, uint8_t column, const uint8_t* rgbData) {
  if (!uploadInProgress) {
    DEBUG_WARN("No upload in progress");
    return false;
  }
  
  if (frameNum >= uploadTotalFrames) {
    Serial.printf("[POV] uploadFrameData: frameNum %d >= totalFrames %d\n", frameNum, uploadTotalFrames);
    return false;
  }
  
  if (column >= POV_COLUMNS) {
    DEBUG_WARN("Column number out of range");
    return false;
  }
  
  POVAnimation* anim = &povState.animations[uploadAnimationId];
  POVFrame* frame = &anim->frames[frameNum];
  POVColumn* col = &frame->columns[column];
  
  // Copy RGB data
  memcpy(col->rgb, rgbData, POV_LEDS * 3);
  
  return true;
}

bool endAnimationUpload() {
  if (!uploadInProgress) {
    DEBUG_WARN("No upload in progress");
    return false;
  }
  
  uploadInProgress = false;
  
  DEBUG_POVF("Upload complete for animation %d", uploadAnimationId);
  
  
  return true;
}

// ============================================================================
// PRE-COMPILED ANIMATIONS
// ============================================================================

// ============================================================================
// HELPER: HSV to RGB (integer, ISR-safe, no float needed at runtime)
// ============================================================================
static void hsvToRgb(uint8_t h, uint8_t s, uint8_t v, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (s == 0) { r = g = b = v; return; }
  uint8_t region = h / 43;
  uint8_t remainder = (h - (region * 43)) * 6;
  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
  switch (region) {
    case 0: r=v; g=t; b=p; break;
    case 1: r=q; g=v; b=p; break;
    case 2: r=p; g=v; b=t; break;
    case 3: r=p; g=q; b=v; break;
    case 4: r=t; g=p; b=v; break;
    default: r=v; g=p; b=q; break;
  }
}

// ============================================================================
// ANIMATION 0: Rainbow disc — full spectrum across columns, fades to center
// ============================================================================
void loadTestAnimation() {

  bool wasEnabled = getPOVEnable();
  setPOVEnable(false);          // stop timer ISR from interfering
  
  if (!startAnimationUpload(0, 1)) return;

  uint8_t rgbData[POV_LEDS * 3];

  for (uint8_t col = 0; col < POV_COLUMNS; col++) {
    uint8_t hue = (uint8_t)((uint32_t)col * 255 / POV_COLUMNS);
    for (uint8_t led = 0; led < POV_LEDS; led++) {
      uint8_t r, g, b;
      // Brightness fades from center (led=0) to edge (led=POV_LEDS-1)
      uint8_t brightness = 60 + (led * 195 / (POV_LEDS - 1));
      hsvToRgb(hue, 255, brightness, r, g, b);
      rgbData[led * 3 + 0] = r;
      rgbData[led * 3 + 1] = g;
      rgbData[led * 3 + 2] = b;
    }
    uploadFrameData(0, col, rgbData);
  }

  endAnimationUpload();
  if (wasEnabled) setPOVEnable(true);
  DEBUG_INFO("✓ Rainbow animation loaded");
}

// ============================================================================
// ANIMATION 1: Spokes — 6 white spokes on black background
// ============================================================================
void loadSpokesAnimation() {
  
  bool wasEnabled = getPOVEnable();
  setPOVEnable(false);          // stop timer ISR from interfering
  
  if (!startAnimationUpload(1, 1)) return;

  uint8_t rgbData[POV_LEDS * 3];
  const uint8_t NUM_SPOKES = 6;
  const uint8_t SPOKE_WIDTH = 1; // columns wide per spoke

  for (uint8_t col = 0; col < POV_COLUMNS; col++) {
    // Is this column part of a spoke?
    bool onSpoke = false;
    for (uint8_t s = 0; s < NUM_SPOKES; s++) {
      uint8_t spokeCol = (s * POV_COLUMNS) / NUM_SPOKES;
      if (col >= spokeCol && col < spokeCol + SPOKE_WIDTH) {
        onSpoke = true; break;
      }
    }

    for (uint8_t led = 0; led < POV_LEDS; led++) {
      if (onSpoke) {
        // White spoke, brighter at edge
        uint8_t brightness = 100 + (led * 155 / (POV_LEDS - 1));
        rgbData[led * 3 + 0] = brightness;
        rgbData[led * 3 + 1] = brightness;
        rgbData[led * 3 + 2] = brightness;
      } else {
        rgbData[led * 3 + 0] = 0;
        rgbData[led * 3 + 1] = 0;
        rgbData[led * 3 + 2] = 0;
      }
    }
    uploadFrameData(1, col, rgbData);
  }

  endAnimationUpload();
  if (wasEnabled) setPOVEnable(true);
  DEBUG_INFO("✓ Spokes animation loaded");
}

// ============================================================================
// ANIMATION 2: Bullseye — concentric colored rings
// ============================================================================
void loadBullseyeAnimation() {

  bool wasEnabled = getPOVEnable();
  setPOVEnable(false);          // stop timer ISR from interfering

  if (!startAnimationUpload(2, 1)) return;

  uint8_t rgbData[POV_LEDS * 3];
  // Ring colors: red, yellow, green, cyan, blue, magenta, white
  const uint8_t ringHues[] = { 0, 43, 85, 128, 170, 213, 0 };
  const uint8_t ringCount = 7;

  for (uint8_t col = 0; col < POV_COLUMNS; col++) {
    for (uint8_t led = 0; led < POV_LEDS; led++) {
      uint8_t ring = (led * ringCount) / POV_LEDS;
      uint8_t r, g, b;
      if (ring >= ringCount - 1) {
        r = g = b = 200; // white center ring
      } else {
        hsvToRgb(ringHues[ring], 255, 200, r, g, b);
      }
      rgbData[led * 3 + 0] = r;
      rgbData[led * 3 + 1] = g;
      rgbData[led * 3 + 2] = b;
    }
    uploadFrameData(2, col, rgbData);
  }

  endAnimationUpload();
  if (wasEnabled) setPOVEnable(true);
  DEBUG_INFO("✓ Bullseye animation loaded");
}

// ============================================================================
// ANIMATION 3: Spiral — hue shifts with both angle and radius
// ============================================================================
void loadSpiralAnimation() {

  bool wasEnabled = getPOVEnable();
  setPOVEnable(false);          // stop timer ISR from interfering

  if (!startAnimationUpload(3, 1)) return;

  uint8_t rgbData[POV_LEDS * 3];

  for (uint8_t col = 0; col < POV_COLUMNS; col++) {
    for (uint8_t led = 0; led < POV_LEDS; led++) {
      // Hue = angle contribution + radius contribution = spiral
      uint8_t angleHue = (uint8_t)((uint32_t)col * 255 / POV_COLUMNS);
      uint8_t radiusHue = (uint8_t)((uint32_t)led * 128 / POV_LEDS);
      uint8_t hue = angleHue + radiusHue; // wraps naturally
      uint8_t r, g, b;
      hsvToRgb(hue, 255, 220, r, g, b);
      rgbData[led * 3 + 0] = r;
      rgbData[led * 3 + 1] = g;
      rgbData[led * 3 + 2] = b;
    }
    uploadFrameData(3, col, rgbData);
  }

  endAnimationUpload();
  if (wasEnabled) setPOVEnable(true);
  DEBUG_INFO("✓ Spiral animation loaded");
}

// ============================================================================
// ANIMATION 4: Clock face — hour and minute hands, updates every revolution
// Uses getLocalTime() from ESP32 RTC (set via NTP or BLE time sync command)
// ============================================================================
void loadClockAnimation() {

  bool wasEnabled = getPOVEnable();
  setPOVEnable(false);          // stop timer ISR from interfering

  // Clock is rendered dynamically each revolution in updateClockAnimation()
  // This just sets up the animation slot with a placeholder frame
  if (!startAnimationUpload(4, 1)) return;

  uint8_t rgbData[POV_LEDS * 3];
  memset(rgbData, 0, sizeof(rgbData));

  // Draw a simple static clock face as placeholder
  for (uint8_t col = 0; col < POV_COLUMNS; col++) {
    memset(rgbData, 0, sizeof(rgbData));

    // Rim: outermost LED always white
    rgbData[(POV_LEDS - 1) * 3 + 0] = 80;
    rgbData[(POV_LEDS - 1) * 3 + 1] = 80;
    rgbData[(POV_LEDS - 1) * 3 + 2] = 80;

    // Hour markers: 12 evenly spaced bright ticks on rim
    uint8_t markerCols[12];
    for (uint8_t m = 0; m < 12; m++) {
      markerCols[m] = (m * POV_COLUMNS) / 12;
    }
    for (uint8_t m = 0; m < 12; m++) {
      if (col == markerCols[m]) {
        rgbData[(POV_LEDS - 1) * 3 + 0] = 255;
        rgbData[(POV_LEDS - 1) * 3 + 1] = 255;
        rgbData[(POV_LEDS - 1) * 3 + 2] = 255;
        // Tick extends 2 LEDs inward
        if (POV_LEDS >= 2) {
          rgbData[(POV_LEDS - 2) * 3 + 0] = 180;
          rgbData[(POV_LEDS - 2) * 3 + 1] = 180;
          rgbData[(POV_LEDS - 2) * 3 + 2] = 180;
        }
      }
    }

    // Hour hand: points to col 0 (12 o'clock = sync position) as placeholder
    // Gets updated dynamically by updateClockAnimation()
    if (col < 2) {
      // Red hour hand — inner 60% of radius
      uint8_t handLen = (POV_LEDS * 6) / 10;
      for (uint8_t led = 0; led < handLen; led++) {
        rgbData[led * 3 + 0] = 220;
        rgbData[led * 3 + 1] = 20;
        rgbData[led * 3 + 2] = 20;
      }
    }

    uploadFrameData(4, col, rgbData);
  }

  endAnimationUpload();
  if (wasEnabled) setPOVEnable(true);
  DEBUG_INFO("✓ Clock animation loaded (static placeholder - call updateClockAnimation() with time)");
}

// Call this with current time to refresh the clock display
void updateClockAnimation(uint8_t hour12, uint8_t minute, uint8_t second) {
  if (!povState.animations[4].active) {
    loadClockAnimation();
  }

  uint8_t rgbData[POV_LEDS * 3];

  // Map time to column indices
  // hour hand: 12 positions × (POV_COLUMNS/12) per hour + minute offset
  uint8_t hourCol   = (uint8_t)(((uint32_t)hour12 * POV_COLUMNS / 12)
                      + ((uint32_t)minute * POV_COLUMNS / (12 * 60))) % POV_COLUMNS;
  uint8_t minuteCol = (uint8_t)((uint32_t)minute * POV_COLUMNS / 60) % POV_COLUMNS;
  uint8_t secondCol = (uint8_t)((uint32_t)second * POV_COLUMNS / 60) % POV_COLUMNS;

  uint8_t hourLen   = (POV_LEDS * 55) / 100; // 55% of radius
  uint8_t minuteLen = (POV_LEDS * 80) / 100; // 80% of radius
  uint8_t secondLen = (POV_LEDS * 85) / 100; // 85% of radius

  for (uint8_t col = 0; col < POV_COLUMNS; col++) {
    memset(rgbData, 0, sizeof(rgbData));

    // Rim
    rgbData[(POV_LEDS - 1) * 3 + 0] = 50;
    rgbData[(POV_LEDS - 1) * 3 + 1] = 50;
    rgbData[(POV_LEDS - 1) * 3 + 2] = 50;

    // Hour markers
    for (uint8_t m = 0; m < 12; m++) {
      uint8_t markerCol = (m * POV_COLUMNS) / 12;
      if (col == markerCol) {
        rgbData[(POV_LEDS - 1) * 3 + 0] = 255;
        rgbData[(POV_LEDS - 1) * 3 + 1] = 255;
        rgbData[(POV_LEDS - 1) * 3 + 2] = 255;
        if (POV_LEDS >= 2) {
          rgbData[(POV_LEDS - 2) * 3 + 0] = 160;
          rgbData[(POV_LEDS - 2) * 3 + 1] = 160;
          rgbData[(POV_LEDS - 2) * 3 + 2] = 160;
        }
      }
    }

    // Second hand (thin, cyan)
    if (col == secondCol) {
      for (uint8_t led = 0; led < secondLen; led++) {
        rgbData[led * 3 + 0] = 0;
        rgbData[led * 3 + 1] = 200;
        rgbData[led * 3 + 2] = 200;
      }
    }

    // Minute hand (white, medium width)
    bool onMinute = (col == minuteCol) ||
                    (col == (minuteCol + 1) % POV_COLUMNS);
    if (onMinute) {
      for (uint8_t led = 0; led < minuteLen; led++) {
        rgbData[led * 3 + 0] = 220;
        rgbData[led * 3 + 1] = 220;
        rgbData[led * 3 + 2] = 220;
      }
    }

    // Hour hand (red, wide)
    bool onHour = (col == hourCol) ||
                  (col == (hourCol + 1) % POV_COLUMNS) ||
                  (col == (hourCol + POV_COLUMNS - 1) % POV_COLUMNS);
    if (onHour) {
      for (uint8_t led = 0; led < hourLen; led++) {
        rgbData[led * 3 + 0] = 240;
        rgbData[led * 3 + 1] = 30;
        rgbData[led * 3 + 2] = 30;
      }
    }

    // Center dot (white)
    rgbData[0] = 255; rgbData[1] = 255; rgbData[2] = 255;

    uploadFrameData(4, col, rgbData);
  }

  DEBUG_POVF("Clock updated: %02d:%02d:%02d", hour12, minute, second);
}

